#include "utils/sdf/sparse_octree_builder.h"
#include "similarity_compression_impl.h"
#include "omp.h"
#include <atomic>
#include <cassert>
#include <chrono>

struct COctreeV3ThreadCtx
{
  std::vector<float> values_tmp;
  std::vector<uint32_t> u_values_tmp;
  std::vector<bool> presence_flags;
  std::vector<bool> requirement_flags;
  std::vector<bool> distance_flags;
};

struct COctreeV3BuildTask
{
  COctreeV3BuildTask() = default;
  COctreeV3BuildTask(unsigned _nodeId, uint3 _p, unsigned _lod_size) : nodeId(_nodeId), p(_p), lod_size(_lod_size) {}
  unsigned nodeId;
  uint3 p;
  unsigned lod_size;
};

struct CompressedClusterInfo
{
  CompressedClusterInfo() = default;
  CompressedClusterInfo(float _add_min, float _add_max) : add_min(_add_min), add_max(_add_max) {}
  std::vector<int> original_node_ids;
  float add_min = 0.0f;
  float add_max = 0.0f;
  bool trivial_transform = true;
};

struct COctreeV3GlobalCtx
{
  std::vector<float> compressed_data;            // data for all leaf nodes remained after similarity compression
  std::vector<uint32_t> compact_leaf_offsets;    // offsets in compressed_data array, for each leaf left after similarity compression
                                                 // filled in global_octree_to_compact_octree_v3_rec
  std::vector<uint32_t> compact_leaf_types;      // leaf types for each leaf left after similarity compression
  std::vector<uint32_t> node_id_cleaf_id_remap;  // ids of leaf node in list of leaves left after similarity compression,
                                                 // for each node from global octree (0 for non-leaf nodes)
  std::vector<uint32_t> node_id_compact_offset;  // offsets in compact array, for each node from global octree
                                                 // that is already processed
  std::vector<uint32_t> node_id_leaf_type;       // leaf type for each leaf node from global octree that is already processed
  std::vector<scom::TransformCompact> transforms;// similarity compression transform codes for each node from global octree
  std::vector<CompressedClusterInfo> cluster_infos;
  std::vector<uint32_t> compact;
};

static std::atomic<unsigned> stat_leaf_bytes(0);
static std::atomic<unsigned> stat_nonleaf_bytes(0);
static constexpr uint32_t m_bitcount[256] = {
    0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8};

static unsigned bitcount(uint32_t x)
{
  return m_bitcount[x & 0xff] + m_bitcount[(x >> 8) & 0xff] + m_bitcount[(x >> 16) & 0xff] + m_bitcount[x >> 24];
}

static bool is_leaf(unsigned offset)
{
  return (offset == 0) || (offset & INVALID_IDX);
}

uint32_t get_transform_code(const COctreeV3Header &header, const scom::TransformCompact &t, float lod_size)
{
  float sz_inv = 1.0f/lod_size;

  assert(t.rotation_id < scom::ROT_COUNT);
  assert(t.add > -1.73205081f*sz_inv && t.add < 1.73205081f*sz_inv);

  uint32_t rot_code = t.rotation_id;
  uint32_t add_code = uint32_t(((0.5f/1.73205081f)*t.add*lod_size + 0.5f) * float(header.add_mask)) & header.add_mask;

  float add_restored = 1.73205081f*sz_inv*(2*(float(add_code & header.add_mask) / float(header.add_mask)) - 1);
  //printf("add, restored: %f, %f\n", t.add, add_restored);

  assert((add_code|rot_code) != 0); //0 transform will be interpreted as non-leaf node, it is not allowed

  return add_code | rot_code;
}

namespace sdf_converter
{
  void BVC_calculate_flags(const GlobalOctree &octree, const COctreeV3Header &header,
                           COctreeV3ThreadCtx &ctx, unsigned idx, unsigned lod_size, uint3 p,
                           float add_min, float add_max)
  {
    int v_size = header.brick_size + 2 * header.brick_pad + 1;
    int p_size = header.brick_size + 2 * header.brick_pad;

    for (int i = 0; i < v_size * v_size * v_size; i++)
      ctx.distance_flags[i] = false;

    for (int i = 0; i < p_size * p_size * p_size; i++)
    {
      ctx.presence_flags[i] = false;
      ctx.requirement_flags[i] = false;
    }

#define V_OFF(off, i, j, k) off + (i) * v_size *v_size + (j) * v_size + (k)
#define P_OFF(off, i, j, k) off + (i) * p_size *p_size + (j) * p_size + (k)

    for (int i = -(int)header.brick_pad; i < (int)(header.brick_size + header.brick_pad); i++)
    {
      for (int j = -(int)header.brick_pad; j < (int)(header.brick_size + header.brick_pad); j++)
      {
        for (int k = -(int)header.brick_pad; k < (int)(header.brick_size + header.brick_pad); k++)
        {
          unsigned off = SBS_v_to_i(i, j, k, v_size, header.brick_pad);
          float min_val = ctx.values_tmp[V_OFF(off, 0, 0, 0)];
          float max_val = ctx.values_tmp[V_OFF(off, 0, 0, 0)];

          min_val = std::min(min_val, ctx.values_tmp[V_OFF(off, 0, 0, 1)]);
          min_val = std::min(min_val, ctx.values_tmp[V_OFF(off, 0, 1, 0)]);
          min_val = std::min(min_val, ctx.values_tmp[V_OFF(off, 0, 1, 1)]);
          min_val = std::min(min_val, ctx.values_tmp[V_OFF(off, 1, 0, 0)]);
          min_val = std::min(min_val, ctx.values_tmp[V_OFF(off, 1, 0, 1)]);
          min_val = std::min(min_val, ctx.values_tmp[V_OFF(off, 1, 1, 0)]);
          min_val = std::min(min_val, ctx.values_tmp[V_OFF(off, 1, 1, 1)]);

          max_val = std::max(max_val, ctx.values_tmp[V_OFF(off, 0, 0, 1)]);
          max_val = std::max(max_val, ctx.values_tmp[V_OFF(off, 0, 1, 0)]);
          max_val = std::max(max_val, ctx.values_tmp[V_OFF(off, 0, 1, 1)]);
          max_val = std::max(max_val, ctx.values_tmp[V_OFF(off, 1, 0, 0)]);
          max_val = std::max(max_val, ctx.values_tmp[V_OFF(off, 1, 0, 1)]);
          max_val = std::max(max_val, ctx.values_tmp[V_OFF(off, 1, 1, 0)]);
          max_val = std::max(max_val, ctx.values_tmp[V_OFF(off, 1, 1, 1)]);

          min_val += add_min;
          max_val += add_max;

          if (min_val <= 0 && max_val >= 0)
          {
            // this voxels contains the surface, set the presence flag
            ctx.presence_flags[(i + (int)header.brick_pad) * p_size * p_size + (j + (int)header.brick_pad) * p_size + k + (int)header.brick_pad] = true;
            ctx.requirement_flags[(i + (int)header.brick_pad) * p_size * p_size + (j + (int)header.brick_pad) * p_size + k + (int)header.brick_pad] = true;

            // if this voxel is inside and we have padding for smooth normals, we should set requirement flags:
            // 1) on this voxel, as it has the actual surface we want to render
            // 2) it's neighbors (both on padding and non-padding voxels), if they contain surface, they
            //    will be needed for normals smoothing
            if (header.brick_pad > 0 &&
                i >= 0 && i < header.brick_size &&
                j >= 0 && j < header.brick_size &&
                k >= 0 && k < header.brick_size)
            {
              int p_off = (i + (int)header.brick_pad) * p_size * p_size + (j + (int)header.brick_pad) * p_size + k + (int)header.brick_pad;

              for (int i = 0; i < 4; i++)
              {
                // if this edge contains surface, we need neighbors of this edge for normals smoothing

                // y axis
                if (ctx.values_tmp[V_OFF(off, 0, i / 2, i % 2)] * ctx.values_tmp[V_OFF(off, 1, i / 2, i % 2)] < 0)
                {
                  ctx.requirement_flags[P_OFF(p_off, 0, i / 2 - 1, i % 2 - 1)] = true;
                  ctx.requirement_flags[P_OFF(p_off, 0, i / 2 - 1, i % 2)] = true;
                  ctx.requirement_flags[P_OFF(p_off, 0, i / 2, i % 2 - 1)] = true;
                  ctx.requirement_flags[P_OFF(p_off, 0, i / 2, i % 2 - 1)] = true;
                }

                // y axis
                if (ctx.values_tmp[V_OFF(off, i / 2, 0, i % 2)] * ctx.values_tmp[V_OFF(off, i >> 1, 1, i & 1)] < 0)
                {
                  ctx.requirement_flags[P_OFF(p_off, i / 2 - 1, 0, i % 2 - 1)] = true;
                  ctx.requirement_flags[P_OFF(p_off, i / 2 - 1, 0, i % 2)] = true;
                  ctx.requirement_flags[P_OFF(p_off, i / 2, 0, i % 2 - 1)] = true;
                  ctx.requirement_flags[P_OFF(p_off, i / 2, 0, i % 2)] = true;
                }

                // z axis
                if (ctx.values_tmp[V_OFF(off, i / 2, i % 2, 0)] * ctx.values_tmp[V_OFF(off, i / 2, i % 2, 1)] < 0)
                {
                  ctx.requirement_flags[P_OFF(p_off, i / 2 - 1, i % 2, 0)] = true;
                  ctx.requirement_flags[P_OFF(p_off, i / 2 - 1, i % 2 - 1, 0)] = true;
                  ctx.requirement_flags[P_OFF(p_off, i / 2, i % 2, 0)] = true;
                  ctx.requirement_flags[P_OFF(p_off, i / 2, i % 2 - 1, 0)] = true;
                }
              }
            }
          }
        }
      }
    }

    // fill distance flags array according to presence flags
    for (int i = -(int)header.brick_pad; i < (int)(header.brick_size + header.brick_pad); i++)
    {
      for (int j = -(int)header.brick_pad; j < (int)(header.brick_size + header.brick_pad); j++)
      {
        for (int k = -(int)header.brick_pad; k < (int)(header.brick_size + header.brick_pad); k++)
        {
          unsigned p_off = (i + (int)header.brick_pad) * p_size * p_size + (j + (int)header.brick_pad) * p_size + k + (int)header.brick_pad;
          unsigned off = SBS_v_to_i(i, j, k, v_size, header.brick_pad);

          ctx.presence_flags[p_off] = ctx.presence_flags[p_off] && ctx.requirement_flags[p_off];

          if (ctx.presence_flags[p_off]) // this voxel is both present and required
          {
            ctx.distance_flags[off] = true;
            ctx.distance_flags[off + 1] = true;
            ctx.distance_flags[off + v_size] = true;
            ctx.distance_flags[off + v_size + 1] = true;
            ctx.distance_flags[off + v_size * v_size] = true;
            ctx.distance_flags[off + v_size * v_size + 1] = true;
            ctx.distance_flags[off + v_size * v_size + v_size] = true;
            ctx.distance_flags[off + v_size * v_size + v_size + 1] = true;
          }
        }
      }
    }

    /*
    for (int i=0;i<p_size;i++)
    {
      printf("slice %d\n", i);
      for (int j=0;j<p_size;j++)
      {
        for (int k=0;k<p_size;k++)
        {
          if (presence_flags[i*p_size*p_size + j*p_size + k])
            printf("#");
          else
            printf("_");
        }
        printf("\n");
      }
      printf("\n\n");
    }
    */
  }

  void BVC_fill_range(const COctreeV3Header &header, COctreeV3ThreadCtx &ctx, unsigned off_3,
                      /*out*/ float &min_active, float &max_active)
  {
    int v_size = header.brick_size + 2 * header.brick_pad + 1;

    min_active = 1000;
    max_active = -1000;
    for (int i = 0; i < v_size * v_size * v_size; i++)
    {
      if (ctx.distance_flags[i])
      {
        min_active = std::min(min_active, ctx.values_tmp[i]);
        max_active = std::max(max_active, ctx.values_tmp[i]);
      }
    }
    float range_active = max_active - min_active;
    assert(range_active > 0 && range_active < 1);
    //we cannot guarante min_active < 0 with similarity compression
    min_active = std::min(min_active, 0.0f);

    uint32_t min_comp = (-min_active) * float(0xFFFFFFFFu);
    uint32_t range_comp = range_active * float(0xFFFFFFFFu);

    ctx.u_values_tmp[off_3 + 0] = min_comp;
    ctx.u_values_tmp[off_3 + 1] = range_comp;
  }

  void BVC_fill_texture_coordinates(const GlobalOctree &octree, const COctreeV3Header &header, COctreeV3ThreadCtx &ctx, 
                                    unsigned idx, unsigned off_4)
  {
    if (header.uv_size > 0)
    {
      assert(header.uv_size == 1); // only 16 bit precision is supported now
      for (int i = 0; i < 8; i++)
      {
        float u_comp = 0xFFFF * LiteMath::clamp(octree.nodes[idx].tex_coords[i].x, 0.0f, 1.0f);
        float v_comp = 0xFFFF * LiteMath::clamp(octree.nodes[idx].tex_coords[i].y, 0.0f, 1.0f);
        ctx.u_values_tmp[off_4 + i] = (unsigned)u_comp << 16 | (unsigned)v_comp;
      }
    }
  }

  void BVC_fill_presence_flags(const COctreeV3Header &header, COctreeV3ThreadCtx &ctx, unsigned off_0)
  {
    int p_size = header.brick_size + 2 * header.brick_pad;
    for (int i = 0; i < p_size * p_size * p_size; i++)
      ctx.u_values_tmp[off_0 + i / 32] |= (unsigned)ctx.presence_flags[i] << (i % 32);
  }

  void BVC_sliced_fill_distance_flags(const COctreeV3Header &header, COctreeV3ThreadCtx &ctx, unsigned off_1, unsigned off_2,
                                      unsigned line_distances_offset_bits, unsigned line_distances_offsets_per_uint,
                                      unsigned slice_distance_flags_uints)
  {
    int v_size = header.brick_size + 2 * header.brick_pad + 1;

    unsigned cur_distances_offset = 0;
    for (int s_i = 0; s_i < v_size; s_i++)
    {
      // offset for distances in this slice
      assert(cur_distances_offset < (1 << line_distances_offset_bits));
      ctx.u_values_tmp[off_2 + s_i / line_distances_offsets_per_uint] |= cur_distances_offset << (line_distances_offset_bits * (s_i % line_distances_offsets_per_uint));
      // printf("offset %u put to slice %u\n", cur_distances_offset, s_i);

      for (int k = 0; k < v_size * v_size; k++)
      {
        unsigned distance_bit = (unsigned)ctx.distance_flags[s_i * v_size * v_size + k];
        // if (distance_bit)
        //   printf("Adistance %d put to %u\n", s_i*v_size*v_size + k, cur_distances_offset);
        ctx.u_values_tmp[off_1 + slice_distance_flags_uints * s_i + k / 32] |= distance_bit << (k % 32);
        cur_distances_offset += distance_bit;
      }
    }
  }

  void BVC_fill_distance_flags(const COctreeV3Header &header, COctreeV3ThreadCtx &ctx, unsigned off_1)
  {
    int v_size = header.brick_size + 2 * header.brick_pad + 1;
    for (int i = 0; i < v_size * v_size * v_size; i++)
      ctx.u_values_tmp[off_1 + i / 32] |= (unsigned)ctx.distance_flags[i] << (i % 32);
  }

  void BVC_fill_distances(const GlobalOctree &octree, const COctreeV3Header &header,
                          COctreeV3ThreadCtx &ctx, unsigned off_5, float min_active, float max_active,
                          /*out*/ unsigned &distances_size_uint)
  {
    int v_size = header.brick_size + 2 * header.brick_pad + 1;
    unsigned bits = header.bits_per_value;
    unsigned max_val = header.bits_per_value == 32 ? 0xFFFFFFFF : ((1 << bits) - 1);
    unsigned vals_per_int = 32 / header.bits_per_value;
    float range_active = max_active - min_active;
    
    unsigned active_distances = 0;
    for (int i = 0; i < v_size * v_size * v_size; i++)
    {
      if (ctx.distance_flags[i])
      {
        unsigned d_compressed = max_val * ((ctx.values_tmp[i] - min_active) / range_active);
        ctx.u_values_tmp[off_5 + active_distances / vals_per_int] |= d_compressed << (bits * (active_distances % vals_per_int));
        // printf("distance %d put to %u\n", i, active_distances);
        active_distances++;
      }
    }

    distances_size_uint = (active_distances + vals_per_int - 1) / vals_per_int;
    assert(distances_size_uint > 0);
  }

  unsigned brick_values_compress(unsigned leaf_type,
                                 const GlobalOctree &octree, const COctreeV3Header &header,
                                 COctreeV3ThreadCtx &ctx, unsigned idx, unsigned lod_size, uint3 p,
                                 float add_min, float add_max)
  {
    int v_size = header.brick_size + 2 * header.brick_pad + 1;
    int p_size = header.brick_size + 2 * header.brick_pad;

    unsigned total_size_uint = 0;

    if (leaf_type != COCTREE_LEAF_TYPE_GRID)
    {
      BVC_calculate_flags(octree, header, ctx, idx, lod_size, p, add_min, add_max);
    }
    else
    {
      //mark everything as active
      for (int i = 0; i < v_size * v_size * v_size; i++)
        ctx.distance_flags[i] = true;
      for (int i = 0; i < p_size * p_size * p_size; i++)
        ctx.presence_flags[i] = true;
    }

    if (leaf_type == COCTREE_LEAF_TYPE_SLICES)
    {
      const unsigned line_distances_offset_bits = 16;
      const unsigned line_distances_offsets_per_uint = 32 / line_distances_offset_bits;
      const unsigned slice_distance_flags_bits = v_size * v_size;
      const unsigned slice_distance_flags_uints = (slice_distance_flags_bits + 32 - 1) / 32;

      const unsigned presence_flags_size_uints = (p_size * p_size * p_size + 32 - 1) / 32;
      const unsigned distance_flags_size_uints = v_size * slice_distance_flags_uints;
      const unsigned distance_offsets_size_uints = (v_size + line_distances_offsets_per_uint - 1) / line_distances_offsets_per_uint; // 16 bits for offset, should be enough for all cases
      const unsigned min_range_size_uints = 2;

      assert(slice_distance_flags_uints <= 2); // if we want slices with more than 64 values, we should change rendering

      //<presence_flags><distance_flags><distance_offsets><min value and range><distances>
      unsigned off_0 = 0;                                   // presence flags
      unsigned off_1 = off_0 + presence_flags_size_uints;   // distance flags
      unsigned off_2 = off_1 + distance_flags_size_uints;   // distance offsets
      unsigned off_3 = off_2 + distance_offsets_size_uints; // min value and range
      unsigned off_4 = off_3 + min_range_size_uints;        // texture coordinates
      unsigned off_5 = off_4 + 8 * header.uv_size;          // distances

      // empty all range that can be used later
      unsigned max_size_uints = v_size * v_size * v_size + off_5;
      for (int i = 0; i < max_size_uints; i++)
        ctx.u_values_tmp[i] = 0;

      float min_active = 1000;
      float max_active = -1000;
      unsigned distances_size_uint = 0;

      BVC_fill_presence_flags(header, ctx, off_0);
      BVC_sliced_fill_distance_flags(header, ctx, off_1, off_2, line_distances_offset_bits, line_distances_offsets_per_uint, slice_distance_flags_uints);
      BVC_fill_range(header, ctx, off_3, min_active, max_active);
      BVC_fill_texture_coordinates(octree, header, ctx, idx, off_4);
      BVC_fill_distances(octree, header, ctx, off_5, min_active, max_active, distances_size_uint);

      total_size_uint = presence_flags_size_uints + distance_flags_size_uints + distance_offsets_size_uints + min_range_size_uints + 8 * header.uv_size + distances_size_uint;

      //printf("SLICED leaf %u (%u + %u + %u + %u + %u + %u)\n", total_size_uint, presence_flags_size_uints, distance_flags_size_uints, 
      //                                                         distance_offsets_size_uints, min_range_size_uints, 8 * header.uv_size,
      //                                                         distances_size_uint);
    }
    else if (leaf_type == COCTREE_LEAF_TYPE_BIT_PACK)
    {
      uint32_t distance_flags_size_uints = (v_size * v_size * v_size + 32 - 1) / 32;
      uint32_t presence_flags_size_uints = (p_size * p_size * p_size + 32 - 1) / 32;
      uint32_t      min_range_size_uints = 2;

      //<presence_flags><distance_flags><><min value and range><tex_coords><distances>
      unsigned off_0 = 0;                                   // presence flags
      unsigned off_1 = off_0 + presence_flags_size_uints;   // distance flags
      unsigned off_3 = off_1 + distance_flags_size_uints;   // min value and range
      unsigned off_4 = off_3 + min_range_size_uints;        // texture coordinates
      unsigned off_5 = off_4 + 8 * header.uv_size;          // distances

      // empty all range that can be used later
      unsigned max_size_uints = v_size * v_size * v_size + off_5;
      for (int i = 0; i < max_size_uints; i++)
        ctx.u_values_tmp[i] = 0;

      float min_active = 1000;
      float max_active = -1000;
      unsigned distances_size_uint = 0;

      BVC_fill_presence_flags(header, ctx, off_0);
      BVC_fill_distance_flags(header, ctx, off_1);
      BVC_fill_range(header, ctx, off_3, min_active, max_active);
      BVC_fill_texture_coordinates(octree, header, ctx, idx, off_4);
      BVC_fill_distances(octree, header, ctx, off_5, min_active, max_active, distances_size_uint);

      total_size_uint = presence_flags_size_uints + distance_flags_size_uints + min_range_size_uints + 8 * header.uv_size + distances_size_uint;

      //printf("BIT PACKED leaf %u (%u + %u + %u + %u + %u)\n", total_size_uint, presence_flags_size_uints, distance_flags_size_uints, 
      //                                                        min_range_size_uints, 8 * header.uv_size, distances_size_uint);
    }
    else if (leaf_type == COCTREE_LEAF_TYPE_GRID)
    {
      const unsigned min_range_size_uints = 2;
      unsigned off_3 = 0;                           // min value and range
      unsigned off_4 = off_3 + min_range_size_uints;// texture coordinates
      unsigned off_5 = off_4 + 8 * header.uv_size;  // distances

      // empty all range that can be used later
      unsigned max_size_uints = v_size * v_size * v_size + off_5;
      for (int i = 0; i < max_size_uints; i++)
        ctx.u_values_tmp[i] = 0;

      float min_active = 1000;
      float max_active = -1000;
      unsigned distances_size_uint = 0;

      BVC_fill_range(header, ctx, off_3, min_active, max_active);
      BVC_fill_texture_coordinates(octree, header, ctx, idx, off_4);
      BVC_fill_distances(octree, header, ctx, off_5, min_active, max_active, distances_size_uint);

      total_size_uint = min_range_size_uints + 8 * header.uv_size + distances_size_uint;

      //printf("GRID leaf %u (%u + %u + %u)\n", total_size_uint, min_range_size_uints, 8 * header.uv_size, distances_size_uint);
    }
    else
    {
      printf("unknown or unsupported leaf type %u\n", leaf_type);
      assert(false);
    }
    return total_size_uint;
  }

  unsigned brick_values_compress_no_packing(COctreeV3Header header, COctreeV3ThreadCtx &ctx,
                                            unsigned idx, unsigned lod_size, uint3 p)
  {
    int v_size = header.brick_size + 2 * header.brick_pad + 1;
    float d = 1.0f / lod_size;

    float d_max = 2 * sqrt(3) / lod_size;
    unsigned bits = header.bits_per_value;
    unsigned max_val = header.bits_per_value == 32 ? 0xFFFFFFFF : ((1 << bits) - 1);
    unsigned vals_per_int = 32 / header.bits_per_value;
    unsigned size_uints = (v_size * v_size * v_size + vals_per_int - 1) / vals_per_int;

    for (int i = 0; i < size_uints; i++)
      ctx.u_values_tmp[i] = 0;

    for (int i = 0; i < v_size * v_size * v_size; i++)
    {
      unsigned d_compressed = std::max(0.0f, max_val * ((ctx.values_tmp[i] + d_max) / (2 * d_max)));
      d_compressed = std::min(d_compressed, max_val);
      ctx.u_values_tmp[i / vals_per_int] |= d_compressed << (bits * (i % vals_per_int));
    }

    return size_uints;
  }

  void check_compact_octree_v3_node_packing(const GlobalOctree &octree, COctreeV3Header header,
                                            COctreeV3GlobalCtx &global_ctx, COctreeV3ThreadCtx &thread_ctx,
                                            unsigned thread_id, const COctreeV3BuildTask &task,
                                            unsigned ofs, unsigned i)
  {
    unsigned compact_leaf_idx = global_ctx.node_id_cleaf_id_remap[ofs + i];
    float add_min = global_ctx.cluster_infos[compact_leaf_idx].add_min;
    float add_max = global_ctx.cluster_infos[compact_leaf_idx].add_max;
    uint3 ch_p = 2 * task.p + uint3((i & 4) >> 2, (i & 2) >> 1, i & 1);

    unsigned leaf_size = brick_values_compress_no_packing(header, thread_ctx, ofs + i, 2 * task.lod_size, ch_p);
    unsigned a_leaf_size = brick_values_compress(0, octree, header, thread_ctx, ofs + i, 2 * task.lod_size, ch_p, add_min, add_max);
    // printf("compressed to %u uints, naive packing would be %u uints\n", a_leaf_size, leaf_size);

    std::vector<unsigned> u_values_tmp_2(thread_ctx.u_values_tmp.size(), 0u);
    std::copy_n(thread_ctx.u_values_tmp.data(), thread_ctx.u_values_tmp.size(), u_values_tmp_2.data());

    int v_size = header.brick_size + 2 * header.brick_pad + 1;
    int p_size = header.brick_size + 2 * header.brick_pad;
    float d_max = 2 * sqrt(3) / (2 * task.lod_size);
    unsigned bits = header.bits_per_value;
    unsigned max_val = header.bits_per_value == 32 ? 0xFFFFFFFF : ((1 << bits) - 1);
    unsigned vals_per_int = 32 / header.bits_per_value;

    const unsigned line_distances_offset_bits = 16;
    const unsigned line_distances_offsets_per_uint = 32 / line_distances_offset_bits;

    unsigned slice_distance_flags_bits = v_size * v_size;
    unsigned slice_distance_flags_uints = (slice_distance_flags_bits + 32 - 1) / 32;
    unsigned distance_flags_size_uints = v_size * slice_distance_flags_uints;
    unsigned presence_flags_size_uints = (p_size * p_size * p_size + 32 - 1) / 32;
    unsigned distance_offsets_size_uints = (v_size + line_distances_offsets_per_uint - 1) / line_distances_offsets_per_uint; // 16 bits for offset, should be enough for all cases

    const unsigned min_range_size_uints = 2;

    assert(slice_distance_flags_uints <= 2); // if we want slices with more than 64 values, we should change rendering

    //<presence_flags><distance_flags><distance_offsets><min value and range><distances>
    unsigned off_0 = 0;                                   // presence flags
    unsigned off_1 = off_0 + presence_flags_size_uints;   // distance flags
    unsigned off_2 = off_1 + distance_flags_size_uints;   // distance offsets
    unsigned off_3 = off_2 + distance_offsets_size_uints; // min value and range
    unsigned off_4 = off_3 + min_range_size_uints;        // texture coordinates
    unsigned off_5 = off_4 + 8 * header.uv_size;          // distances

    float min_val = -float(u_values_tmp_2[off_3 + 0]) / float(0xFFFFFFFFu);
    float range = (float(u_values_tmp_2[off_3 + 1]) / float(0xFFFFFFFFu)) / max_val;

    for (int x = 0; x < header.brick_size; x++)
    {
      for (int y = 0; y < header.brick_size; y++)
      {
        for (int z = 0; z < header.brick_size; z++)
        {
          uint3 voxelPosU = uint3(x, y, z);
          uint32_t p_size = header.brick_size + 2 * header.brick_pad;
          uint32_t PFlagPos = (voxelPosU.x + header.brick_pad) * p_size * p_size + (voxelPosU.y + header.brick_pad) * p_size + voxelPosU.z + header.brick_pad;
          if ((u_values_tmp_2[off_0 + PFlagPos / 32] & (1u << (PFlagPos % 32))) == 0)
            continue;

          for (int i = 0; i < 8; i++)
          {
            uint3 vPos = uint3(x, y, z) + uint3((i & 4) >> 2, (i & 2) >> 1, i & 1);
            uint32_t sliceId = vPos.x + header.brick_pad;
            uint32_t localId = (vPos.y + header.brick_pad) * v_size + vPos.z + header.brick_pad;

            uint32_t b0 = u_values_tmp_2[off_1 + slice_distance_flags_uints * sliceId];
            b0 = localId > 31 ? b0 : b0 & ((1u << localId) - 1);
            b0 = bitcount(b0);

            uint32_t b1 = localId > 32 ? bitcount(u_values_tmp_2[off_1 + slice_distance_flags_uints * sliceId + 1] & ((1u << (localId - 32)) - 1))
                                       : 0;

            uint32_t sliceOffset = (u_values_tmp_2[off_2 + sliceId / line_distances_offsets_per_uint] >>
                                    (line_distances_offset_bits * (sliceId % line_distances_offsets_per_uint))) &
                                   (((1u << line_distances_offset_bits) - 1));
            uint32_t localOffset = b0 + b1;

            uint32_t vId = sliceOffset + localOffset;
            uint32_t dist = ((u_values_tmp_2[off_5 + vId / vals_per_int] >> (bits * (vId % vals_per_int))) & max_val);
            uint32_t real_vId = SBS_v_to_i(vPos.x, vPos.y, vPos.z, v_size, header.brick_pad);

            float real_d = thread_ctx.values_tmp[real_vId];
            float d = min_val + range * dist;

            if (std::abs(d - real_d) > 1.5f * range)
            {
              printf("error unpacking dist\n");
              printf("reading distance %u from %u (%u %u - %u %u)\n", real_vId, vId, sliceOffset, localOffset, b0, b1);
              printf("dist = %f, real dist = %f\n", d, real_d);
              assert(false);
            }
          }
        }
      }
    }
  }

  static std::atomic<int> type_stat[4] = {0, 0, 0, 0};

  void execute_compression_task_leaf(const GlobalOctree &octree, const COctreeV3Header &header,
                                     COctreeV3GlobalCtx &global_ctx, COctreeV3ThreadCtx &thread_ctx,
                                     unsigned thread_id, const COctreeV3BuildTask &task)
  {
    //should not be multithreaded, manipulation with compact_leaf_offsets is not thread-safe
    assert(thread_id == 0);

    constexpr bool check_packing = false; // for debugging

    const int v_size = header.brick_size + 2 * header.brick_pad + 1;
    const int vox_count = v_size * v_size * v_size;

    unsigned leaf_offset, leaf_size, leaf_type;

    unsigned compact_leaf_idx = global_ctx.node_id_cleaf_id_remap[task.nodeId];
    if (global_ctx.compact_leaf_offsets[compact_leaf_idx] == 0u) // this is a new node, we should compress it and add to compact octree
    {
      // copy original values to temporary buffer and check if this leaf contain surface
      std::copy_n(global_ctx.compressed_data.data() + compact_leaf_idx * vox_count, vox_count, thread_ctx.values_tmp.begin());

      // we can check node packing for debug purposes.
      // if (check_packing)
      //  check_compact_octree_v3_node_packing(octree, header, global_ctx, thread_ctx, thread_id, task, ofs, i);

      // fill the compact octree with compressed leaf data
      float add_min = global_ctx.cluster_infos[compact_leaf_idx].add_min;
      float add_max = global_ctx.cluster_infos[compact_leaf_idx].add_max;

      unsigned uncompressed_leaf_size = 2 + (v_size*v_size*v_size + (32/header.bits_per_value - 1))/(32/header.bits_per_value);
      
      leaf_size = brick_values_compress(header.default_leaf_type, octree, header, thread_ctx, task.nodeId, task.lod_size, task.p, add_min, add_max);
      leaf_type = header.default_leaf_type;
      if (leaf_size >= uncompressed_leaf_size && header.fallback_leaf_type != header.default_leaf_type)
      {
        int old_leaf_size = leaf_size;
        leaf_size = brick_values_compress(header.fallback_leaf_type, octree, header, thread_ctx, task.nodeId, task.lod_size, task.p, add_min, add_max);
        leaf_type = header.fallback_leaf_type;
      }
      //printf("%u] type = %u, size = %u/%u\n", task.nodeId, leaf_type, leaf_size, uncompressed_leaf_size);
      type_stat[leaf_type]++;
      // printf("leaf type %d %d %d %d\n", type_stat[0], type_stat[1], type_stat[2], type_stat[3]);
      assert(leaf_size > 0);
      // printf("leaf size %u/%u\n", leaf_size, uncompressed_leaf_size);

      // resize compact octree to accommodate the new leaf
      #pragma omp critical
      {
        leaf_offset = global_ctx.compact.size();
        global_ctx.compact.resize(leaf_offset + leaf_size, 0u); // space for the leaf child
      }

      // copy leaf data to the compact octree
      for (int j = 0; j < leaf_size; j++)
        global_ctx.compact[leaf_offset + j] = thread_ctx.u_values_tmp[j];

      // add info about this leaf to the context
      // TODO : make it thread-safe
      global_ctx.compact_leaf_offsets[compact_leaf_idx] = leaf_offset;
      global_ctx.compact_leaf_types[compact_leaf_idx] = leaf_type;
      stat_leaf_bytes += leaf_size * sizeof(uint32_t);
    }
    else // this leaf node already exist, just get its offset and size
    {
      leaf_offset = global_ctx.compact_leaf_offsets[compact_leaf_idx];
      leaf_type   = global_ctx.compact_leaf_types[compact_leaf_idx];
    }

    global_ctx.node_id_compact_offset[task.nodeId] = leaf_offset;
    global_ctx.node_id_leaf_type[task.nodeId] = leaf_type;
  }

  void execute_compression_task_node(const GlobalOctree &octree, const COctreeV3Header &header,
                                     COctreeV3GlobalCtx &global_ctx, COctreeV3ThreadCtx &thread_ctx,
                                     unsigned thread_id, const COctreeV3BuildTask &task)
  {
    const int v_size = header.brick_size + 2 * header.brick_pad + 1;
    const int vox_count = v_size*v_size*v_size;
    const unsigned ofs = octree.nodes[task.nodeId].offset;
    assert(!is_leaf(ofs)); // octree must have at least two levels

    unsigned ch_count = header.lods;
    unsigned ch_is_active = 0u; // one bit per child
    unsigned ch_leaf_type = 0u;   // COCTREE_LEAF_TYPE_BITS bit per child
    unsigned tmp_node_buf[32];

    for (int i = 0; i < 8; i++)
    {
      if (is_leaf(octree.nodes[ofs + i].offset)) // process leaf
      {
        bool is_border_leaf = octree.nodes[ofs + i].is_not_void;
        if (is_border_leaf)
        {
          unsigned leaf_offset = global_ctx.node_id_compact_offset[ofs + i];
          unsigned leaf_type = global_ctx.node_id_leaf_type[ofs + i];
          assert(leaf_offset != 0xFFFFFFFFu && leaf_type != COCTREE_LEAF_TYPE_NOT_A_LEAF);

          tmp_node_buf[1 + header.uints_per_child_info*ch_count] = 0;
          tmp_node_buf[1 + header.uints_per_child_info*ch_count + header.trans_off] = 0;

          //if we use small nodes, we have limited space for leaf_offset. Make sure it fits
          assert((leaf_offset & (header.idx_mask >> header.idx_sh)) == leaf_offset);
          tmp_node_buf[1 + header.uints_per_child_info*ch_count] |= leaf_offset << header.idx_sh;

          if (header.sim_compression > 0) //if transform code is required, we save it here
          {
            uint32_t code = get_transform_code(header, global_ctx.transforms[ofs + i], task.lod_size);
            tmp_node_buf[1 + header.uints_per_child_info*ch_count + header.trans_off] |= code;
          }
          ch_is_active |= (1 << i);
          ch_leaf_type |= (leaf_type << i*COCTREE_LEAF_TYPE_BITS);
          ch_count++;
        }
      }
      else
      {
        // non-leaf child
        unsigned child_offset = global_ctx.node_id_compact_offset[ofs + i];
        assert(child_offset != 0xFFFFFFFFu);

        tmp_node_buf[1 + header.uints_per_child_info*ch_count] = child_offset;
        if (header.trans_off > 0) //if transform code is stored in separate uint, set it to zero to avoid UB
          tmp_node_buf[1 + header.uints_per_child_info*ch_count + header.trans_off] = 0;

        ch_is_active |= (1 << i);
        ch_leaf_type |= (COCTREE_LEAF_TYPE_NOT_A_LEAF << i*COCTREE_LEAF_TYPE_BITS);
        ch_count++;
      }
    }
    assert(ch_is_active > 0); // we must have at least one active child
    tmp_node_buf[0] = ch_is_active | (ch_leaf_type << 8u);

    if (header.lods > 0)
    {
      unsigned leaf_offset = global_ctx.node_id_compact_offset[task.nodeId];
      unsigned leaf_type = global_ctx.node_id_leaf_type[task.nodeId];
      assert(leaf_offset != 0xFFFFFFFFu && leaf_type != COCTREE_LEAF_TYPE_NOT_A_LEAF);

      tmp_node_buf[1] = 0;
      tmp_node_buf[1 + header.trans_off] = 0;

      // if we use small nodes, we have limited space for leaf_offset. Make sure it fits
      assert((leaf_offset & (header.idx_mask >> header.idx_sh)) == leaf_offset);
      tmp_node_buf[1] |= leaf_offset << header.idx_sh;

      if (header.sim_compression > 0) // if transform code is required, we save it here
      {
        uint32_t code = get_transform_code(header, global_ctx.transforms[task.nodeId], task.lod_size);
        tmp_node_buf[1 + header.trans_off] |= code;
      }

      assert(tmp_node_buf[0] & (COCTREE_LEAF_TYPE_MASK << COCTREE_LOD_LEAF_TYPE_SHIFT) == 0);
      tmp_node_buf[0] |= (leaf_type << COCTREE_LOD_LEAF_TYPE_SHIFT);
    }

    int total_uints = header.uints_per_child_info*ch_count + 1;
    stat_nonleaf_bytes += total_uints * sizeof(uint32_t);
    
    unsigned node_offset;
    if (task.nodeId == 0) //special case for root node, it has it's own place in the beginning of compact array
    {
      node_offset = 0;
    }
    else
    {
      #pragma omp critical
      {
        node_offset = global_ctx.compact.size();
        global_ctx.compact.resize(node_offset + total_uints);
      }
    }

    for (int i = 0; i < total_uints; i++)
      global_ctx.compact[node_offset + i] = tmp_node_buf[i];
    
    global_ctx.node_id_compact_offset[task.nodeId] = node_offset;
    global_ctx.node_id_leaf_type[task.nodeId] = COCTREE_LEAF_TYPE_NOT_A_LEAF;

    // printf("Node = ");
    // for (int i = 0; i < total_uints; i++)
    //   printf("%x ", global_ctx.compact[task.nodeId + i]);
    // printf("\n");
    // printf("[%u]NODE[%u] flags = %x leaf types=( ", task.lod_size, task.nodeId, tmp_node_buf[0] & 0xFF);
    // for (int i = 0; i < 8; i++)
    //   printf("%u ", (tmp_node_buf[0] >> (8 + i*COCTREE_LEAF_TYPE_BITS)) & COCTREE_LEAF_TYPE_MASK);
    // printf(")\n");
  }

  void fill_task_layers_rec(const GlobalOctree &octree, bool use_lods,
                            std::vector<std::vector<COctreeV3BuildTask>> &node_layers, 
                            std::vector<COctreeV3BuildTask> &leaf_layer,
                            unsigned idx, unsigned level, uint3 p, unsigned lod_size)
  {
    if (octree.nodes[idx].is_not_void)
    {
      if (is_leaf(octree.nodes[idx].offset) || use_lods)
      {
        leaf_layer.push_back(COctreeV3BuildTask(idx, p, lod_size));
      }
      
      if (!is_leaf(octree.nodes[idx].offset))
      {
        if (level >= node_layers.size())
          node_layers.resize(level+1);
        
        node_layers[level].push_back(COctreeV3BuildTask(idx, p, lod_size));

        for (int i = 0; i < 8; i++)
        {
          unsigned ch_idx = octree.nodes[idx].offset + i;
          uint3 ch_p = 2 * p + uint3((i & 4) >> 2, (i & 2) >> 1, i & 1);
          if (octree.nodes[ch_idx].is_not_void)
          fill_task_layers_rec(octree, use_lods, node_layers, leaf_layer, ch_idx, level + 1, ch_p, 2*lod_size);
        }
      }
    }
  }

  void global_octree_to_COctreeV3(const GlobalOctree &octree, COctreeV3 &compact_octree, COctreeV3Settings co_settings)
  {
    global_octree_to_COctreeV3(octree, compact_octree, co_settings, scom::Settings());
  }

  void global_octree_to_COctreeV3(const GlobalOctree &octree, COctreeV3 &compact_octree, 
                                  COctreeV3Settings co_settings, scom::Settings settings)
  {
#if ON_CPU == 1
    assert(COctreeV3::VERSION == 5); // if version is changed, this function should be revisited, as some changes may be needed
#endif
    stat_leaf_bytes.store(0);
    stat_nonleaf_bytes.store(0);

    for (int i=0;i<4;i++)
      type_stat[i].store(0);

    auto t1 = std::chrono::high_resolution_clock::now();

    //Initialize thread contexts 
    const int v_size = octree.header.brick_size + 2 * octree.header.brick_pad + 1;
    const int p_size = octree.header.brick_size + 2 * octree.header.brick_pad;
    int max_threads = omp_get_max_threads();
    COctreeV3GlobalCtx global_ctx;
    std::vector<COctreeV3ThreadCtx> all_ctx(max_threads);
    for (int i = 0; i < max_threads; i++)
    {
      all_ctx[i].values_tmp = std::vector<float>(v_size * v_size * v_size * max_threads);
      all_ctx[i].u_values_tmp = std::vector<uint32_t>(3 * v_size * v_size * v_size * max_threads);
      all_ctx[i].presence_flags = std::vector<bool>(p_size * p_size * p_size, false);
      all_ctx[i].requirement_flags = std::vector<bool>(p_size * p_size * p_size, false);
      all_ctx[i].distance_flags = std::vector<bool>(v_size * v_size * v_size, false);
    }

    //Performing similarity compression if needed
    if (co_settings.sim_compression)
    {
      scom::CompressionOutput scom_output;
      scom::similarity_compression(octree, settings, scom_output);

      global_ctx.compressed_data = scom_output.compressed_data;
      global_ctx.node_id_cleaf_id_remap = scom_output.node_id_cleaf_id_remap;
      global_ctx.transforms = scom_output.tranforms;

      global_ctx.compact_leaf_offsets.resize(scom_output.leaf_count, 0);

      //fill cluster infos to use it for leaves compression
      global_ctx.cluster_infos.resize(scom_output.leaf_count, CompressedClusterInfo(1000.0f, -1000.0f)); 
      for (int i = 0; i < octree.nodes.size(); i++)
      {
        if ((is_leaf(octree.nodes[i].offset) || co_settings.use_lods) && octree.nodes[i].is_not_void)
        {
          float add = global_ctx.transforms[i].add;

          int cleaf_id = global_ctx.node_id_cleaf_id_remap[i];
          global_ctx.cluster_infos[cleaf_id].original_node_ids.push_back(i);
          global_ctx.cluster_infos[cleaf_id].trivial_transform &= std::abs(global_ctx.transforms[i].add) < 1e-5f &&
                                                                  global_ctx.transforms[i].rotation_id == scom::ROT_ID_IDENTITY;
          global_ctx.cluster_infos[cleaf_id].add_min = std::min(global_ctx.cluster_infos[cleaf_id].add_min, add);
          global_ctx.cluster_infos[cleaf_id].add_max = std::max(global_ctx.cluster_infos[cleaf_id].add_max, add);
        }
      }

    }
    else
    {
      //set default
      global_ctx.compressed_data.reserve(octree.values_f.size()); 
      global_ctx.compact_leaf_offsets.reserve(octree.nodes.size());
      global_ctx.cluster_infos.reserve(octree.nodes.size());

      global_ctx.node_id_cleaf_id_remap = std::vector<unsigned>(octree.nodes.size(), 0u);
      global_ctx.transforms = std::vector<scom::TransformCompact>(octree.nodes.size(), scom::get_identity_transform());

      int vox_count = v_size*v_size*v_size;
      for (int i = 0; i < octree.nodes.size(); i++)
      {
        if ((is_leaf(octree.nodes[i].offset) || co_settings.use_lods) && octree.nodes[i].is_not_void)
        {
          global_ctx.node_id_cleaf_id_remap[i] = global_ctx.compact_leaf_offsets.size();
          global_ctx.transforms[i] = scom::get_identity_transform(); //identity transform
          
          CompressedClusterInfo info;
          info.trivial_transform = true;
          info.original_node_ids = {i};

          global_ctx.compact_leaf_offsets.push_back(0);
          global_ctx.cluster_infos.push_back(info);
          global_ctx.compressed_data.insert(global_ctx.compressed_data.end(), 
                                            octree.values_f.data() + octree.nodes[i].val_off, 
                                            octree.values_f.data() + octree.nodes[i].val_off + vox_count);
        }
      }
    }

    auto t2 = std::chrono::high_resolution_clock::now();
    printf("similarity compression %.2f ms\n", 1e-6f*std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count());

    //Determine data layout and fill header
    compact_octree.header = get_default_coctree_v3_header();
    compact_octree.header.bits_per_value = co_settings.bits_per_value;
    compact_octree.header.brick_size = co_settings.brick_size;
    compact_octree.header.brick_pad = co_settings.brick_pad;
    compact_octree.header.sim_compression = co_settings.sim_compression;
    compact_octree.header.uv_size = co_settings.uv_size;
    compact_octree.header.lods = co_settings.use_lods ? 1 : 0;

    //determine leaf packing modes
    bool can_use_bitpack = v_size*v_size*v_size <= 64; //we have 2 bit max on bitfield.
    if (co_settings.default_leaf_type == COCTREE_USE_BEST_LEAF_TYPE)
    {
      compact_octree.header.default_leaf_type = can_use_bitpack ? COCTREE_LEAF_TYPE_BIT_PACK : COCTREE_LEAF_TYPE_SLICES;
    }
    else if (co_settings.default_leaf_type == COCTREE_LEAF_TYPE_BIT_PACK && !can_use_bitpack)
    {
      printf("WARNING: bit pack leaf type is not supported for v_size > 4, switching to slices\n");
      compact_octree.header.default_leaf_type = COCTREE_LEAF_TYPE_SLICES;
    }
    else
    {
      compact_octree.header.default_leaf_type = co_settings.default_leaf_type;
    }

    compact_octree.header.fallback_leaf_type = co_settings.allow_fallback_to_unpacked_leaves ?
                                               COCTREE_LEAF_TYPE_GRID : compact_octree.header.default_leaf_type;
                                                                                                 
    //prepare tasks for compression
    std::vector<std::vector<COctreeV3BuildTask>> node_layers;
    std::vector<COctreeV3BuildTask> leaf_layer;
    fill_task_layers_rec(octree, co_settings.use_lods, node_layers, leaf_layer, 0, 0, uint3(0, 0, 0), 1);
    
    auto t3 = std::chrono::high_resolution_clock::now();
    printf("prepare compression tasks %.2f ms\n", 1e-6f*std::chrono::duration_cast<std::chrono::nanoseconds>(t3 - t2).count());

    //Initialize global context
    global_ctx.compact.reserve((1+8*COCTREE_MAX_CHILD_INFO_SIZE) * octree.nodes.size());
    global_ctx.compact.resize(( 1+8*COCTREE_MAX_CHILD_INFO_SIZE), 0u);
    global_ctx.node_id_compact_offset = std::vector<unsigned>(octree.nodes.size(), 0xFFFFFFFFu);
    global_ctx.node_id_leaf_type = std::vector<unsigned>(octree.nodes.size(), COCTREE_LEAF_TYPE_NOT_A_LEAF);
    global_ctx.compact_leaf_types = std::vector<unsigned>(global_ctx.compact_leaf_offsets.size(), COCTREE_LEAF_TYPE_NOT_A_LEAF);
    
    //compress leaves (isn't thread safe)
    for (int i = 0; i < leaf_layer.size(); i++)
      execute_compression_task_leaf(octree, compact_octree.header, global_ctx, all_ctx[0], 0, leaf_layer[i]);

    auto t4 = std::chrono::high_resolution_clock::now();
    printf("compress leaves %.2f ms\n", 1e-6f*std::chrono::duration_cast<std::chrono::nanoseconds>(t4 - t3).count());

    //check whether we can use small nodes (i.e. leaves takes not much space and we can use <32 bit indices)
    bool can_use_small_nodes = false;
    {
      COctreeV3Header tmp_header = compact_octree.header;
      tmp_header.node_pack_mode = COCTREE_NODE_PACK_MODE_SIM_COMP_SMALL;
      fill_coctree_v3_header(tmp_header);

      int leaves_size = global_ctx.compact.size();
      if (leaves_size < (tmp_header.idx_mask >> tmp_header.idx_sh))
        can_use_small_nodes = true;
    }

    //determine (non-leaf) nodes packing mode
    if (co_settings.sim_compression == false)
      compact_octree.header.node_pack_mode = COCTREE_NODE_PACK_MODE_DEFAULT;
    else if (co_settings.sim_compression == true && can_use_small_nodes == false)
      compact_octree.header.node_pack_mode = COCTREE_NODE_PACK_MODE_SIM_COMP_FULL;
    else if (co_settings.sim_compression == true && can_use_small_nodes == true)
      compact_octree.header.node_pack_mode = COCTREE_NODE_PACK_MODE_SIM_COMP_SMALL;
    else
      printf("Failed to determine coctree node pack mode\n");

    //precompute masks and other stuff in header
    fill_coctree_v3_header(compact_octree.header);
    
    //compress non-leaves in reverse order (we should compress nodes below to have children offsets for their parents)
    for (int layer = node_layers.size()-1; layer >= 0; layer--)
    {
      #pragma omp parallel for num_threads(max_threads)
      for (int i = 0; i < node_layers[layer].size(); i++)
      {
        int thread_id = omp_get_thread_num();
        execute_compression_task_node(octree, compact_octree.header, global_ctx, all_ctx[thread_id], thread_id, node_layers[layer][i]);
      }
    }

    auto t5 = std::chrono::high_resolution_clock::now();
    printf("compress nodes %.2f ms\n", 1e-6f*std::chrono::duration_cast<std::chrono::nanoseconds>(t5 - t4).count());

    printf("leaf types [%d %d %d %d]\n", type_stat[0].load(), type_stat[1].load(), type_stat[2].load(), type_stat[3].load());
    printf("compact octree %.1f Kb leaf, %.1f Kb non-leaf\n", stat_leaf_bytes.load() / 1024.0f, stat_nonleaf_bytes.load() / 1024.0f);

    global_ctx.compact.shrink_to_fit();
    compact_octree.data = std::move(global_ctx.compact);
  }
}