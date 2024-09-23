#include "sparse_octree_2.h"
#include "omp.h"
#include <set>
#include <chrono>
#include <unordered_map>

static constexpr unsigned INVALID_IDX = 1u<<31u;

namespace LiteMath
{
  static inline float2 to_float2(float3 f3)         
  { 
    return float2(f3.x, f3.y);
  }
  static inline float to_degrees(float radians)
  {
    return radians * 57.295779513082320876798154814105;
  }
  static inline float to_radians(float degrees)
  {
    return degrees * 0.01745329251994329576923690768489;
  }

  static inline int2 operator%(const int2 a, const int2 b) { return int2{a.x % b.x, a.y % b.y}; }
  static inline int3 operator%(const int3 a, const int3 b) { return int3{a.x % b.x, a.y % b.y, a.z % b.z}; }
  static inline int4 operator%(const int4 a, const int4 b) { return int4{a.x % b.x, a.y % b.y, a.z % b.z, a.w % b.w}; }

  static inline uint2 operator%(const uint2 a, const uint2 b) { return uint2{a.x % b.x, a.y % b.y}; }
  static inline uint3 operator%(const uint3 a, const uint3 b) { return uint3{a.x % b.x, a.y % b.y, a.z % b.z}; }
  static inline uint4 operator%(const uint4 a, const uint4 b) { return uint4{a.x % b.x, a.y % b.y, a.z % b.z, a.w % b.w}; }
}

static double urand(double from=0, double to=1)
{
  return ((double)rand() / RAND_MAX) * (to - from) + from;
}

static bool is_border(float distance, int level)
{
  return level < 2  ? true : std::abs(distance) < sqrt(3)*pow(2, -level);
}

static bool is_leaf(unsigned offset)
{
  return (offset == 0) || (offset & INVALID_IDX);
}

//level_size is (1 << level)
static bool is_border_node(float min_val, float max_val, unsigned level_size)
{
  float d_max = 2*sqrt(3)/level_size;
  return (min_val <= 0) && (max_val > -d_max);
}

namespace sdf_converter
{
  struct LargeNode
  {
    float3 p;
    float d;
    unsigned level;
    unsigned group_idx;
    unsigned thread_id;
    unsigned children_idx;
    float value;
  };

  void add_node_rec(std::vector<SdfOctreeNode> &nodes, SparseOctreeSettings settings, MultithreadedDistanceFunction sdf,
                    unsigned thread_id, unsigned node_idx, unsigned depth, unsigned max_depth, float3 p, float d)
  {
    nodes[node_idx].value = sdf(2.0f * ((p + float3(0.5, 0.5, 0.5)) * d) - float3(1, 1, 1), thread_id);

    if (depth < max_depth && is_border(nodes[node_idx].value, depth))
    {
      nodes[node_idx].offset = nodes.size();
      
      nodes.resize(nodes.size() + 8);
      unsigned idx = nodes[node_idx].offset;
      for (unsigned cid = 0; cid < 8; cid++)
      {
        add_node_rec(nodes, settings, sdf, thread_id, 
                     idx + cid, depth + 1, max_depth, 2 * p + float3((cid & 4) >> 2, (cid & 2) >> 1, cid & 1), d / 2);
      }
    }
  }

  void check_and_fix_sdf_sign(std::vector<SdfOctreeNode> &nodes, float d_thr, unsigned idx, float d)
  {
    unsigned ofs = nodes[idx].offset;
    //printf("idx ofs size %u %u %u\n", idx, ofs, (unsigned)nodes.size());
    if (!is_leaf(ofs))
    {
      for (int i=0;i<8;i++)
      {
        if (std::abs(nodes[ofs+i].value - nodes[idx].value)> 0.5*sqrt(3)*d)
          nodes[ofs+i].value *= -1;
        //if (nodes[idx].value < -d_thr)
        //  printf("%u lol %f ch %d %f\n", idx, nodes[idx].value, i, nodes[ofs+i].value);
      }

      for (int i=0;i<8;i++)
        check_and_fix_sdf_sign(nodes, d_thr, ofs+i, d/2);
    }
  }

  std::vector<SdfOctreeNode> construct_sdf_octree(SparseOctreeSettings settings, MultithreadedDistanceFunction sdf, 
                                                  unsigned max_threads)
  {
std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    omp_set_num_threads(max_threads);

    unsigned min_remove_level = std::min(settings.depth, 4u);
    std::vector<LargeNode> large_nodes;
    int lg_size = pow(2, min_remove_level);
    large_nodes.push_back({float3(0,0,0), 1.0f, 0u, 0u, 0u, 0u, 1000.0f});

    unsigned i = 0;
    while (i < large_nodes.size())
    {
      if (large_nodes[i].level < min_remove_level)
      {
        large_nodes[i].children_idx = large_nodes.size();
        for (int j=0;j<8;j++)
        {
          float ch_d = large_nodes[i].d / 2;
          float3 ch_p = 2 * large_nodes[i].p + float3((j & 4) >> 2, (j & 2) >> 1, j & 1);
          large_nodes.push_back({ch_p, ch_d, large_nodes[i].level+1, 0u, 0u, 0u, 1000.0f});
        }
      }
      i++;
    }

    std::vector<unsigned> border_large_nodes;

    for (int i=0;i<large_nodes.size();i++)
    {
      float3 pos = 2.0f * ((large_nodes[i].p + float3(0.5, 0.5, 0.5)) * large_nodes[i].d) - float3(1, 1, 1); 
      large_nodes[i].value = sdf(pos, 0);
      if (large_nodes[i].children_idx == 0 && is_border(large_nodes[i].value, large_nodes[i].level))
        border_large_nodes.push_back(i);
    }

    unsigned step = (border_large_nodes.size() + max_threads - 1) / max_threads;
    std::vector<std::vector<SdfOctreeNode>> all_nodes(max_threads);
    std::vector<std::vector<uint2>> all_groups(max_threads);

    for (int i=0;i<max_threads;i++)
    {
      all_nodes[i].reserve(std::min(1ul << 20, 1ul << 3*settings.depth));
    }

std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    #pragma omp parallel for
    for (int thread_id=0;thread_id<max_threads;thread_id++)
    {
      unsigned start = thread_id * step;
      unsigned end = std::min(start + step, (unsigned)border_large_nodes.size());
      for (unsigned i=start;i<end;i++)
      {
        unsigned idx = border_large_nodes[i];
        unsigned local_root_idx = all_nodes[thread_id].size();
        large_nodes[idx].group_idx = all_groups[thread_id].size();
        large_nodes[idx].thread_id = thread_id;
        all_nodes[thread_id].emplace_back();
        add_node_rec(all_nodes[thread_id], settings, sdf, thread_id, 
                    local_root_idx, large_nodes[idx].level, settings.depth, large_nodes[idx].p, large_nodes[idx].d);
      
        all_groups[thread_id].push_back(uint2(local_root_idx, all_nodes[thread_id].size()));
      }
    }

std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();

    std::vector<SdfOctreeNode> res_nodes(large_nodes.size());

    for (int i=0;i<large_nodes.size();i++)
    {
      res_nodes[i].value = large_nodes[i].value;
      res_nodes[i].offset = large_nodes[i].children_idx;
      if (large_nodes[i].children_idx == 0 && is_border(large_nodes[i].value, large_nodes[i].level))
      {
        unsigned start = all_groups[large_nodes[i].thread_id][large_nodes[i].group_idx].x + 1;
        unsigned end = all_groups[large_nodes[i].thread_id][large_nodes[i].group_idx].y;
        
        if (start == end) //this region is empty
          continue;
        
        unsigned prev_size = res_nodes.size();
        int shift = int(prev_size) - int(start);
        res_nodes[i].offset = all_nodes[large_nodes[i].thread_id][start-1].offset + shift;
        //printf("%d offset %u start %u shift %d\n", i, res_nodes[i].offset, start, shift);
        res_nodes.insert(res_nodes.end(), all_nodes[large_nodes[i].thread_id].begin() + start, all_nodes[large_nodes[i].thread_id].begin() + end);
        
        for (int j=prev_size;j<res_nodes.size();j++)
        {
          if (res_nodes[j].offset != 0)
            res_nodes[j].offset += shift;
        }
      }
    }

std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();

    check_and_fix_sdf_sign(res_nodes, pow(2,-1.0*min_remove_level), 0, 1.0f);
  
std::chrono::steady_clock::time_point t5 = std::chrono::steady_clock::now();
  
    float time_1 = 1e-3f*std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
    float time_2 = 1e-3f*std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();
    float time_3 = 1e-3f*std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count();
    float time_4 = 1e-3f*std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count();

    //printf("total nodes = %d, time = %6.2f ms (%.1f+%.1f+%.1f+%.1f)\n", 
    //       (int)res_nodes.size(), time_1 + time_2 + time_3 + time_4, time_1, time_2, time_3, time_4);

    omp_set_num_threads(omp_get_max_threads());

    return res_nodes;
  }

  void fill_octree_frame_rec(MultithreadedDistanceFunction sdf,   
                             const std::vector<SdfOctreeNode> &nodes,
                             std::vector<SdfFrameOctreeNode> &frame,
                             unsigned thread_id, unsigned idx, float3 p, float d)
  {
    unsigned ofs = nodes[idx].offset;
    frame[idx].offset = ofs;
    float min_val = nodes[idx].value;
    float max_val = nodes[idx].value;

    float3 pos = 2.0f * (d * p) - 1.0f;
    for (int i = 0; i < 8; i++)
    {
      float3 ch_pos = pos + 2 * d * float3((i & 4) >> 2, (i & 2) >> 1, i & 1);
      frame[idx].values[i] = sdf(ch_pos, thread_id);
      min_val = std::min(min_val, frame[idx].values[i]);
      max_val = std::max(max_val, frame[idx].values[i]);
    }

    if (max_val - min_val > 2 * sqrt(3) * d)
    {
      // printf("inconsistent distance %f - %f with d=%f\n", max_val, min_val, d);
      for (int i = 0; i < 8; i++)
        frame[idx].values[i] = LiteMath::sign(max_val) * std::abs(frame[idx].values[i]);
    }

    if (!is_leaf(ofs)) 
    {
      for (int i = 0; i < 8; i++)
      {
        float ch_d = d / 2;
        float3 ch_p = 2 * p + float3((i & 4) >> 2, (i & 2) >> 1, i & 1);
        fill_octree_frame_rec(sdf, nodes, frame, thread_id, ofs + i, ch_p, ch_d);
      }
    }
  }

  void octree_to_layers(const std::vector<SdfOctreeNode> &nodes, std::vector<float4> &layers, unsigned idx, float3 p, float d)
  {
    layers[idx] = float4(p.x, p.y, p.z, d);
    unsigned ofs = nodes[idx].offset;
    if (!is_leaf(ofs)) 
    {
      for (int i = 0; i < 8; i++)
      {
        float ch_d = d / 2;
        float3 ch_p = 2 * p + float3((i & 4) >> 2, (i & 2) >> 1, i & 1);
        octree_to_layers(nodes, layers, ofs + i, ch_p, ch_d);
      }
    }
  }

  struct PositionHasher
  {
    std::size_t operator()(const float3& k) const
    {
      // Compute individual hash values for first,
      // second and third and combine them using XOR
      // and bit shifting:

      return (  (std::hash<float>()(k.x)
              ^ (std::hash<float>()(k.y) << 1)) >> 1)
              ^ (std::hash<float>()(k.z) << 1);
    }
  };

  struct PositionEqual
  {
    bool operator()(const float3& lhs, const float3& rhs) const
    {
      return std::abs(lhs.x - rhs.x) < 1e-12f && std::abs(lhs.y - rhs.y) < 1e-12f && std::abs(lhs.z - rhs.z) < 1e-12f;
    }
  };

  std::vector<SdfFrameOctreeNode> convert_to_frame_octree(SparseOctreeSettings settings, MultithreadedDistanceFunction sdf, unsigned max_threads,
                                                          const std::vector<SdfOctreeNode> &nodes)
  {
std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    omp_set_num_threads(max_threads);
    std::vector<float4> layers(nodes.size());
    std::vector<SdfFrameOctreeNode> frame(nodes.size());
    octree_to_layers(nodes, layers, 0, float3(0,0,0), 1.0f);

std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    unsigned step = (nodes.size() + max_threads - 1) / max_threads;

    #pragma omp parallel for
    for (int thread_id=0;thread_id<max_threads;thread_id++)
    {
      std::unordered_map<float3, float, PositionHasher, PositionEqual> distance_cache;

      unsigned start = thread_id * step;
      unsigned end = std::min(start + step, (unsigned)nodes.size());
      for (int idx = start; idx < end; idx++)
      {
        unsigned ofs = nodes[idx].offset;
        float3 p = float3(layers[idx].x, layers[idx].y, layers[idx].z);
        float d = layers[idx].w;
        frame[idx].offset = ofs;
        float min_val = nodes[idx].value;
        float max_val = nodes[idx].value;

        float3 pos = 2.0f * (d * p) - 1.0f;
        for (int i = 0; i < 8; i++)
        {
          float3 ch_pos = pos + 2 * d * float3((i & 4) >> 2, (i & 2) >> 1, i & 1);
          auto it = distance_cache.find(ch_pos);
          float distance = 0.0f;
          if (it == distance_cache.end())
          {
            distance = sdf(ch_pos, thread_id);
            distance_cache[ch_pos] = distance;
          }
          else
            distance = it->second;
          frame[idx].values[i] = distance;
          min_val = std::min(min_val, frame[idx].values[i]);
          max_val = std::max(max_val, frame[idx].values[i]);
        }

        if (max_val - min_val > 2 * sqrt(3) * d)
        {
          for (int i = 0; i < 8; i++)
            frame[idx].values[i] = LiteMath::sign(max_val) * std::abs(frame[idx].values[i]);
        }
      }
    }

std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();

    float time_1 = 1e-3f*std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
    float time_2 = 1e-3f*std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();
    //printf("octree to frame octree: time = %6.2f ms (%.1f+%.1f)\n", time_1 + time_2, time_1, time_2);

    omp_set_num_threads(omp_get_max_threads());

    return frame;
  }

  void octree_to_layers(const std::vector<SdfFrameOctreeNode> &nodes, std::vector<float4> &layers, unsigned idx, float3 p, float d)
  {
    layers[idx] = float4(p.x, p.y, p.z, d);
    unsigned ofs = nodes[idx].offset;
    if (!is_leaf(ofs)) 
    {
      for (int i = 0; i < 8; i++)
      {
        float ch_d = d / 2;
        float3 ch_p = 2 * p + float3((i & 4) >> 2, (i & 2) >> 1, i & 1);
        octree_to_layers(nodes, layers, ofs + i, ch_p, ch_d);
      }
    }
  }

  SdfSBS frame_octree_to_SBS(MultithreadedDistanceFunction sdf, 
                             unsigned max_threads,
                             const std::vector<SdfFrameOctreeNode> &nodes,
                             const SdfSBSHeader &header)
  {
std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    omp_set_num_threads(max_threads);
    std::vector<float4> layers(nodes.size());
    octree_to_layers(nodes, layers, 0, float3(0,0,0), 1.0f);

std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    SdfSBS sbs;
    uint32_t v_size = header.brick_size + 2*header.brick_pad + 1;
    sbs.header = header;
    sbs.nodes.reserve(nodes.size());
    sbs.values.reserve(nodes.size() * v_size * v_size * v_size);

    unsigned step = (nodes.size() + max_threads - 1) / max_threads;

    #pragma omp parallel for
    for (int thread_id=0;thread_id<max_threads;thread_id++)
    {
      std::vector<float> values(v_size*v_size*v_size, 1000.0f);
      unsigned start = thread_id * step;
      unsigned end = std::min(start + step, (unsigned)nodes.size());
      for (int idx = start; idx < end; idx++)
      {
        unsigned ofs = nodes[idx].offset;
        if (is_leaf(ofs)) 
        {
          uint3 p = uint3(layers[idx].x, layers[idx].y, layers[idx].z);
          float d = layers[idx].w;

          float min_val = 1000;
          float max_val = -1000;
          for (int i=0;i<8;i++)
          {
            min_val = std::min(min_val, nodes[idx].values[i]);
            max_val = std::max(max_val, nodes[idx].values[i]);
          }
          float3 p0 = 2.0f*(d*float3(p)) - 1.0f;
          float dp = 2.0f*d/header.brick_size;

          for (int i=-header.brick_pad; i<=header.brick_size + header.brick_pad; i++)
          {
            for (int j=-header.brick_pad; j<=header.brick_size + header.brick_pad; j++)
            {
              for (int k=-header.brick_pad; k<=header.brick_size + header.brick_pad; k++)
              {
                float val = 2e6f;
                // corners, reuse values
                if (i == 0)
                {
                  if (j == 0)
                  {
                    if (k == 0)
                      val = nodes[idx].values[0];
                    else if (k == header.brick_size)
                      val = nodes[idx].values[1];
                  }
                  else if (j == header.brick_size)
                  {
                    if (k == 0)
                      val = nodes[idx].values[2];
                    else if (k == header.brick_size)
                      val = nodes[idx].values[3];
                  }
                }
                else if (i == header.brick_size)
                {
                  if (j == 0)
                  {
                    if (k == 0)
                      val = nodes[idx].values[4];
                    else if (k == header.brick_size)
                      val = nodes[idx].values[5];
                  }
                  else if (j == header.brick_size)
                  {
                    if (k == 0)
                      val = nodes[idx].values[6];
                    else if (k == header.brick_size)
                      val = nodes[idx].values[7];
                  }
                }

                //new points
                if (val > 1e6f)
                {
                  float3 pos = p0 + dp*float3(i,j,k);
                  val = sdf(pos, thread_id);
                }

                values[i*v_size*v_size + j*v_size + k] = val;
              }
            }      
          }

          //fix for inconsistent distances
          if (max_val - min_val > 2 * sqrt(3) * d)
          {
            //printf("inconsistent distance %f - %f with d=%f\n", max_val, min_val, d);
            for (int i = 0; i < values.size(); i++)
              values[i] = LiteMath::sign(max_val) * std::abs(values[i]);
          }

          //add not only if there is really a border
          if (is_border_node(min_val, max_val, 1/d))
          {
            unsigned off=0, n_off=0;
            unsigned lod_size = 1.0f/d;
            float d_max = 2*sqrt(3)/lod_size;
            unsigned bits = 8*header.bytes_per_value;
            unsigned max_val = (1 << bits) - 1;
            unsigned vals_per_int = 4/header.bytes_per_value;

            #pragma omp critical
            {
              off = sbs.values.size();
              n_off = sbs.nodes.size();
              sbs.nodes.emplace_back();
              sbs.values.resize(sbs.values.size() + (values.size()+vals_per_int-1)/vals_per_int);
            }

            sbs.nodes[n_off].data_offset = off;
            sbs.nodes[n_off].pos_xy = (p.x << 16) | p.y;
            sbs.nodes[n_off].pos_z_lod_size = (p.z << 16) | lod_size;

            for (int i=0;i<values.size();i++)
            {
              unsigned d_compressed = std::max(0.0f, max_val*((values[i]+d_max)/(2*d_max)));
              d_compressed = std::min(d_compressed, max_val);
              sbs.values[off + i/vals_per_int] |= d_compressed << (bits*(i%vals_per_int));
            }
          }
        } //end if is leaf
      }
    }
std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();

    float time_1 = 1e-3f*std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
    float time_2 = 1e-3f*std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();
    //printf("frame octree to SBS: time = %6.2f ms (%.1f+%.1f)\n", time_1 + time_2, time_1, time_2);

    sbs.nodes.shrink_to_fit();
    sbs.values.shrink_to_fit();
    omp_set_num_threads(omp_get_max_threads());

    return sbs;
  }

  void mesh_octree_to_sdf_frame_octree_rec(const cmesh4::SimpleMesh &mesh,
                                         const cmesh4::TriangleListOctree &tl_octree,
                                         std::vector<SdfFrameOctreeNode> &frame,
                                         unsigned idx, float3 p, float d)
  {
    unsigned ofs = tl_octree.nodes[idx].offset;
    frame[idx].offset = ofs;

    if (is_leaf(ofs)) 
    {
      float3 pos = 2.0f*(d*p) - 1.0f;
      for (int i = 0; i < 8; i++)
      {
        float3 ch_pos = pos + 2*d*float3((i & 4) >> 2, (i & 2) >> 1, i & 1);
        float min_dist_sq = 1000;
        int min_ti = -1;
        for (int j=0; j<tl_octree.nodes[idx].tid_count; j++)
        {
          int t_i = tl_octree.triangle_ids[tl_octree.nodes[idx].tid_offset+j];

          float3 a = to_float3(mesh.vPos4f[mesh.indices[3*t_i+0]]);
          float3 b = to_float3(mesh.vPos4f[mesh.indices[3*t_i+1]]);
          float3 c = to_float3(mesh.vPos4f[mesh.indices[3*t_i+2]]);
          float3 vt = ch_pos - cmesh4::closest_point_triangle(ch_pos, a, b, c);
          float dist_sq = LiteMath::dot(vt, vt);

          if (dist_sq < min_dist_sq)
          {
            min_dist_sq = dist_sq; 
            min_ti = t_i;
          }
        }

        if (min_ti >= 0)
        {
          float3 a = to_float3(mesh.vPos4f[mesh.indices[3*min_ti+0]]);
          float3 b = to_float3(mesh.vPos4f[mesh.indices[3*min_ti+1]]);
          float3 c = to_float3(mesh.vPos4f[mesh.indices[3*min_ti+2]]);
          float3 vt = ch_pos - cmesh4::closest_point_triangle(ch_pos, a, b, c);
          float3 n = (1.0f/3.0f)*(to_float3(mesh.vNorm4f[mesh.indices[3*min_ti+0]]) + 
                                  to_float3(mesh.vNorm4f[mesh.indices[3*min_ti+1]]) + 
                                  to_float3(mesh.vNorm4f[mesh.indices[3*min_ti+2]]));

          frame[idx].values[i] = dot(normalize(n), vt) > 0 ? sqrt(min_dist_sq) : -sqrt(min_dist_sq);
        }
        else
          frame[idx].values[i] = 1000;
      }
    }
    else
    {
      for (int i = 0; i < 8; i++)
      {
        float ch_d = d / 2;
        float3 ch_p = 2 * p + float3((i & 4) >> 2, (i & 2) >> 1, i & 1);
        mesh_octree_to_sdf_frame_octree_rec(mesh, tl_octree, frame, ofs + i, ch_p, ch_d);
      }
    }
  }

  void mesh_octree_to_sdf_frame_octree(const cmesh4::SimpleMesh &mesh,
                                       const cmesh4::TriangleListOctree &tl_octree, 
                                       std::vector<SdfFrameOctreeNode> &out_frame)
  {
    out_frame.resize(tl_octree.nodes.size());
    mesh_octree_to_sdf_frame_octree_rec(mesh, tl_octree, out_frame, 0, float3(0,0,0), 1);
  }

  struct LayerFrameNodeInfo
  {
    unsigned idx;
    bool is_leaf;
    bool is_border;
    unsigned border_children;
  };

  struct FrameNodeQualitySort
  {
    unsigned idx;
    unsigned active_children; //active children count
    float weighted_diff; //average sdf diff. on children sample points / weight
  };

  bool fill_layers_rec(const std::vector<SdfFrameOctreeNode> &frame, std::vector<std::vector<LayerFrameNodeInfo>> &layers, 
                      unsigned level, unsigned idx)
  {  
    if (level >= layers.size())
      layers.resize(level+1);

    unsigned ofs = frame[idx].offset;
    bool leaf = is_leaf(ofs);
    float min_val = frame[idx].values[0];
    float max_val = frame[idx].values[0];
    for (int i=0;i<8;i++)
    {
      min_val = std::min(min_val, frame[idx].values[i]);
      max_val = std::max(max_val, frame[idx].values[i]);
    }
    bool border = is_border_node(min_val, max_val, 1 << level);
    unsigned border_children = 0;

    if (!leaf)
    {
      for (unsigned i=0;i<8;i++)
        border_children += fill_layers_rec(frame, layers, level+1, ofs+i);
    }

    //printf("layer %u, idx %u, leaf = %d, border = %d, border_children = %u\n", level, idx, leaf, border, border_children);
    LayerFrameNodeInfo node = {idx, leaf, border, border_children};
    layers[level].push_back(node);

    return border;
  }

  static float trilinear_interpolation(const float values[8], float3 dp)
  {
    return (1-dp.x)*(1-dp.y)*(1-dp.z)*values[0] + 
          (1-dp.x)*(1-dp.y)*(  dp.z)*values[1] + 
          (1-dp.x)*(  dp.y)*(1-dp.z)*values[2] + 
          (1-dp.x)*(  dp.y)*(  dp.z)*values[3] + 
          (  dp.x)*(1-dp.y)*(1-dp.z)*values[4] + 
          (  dp.x)*(1-dp.y)*(  dp.z)*values[5] + 
          (  dp.x)*(  dp.y)*(1-dp.z)*values[6] + 
          (  dp.x)*(  dp.y)*(  dp.z)*values[7];
  }

  float node_RMSE_linear(const float node_values[8], MultithreadedDistanceFunction sdf, float3 corner, float3 offset)
  {
    float rmse = 0.0f;
    for (int x = 0; x <= 2; x += 1)
    {
      for (int y = 0; y <= 2; y += 1)
      {
        for (int z = 0; z <= 2; z += 1)
        {
          float3 off_idx = float3((float)x / 2.0, (float)y / 2.0, (float)z / 2.0);
          float coeff = 1.0;
          if (x == 1) coeff *= 2.0;
          if (y == 1) coeff *= 2.0;
          if (z == 1) coeff *= 2.0;
          float sdf_val = sdf(corner + offset * off_idx, 0);//TODO save it to cash and take
          rmse += pow((sdf_val - trilinear_interpolation(node_values, off_idx)), 2);
        }
      }
    }
    return sqrt(rmse / 64.0);
    
  }

  void fill_frame(SdfFrameOctreeNode &node, MultithreadedDistanceFunction sdf, float3 p, float d)//change for take interpolate values if it needs
  {
    float3 pos = 2.0f * (d * p) - 1.0f;
    for (int i = 0; i < 8; i++)
    {
      float3 ch_pos = pos + 2 * d * float3((i & 4) >> 2, (i & 2) >> 1, i & 1);
      node.values[i] = sdf(ch_pos, 0);
    }
  }

  struct dirs {unsigned neighbour[3*2]; float3 p; float d; unsigned parent;};//x+x-y+y-z+z-

  void fill_neighbours_til_one_dir(std::vector<struct dirs> &directions, const std::set<unsigned> &divided, 
                                   const std::vector<SdfFrameOctreeNode> &result,
                                   unsigned idx, unsigned num, int dir, unsigned counted_idx, unsigned another_idx)
  {
    directions[result[idx].offset + num].neighbour[counted_idx] = result[idx].offset + num + dir;
    if (divided.find(directions[idx].neighbour[another_idx]) != divided.end() && directions[idx].neighbour[another_idx] != 0) 
        directions[result[idx].offset + num].neighbour[another_idx] = 
        result[directions[idx].neighbour[another_idx]].offset + num + dir;
    else
        directions[result[idx].offset + num].neighbour[another_idx] = directions[idx].neighbour[another_idx];
  }

  void interpolate_vertex(const std::vector<struct dirs> &directions, 
                                   std::vector<SdfFrameOctreeNode> &result,
                                   unsigned idx, unsigned num, unsigned dir, unsigned vert)
  {
    float3 p = directions[directions[result[idx].offset + num].neighbour[dir]].p;
    float d = directions[directions[result[idx].offset + num].neighbour[dir]].d;
    float3 ch_pos_1 = 2.0f * (d * p) - 1.0f;
    float3 ch_pos_2 = ch_pos_1 + 2 * d;

    p = directions[result[idx].offset + num].p;
    d = directions[result[idx].offset + num].d;
    float3 ch_pos = 2.0f * (d * p) - 1.0f + 2 * d * float3((vert & 4) >> 2, (vert & 2) >> 1, vert & 1);

    float3 coeff = 1.0f - (ch_pos_2 - ch_pos) / (ch_pos_2 - ch_pos_1);
    
    result[result[idx].offset + num].values[vert] = 
      trilinear_interpolation(result[directions[result[idx].offset + num].neighbour[dir]].values, coeff);
  }

  std::vector<SdfFrameOctreeNode> construct_sdf_frame_octree(SparseOctreeSettings settings, MultithreadedDistanceFunction sdf, float eps, 
                                                             unsigned max_threads/*while unused*/, bool is_smooth)
  {
    //struct dirs {unsigned neighbour[3*2]; float3 p; float d; unsigned parent;};//x+x-y+y-z+z-
    std::vector<SdfFrameOctreeNode> result(1);
    fill_frame(result[0], sdf, float3(0, 0, 0), 1);
    std::vector<struct dirs> directions(1);
    directions[0].d = 1.0f;
    directions[0].p = float3(0, 0, 0);
    directions[0].parent = 0;
    memset(directions[0].neighbour, 0, sizeof(unsigned) * 8);
    std::set<unsigned> divided = {};
    std::set<unsigned> last_level = {0};

    for (int i = 1; i < settings.depth; ++i)
    {
      //TODO
      bool is_end = true;
      printf("%d\n", i);
      for (auto node_idx : last_level)
      {
        float3 corner = 2.0 * directions[node_idx].p * directions[node_idx].d - 1.0;
        if (node_RMSE_linear(result[node_idx].values, sdf, corner, 2 * directions[node_idx].d * float3(1, 1, 1)) >= eps)
        {
          divided.insert(node_idx);
          is_end = false;
        }
      }
      if (is_end) break;
      last_level.clear();
      for (auto div_idx : divided)
      {
        result[div_idx].offset = result.size();
        result.resize(result.size() + 8);
        directions.resize(directions.size() + 8);
        for (unsigned num = 0; num < 8; ++num)
        {
          last_level.insert(result[div_idx].offset + num);
          //initialize
          result[result[div_idx].offset + num].offset = 0;
          directions[result[div_idx].offset + num].p = 2 * directions[div_idx].p + float3((num & 4) >> 2, (num & 2) >> 1, num & 1);
          directions[result[div_idx].offset + num].d = directions[div_idx].d / 2.0;
          directions[result[div_idx].offset + num].parent = div_idx;
        }

      }
      for (auto div_idx : divided)
      {
        for (unsigned num = 0; num < 8; ++num)
        {
          if ((num & 4) != 0) fill_neighbours_til_one_dir(directions, divided, result, div_idx, num, -4, 1, 0);
          else fill_neighbours_til_one_dir(directions, divided, result, div_idx, num, 4, 0, 1);

          if ((num & 2) != 0) fill_neighbours_til_one_dir(directions, divided, result, div_idx, num, -2, 3, 2);
          else fill_neighbours_til_one_dir(directions, divided, result, div_idx, num, 2, 2, 3);

          if ((num & 1) != 0) fill_neighbours_til_one_dir(directions, divided, result, div_idx, num, -1, 5, 4);
          else fill_neighbours_til_one_dir(directions, divided, result, div_idx, num, 1, 4, 5);

          for (unsigned vert = 0; vert < 8; ++vert)
          {
            unsigned x_dir = (vert & 4) >> 2;
            unsigned y_dir = 2 + ((vert & 2) >> 1);
            unsigned z_dir = 4 + (vert & 1);
            if (is_smooth && last_level.find(directions[result[div_idx].offset + num].neighbour[x_dir]) == last_level.end() && directions[result[div_idx].offset + num].neighbour[x_dir] != 0)
            {
              //take vertex from x_dir neighbour
              interpolate_vertex(directions, result, div_idx, num, x_dir, vert);
            }
            else if (is_smooth && last_level.find(directions[result[div_idx].offset + num].neighbour[y_dir]) == last_level.end() && directions[result[div_idx].offset + num].neighbour[y_dir] != 0)
            {
              //take vertex from y_dir neighbour
              interpolate_vertex(directions, result, div_idx, num, y_dir, vert);
            }
            else if (is_smooth && last_level.find(directions[result[div_idx].offset + num].neighbour[z_dir]) == last_level.end() && directions[result[div_idx].offset + num].neighbour[z_dir] != 0)
            {
              //take vertex from z_dir neighbour
              interpolate_vertex(directions, result, div_idx, num, z_dir, vert);
            }
            else// all neighbours on the last level
            {
              float3 p = directions[result[div_idx].offset + num].p;
              float d = directions[result[div_idx].offset + num].d;
              float3 ch_pos = 2.0f * (d * p) - 1.0f + 2 * d * float3((vert & 4) >> 2, (vert & 2) >> 1, vert & 1);
              result[result[div_idx].offset + num].values[vert] = sdf(ch_pos, 0);
            }
          }
        }
      }
      divided.clear();
    }
    return result;
  }

  static void print_layers(const std::vector<std::vector<LayerFrameNodeInfo>> &layers, bool count_only_border_nodes)
  {
    for (auto &layer : layers)
    {
      unsigned l_cnt = 0;
      for (auto &node : layer)
      {
        if (node.is_leaf && (!count_only_border_nodes || node.is_border))
          l_cnt++;
      }
      //printf("layer %u (%u)\n", (unsigned)layer.size(), l_cnt);
    }
  }

  void frame_octree_limit_nodes(std::vector<SdfFrameOctreeNode> &frame, unsigned nodes_limit,
                                bool count_only_border_nodes)
  {
    if (nodes_limit >= frame.size())
      return;
    
    assert(nodes_limit > 0);
    assert(frame.size() > 0);

    std::vector<std::vector<LayerFrameNodeInfo>> layers;
    fill_layers_rec(frame, layers, 0, 0);

    unsigned cnt = 0;
    for (auto &layer : layers)
      for (auto &node : layer)
        if (node.is_leaf && (!count_only_border_nodes || node.is_border))
          cnt++;

    //print_layers(layers, count_only_border_nodes);
    //printf("cnt = %u, nodes_limit = %u\n", cnt, nodes_limit);

    //layer from which we are deleting nodes, to do so, their parent nodes are evaluated
    unsigned active_layer = layers.size() - 1;

    while (cnt > nodes_limit && active_layer > 0)
    {
      std::vector<FrameNodeQualitySort> merge_candidates;
      for (auto &p_idx : layers[active_layer-1])
      {
        //printf("%u %d %d %u\n", p_idx.idx, p_idx.is_leaf, p_idx.is_border, p_idx.border_children);
        unsigned ofs = frame[p_idx.idx].offset;
        unsigned active_children = count_only_border_nodes ? p_idx.border_children : 8;
        //if (active_children < 2)
        //  printf("ERROR: %u %u\n", p_idx.idx, active_children);
        if (is_leaf(ofs))
          continue;

        float diff = 0;
        for (unsigned i=0;i<8;i++)
        {
          SdfFrameOctreeNode &child = frame[ofs+i];
          for (unsigned j=0;j<8;j++)
          {
            float child_val = child.values[j];
            float3 parent_q = 0.5f*(float3((i & 4) >> 2, (i & 2) >> 1, i & 1) + float3((j & 4) >> 2, (j & 2) >> 1, j & 1));
            float parent_val = trilinear_interpolation(frame[p_idx.idx].values, parent_q);
            diff += std::abs(child_val - parent_val);
          }
        }
        merge_candidates.push_back({p_idx.idx, active_children, active_children > 1 ? diff/(active_children - 1) : 1000});
      }

      assert(merge_candidates.size() > 0);

      unsigned delete_potential = 0;
      for (auto &mc : merge_candidates)
        delete_potential += mc.active_children-1;

      if (cnt - delete_potential < nodes_limit)
      {
        std::sort(merge_candidates.begin(), merge_candidates.end(), 
                  [](const FrameNodeQualitySort &a, const FrameNodeQualitySort &b){return a.weighted_diff < b.weighted_diff;});
      }
      //else we have to delete the whole layer

      int c_i = 0;
      while (cnt > nodes_limit && c_i < merge_candidates.size())
      {
        unsigned idx = merge_candidates[c_i].idx;
        for (unsigned i=0;i<8;i++)
          frame[frame[idx].offset+i].offset = INVALID_IDX;
        frame[idx].offset = 0;
        //printf("removing %u %u %f, left %d\n", idx, merge_candidates[c_i].active_children, 
        //merge_candidates[c_i].weighted_diff, cnt);
        cnt -= (merge_candidates[c_i].active_children - 1);
        c_i++;
      }

      //for (int i=0;i<merge_candidates.size();i++)
      //  printf("%u %u %f\n", merge_candidates[i].idx, merge_candidates[i].weight, merge_candidates[i].weighted_diff);
      layers = {};
      fill_layers_rec(frame, layers, 0, 0);
      cnt = 0;
      for (auto &layer : layers)
        for (auto &node : layer)
          if (node.is_leaf && (!count_only_border_nodes || node.is_border))
            cnt++;

      //print_layers(layers, count_only_border_nodes);
      active_layer--;
    }

    std::vector<unsigned> idx_remap(frame.size(), 0);
    std::vector<SdfFrameOctreeNode> new_frame;

    new_frame.reserve(frame.size());
    for (unsigned i=0;i<frame.size();i++)
    {
      if (frame[i].offset != INVALID_IDX)
      {
        idx_remap[i] = new_frame.size();
        new_frame.push_back(frame[i]);
      }
    }

    for (unsigned i=0;i<new_frame.size();i++)
    {
      if (new_frame[i].offset > 0)
        new_frame[i].offset = idx_remap[new_frame[i].offset];
    }

    frame = new_frame;

    //printf("cnt left %u\n", cnt);
  }

  struct LayerOctreeNodeInfo
  {
    unsigned idx;
    bool is_leaf;
    float3 pos; //center
  };

  struct OctreeNodeQualitySort
  {
    unsigned idx;
    float diff;
  };

  void fill_layers_rec(const std::vector<SdfOctreeNode> &frame, std::vector<std::vector<LayerOctreeNodeInfo>> &layers, 
                      unsigned level, unsigned idx, float3 p, float d)
  {
    if (level >= layers.size())
      layers.resize(level+1);

    bool leaf = is_leaf(frame[idx].offset);
    float3 pos = 2.0f * (d * (p + 0.5f)) - 1.0f;
    layers[level].push_back({idx, leaf, pos});

    if (!leaf)
    {
      for (unsigned i=0;i<8;i++)
        fill_layers_rec(frame, layers, level+1, frame[idx].offset + i, 2*p + float3((i & 4) >> 2, (i & 2) >> 1, i & 1), d/2);
    }
  }

  void octree_limit_nodes(std::vector<SdfOctreeNode> &frame, unsigned nodes_limit)
  {
    bool verbose = false;

    if (nodes_limit >= frame.size())
      return;
    
    assert(nodes_limit > 0);
    assert(frame.size() > 0);

    std::vector<std::vector<LayerOctreeNodeInfo>> layers;
    fill_layers_rec(frame, layers, 0, 0, float3(0,0,0), 1.0f);

    unsigned cnt = 0;
    for (auto &layer : layers)
    {
      if (verbose)
        printf("layer %u\n", (unsigned)layer.size());
      for (auto &node : layer)
      {
        if (node.is_leaf)
          cnt++;
      }
    }
    if (verbose)
      printf("cnt = %u, nodes_limit = %u\n", cnt, nodes_limit);

    //layer from which we are deleting nodes, to do so, their parent nodes are evaluated
    unsigned active_layer = layers.size() - 1;

    while (cnt > nodes_limit && active_layer > 0)
    {
      std::shared_ptr<ISdfOctreeFunction> octree_f = get_SdfOctreeFunction(frame);
      std::vector<OctreeNodeQualitySort> merge_candidates;
      for (auto &p_idx : layers[active_layer-1])
      {
        //printf("%u %d %d %u\n", p_idx.idx, p_idx.is_leaf, p_idx.is_border, p_idx.border_children);
        unsigned ofs = frame[p_idx.idx].offset;
        if (is_leaf(ofs))
          continue;

        float diff = 0;
        for (unsigned i=0;i<8;i++)
        {
          float d = 1.0f / (1 << (active_layer - 1));
          float child_val = frame[ofs+i].value;
          float parent_val = octree_f->eval_distance_level(p_idx.pos + d * float3((i & 4) >> 2, (i & 2) >> 1, i & 1), active_layer - 1);
          diff += std::abs(child_val - parent_val);
        }
        merge_candidates.push_back({p_idx.idx, diff});
      }

      assert(merge_candidates.size() > 0);

      unsigned delete_potential = 0;
      for (auto &mc : merge_candidates)
        delete_potential += 8-1;

      if (cnt - delete_potential < nodes_limit)
      {
        std::sort(merge_candidates.begin(), merge_candidates.end(), 
                  [](const OctreeNodeQualitySort &a, const OctreeNodeQualitySort &b){return a.diff < b.diff;});
      }
      //else we have to delete the whole layer

      int c_i = 0;
      while (cnt > nodes_limit && c_i < merge_candidates.size())
      {
        unsigned idx = merge_candidates[c_i].idx;
        for (unsigned i=0;i<8;i++)
          frame[frame[idx].offset+i].offset = INVALID_IDX;
        frame[idx].offset = 0;
        //printf("removing %u %u %f, left %d\n", idx, merge_candidates[c_i].active_children, 
        //merge_candidates[c_i].weighted_diff, cnt);
        cnt -= (8 - 1);
        c_i++;
      }

      //for (int i=0;i<merge_candidates.size();i++)
      //  printf("%u %u %f\n", merge_candidates[i].idx, merge_candidates[i].weight, merge_candidates[i].weighted_diff);
      active_layer--;
    }

    std::vector<unsigned> idx_remap(frame.size(), 0);
    std::vector<SdfOctreeNode> new_frame;

    new_frame.reserve(frame.size());
    for (unsigned i=0;i<frame.size();i++)
    {
      if (frame[i].offset != INVALID_IDX)
      {
        idx_remap[i] = new_frame.size();
        new_frame.push_back(frame[i]);
      }
    }

    for (unsigned i=0;i<new_frame.size();i++)
    {
      if (new_frame[i].offset > 0)
        new_frame[i].offset = idx_remap[new_frame[i].offset];
    }

    frame = new_frame;
  }

  void frame_octree_to_SVS_rec(const std::vector<SdfFrameOctreeNode> &frame,
                               std::vector<SdfSVSNode> &nodes,
                               unsigned idx, uint3 p, unsigned lod_size)
  {
    unsigned ofs = frame[idx].offset;
    if (is_leaf(ofs)) 
    {
      float d_max = 2*sqrt(3)/lod_size;
      float min_val = 1000;
      float max_val = -1000;
      for (int i=0;i<8;i++)
      {
        min_val = std::min(min_val, frame[idx].values[i]);
        max_val = std::max(max_val, frame[idx].values[i]);
      }
      
      if (is_border_node(min_val, max_val, lod_size))
      {
        nodes.emplace_back();
        nodes.back().pos_xy = (p.x << 16) | p.y;
        nodes.back().pos_z_lod_size = (p.z << 16) | lod_size;

        nodes.back().values[0] = 0u;
        nodes.back().values[1] = 0u;
        for (int i=0;i<8;i++)
        {
          unsigned d_compressed = std::max(0.0f, 255*((frame[idx].values[i]+d_max)/(2*d_max)) + 0.5f);
          d_compressed = std::min(d_compressed, 255u);
          //assert(d_compressed < 256);
          nodes.back().values[i/4] |= d_compressed << (8*(i%4));
        }
      }
    }
    else
    {
      for (int i = 0; i < 8; i++)
      {
        uint3 ch_p = 2 * p + uint3((i & 4) >> 2, (i & 2) >> 1, i & 1);
        frame_octree_to_SVS_rec(frame, nodes, ofs + i, ch_p, 2*lod_size);
      }
    }
  }

  static void closest_triangle_idx(const cmesh4::SimpleMesh &mesh, const cmesh4::TriangleListOctree &tl_octree,
                                   float3 pos, unsigned idx, int &min_ti, float &min_dist_sq)
  {
    for (int j = 0; j < tl_octree.nodes[idx].tid_count; j++)
    {
      int t_i = tl_octree.triangle_ids[tl_octree.nodes[idx].tid_offset + j];

      float3 a = to_float3(mesh.vPos4f[mesh.indices[3 * t_i + 0]]);
      float3 b = to_float3(mesh.vPos4f[mesh.indices[3 * t_i + 1]]);
      float3 c = to_float3(mesh.vPos4f[mesh.indices[3 * t_i + 2]]);
      float3 vt = pos - cmesh4::closest_point_triangle(pos, a, b, c);
      float dist_sq = LiteMath::dot(vt, vt);

      if (dist_sq < min_dist_sq)
      {
        min_dist_sq = dist_sq;
        min_ti = t_i;
      }
    }
  }

  void mesh_octree_to_sdf_frame_octree_tex_rec(const cmesh4::SimpleMesh &mesh,
                                               const cmesh4::TriangleListOctree &tl_octree,
                                               std::vector<SdfFrameOctreeTexNode> &frame,
                                               unsigned idx, float3 p, float d)
  {
    unsigned ofs = tl_octree.nodes[idx].offset;
    frame[idx].offset = ofs;

    if (is_leaf(ofs)) 
    {
      float3 pos = 2.0f*(d*p) - 1.0f;
      for (int i = 0; i < 8; i++)
      {
        float3 ch_pos = pos + 2*d*float3((i & 4) >> 2, (i & 2) >> 1, i & 1);
        float min_dist_sq = 1000;
        int min_ti = -1;
        closest_triangle_idx(mesh, tl_octree, ch_pos, idx, min_ti, min_dist_sq);

        if (min_ti >= 0)
        {
          float3 a = to_float3(mesh.vPos4f[mesh.indices[3*min_ti+0]]);
          float3 b = to_float3(mesh.vPos4f[mesh.indices[3*min_ti+1]]);
          float3 c = to_float3(mesh.vPos4f[mesh.indices[3*min_ti+2]]);
          float3 surface_pos = cmesh4::closest_point_triangle(ch_pos, a, b, c);
          //float3 n = normalize(cross(b-a, c-a));
          float3 n = (1.0f/3.0f)*(to_float3(mesh.vNorm4f[mesh.indices[3*min_ti+0]]) + 
                                  to_float3(mesh.vNorm4f[mesh.indices[3*min_ti+1]]) + 
                                  to_float3(mesh.vNorm4f[mesh.indices[3*min_ti+2]]));
          frame[idx].values[i] = dot(normalize(n), ch_pos - surface_pos) > 0 ? sqrt(min_dist_sq) : -sqrt(min_dist_sq);

          float3 bc = cmesh4::barycentric(surface_pos, a, b, c);
          float2 tc = bc.x*mesh.vTexCoord2f[mesh.indices[3*min_ti+0]] + bc.y*mesh.vTexCoord2f[mesh.indices[3*min_ti+1]] + bc.z*mesh.vTexCoord2f[mesh.indices[3*min_ti+2]];
          frame[idx].tex_coords[2*i] = tc.x;
          frame[idx].tex_coords[2*i+1] = tc.y;
        }
        else
        {
          frame[idx].values[i] = 1000;
          frame[idx].tex_coords[2*i] = 0;
          frame[idx].tex_coords[2*i+1] = 0;
        }
      }

      //material id from triangle closest to the center
      if (mesh.matIndices.size() == mesh.TrianglesNum())
      {
        float3 center = pos + d*float3(1,1,1);
        float min_dist_sq = 1000;
        int min_ti = -1;
        closest_triangle_idx(mesh, tl_octree, center, idx, min_ti, min_dist_sq);
        if (min_ti >= 0)
          frame[idx].material_id = mesh.matIndices[min_ti];
        else
          frame[idx].material_id = 0;
        //printf("material id %u\n", frame[idx].material_id);
      }
      else
      {
        frame[idx].material_id = 0;
      }

    }
    else
    {
      for (int i = 0; i < 8; i++)
      {
        float ch_d = d / 2;
        float3 ch_p = 2 * p + float3((i & 4) >> 2, (i & 2) >> 1, i & 1);
        mesh_octree_to_sdf_frame_octree_tex_rec(mesh, tl_octree, frame, ofs + i, ch_p, ch_d);
      }
    }
  }

  void mesh_octree_to_sdf_frame_octree_tex(const cmesh4::SimpleMesh &mesh,
                                           const cmesh4::TriangleListOctree &tl_octree, 
                                           std::vector<SdfFrameOctreeTexNode> &out_frame)
  {
    out_frame.resize(tl_octree.nodes.size());
    mesh_octree_to_sdf_frame_octree_tex_rec(mesh, tl_octree, out_frame, 0, float3(0,0,0), 1);
  }

  void octree_to_layers_tex(const std::vector<SdfFrameOctreeTexNode> &nodes, std::vector<float4> &layers, unsigned idx, float3 p, float d)
  {
    layers[idx] = float4(p.x, p.y, p.z, d);
    unsigned ofs = nodes[idx].offset;
    if (!is_leaf(ofs)) 
    {
      for (int i = 0; i < 8; i++)
      {
        float ch_d = d / 2;
        float3 ch_p = 2 * p + float3((i & 4) >> 2, (i & 2) >> 1, i & 1);
        octree_to_layers_tex(nodes, layers, ofs + i, ch_p, ch_d);
      }
    }
  }

  SdfSBS frame_octree_to_SBS_tex(MultithreadedDistanceFunction sdf, 
                                 unsigned max_threads,
                                 const std::vector<SdfFrameOctreeTexNode> &nodes,
                                 const SdfSBSHeader &header)
  {
std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    omp_set_num_threads(max_threads);
    std::vector<float4> layers(nodes.size());
    octree_to_layers_tex(nodes, layers, 0, float3(0,0,0), 1.0f);

std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    SdfSBS sbs;
    uint32_t v_size = header.brick_size + 2*header.brick_pad + 1;
    sbs.header = header;
    sbs.header.aux_data = SDF_SBS_NODE_LAYOUT_DX_UV16;
    sbs.nodes.reserve(nodes.size());
    sbs.values.reserve(nodes.size() * v_size * v_size * v_size);

    unsigned step = (nodes.size() + max_threads - 1) / max_threads;

    #pragma omp parallel for
    for (int thread_id=0;thread_id<max_threads;thread_id++)
    {
      std::vector<float> values(v_size*v_size*v_size, 1000.0f);
      unsigned start = thread_id * step;
      unsigned end = std::min(start + step, (unsigned)nodes.size());
      for (int idx = start; idx < end; idx++)
      {
        unsigned ofs = nodes[idx].offset;
        if (is_leaf(ofs)) 
        {
          uint3 p = uint3(layers[idx].x, layers[idx].y, layers[idx].z);
          float d = layers[idx].w;

          float min_val = 1000;
          float max_val = -1000;
          for (int i=0;i<8;i++)
          {
            min_val = std::min(min_val, nodes[idx].values[i]);
            max_val = std::max(max_val, nodes[idx].values[i]);
          }
          float3 p0 = 2.0f*(d*float3(p)) - 1.0f;
          float dp = 2.0f*d/header.brick_size;

          for (int i=-header.brick_pad; i<=header.brick_size + header.brick_pad; i++)
          {
            for (int j=-header.brick_pad; j<=header.brick_size + header.brick_pad; j++)
            {
              for (int k=-header.brick_pad; k<=header.brick_size + header.brick_pad; k++)
              {
                float val = 2e6f;
                // corners, reuse values
                if (i == 0)
                {
                  if (j == 0)
                  {
                    if (k == 0)
                      val = nodes[idx].values[0];
                    else if (k == header.brick_size)
                      val = nodes[idx].values[1];
                  }
                  else if (j == header.brick_size)
                  {
                    if (k == 0)
                      val = nodes[idx].values[2];
                    else if (k == header.brick_size)
                      val = nodes[idx].values[3];
                  }
                }
                else if (i == header.brick_size)
                {
                  if (j == 0)
                  {
                    if (k == 0)
                      val = nodes[idx].values[4];
                    else if (k == header.brick_size)
                      val = nodes[idx].values[5];
                  }
                  else if (j == header.brick_size)
                  {
                    if (k == 0)
                      val = nodes[idx].values[6];
                    else if (k == header.brick_size)
                      val = nodes[idx].values[7];
                  }
                }

                //new points
                if (val > 1e6f)
                {
                  float3 pos = p0 + dp*float3(i,j,k);
                  val = sdf(pos, thread_id);
                }

                values[i*v_size*v_size + j*v_size + k] = val;
              }
            }      
          }

          //fix for inconsistent distances
          if (max_val - min_val > 2 * sqrt(3) * d)
          {
            //printf("inconsistent distance %f - %f with d=%f\n", max_val, min_val, d);
            for (int i = 0; i < values.size(); i++)
              values[i] = LiteMath::sign(max_val) * std::abs(values[i]);
          }

          //add node only if there is really a border
          if (is_border_node(min_val, max_val, 1/d) /*true*/)
          {
            unsigned off=0, n_off=0;
            unsigned lod_size = 1.0f/d;
            float d_max = 2*sqrt(3)/lod_size;
            unsigned bits = 8*header.bytes_per_value;
            unsigned max_val = (1 << bits) - 1;
            unsigned vals_per_int = 4/header.bytes_per_value;
            unsigned dist_size = (v_size*v_size*v_size+vals_per_int-1)/vals_per_int;
            unsigned tex_size = 8;

            #pragma omp critical
            {
              off = sbs.values.size();
              n_off = sbs.nodes.size();
              sbs.nodes.emplace_back();
              sbs.values.resize(sbs.values.size() + dist_size + tex_size);
            }

            sbs.nodes[n_off].data_offset = off;
            sbs.nodes[n_off].pos_xy = (p.x << 16) | p.y;
            sbs.nodes[n_off].pos_z_lod_size = (p.z << 16) | lod_size;

            //add distances
            for (int i=0;i<values.size();i++)
            {
              unsigned d_compressed = std::max(0.0f, max_val*((values[i]+d_max)/(2*d_max)));
              d_compressed = std::min(d_compressed, max_val);
              sbs.values[off + i/vals_per_int] |= d_compressed << (bits*(i%vals_per_int));
            }

            //add texture coordinates
            for (int i=0;i<8;i++)
            {
              //printf("id = %u, tc= %f %f\n", off, nodes[idx].tex_coords[2*i+0], nodes[idx].tex_coords[2*i+1]);
              unsigned packed_u = ((1<<16) - 1)*LiteMath::clamp(nodes[idx].tex_coords[2*i+0], 0.0f, 1.0f);
              unsigned packed_v = ((1<<16) - 1)*LiteMath::clamp(nodes[idx].tex_coords[2*i+1], 0.0f, 1.0f);
              sbs.values[off + dist_size + i] = (packed_u << 16) | (packed_v & 0x0000FFFF);
              //printf("hex sbs.values[%d] = %x\n", off + dist_size + i, sbs.values[off + dist_size + i]);
            }
          }
        } //end if is leaf
      }
    }
std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();

    float time_1 = 1e-3f*std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
    float time_2 = 1e-3f*std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();
    //printf("frame octree to SBS: time = %6.2f ms (%.1f+%.1f)\n", time_1 + time_2, time_1, time_2);

    sbs.nodes.shrink_to_fit();
    sbs.values.shrink_to_fit();
    omp_set_num_threads(omp_get_max_threads());

    return sbs;
  }

  SdfSBS SBS_col_to_SBS_ind(const SdfSBS &sbs)
  {
    SdfSBS sbs_ind;
    //same header, except for layout
    //same nodes and offsets (UV and RGB occupy the same space)
    //completely different data structure (it is now only indices) + 
    //additional array for actual values (floats)
    sbs_ind.header = sbs.header;
    sbs_ind.header.aux_data = SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F;

    sbs_ind.nodes = sbs.nodes;

    unsigned vals_per_int = 4/sbs.header.bytes_per_value;
    unsigned v_size = sbs.header.brick_size + 2*sbs.header.brick_pad + 1;
    unsigned dist_size = (v_size*v_size*v_size+vals_per_int-1)/vals_per_int;
    unsigned bits = 8*sbs.header.bytes_per_value;
    unsigned max_val = sbs.header.bytes_per_value == 4 ? 0xFFFFFFFF : ((1 << bits) - 1);

    sbs_ind.values.resize(sbs.nodes.size()*(v_size*v_size*v_size + 8)); //v_size^3 indices for distances + 8 for textures
    unsigned f_offset = 0;
    unsigned i_offset = 0;

    std::unordered_map<float3, unsigned, PositionHasher, PositionEqual> distance_indices;
    std::unordered_map<float3, unsigned, PositionHasher, PositionEqual> color_indices;

    for (int n_idx = 0; n_idx < sbs.nodes.size(); n_idx++)
    {
      const SdfSBSNode &n = sbs.nodes[n_idx];
      float px = n.pos_xy >> 16;
      float py = n.pos_xy & 0x0000FFFF;
      float pz = n.pos_z_lod_size >> 16;
      float sz = n.pos_z_lod_size & 0x0000FFFF;
      float sz_inv = 2.0f/sz;
      float d = 2.0f/(sz*sbs.header.brick_size);

      sbs_ind.nodes[n_idx].data_offset = i_offset;

      unsigned col_off = n.data_offset + dist_size;
      for (unsigned i = 0; i < v_size*v_size*v_size; i++)
      {
        uint32_t v_off = n.data_offset;
        float d_max = 2*1.73205081f/sz;
        float mult = 2*d_max/max_val;
        float val = -d_max + mult*((sbs.values[n.data_offset + i/vals_per_int] >> (bits*(i%vals_per_int))) & max_val);

        uint3 voxelPos = uint3(i/(v_size*v_size), i/v_size%v_size, i%v_size);
        float3 pos = float3(-1,-1,-1) + 2.0f*(float3(px,py,pz)/sz + float3(voxelPos)/(sz*sbs.header.brick_size));
                      
        //printf("%u %u val = %f pos = %f %f %f\n", n.data_offset, i, val, pos.x, pos.y, pos.z);
        //printf("min_pos, pos, sz = %f %f %f %f %f %f %f\n", px, py, pz, pos.x, pos.y, pos.z, sz);
        auto it = distance_indices.find(pos);
        if (it == distance_indices.end())
        {
          sbs_ind.values_f.push_back(val);
          distance_indices[pos] = sbs_ind.values_f.size() - 1;
          sbs_ind.values[i_offset + i] = sbs_ind.values_f.size() - 1;
        }
        else
        {
          sbs_ind.values[i_offset + i] = it->second;
        }
      }
      for (unsigned i = 0; i < 8; i++)
      {
        unsigned col_packed = sbs.values[col_off + i];
        float3 f_color = float3(col_packed & 0xFF, (col_packed >> 8) & 0xFF, (col_packed >> 16) & 0xFF)/255.0f;

        float3 pos = float3(px, py, pz) + float3((i & 4) >> 2, (i & 2) >> 1, i & 1)/sz;
        auto it = distance_indices.find(pos);
        if (it == color_indices.end())
        {
          sbs_ind.values_f.push_back(f_color.x);
          sbs_ind.values_f.push_back(f_color.y);
          sbs_ind.values_f.push_back(f_color.z);
          color_indices[pos] = sbs_ind.values_f.size() - 3;
          sbs_ind.values[i_offset + v_size*v_size*v_size + i] = sbs_ind.values_f.size() - 3;
        }
        else
        {
          sbs_ind.values[i_offset + v_size*v_size*v_size + i] = it->second;
        }
      }
      i_offset += v_size*v_size*v_size + 8;
    }

    return sbs_ind;
  }
}