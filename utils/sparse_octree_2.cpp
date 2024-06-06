#include "sparse_octree_2.h"
#include "omp.h"
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

    printf("total nodes = %d, time = %6.2f ms (%.1f+%.1f+%.1f+%.1f)\n", 
           (int)res_nodes.size(), time_1 + time_2 + time_3 + time_4, time_1, time_2, time_3, time_4);

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
    printf("octree to frame octree: time = %6.2f ms (%.1f+%.1f)\n", time_1 + time_2, time_1, time_2);

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
    std::vector<SdfFrameOctreeNode> frame(nodes.size());
    octree_to_layers(nodes, layers, 0, float3(0,0,0), 1.0f);

std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    SdfSBS sbs;
    sbs.header = header;
    sbs.nodes.reserve(nodes.size());
    sbs.values.reserve(nodes.size() * header.v_size * header.v_size * header.v_size);

    unsigned step = (nodes.size() + max_threads - 1) / max_threads;

    #pragma omp parallel for
    for (int thread_id=0;thread_id<max_threads;thread_id++)
    {
      std::vector<float> values(header.v_size*header.v_size*header.v_size, 1000.0f);
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

                values[i*header.v_size*header.v_size + j*header.v_size + k] = val;
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
    printf("frame octree to SBS: time = %6.2f ms (%.1f+%.1f)\n", time_1 + time_2, time_1, time_2);

    sbs.nodes.shrink_to_fit();
    sbs.values.shrink_to_fit();
    omp_set_num_threads(omp_get_max_threads());

    return sbs;
  }
}