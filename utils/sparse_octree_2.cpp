#include "sparse_octree_2.h"
#include "omp.h"
#include <chrono>

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
          frame[idx].values[i] = sdf(ch_pos, thread_id);
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
}