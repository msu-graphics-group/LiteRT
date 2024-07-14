#include "sparse_octree.h"
#include "stat_box.h"
#include <cassert>
#include <map>
#include <chrono>

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

SparseOctreeBuilder::SparseOctreeBuilder()
{
  std::vector<Node> nodes;
  octree_f = get_SdfOctreeFunction(nodes);
}

bool SparseOctreeBuilder::is_border(float distance, int level)
{
  return level < 2  ? true : std::abs(distance) < sqrt(3)*pow(2, -level);
}

static constexpr unsigned INVALID_IDX = 1u<<31u;
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

void SparseOctreeBuilder::add_node_rec(unsigned node_idx, unsigned depth, unsigned max_depth, float3 p, float d)
{
  auto &nodes = get_nodes();
  nodes[node_idx].value = sdf(2.0f * ((p + float3(0.5, 0.5, 0.5)) * d) - float3(1, 1, 1));

  if (depth < max_depth && is_border(nodes[node_idx].value, depth))
  {
    nodes[node_idx].offset = nodes.size();
    
    nodes.resize(nodes.size() + 8);
    unsigned idx = nodes[node_idx].offset;
    for (unsigned cid = 0; cid < 8; cid++)
    {
      add_node_rec(idx + cid, depth + 1, max_depth, 2 * p + float3((cid & 4) >> 2, (cid & 2) >> 1, cid & 1), d / 2);
    }
  }
}

bool DBG = false;

SparseOctreeBuilder::T SparseOctreeBuilder::sample(const float3 &position, unsigned max_level) const
{
  return octree_f->eval_distance_level(position, max_level);
}

SparseOctreeBuilder::T SparseOctreeBuilder::sample_closest(const float3 &position) const
{
  auto &nodes = get_nodes();
  //if (std::abs(position).x > 0.9 || std::abs(position).y > 0.9 || std::abs(position).z > 0.9)
  //  return 0.1;
  float3 pos = LiteMath::clamp(0.5f*(position + 1.0f), 0.0f, 1.0f);
  unsigned idx = 0;
  float d = 1;
  float3 p = float3(0,0,0);
  while (nodes[idx].offset != 0)
  {
    float3 pindf = pos/d - p;
    unsigned ch_index = 4*(pindf.x >= 0.5) + 2*(pindf.y >= 0.5) + (pindf.z >= 0.5);
    //printf("%u pindf %f %f %f %u\n",idx, pindf.x, pindf.y, pindf.z, ch_index);
    idx = nodes[idx].offset + ch_index;
    d = d/2;
    p = 2*p + float3((ch_index & 4) >> 2, (ch_index & 2) >> 1, ch_index & 1);
  }
  //printf("\n");
  //printf("%u last pindf \n",idx);

  return nodes[idx].value;
}

struct SparseOctreeCounts
{
  std::vector<unsigned> count_all;
  std::vector<unsigned> count_border;
  std::vector<unsigned> count_leaf;
  std::vector<unsigned> count_border_leaf;
};

void check_border_reappearance_rec(const std::vector<SparseOctreeBuilder::Node> &nodes, unsigned idx, unsigned level)
{
  bool b = SparseOctreeBuilder::is_border(nodes[idx].value, level); 
  if (!is_leaf(nodes[idx].offset))
  {
    for (unsigned ch_idx=0; ch_idx<8; ch_idx++)
    {
      bool ch_b = SparseOctreeBuilder::is_border(nodes[nodes[idx].offset + ch_idx].value, level+1);
      if (!b && ch_b)
        printf("reappeared border (level %d-%d): %f %f\n", level,level+1, nodes[idx].value, nodes[nodes[idx].offset + ch_idx].value);
      check_border_reappearance_rec(nodes, nodes[idx].offset + ch_idx, level+1);
    }
  }
}

void print_stat_rec(const std::vector<SparseOctreeBuilder::Node> &nodes, SparseOctreeCounts &counts, unsigned idx, unsigned level)
{
  if (level == counts.count_all.size())
  {
    counts.count_all.push_back(0);
    counts.count_border.push_back(0);
    counts.count_leaf.push_back(0);
    counts.count_border_leaf.push_back(0);
  }
  counts.count_all[level] += 1;
  counts.count_border[level] += SparseOctreeBuilder::is_border(nodes[idx].value, level);
  counts.count_leaf[level] += is_leaf(nodes[idx].offset);
  counts.count_border_leaf[level] += SparseOctreeBuilder::is_border(nodes[idx].value, level) && is_leaf(nodes[idx].offset);
  if (!is_leaf(nodes[idx].offset))
  {
    for (unsigned ch_idx=0; ch_idx<8; ch_idx++)
      print_stat_rec(nodes, counts, nodes[idx].offset + ch_idx, level+1);
  }
}

void SparseOctreeBuilder::print_stat() const
{
  auto &nodes = get_nodes();
  check_border_reappearance_rec(nodes, 0, 0);

  SparseOctreeCounts counts;
  print_stat_rec(nodes, counts, 0, 0);
  printf("SparseOctreeBuilder::print_stat\n");
  printf("Levels: %d\n", (int)counts.count_all.size());
  float tmax = 1;
  for (int i=0;i<counts.count_all.size();i++)
  {
    float m = 100.0f/counts.count_all[i];
    printf("Level %2d: cnt %6u, b %5u (%5.1f%%), l %5u (%5.1f%%), bl %5u (%5.1f%%)  occ %5.1f%%\n", i, counts.count_all[i], 
           counts.count_border[i], m*counts.count_border[i], 
           counts.count_leaf[i], m*counts.count_leaf[i],
           counts.count_border_leaf[i], m*counts.count_border_leaf[i],
           100.0f*counts.count_all[i]/tmax);
    tmax *= 8;
  }
}

std::pair<float,float> SparseOctreeBuilder::estimate_quality(float dist_thr, unsigned samples) const
{
  stat::Bins<float> stat_box(0.0f, 0.1f);
  unsigned i = 0;
  std::vector<float> differences(samples, 0);
  while (i < samples)
  {
    float3 p = float3(urand(-1,1),urand(-1,1),urand(-1,1));
    float ref_d = sdf(p);
    if (std::abs(ref_d) < dist_thr)
    {
      differences[i] = sample(p) - ref_d;
      stat_box.add(std::abs(differences[i]));
      i++;
    }
  }

  double av_diff = 0;
  double max_diff = 0;
  for (auto &d : differences)
  {
    av_diff += std::abs(d);
    max_diff = std::max(max_diff, (double)d);
  }
  av_diff /= samples;
  //stat_box.print_bins();
  return {av_diff, max_diff};
}

float2 invalidate_node_rec(std::vector<SparseOctreeBuilder::Node> &nodes, unsigned idx)
{
  unsigned ofs = nodes[idx].offset;
  if (!is_leaf(ofs)) 
  {
    float min_val = 1e9;
    float avg_val = 0;
    for (int i=0;i<8;i++)
    {
      float2 min_avg = invalidate_node_rec(nodes, ofs + i);
      min_val = std::min(min_val, min_avg.x);
      avg_val += min_avg.y;
      nodes[ofs + i].offset |= INVALID_IDX;
    }
    return float2(min_val, avg_val/8);
  }
  else
    return float2(nodes[idx].value);
}

void remove_linear_rec(SparseOctreeBuilder &octree, float thr, unsigned min_level_to_remove, unsigned idx, unsigned level, float3 p, float d)
{
  unsigned ofs = octree.get_nodes()[idx].offset;
  if (is_leaf(ofs)) 
    return;
  
  bool diff_less = true;
  if (level >= min_level_to_remove)
  {
    for (int i=0;i<8;i++)
    {
      float ch_d = d/2;
      float3 ch_p = 2*p + float3((i & 4) >> 2, (i & 2) >> 1, i & 1);
      float dist = octree.get_nodes()[ofs + i].value;
      float sampled_dist = octree.sample(2.0f*(ch_d*(ch_p + float3(0.5,0.5,0.5))) - 1.0f, level-1);
      //printf("d sd %f %f\n",dist, sampled_dist);
      if (std::abs(dist - sampled_dist) > thr)
      {
        diff_less = false;
        break;
      }
    }  
  }
  else
    diff_less = false;

  if (!diff_less)
  {
    for (int i=0;i<8;i++)
    {
      float ch_d = d/2;
      float3 ch_p = 2*p + float3((i & 4) >> 2, (i & 2) >> 1, i & 1);
      remove_linear_rec(octree, thr, min_level_to_remove, ofs + i, level+1, ch_p, ch_d);
    }
  }
  else
  {
    octree.get_nodes()[idx].value = invalidate_node_rec(octree.get_nodes(), idx).y;
    octree.get_nodes()[idx].offset |= INVALID_IDX;
  }
}

void all_to_valid_nodes_remap_rec(const std::vector<SparseOctreeBuilder::Node> &all_nodes, 
                                  std::vector<int> &all_to_valid_remap,
                                  unsigned *valid_counter,
                                  unsigned idx)
{
  unsigned ch_idx = all_nodes[idx].offset;
  if (is_leaf(ch_idx))
    return;
  else
  {
    for (int i=0;i<8;i++)
      all_to_valid_remap[ch_idx+i] = (*valid_counter) + i;
    
    (*valid_counter) += 8;

    for (int i=0;i<8;i++)
      all_to_valid_nodes_remap_rec(all_nodes, all_to_valid_remap, valid_counter, ch_idx+i);
  }
}

void SparseOctreeBuilder::construct_bottom_up_finish()
{
  auto &nodes = get_nodes();
  //auto p = estimate_quality(10, 100000);
  //printf("estimate_quality: %f %f\n", (float)p.first, (float)p.second);

  std::vector<int> valid_remap(nodes.size(), -1);
  unsigned valid_count = 1;

  valid_remap[0] = 0; //root is always valid
  all_to_valid_nodes_remap_rec(nodes, valid_remap, &valid_count, 0);

  std::vector<Node> valid_nodes(valid_count);
  for (int i=0;i<nodes.size();i++)
  {
    if (valid_remap[i] >= 0)
    {
      valid_nodes[valid_remap[i]] = nodes[i];
      if (is_leaf(valid_nodes[valid_remap[i]].offset))
        valid_nodes[valid_remap[i]].offset = 0;
      else
        valid_nodes[valid_remap[i]].offset = valid_remap[valid_nodes[valid_remap[i]].offset];
    }
  }
  
  nodes = valid_nodes;

  //printf("SDF octree created with %u (%5.1f%%) nodes (dense one would have %u)\n", 
  //       valid_count, 100.0f*valid_count/(unsigned)valid_remap.size(), (unsigned)valid_remap.size());
}

void SparseOctreeBuilder::construct_bottom_up_base(unsigned start_depth, float3 start_p, float start_d)
{
  auto &nodes = get_nodes();
  nodes.clear();

  // create octree on border nodes
  nodes.emplace_back();
  add_node_rec(0, start_depth, settings.depth, start_p, start_d);

  //remove some nodes with estimated error is lower than given threshold
  //currently not used, use nodes_limit instead
  //if (settings.remove_thr > 0)
  //  remove_linear_rec(*this, settings.remove_thr, min_remove_level, 0, start_depth, start_p, start_d);  
}

struct cmpUint3 {
    bool operator()(const uint3& a, const uint3& b) const 
    {
      if (a.x < b.x)
        return true;
      else if (a.x > b.x)
        return false;
      if (a.y < b.y)
        return true;
      else if (a.y > b.y)
        return false;
      return a.z < b.z;
      
    }
};

void add_all_blocks_rec(std::vector<std::vector<BlockSparseOctree<float>::BlockInfo>> &all_blocks, 
                        std::vector<std::vector<bool>> &block_active, 
                        std::vector<std::map<uint3, unsigned, cmpUint3>> &block_idx_to_block_n, 
                        unsigned mip, unsigned idx)
{
  if (all_blocks[mip][idx].mip > 0)
  {
    for (int i=0;i<8;i++)
    {
      uint3 off = uint3((i & 4) >> 2, (i & 2) >> 1, i & 1); 
      uint3 pidx = 2*all_blocks[mip][idx].coords + off;
      all_blocks[mip-1].push_back({pidx, mip-1, 0});
      block_active[mip-1].push_back(false);
      block_idx_to_block_n[mip-1][pidx] = all_blocks[mip-1].size()-1;
      add_all_blocks_rec(all_blocks, block_active, block_idx_to_block_n, mip-1, all_blocks[mip-1].size()-1);
    }
  }
}

void find_active_blocks_rec(const SparseOctreeBuilder &octree, std::vector<std::vector<bool>> &block_active, 
                            std::vector<std::map<uint3, unsigned, cmpUint3>> &block_idx_to_block_n,
                            unsigned min_depth, unsigned max_block_mip, 
                            unsigned idx, unsigned level, uint3 pixel_idx)
{
  if ((octree.get_nodes()[idx].offset & INVALID_IDX) == 0)
  {
    if (level >= min_depth)
    {
      unsigned block_mip = max_block_mip - (level - min_depth);
      uint3 block_idx = pixel_idx/uint3(BlockSparseOctree<float>::BLOCK_SIZE_X, BlockSparseOctree<float>::BLOCK_SIZE_Y, BlockSparseOctree<float>::BLOCK_SIZE_Z);
      block_active[block_mip][block_idx_to_block_n[block_mip][block_idx]] = true;
    }
    if (!is_leaf(octree.get_nodes()[idx].offset))
    {
      for (int i=0;i<8;i++)
      {
        uint3 off = uint3((i & 4) >> 2, (i & 2) >> 1, i & 1); 
        find_active_blocks_rec(octree, block_active, block_idx_to_block_n, min_depth, max_block_mip, octree.get_nodes()[idx].offset+i, level+1, 2*pixel_idx+off);
      }
    }
  }
}

void restore_invalid_nodes_in_active_blocks_rec(SparseOctreeBuilder &octree, std::vector<std::vector<bool>> &block_active, 
                                                std::vector<std::map<uint3, unsigned, cmpUint3>> &block_idx_to_block_n,
                                                unsigned min_depth, unsigned max_block_mip, 
                                                unsigned idx, unsigned level, uint3 pixel_idx)
{
  unsigned real_offset = octree.get_nodes()[idx].offset & (~INVALID_IDX);
  //printf("offset %u %u \n", octree.get_nodes()[idx].offset, real_offset);

  if (level >= min_depth)
  {
    unsigned block_mip = max_block_mip - (level - min_depth);
    uint3 block_idx = pixel_idx/uint3(BlockSparseOctree<float>::BLOCK_SIZE_X, BlockSparseOctree<float>::BLOCK_SIZE_Y, BlockSparseOctree<float>::BLOCK_SIZE_Z);

    //invalid node in active block
    if ((octree.get_nodes()[idx].offset & INVALID_IDX) != 0 && block_active[block_mip][block_idx_to_block_n[block_mip][block_idx]])
    {
      //printf("reactivated node %u in block %u %u %u\n", idx, block_idx.x, block_idx.y, block_idx.z);
      octree.get_nodes()[idx].offset = real_offset;
    }
  }

  if (real_offset != 0)
  {
    for (int i=0;i<8;i++)
    {
      uint3 off = uint3((i & 4) >> 2, (i & 2) >> 1, i & 1); 
      restore_invalid_nodes_in_active_blocks_rec(octree, block_active, block_idx_to_block_n, min_depth, max_block_mip, real_offset+i, level+1, 2*pixel_idx+off);
    }    
  }
}

void node_values_to_blocks_rec(SparseOctreeBuilder &octree, std::vector<std::vector<bool>> &block_active, 
                               std::vector<std::vector<BlockSparseOctree<float>::BlockInfo>> &all_blocks,
                               std::vector<std::map<uint3, unsigned, cmpUint3>> &block_idx_to_block_n,
                               std::vector<float> &blocks_data,
                               unsigned min_depth, unsigned max_block_mip, 
                               unsigned idx, unsigned level, uint3 pixel_idx)
{
  unsigned real_offset = octree.get_nodes()[idx].offset & (~INVALID_IDX);

  if (level >= min_depth)
  {
    unsigned block_mip = max_block_mip - (level - min_depth);
    uint3 block_idx = pixel_idx/uint3(BlockSparseOctree<float>::BLOCK_SIZE_X, BlockSparseOctree<float>::BLOCK_SIZE_Y, BlockSparseOctree<float>::BLOCK_SIZE_Z);
    uint3 local_idx = pixel_idx % uint3(BlockSparseOctree<float>::BLOCK_SIZE_X, BlockSparseOctree<float>::BLOCK_SIZE_Y, BlockSparseOctree<float>::BLOCK_SIZE_Z);
    unsigned local_offset = local_idx.z * BlockSparseOctree<float>::BLOCK_SIZE_X * BlockSparseOctree<float>::BLOCK_SIZE_Y +
                            local_idx.y * BlockSparseOctree<float>::BLOCK_SIZE_X + 
                            local_idx.x;

    unsigned block_pos = block_idx_to_block_n[block_mip][block_idx];
    if (block_active[block_mip][block_pos])
      blocks_data[all_blocks[block_mip][block_pos].data_offset + local_offset] = octree.get_nodes()[idx].value;
  }

  if (real_offset != 0)
  {
    for (int i=0;i<8;i++)
    {
      uint3 off = uint3((i & 4) >> 2, (i & 2) >> 1, i & 1); 
      node_values_to_blocks_rec(octree, block_active, all_blocks, block_idx_to_block_n, blocks_data, min_depth, max_block_mip, 
                                real_offset+i, level+1, 2*pixel_idx+off);
    }    
  }
}

void SparseOctreeBuilder::construct_bottom_up_blocks(DistanceFunction _sdf, SparseOctreeSettings _settings, 
                                              BlockSparseOctree<T> &out_bso)
{
  sdf = _sdf;
  settings = _settings;
  min_remove_level = std::min(settings.depth, 4u);
  octree_f->get_nodes().clear();

  unsigned max_block_size = std::max(BlockSparseOctree<float>::BLOCK_SIZE_X, std::max(BlockSparseOctree<float>::BLOCK_SIZE_Y, BlockSparseOctree<float>::BLOCK_SIZE_Z));
  unsigned max_depth_blocks = settings.depth - log2(max_block_size);

  //I - construct octree, leaving removed modes intact but with INVALID_IDX flad for their offsets
  construct_bottom_up_base(0u, float3(0,0,0), 1.0f);

  //II - restore invalid nodes in active blocks
  std::vector<std::vector<bool>> block_active;
  std::vector<std::vector<BlockSparseOctree<float>::BlockInfo>> all_blocks;
  std::vector<std::map<uint3, unsigned, cmpUint3>> block_idx_to_block_n;
  all_blocks.resize(max_depth_blocks + 1);
  block_active.resize(max_depth_blocks + 1);
  block_idx_to_block_n.resize(max_depth_blocks + 1);

  //convert SparseOctree to BlockSparseOctree
  out_bso.top_mip = max_depth_blocks;

  //create two root blocks and fill all_blocks with all possible blocks
  unsigned z_off = 0;
  while (z_off*BlockSparseOctree<float>::BLOCK_SIZE_Z != BlockSparseOctree<float>::BLOCK_SIZE_X)
  {
    all_blocks[max_depth_blocks].push_back({uint3(0,0,z_off), max_depth_blocks, 0}); 
    block_active[max_depth_blocks].push_back(false);
    block_idx_to_block_n[max_depth_blocks][uint3(0,0,z_off)] = z_off;
    add_all_blocks_rec(all_blocks, block_active, block_idx_to_block_n, max_depth_blocks, z_off);
    z_off++;
  }

  //check all nodes by block to find active blocks
  find_active_blocks_rec(*this, block_active, block_idx_to_block_n, log2(BlockSparseOctree<float>::BLOCK_SIZE_X), max_depth_blocks, 0, 0, uint3(0,0,0));

  //restore invalid nodes in active blocks
  restore_invalid_nodes_in_active_blocks_rec(*this, block_active, block_idx_to_block_n, 
                                             log2(BlockSparseOctree<float>::BLOCK_SIZE_X), max_depth_blocks, 
                                             0, 0, uint3(0,0,0));

  //calculate offsets for active blocks and allocate memory for it
  unsigned offset = 0;
  for (int i=0;i<max_depth_blocks + 1; i++)
  {
    for (int j=0;j<all_blocks[i].size();j++)
    {
      if (block_active[i][j])
      {
        all_blocks[i][j].data_offset = offset;
        offset += out_bso.block_size.x*out_bso.block_size.y*out_bso.block_size.z;
      }
    }
  }
  out_bso.data.resize(offset, T(1e9));

  //fill buffer for data per block
  node_values_to_blocks_rec(*this, block_active, all_blocks, block_idx_to_block_n, out_bso.data, 
                            log2(BlockSparseOctree<float>::BLOCK_SIZE_X), max_depth_blocks, 
                            0, 0, uint3(0,0,0));

  //collect all active 
  for (int i=0;i<max_depth_blocks + 1; i++)
  {
    for (int j=0;j<all_blocks[i].size();j++)
    {
      if (block_active[i][j])
        out_bso.blocks.push_back(all_blocks[i][j]);
    }
  }
  
  //DEBUG:  check is all the data for blocks is collected
  /*
  for (int i=0;i<max_depth_blocks + 1; i++)
  {
    for (int j=0;j<all_blocks[i].size();j++)
    {
      if (block_active[i][j])
      {
        for (int z=0;z<16;z++)
        {
          for (int y=0;y<32;y++)
          {
            for (int x=0;x<32;x++)
            {
              unsigned off = all_blocks[i][j].data_offset + z*32*32 + y*32 + x;
              float val = out_bso.data[off];
              if (val > 1e9)
              {
                printf("block [%u %u %u] in lod %u missed value in (%d %d %d)\n",
                       all_blocks[i][j].coords.x, all_blocks[i][j].coords.y, all_blocks[i][j].coords.z,
                       all_blocks[i][j].mip, x,y,z);
              }
            }
          }
        }
      }
    }
  }
  */

  //III - finish building octree
  construct_bottom_up_finish();
}

void fill_octree_frame_rec(std::function<SparseOctreeBuilder::T(const float3 &)> f,   
                           const std::vector<SdfOctreeNode> &nodes,
                           std::vector<SdfFrameOctreeNode> &frame,
                           unsigned idx, float3 p, float d)
{
  unsigned ofs = nodes[idx].offset;
  frame[idx].offset = ofs;
  float min_val = nodes[idx].value;
  float max_val = nodes[idx].value;

  float3 pos = 2.0f * (d * p) - 1.0f;
  for (int i = 0; i < 8; i++)
  {
    float3 ch_pos = pos + 2 * d * float3((i & 4) >> 2, (i & 2) >> 1, i & 1);
    frame[idx].values[i] = f(ch_pos);
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
      fill_octree_frame_rec(f, nodes, frame, ofs + i, ch_p, ch_d);
    }
  }
}

void SparseOctreeBuilder::construct_large_cell_rec(std::vector<Node> &final_nodes, unsigned root_idx, unsigned level, float3 p, float d)
{
  //construct octree in this large cell
  if (level == min_remove_level)
  {

  }
}

//large node is a smallest unit for which octree can be built independently from other units
struct LargeNode
{
  float3 p;
  float d;
  unsigned level;
  unsigned root_idx;
  unsigned children_idx;
};

void check_and_fix_sdf_sign(std::vector<SparseOctreeBuilder::Node> &nodes, float d_thr, unsigned idx, float d)
{
  unsigned ofs = nodes[idx].offset;
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

void SparseOctreeBuilder::construct(DistanceFunction _sdf, SparseOctreeSettings _settings)
{
  sdf = _sdf;
  settings = _settings;
  min_remove_level = std::min(settings.depth, 4u);

std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

  std::vector<LargeNode> large_nodes;
  int lg_size = pow(2, min_remove_level);
  std::vector<unsigned> large_grid(lg_size*lg_size*lg_size);
  large_nodes.push_back({float3(0,0,0), 1.0f, 0u, 0u, 0u});

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
        large_nodes.push_back({ch_p, ch_d, large_nodes[i].level+1, i, 0u});
      }
    }
    i++;
  }

  std::vector<Node> all_nodes;
  all_nodes.resize(large_nodes.size());
  for (int i=0;i<large_nodes.size();i++)
  {
    float val = sdf(2.0f * ((large_nodes[i].p + float3(0.5, 0.5, 0.5)) * large_nodes[i].d) - float3(1, 1, 1));
    all_nodes[i].offset = large_nodes[i].children_idx;
    all_nodes[i].value = val;

    if (large_nodes[i].level == min_remove_level)
      large_grid[large_nodes[i].p.x*lg_size*lg_size + large_nodes[i].p.y*lg_size + large_nodes[i].p.z] = i;
  }

  for (int i=0;i<lg_size;i++)
  {
  for (int j=0;j<lg_size;j++)
  {
  for (int k=0;k<lg_size;k++)
  {
    float val = all_nodes[large_grid[i*lg_size*lg_size + j*lg_size + k]].value;
    float max_val = val;
    if (val < 0)
    {
      for (int i1=max(0,i-1);i1<min(lg_size,i+2);i1++)
        for (int j1=max(0,j-1);j1<min(lg_size,j+2);j1++)
          for (int k1=max(0,k-1);k1<min(lg_size,k+2);k1++)
            max_val = max(max_val, all_nodes[large_grid[i1*lg_size*lg_size + j1*lg_size + k1]].value);
    
      //if (max_val - val > sqrt(3)*2/lg_size)
      //  all_nodes[large_grid[i*lg_size*lg_size + j*lg_size + k]].value *= -1;
    }
    //printf("(%5.2f %5.2f)", max_val, all_nodes[large_grid[i*lg_size*lg_size + j*lg_size + k]].value);
  }    
  //printf("\n");
  }   
  //printf("\n"); 
  }

  //printf("THRRRR %f\n",sqrtf(3)*2.0f/lg_size);

  for (int i=0;i<large_nodes.size();i++)
  {
    //if it is a leaf (as a large node) and it can contain borders (with some additional confidence mult 1.05)
    if (large_nodes[i].children_idx == 0 && is_border(all_nodes[i].value, large_nodes[i].level))
    {
      construct_bottom_up_base(large_nodes[i].level, large_nodes[i].p, large_nodes[i].d);
      construct_bottom_up_finish();

      if (get_nodes().size() > 1)
      {
        //printf("create octree for node (%f %f %f) - %d nodes\n", large_nodes[i].p.x, large_nodes[i].p.y, large_nodes[i].p.z,
        //       (int)(get_nodes().size()));
        unsigned off = all_nodes.size();
        all_nodes[i].offset = off;
        all_nodes.reserve(all_nodes.size() + get_nodes().size() - 1);
        all_nodes.insert(all_nodes.end(), get_nodes().begin()+1, get_nodes().end()); //skip root node of get_nodes() as it is already in all_nodes
        for (int j=off; j<all_nodes.size(); j++) 
        {
          if (!is_leaf(all_nodes[j].offset))
            all_nodes[j].offset += off-1;
        }
      }
    }
  }

  check_and_fix_sdf_sign(all_nodes, pow(2,-1.0*min_remove_level), 0, 1.0f);

  get_nodes() = all_nodes;
  
std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  
  float time_1 = 1e-3f*std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
  unsigned long n_real = get_nodes().size();
  unsigned long n_max = pow(8, settings.depth);
  
  //printf("SDF octree created with %lu (%5.2lf%%) nodes (dense one would have %lu)\n", n_real, 100.0*n_real/n_max, n_max);
  //printf("time spent (ms) %.1f\n", time_1);
}

void SparseOctreeBuilder::convert_to_frame_octree(std::vector<SdfFrameOctreeNode> &out_frame)
{
  auto &nodes = get_nodes();
  out_frame.resize(nodes.size());
  fill_octree_frame_rec(sdf, nodes, out_frame, 0, float3(0,0,0), 1);
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

void SparseOctreeBuilder::convert_to_sparse_voxel_set(std::vector<SdfSVSNode> &out_nodes)
{
  auto &nodes = get_nodes();
  std::vector<SdfFrameOctreeNode> frame(nodes.size());
  fill_octree_frame_rec(sdf, nodes, frame, 0, float3(0,0,0), 1);
  frame_octree_to_SVS_rec(frame, out_nodes, 0, uint3(0,0,0), 1);
}

void frame_octree_to_SBS_rec(std::function<SparseOctreeBuilder::T(const float3 &)> sdf, 
                             const std::vector<SdfFrameOctreeNode> &nodes,
                             const SdfSBSHeader &header,
                             std::vector<SdfSBSNode> &out_nodes, 
                             std::vector<uint32_t> &out_values,
                             unsigned idx, uint3 p, unsigned level, float d)
{
  unsigned ofs = nodes[idx].offset;
  if (is_leaf(ofs)) 
  {
    uint32_t v_size = header.brick_size + 2*header.brick_pad + 1;
    std::vector<float> values(v_size*v_size*v_size, 1000.0f);
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
          float3 p = p0 + dp*float3(i,j,k);
          float val = sdf(p);
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
    if (is_border_node(min_val, max_val, 1 << level))
    {
      unsigned off = out_values.size();
      out_nodes.emplace_back();

      unsigned lod_size = 1.0f/d;
      out_nodes.back().data_offset = off;
      out_nodes.back().pos_xy = (p.x << 16) | p.y;
      out_nodes.back().pos_z_lod_size = (p.z << 16) | lod_size;

      float d_max = 2*sqrt(3)/lod_size;
      unsigned bits = 8*header.bytes_per_value;
      unsigned max_val = (1 << bits) - 1;
      unsigned vals_per_int = 4/header.bytes_per_value;

      out_values.resize(out_values.size() + (values.size()+vals_per_int-1)/vals_per_int);
      for (int i=0;i<values.size();i++)
      {
        unsigned d_compressed = std::max(0.0f, max_val*((values[i]+d_max)/(2*d_max)));
        d_compressed = std::min(d_compressed, max_val);
        out_values[off + i/vals_per_int] |= d_compressed << (bits*(i%vals_per_int));
      }
    }
  }
  else
  {
    for (int i = 0; i < 8; i++)
    {
      uint3 ch_p = 2 * p + uint3((i & 4) >> 2, (i & 2) >> 1, i & 1);
      frame_octree_to_SBS_rec(sdf, nodes, header, out_nodes, out_values, ofs + i, ch_p, level + 1, d/2);
    }
  }
}

void SparseOctreeBuilder::convert_to_sparse_brick_set(SdfSBSHeader &header, 
                                                      std::vector<SdfSBSNode> &out_nodes, 
                                                      std::vector<uint32_t> &out_values)
{
  assert(header.brick_size >= 1 && header.brick_size <= 16);
  assert(header.brick_pad == 0 || header.brick_pad == 1);
  assert(header.bytes_per_value == 1 || header.bytes_per_value == 2 || header.bytes_per_value == 4);

  auto &nodes = get_nodes();
  std::vector<SdfFrameOctreeNode> frame(nodes.size());
  fill_octree_frame_rec(sdf, nodes, frame, 0, float3(0,0,0), 1);

  frame_octree_to_SBS_rec(sdf, frame, header, out_nodes, out_values, 0, uint3(0,0,0), 0, 1);
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

void SparseOctreeBuilder::mesh_octree_to_sdf_frame_octree(const cmesh4::SimpleMesh &mesh,
                                                          const cmesh4::TriangleListOctree &tl_octree, 
                                                          std::vector<SdfFrameOctreeNode> &out_frame)
{
  out_frame.resize(tl_octree.nodes.size());
  mesh_octree_to_sdf_frame_octree_rec(mesh, tl_octree, out_frame, 0, float3(0,0,0), 1);
}

void SparseOctreeBuilder::mesh_octree_to_SVS(const cmesh4::SimpleMesh &mesh,
                                             const cmesh4::TriangleListOctree &tl_octree, 
                                             std::vector<SdfSVSNode> &out_SVS)
{
  std::vector<SdfFrameOctreeNode> frame; 
  mesh_octree_to_sdf_frame_octree(mesh, tl_octree, frame);
  frame_octree_to_SVS_rec(frame, out_SVS, 0, uint3(0,0,0), 1);
}

float SparseOctreeBuilder::check_quality(std::function<float(const float3 &)> f, const std::vector<SdfSVSNode> &nodes)
{
  double diff = 0;
  double diff_abs = 0;
  int cnt = 0;
  for (int i=0;i<nodes.size();i++)
  {
    float px = nodes[i].pos_xy >> 16;
    float py = nodes[i].pos_xy & 0x0000FFFF;
    float pz = nodes[i].pos_z_lod_size >> 16;
    float sz = nodes[i].pos_z_lod_size & 0x0000FFFF;
    float d_max = 2*1.41421356f/sz;

    float3 boxMin = float3(-1,-1,-1) + 2.0f*float3(px,py,pz)/sz;
    float3 boxMax = boxMin + 2.0f*float3(1,1,1)/sz;
    //printf("box (%f %f %f) - (%f %f %f)\n", boxMin.x, boxMin.y, boxMin.z, boxMax.x, boxMax.y, boxMax.z);
    float local_diff = 0;
    float local_diff_abs = 0;
    float max_val = -1000;
    for (int j=0;j<8;j++)
    {
      float3 p = boxMin + float3((j & 4) >> 2, (j & 2) >> 1, j & 1)*2.0f/sz;
      unsigned q_val = (nodes[i].values[j/4] >> (8*(j%4))) & 0xFF;
      float val = -d_max + 2*d_max*(1.0/255.0f)*q_val;
      float val_real = f(p);

      //if (std::abs(val_real - val) > 0.02)
      //printf("duff (%d %f) - %f\n", q_val, val, val_real);
      max_val = std::max(max_val, val_real);
      local_diff += val_real - val;
      local_diff_abs += std::abs(val_real - val);
    }

    //if we have a "solid" with only negative values, we don't really expect it to be accurate, it's just a box
    if (max_val >= 0)
    {
      diff += local_diff;
      diff_abs += local_diff_abs;
      cnt += 8;
    }
    //printf("\n");
  }

  printf("diff = %f, diff_abs = %f\n", diff/cnt, diff_abs/cnt);

  return diff_abs / cnt;
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
    printf("layer %u (%u)\n", (unsigned)layer.size(), l_cnt);
  }
}

void frame_octree_limit_nodes(std::vector<SdfFrameOctreeNode> &frame, unsigned nodes_limit,
                              bool count_only_border_nodes = false)
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

  if (false)
  {
    std::vector<std::vector<LayerFrameNodeInfo>> layers2;
    fill_layers_rec(frame, layers2, 0, 0);

    unsigned cnt = 0;
    unsigned cur_l = 0;
    for (auto &layer : layers2)
    {
      printf("layer %u\n", (unsigned)layer.size());
      for (auto &node : layer)
        printf("%d, node[%u] %d %d %u values %f %f\n", cur_l, node.idx, node.is_leaf, node.is_border,
               node.border_children, frame[node.idx].values[0], frame[node.idx].values[7]);
      cur_l++;
    }
  }

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