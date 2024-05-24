#pragma once
#include <vector>
#include <functional>
#include <fstream>
#include "../sdfScene/sdf_scene.h"
#include "../utils/mesh.h"
/*
A class that is able to represent and arbitrary function f : R^3 -> T 
as a sparse octree, where every leaf contains value of function in it's
center. 
T - is a type that can be meaningfully interpolated (i.e. float, double, float2, float3)
It should have T+T and float*T operator and be POD of course. 
Octree always represents unit cube [-1,1]^3
*/

enum class SparseOctreeBuildType
{
  DEFAULT = 0, //build from abstrace distance function, quite slow, but reliable
  MESH_TLO = 1 //works only if building from mesh, faster for detailed octrees and medium-sized meshes
};

struct SparseOctreeSettings
{
  SparseOctreeSettings() = default;
  SparseOctreeSettings(SparseOctreeBuildType type, unsigned _depth)
  {
    build_type = type;
    depth = _depth;
  }
  unsigned depth = 1;
  unsigned min_remove_level = 4; //used only with SparseOctreeBuildType::DEFAULT
  float remove_thr = 0.0001; //used only with SparseOctreeBuildType::DEFAULT
  SparseOctreeBuildType build_type = SparseOctreeBuildType::DEFAULT;
};

template <typename T>
struct BlockSparseOctree
{
  //Page size in Vulkan sparse texture
  static constexpr unsigned BLOCK_SIZE_X = 32;
  static constexpr unsigned BLOCK_SIZE_Y = 32;
  static constexpr unsigned BLOCK_SIZE_Z = 16;
  struct BlockInfo
  {
    uint3 coords; //offset in blocks inside mip
    unsigned mip;
    unsigned data_offset; //offset in T array (not bytes!). All block data stored together starting with this offset
  };

  uint3 block_size = uint3(BLOCK_SIZE_X, BLOCK_SIZE_Y, BLOCK_SIZE_Z);
  uint3 top_mip_size = uint3(1,1,2); //to create cube grid from 32*32*16 blocks
  unsigned top_mip;

  std::vector<BlockInfo> blocks;
  std::vector<T> data; //for each block values is stored in z y x order to comply with Vulkan sparse texture
};

template <typename T>
void save_block_sparse_octree(const std::string &path, const BlockSparseOctree<T> &bso)
{
  std::ofstream fs(path, std::ios::binary);
  std::vector<unsigned> metadata(9);
  metadata[0] = bso.block_size.x; 
  metadata[1] = bso.block_size.y;
  metadata[2] = bso.block_size.z;

  metadata[3] = bso.top_mip_size.x; 
  metadata[4] = bso.top_mip_size.y;
  metadata[5] = bso.top_mip_size.z;  

  metadata[6] = bso.top_mip; 
  metadata[7] = bso.blocks.size();
  metadata[8] = bso.data.size();

  fs.write((const char *)metadata.data(), metadata.size()*sizeof(unsigned));
  fs.write((const char *)bso.blocks.data(), bso.blocks.size() * sizeof(typename BlockSparseOctree<T>::BlockInfo));
  fs.write((const char *)bso.data.data(), bso.data.size() * sizeof(T));
  fs.flush();
  fs.close();
}

template <typename T>
void load_block_sparse_octree(const std::string &path, /*out*/BlockSparseOctree<T> &bso)
{
  std::ifstream fs(path, std::ios::binary);
  std::vector<unsigned> metadata(9);

  fs.read((char *)metadata.data(), metadata.size()*sizeof(unsigned));

  bso.block_size.x = metadata[0];
  bso.block_size.y = metadata[1];
  bso.block_size.z = metadata[2];

  bso.top_mip_size.x = metadata[3];
  bso.top_mip_size.y = metadata[4];
  bso.top_mip_size.z = metadata[5];

  bso.top_mip = metadata[6];
  bso.blocks.resize(metadata[7]);
  bso.data.resize(metadata[8]);

  fs.read((char *)bso.blocks.data(), bso.blocks.size() * sizeof(typename BlockSparseOctree<T>::BlockInfo));
  fs.read((char *)bso.data.data(), bso.data.size() * sizeof(T));

  fs.close();
}

class SparseOctreeBuilder
{
public:
  using Node = SdfOctreeNode;
  using T = float;
  using DistanceFunction = std::function<T(const float3 &)>;

  static bool is_border(float distance, int level);
  static void mesh_octree_to_sdf_frame_octree(const cmesh4::SimpleMesh &mesh,
                                              const cmesh4::TriangleListOctree &tl_octree, 
                                              std::vector<SdfFrameOctreeNode> &out_frame);
  static void mesh_octree_to_SVS(const cmesh4::SimpleMesh &mesh,
                                 const cmesh4::TriangleListOctree &tl_octree, 
                                 std::vector<SdfSVSNode> &out_frame);
  SparseOctreeBuilder();
  void construct(DistanceFunction f, SparseOctreeSettings settings);
  void construct_bottom_up(DistanceFunction f, SparseOctreeSettings settings);
  void construct_bottom_up_blocks(DistanceFunction f, SparseOctreeSettings settings, 
                                  BlockSparseOctree<T> &out_bso);
  void construct_bottom_up_frame(DistanceFunction f, SparseOctreeSettings settings, 
                                 std::vector<SdfFrameOctreeNode> &out_frame);
  
  void convert_to_frame_octree(std::vector<SdfFrameOctreeNode> &out_frame);
  void convert_to_sparse_voxel_set(std::vector<SdfSVSNode> &out_nodes);
  void convert_to_sparse_brick_set(SdfSBSHeader &header, std::vector<SdfSBSNode> &out_nodes, std::vector<uint32_t> &out_values);

  float check_quality(DistanceFunction f, const std::vector<SdfSVSNode> &nodes);

  T sample(const float3 &pos, unsigned max_level = 1000) const;
  T sample_closest(const float3 &pos) const;

  void print_stat() const;
  std::pair<float,float> estimate_quality(float dist_thr = 0.01, unsigned samples = 10000) const;
  std::vector<Node> &get_nodes() { return octree_f->get_nodes(); }
  const std::vector<Node> &get_nodes() const { return octree_f->get_nodes(); }

  Node &get_node(unsigned idx) { return octree_f->get_nodes()[idx]; }
  const Node &get_node(unsigned idx) const { return octree_f->get_nodes()[idx]; }
protected:
  void add_node_rec(unsigned node_idx, unsigned depth, unsigned max_depth, float3 p, float d);
  void split_children(unsigned node_idx, float threshold, float3 p, float d, unsigned level);

  void construct_bottom_up_base(unsigned start_depth, float3 start_p, float start_d);
  void construct_bottom_up_finish();
  void construct_large_cell_rec(std::vector<Node> &final_nodes, unsigned root_idx, unsigned level, float3 p, float d);

  std::shared_ptr<ISdfOctreeFunction> octree_f; //0 node is root
  DistanceFunction sdf;
  SparseOctreeSettings settings;
};