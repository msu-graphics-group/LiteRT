#pragma once
#include "LiteMath/LiteMath.h"
#include <vector>
#include <string>
#include <memory>
#include <cassert>

using LiteMath::float2;
using LiteMath::float3;
using LiteMath::float4;
using LiteMath::uint2;
using LiteMath::uint3;
using LiteMath::uint4;
using LiteMath::int2;
using LiteMath::int3;
using LiteMath::int4;
using LiteMath::float4x4;
using LiteMath::float3x3;
using LiteMath::cross;
using LiteMath::dot;
using LiteMath::length;
using LiteMath::normalize;
using LiteMath::to_float4;
using LiteMath::to_float3;
using LiteMath::max;
using LiteMath::min;

//################################################################################
// Constants and plain data structures definitions. Used both on GPU with slicer 
// and on CPU.
//################################################################################
// enum SdfPrimitiveType
static constexpr unsigned SDF_PRIM_SPHERE = 0;
static constexpr unsigned SDF_PRIM_BOX = 1;
static constexpr unsigned SDF_PRIM_CYLINDER = 2;
static constexpr unsigned SDF_PRIM_SIREN = 3;

static constexpr unsigned NEURAL_SDF_MAX_LAYERS = 8;
static constexpr unsigned NEURAL_SDF_MAX_LAYER_SIZE = 1024;
static constexpr float SIREN_W0 = 30;
static constexpr unsigned SDF_SBS_ADAPT_MAX_UNITS = 1 << 15u;

static constexpr unsigned INVALID_IDX = 1u<<31u;

// enum SBSInSide
static constexpr unsigned SBS_IN_SIDE_X_NEG = 0u;
static constexpr unsigned SBS_IN_SIDE_X_POS = 1u;
static constexpr unsigned SBS_IN_SIDE_Y_NEG = 2u;
static constexpr unsigned SBS_IN_SIDE_Y_POS = 3u;
static constexpr unsigned SBS_IN_SIDE_Z_NEG = 4u;
static constexpr unsigned SBS_IN_SIDE_Z_POS = 5u;

// enum SdfSBSNodeLayout
static constexpr unsigned SDF_SBS_NODE_LAYOUT_UNDEFINED        = 0 << 24u; //should be interpreted as SDF_SBS_NODE_LAYOUT_D for legacy reasons
static constexpr unsigned SDF_SBS_NODE_LAYOUT_DX               = 1 << 24u; //v_size^3 distance values (<bytes_per_value> bytes each)
static constexpr unsigned SDF_SBS_NODE_LAYOUT_DX_UV16          = 2 << 24u; //v_size^3 distance values (<bytes_per_value> bytes each), 8 tex coords (2*2 bytes each)
static constexpr unsigned SDF_SBS_NODE_LAYOUT_DX_RGB8          = 3 << 24u; //v_size^3 distance values (<bytes_per_value> bytes each), 8 RBG colors (4 bytes, with padding)
static constexpr unsigned SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F    = 4 << 24u; //v_size^3 indices to distance values (1 float each), 8 indices to RBG colors (3 float)
static constexpr unsigned SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F_IN = 5 << 24u; //v_size^3 indices to distance values (1 float each), 8 indices to RBG colors (3 float),
                                                                           //27 indices to adjacent bricks, INVALID_IDX indicates that the is no adjacent 
                                                                           //voxel on this side, otherwise it is an index of this brick in nodes array
static constexpr unsigned SDF_SBS_NODE_LAYOUT_MASK          = 0xFF000000;

// enum SdfOctreeNodeFlags
static constexpr unsigned OCTREE_FLAG_NODE_BORDER = 0; //the is a border in this node, proper intersection calculation required
static constexpr unsigned OCTREE_FLAG_NODE_EMPTY  = 1; //the node is empty, no need to intersect it
static constexpr unsigned OCTREE_FLAG_NODE_FULL   = 2; //the node is full, intersection is guaranteed to be on node's border
static constexpr unsigned OCTREE_FLAG_NODE_PARENT = 3;

struct SdfObject
{
  unsigned type;          // from enum SdfPrimitiveType
  unsigned params_offset; // in parameters vector
  unsigned params_count;
  unsigned neural_id; // index in neural_properties if type is neural

  float distance_mult;
  float distance_add;
  unsigned complement; // 0 or 1
  unsigned _pad; 

  float4 max_pos;    //float4 to prevent padding issues
  float4 min_pos;    //float4 to prevent padding issues

  float4x4 transform;
};
struct SdfConjunction
{
  float4 max_pos;
  float4 min_pos;
  unsigned offset; // in objects vector
  unsigned size;
  unsigned _pad[2];
};

struct NeuralDenseLayer
{
  unsigned offset;
  unsigned in_size;
  unsigned out_size;
};
struct NeuralProperties
{
  unsigned layer_count;
  NeuralDenseLayer layers[NEURAL_SDF_MAX_LAYERS];
};

struct SdfHit
{
  float4 hit_pos;  // hit_pos.w < 0 if no hit, hit_pos.w > 0 otherwise
  float4 hit_norm; // hit_norm.w can store different types of things for debug/visualization purposes
};

struct SdfFrameOctreeNode
{
  float values[8];
  unsigned offset; // offset for children (they are stored together). 0 offset means it's a leaf  
};

struct SdfCompactOctreeNode
{
  uint32_t offset; // offset for children (they are stored together). 0 offset means it's a leaf
  uint32_t flags;  // enum SdfOctreeNodeFlags
  uint32_t values[2]; //compressed distance values, 1 byte per value
};

//node for SparseVoxelSet, basically the same as SdfFrameOctree, but more compact
struct SdfSVSNode
{
  uint32_t pos_xy; //position of voxel in it's LOD
  uint32_t pos_z_lod_size; //size of it's LOD, (i.e. 2^LOD)
  uint32_t values[2]; //compressed distance values, 1 byte per value
};

//node for SparseBrickSet, similar idea to SparseVoxelSet, but values stored in bricks of KxKxK voxels,
//and values are shared between voxels, which leads to smaller memory footprint
struct SdfSBSNode
{
  uint32_t pos_xy; //position of start voxel of the block in it's LOD
  uint32_t pos_z_lod_size; //size of it's LOD, (i.e. 2^LOD)
  uint32_t data_offset; //offset in data vector for block with distance values, offset is in uint32_t, not bytes 
  uint32_t _pad;
};

//node for Adaptive SBS, all brick offsets are in units.
struct SdfSBSAdaptNode
{
  uint32_t pos_xy; // per-axis positions of the block's start voxel, in units. Limits: [0, 2^15). 2^15 means 1.0f (sdf scene's upper bound).
  uint32_t pos_z_vox_size; // vox_size -- voxel size in units ('tis always a cube)
  uint32_t data_offset;  //offset in data vector for block with distance values, offset is in uint32_t, not bytes
  uint32_t vox_count_xyz_pad; // _pad - 1 byte | voxel_numbers_xyz - 3 bytes
};

//headed contains some info about particular SBS, such as size of a brick, precision of stored values etc
//and also some precomputed values based on them to reduce the number of calculations during rendering
struct SdfSBSHeader
{
  uint32_t brick_size;      //number of voxels in each brick, 1 to 16
  uint32_t brick_pad;       //how many additional voxels are stored on the borders, 0 is default, 1 is required for tricubic filtration
  uint32_t bytes_per_value; //1, 2 or 4 bytes per value is allowed
  uint32_t aux_data;        //SdfSBSNodeLayout
};

struct SdfSBSAdaptHeader
{
  uint32_t brick_pad;       //how many additional voxels are stored on the borders, 0 is default, 1 is required for tricubic filtration
  uint32_t bytes_per_value; //1, 2 or 4 bytes per value is allowed
  uint32_t aux_data;        //SdfSBSNodeLayout
  uint32_t _pad;
};

struct SdfFrameOctreeTexNode
{
  float tex_coords[16];
  float values[8];
  unsigned offset; // offset for children (they are stored together). 0 offset means it's a leaf  
  unsigned material_id;
};

struct OTStackElement
{
  uint32_t nodeId;
  uint32_t info;
  uint2 p_size;
};

//enum COctreeNodePackMode
static constexpr unsigned COCTREE_NODE_PACK_MODE_DEFAULT        = 0; //32 bit per child info (32 bit child offset)
static constexpr unsigned COCTREE_NODE_PACK_MODE_SIM_COMP_FULL  = 1; //64 bit per child info (32 bit child offset, (8+24) bit transform code)
static constexpr unsigned COCTREE_NODE_PACK_MODE_SIM_COMP_SMALL = 2; //32 bit per child info (18 bit child offset, (6+8 ) bit transform code)

//enum COctreeLeafType
static constexpr unsigned COCTREE_LEAF_TYPE_NOT_A_LEAF          = 0;
static constexpr unsigned COCTREE_LEAF_TYPE_GRID                = 1; //all values are present, no bit fields, only range and quantized values
static constexpr unsigned COCTREE_LEAF_TYPE_BIT_PACK            = 2; //presence bit field for voxels/values, only for bricks with <= 64 values
static constexpr unsigned COCTREE_LEAF_TYPE_SLICES              = 3; //separate bit fields for each slice, better for big and/or padded bricks

static constexpr unsigned COCTREE_LEAF_TYPE_BITS                = 2;
static constexpr unsigned COCTREE_LEAF_TYPE_MASK                = 0x3;
static constexpr unsigned COCTREE_LOD_LEAF_TYPE_SHIFT           = 30;
static constexpr unsigned COCTREE_MAX_CHILD_INFO_SIZE           = 2; //size in uints
static constexpr unsigned COCTREE_USE_BEST_LEAF_TYPE            = 1000;

//Header is an ultimate descriptor of how COctreeV3 is stored in memory
//It has a lot of redundance and should not be filled manually
//Created in global_octree_to_COctreeV3 and used in render mostly
struct COctreeV3Header
{
  uint32_t brick_size;      //number of voxels in each brick, 1 to 16
  uint32_t brick_pad;       //how many additional voxels are stored on the borders, 0 is default, 1 is for tricubic filtration or normals smoothing
  uint32_t bits_per_value;  //6, 8, 10, 16, 32 bits per value is allowed
  uint32_t uv_size;         //0 if COctreeV3 is not textured, 1 for default (16 for u and v) and 2 for more precision (32 for u and v, not supported)
  uint32_t sim_compression; //0 or 1, indicates if similarity compression is used
  uint32_t lods;            //0 or 1
  
  uint32_t default_leaf_type; //enum COctreeLeafType
  uint32_t fallback_leaf_type;//enum COctreeLeafType
  
  uint32_t node_pack_mode;  
  // precomputed values for non-leaf nodes, fully determined by node_pack_mode
  uint32_t uints_per_child_info;
  uint32_t idx_mask;
  uint32_t idx_sh;
  uint32_t trans_off;
  uint32_t rot_mask;
  uint32_t add_mask;
};

struct OpenVDBHeader
{
  uint32_t offset;
};

//voxel position (i,j,k) to linear index
static unsigned SBS_v_to_i(float i, float j, float k, unsigned v_size, unsigned pad)
{
  return ((uint32_t)i+pad)*v_size*v_size + ((uint32_t)j+pad)*v_size + ((uint32_t)k+pad);
}

//################################################################################
// CPU-specific functions and data structures
//################################################################################
#ifndef KERNEL_SLICER

// Determines how the octree should be packed into COctreeV3.
// Doesn't deal with similarity compression (it is defined by scom::Settings)
// For other SDF types Settings and Header are the same thing, because they are simplier
struct COctreeV3Settings
{
  uint32_t brick_size = 1;      //number of voxels in each brick, 1 to 16
  uint32_t brick_pad = 0;       //how many additional voxels are stored on the borders, 0 is default, 1 is for tricubic filtration or normals smoothing
  uint32_t bits_per_value = 8;  //6, 8, 10, 16, 32 bits per value is allowed
  uint32_t uv_size = 0;         //0 if COctreeV3 is not textured, 1 for default (16 for u and v) and 2 for more precision (32 for u and v, not supported)
  uint32_t sim_compression = 0; //0 or 1, indicates if similarity compression is used
  uint32_t default_leaf_type = COCTREE_USE_BEST_LEAF_TYPE; //enum COctreeLeafType or COCTREE_USE_BEST_TYPE
  // If true, bricks can be saved as dense grid (COCTREE_LEAF_TYPE_GRID), if default_leaf_type
  // will result in larger memory footprint. It causes branching in shader and may slow it down.
  bool allow_fallback_to_unpacked_leaves = true;
  bool use_lods = false;
};

//based on node_pack_mode and leaf_pack_mode, fill masks and other values dependant on them
static void fill_coctree_v3_header(COctreeV3Header &header)
{
  if (header.node_pack_mode == COCTREE_NODE_PACK_MODE_DEFAULT)
  {
    header.uints_per_child_info = 1;
    header.idx_sh = 0;
    header.trans_off = 0;
    header.idx_mask = 0xFFFFFFFFu;
    header.rot_mask = 0x00000000u;
    header.add_mask = 0x00000000u;
  }
  else if (header.node_pack_mode == COCTREE_NODE_PACK_MODE_SIM_COMP_FULL)
  {
    header.uints_per_child_info = 2;
    header.idx_sh = 0;
    header.trans_off = 1;
    header.idx_mask = 0xFFFFFFFFu;
    header.rot_mask = 0x0000003Fu;
    header.add_mask = 0x3FFFFF00u;
  }
  else if (header.node_pack_mode == COCTREE_NODE_PACK_MODE_SIM_COMP_SMALL)
  {
    header.uints_per_child_info = 1;
    header.idx_sh = 14;
    header.trans_off = 0;
    header.idx_mask = 0xFFFFC000u; //14-32 bits
    header.rot_mask = 0x0000003Fu; // 0- 6 bits
    header.add_mask = 0x00003FC0u; // 6-14 bits
  }

  //in OctreeIntersectV3 function, childInfo contains leaf type, rotation and add codes simultaneously
  assert((((header.rot_mask | header.add_mask) << COCTREE_LEAF_TYPE_BITS) >> COCTREE_LEAF_TYPE_BITS) == (header.rot_mask | header.add_mask));
  //even if child mask is shifted to the left, we should be able to detect non-leaf nodes with simple == 0 check
  assert(COCTREE_LEAF_TYPE_NOT_A_LEAF == 0);
}

static COctreeV3Header get_default_coctree_v3_header()
{
  COctreeV3Header header;

  header.brick_size = 1;
  header.brick_pad = 0;
  header.bits_per_value = 8;
  header.uv_size = 0;
  header.sim_compression = 0;

  header.node_pack_mode = COCTREE_NODE_PACK_MODE_DEFAULT;
  
  header.default_leaf_type  = COCTREE_LEAF_TYPE_SLICES;
  header.fallback_leaf_type = COCTREE_LEAF_TYPE_GRID;

  header.lods = 0;

  return header;
}

struct SdfGrid
{
  uint3 size;
  std::vector<float> data; //size.x*size.y*size.z values 
};

struct SdfSBS
{
  SdfSBSHeader header;
  std::vector<SdfSBSNode> nodes;
  std::vector<uint32_t> values;
  std::vector<float> values_f; //used with indexed SBS layouts, as it is easier to have float values as a separate vector
};

struct SdfSBSAdapt
{
  SdfSBSAdaptHeader header;
  std::vector<SdfSBSAdaptNode> nodes;
  std::vector<uint32_t> values;
  std::vector<float> values_f; //used with indexed SBS layouts, as it is easier to have float values as a separate vector
};

// structure to actually store SdfScene data
struct SdfScene
{
  std::vector<float> parameters;
  std::vector<SdfObject> objects;
  std::vector<SdfConjunction> conjunctions;
  std::vector<NeuralProperties> neural_properties;
};

struct SdfGridView
{
  SdfGridView() = default;
  SdfGridView(const SdfGrid &grid)
  {
    size = grid.size;
    data = grid.data.data();
  }
  SdfGridView(uint3 a_size, const std::vector<float> &a_data)
  {
    size = a_size;
    data = a_data.data();
  }
  uint3 size;
  const float *data; //size.x*size.y*size.z values 
};

struct SdfFrameOctreeView
{
  SdfFrameOctreeView() = default;
  SdfFrameOctreeView(const std::vector<SdfFrameOctreeNode> &a_nodes)
  {
    size = a_nodes.size();
    nodes = a_nodes.data();
  }
  unsigned size;
  const SdfFrameOctreeNode *nodes;
};

struct SdfSVSView
{
  SdfSVSView() = default;
  SdfSVSView(const std::vector<SdfSVSNode> &a_nodes)
  {
    size = a_nodes.size();
    nodes = a_nodes.data();
  }
  unsigned size;
  const SdfSVSNode *nodes;
};

struct SdfSBSView
{
  SdfSBSView() = default;
  SdfSBSView(const SdfSBS &sbs)
  {
    header = sbs.header;
    size = sbs.nodes.size();
    nodes = sbs.nodes.data();
    values_count = sbs.values.size();
    values = sbs.values.data();
    values_f_count = sbs.values_f.size();
    values_f = sbs.values_f.data();
  }
  SdfSBSView(SdfSBSHeader a_header, const std::vector<SdfSBSNode> &a_nodes, 
             const std::vector<uint32_t> &a_values)
  {
    header = a_header;
    size = a_nodes.size();
    nodes = a_nodes.data();
    values_count = a_values.size();
    values = a_values.data();
    values_f_count = 0;
    values_f = nullptr;
  }

  SdfSBSView(SdfSBSHeader a_header, const std::vector<SdfSBSNode> &a_nodes,
             const std::vector<uint32_t> &a_values, const std::vector<float> &a_values_f)
  {
    header = a_header;
    size = a_nodes.size();
    nodes = a_nodes.data();
    values_count = a_values.size();
    values = a_values.data();
    values_f_count = a_values_f.size();
    values_f = a_values_f.data();
  }

  SdfSBSHeader header;
  unsigned size;
  const SdfSBSNode *nodes;
  unsigned values_count;
  const uint32_t *values;
  unsigned values_f_count;
  const float *values_f;
};

struct SdfSBSAdaptView
{
  SdfSBSAdaptView() = default;
  SdfSBSAdaptView(const SdfSBSAdapt &sbs_adapt)
  {
    header = sbs_adapt.header;
    size = sbs_adapt.nodes.size();
    nodes = sbs_adapt.nodes.data();
    values_count = sbs_adapt.values.size();
    values = sbs_adapt.values.data();
    values_f_count = sbs_adapt.values_f.size();
    values_f = sbs_adapt.values_f.data();
  }
  SdfSBSAdaptView(SdfSBSAdaptHeader a_header, const std::vector<SdfSBSAdaptNode> &a_nodes, 
             const std::vector<uint32_t> &a_values)
  {
    header = a_header;
    size = a_nodes.size();
    nodes = a_nodes.data();
    values_count = a_values.size();
    values = a_values.data();
    values_f_count = 0;
    values_f = nullptr;
  }

  SdfSBSAdaptView(SdfSBSAdaptHeader a_header, const std::vector<SdfSBSAdaptNode> &a_nodes,
             const std::vector<uint32_t> &a_values, const std::vector<float> &a_values_f)
  {
    header = a_header;
    size = a_nodes.size();
    nodes = a_nodes.data();
    values_count = a_values.size();
    values = a_values.data();
    values_f_count = a_values_f.size();
    values_f = a_values_f.data();
  }

  SdfSBSAdaptHeader header;
  unsigned size;
  const SdfSBSAdaptNode *nodes;
  unsigned values_count;
  const uint32_t *values;
  unsigned values_f_count;
  const float *values_f;
};

// structure to access and transfer SdfScene data
// all interfaces use SdfSceneView to be independant of how exactly SDF scenes are stored
struct SdfSceneView
{
  SdfSceneView() = default;
  SdfSceneView(const SdfScene &scene)
  {
    parameters = scene.parameters.data();
    objects = scene.objects.data();
    conjunctions = scene.conjunctions.data();
    neural_properties = scene.neural_properties.data();

    parameters_count = scene.parameters.size();
    objects_count = scene.objects.size();
    conjunctions_count = scene.conjunctions.size();
    neural_properties_count = scene.neural_properties.size();
  }
  SdfSceneView(const std::vector<float> &_parameters,
               const std::vector<SdfObject> &_objects,
               const std::vector<SdfConjunction> &_conjunctions,
               const std::vector<NeuralProperties> &_neural_properties)
  {
    parameters = _parameters.data();
    objects = _objects.data();
    conjunctions = _conjunctions.data();
    neural_properties = _neural_properties.data();

    parameters_count = _parameters.size();
    objects_count = _objects.size();
    conjunctions_count = _conjunctions.size();
    neural_properties_count = _neural_properties.size();
  }

  const float *parameters;
  const SdfObject *objects;
  const SdfConjunction *conjunctions;
  const NeuralProperties *neural_properties;

  unsigned parameters_count;
  unsigned objects_count;
  unsigned conjunctions_count;
  unsigned neural_properties_count;
};

struct SdfFrameOctreeTexView
{
  SdfFrameOctreeTexView() = default;
  SdfFrameOctreeTexView(const std::vector<SdfFrameOctreeTexNode> &a_nodes)
  {
    size = a_nodes.size();
    nodes = a_nodes.data();
  }
  unsigned size;
  const SdfFrameOctreeTexNode *nodes;
};

struct COctreeV1View
{
  COctreeV1View() = default;
  COctreeV1View(const std::vector<SdfCompactOctreeNode> &a_nodes)
  {
    size = a_nodes.size();
    nodes = a_nodes.data();
  }
  unsigned size;
  const SdfCompactOctreeNode *nodes;
};

struct COctreeV2
{
  std::vector<uint32_t> data;
};

struct COctreeV2View
{
  COctreeV2View() = default;
  COctreeV2View(const COctreeV2 &a_octree)
  {
    data = a_octree.data.data();
    size = a_octree.data.size();
  }
  explicit COctreeV2View(const std::vector<uint32_t> &a_data)
  {
    data = a_data.data();
    size = a_data.size();
  }
  const uint32_t *data = nullptr;
  uint32_t size = 0;
};

struct COctreeV3
{
  static constexpr unsigned VERSION = 5; // change version if structure changes
  COctreeV3Header header = get_default_coctree_v3_header();
  std::vector<uint32_t> data;
};

struct COctreeV3View
{
  COctreeV3View() = default;
  COctreeV3View(const COctreeV3 &a_octree)
  {
    header = a_octree.header;
    data = a_octree.data.data();
    size = a_octree.data.size();
  }
  COctreeV3View(COctreeV3Header a_header, const std::vector<uint32_t> &a_data)
  {
    header = a_header;
    data = a_data.data();
    size = a_data.size();
  }
  COctreeV3Header header;
  const uint32_t *data = nullptr;
  uint32_t size = 0;
};

// interface to evaluate SdfGrid out of context of rendering
class ISdfGridFunction
{
public:
  virtual void init(SdfGridView octree) = 0; 
  virtual float eval_distance(float3 pos) = 0;
};
std::shared_ptr<ISdfGridFunction> get_SdfGridFunction(SdfGridView scene);

struct ModelInfo
{
  std::string name;
  unsigned bytesize;
  unsigned num_primitives;
};

enum class DemoScene
{
  SINGLE_OBJECT,
  CORNELL_BOX,
  SINGLE_OBJECT_CUBEMAP
};

namespace cmesh4
{
  struct SimpleMesh;
};

// for every type of SDF representation, we need three functions:
// save, load, info
// the last one is requires to generate xml file with model of this type
void save_sdf_scene(const SdfScene &scene, const std::string &path);
void load_sdf_scene(SdfScene &scene, const std::string &path);
ModelInfo get_info_sdf_scene(const SdfScene &scene);

void save_sdf_grid(const SdfGridView &scene, const std::string &path);
void load_sdf_grid(SdfGrid &scene, const std::string &path);
ModelInfo get_info_sdf_grid(const SdfGridView &scene);

void save_sdf_frame_octree(const SdfFrameOctreeView &scene, const std::string &path);
void load_sdf_frame_octree(std::vector<SdfFrameOctreeNode> &scene, const std::string &path);
ModelInfo get_info_sdf_frame_octree(const SdfFrameOctreeView &scene);

void save_sdf_SVS(const SdfSVSView &scene, const std::string &path);
void load_sdf_SVS(std::vector<SdfSVSNode> &scene, const std::string &path);
ModelInfo get_info_sdf_SVS(const SdfSVSView &scene);

void save_sdf_SBS(const SdfSBSView &scene, const std::string &path);
void load_sdf_SBS(SdfSBS &scene, const std::string &path);
ModelInfo get_info_sdf_SBS(const SdfSBSView &scene);

void save_sdf_frame_octree_tex(const SdfFrameOctreeTexView &scene, const std::string &path);
void load_sdf_frame_octree_tex(std::vector<SdfFrameOctreeTexNode> &scene, const std::string &path);
ModelInfo get_info_sdf_frame_octree_tex(const SdfFrameOctreeTexView &scene);

void save_coctree_v3(const COctreeV3View &scene, const std::string &path);
void load_coctree_v3(COctreeV3 &scene, const std::string &path);
ModelInfo get_info_coctree_v3(const COctreeV3View &scene);

void save_scene_xml(std::string path, std::string bin_file_name, ModelInfo info, int mat_id,
                    DemoScene scene = DemoScene::CORNELL_BOX); //for all models except mesh
void save_scene_xml(std::string path, std::string bin_file_name, const cmesh4::SimpleMesh &mesh,
                    DemoScene scene = DemoScene::CORNELL_BOX);

void load_neural_sdf_scene_SIREN(SdfScene &scene, const std::string &path); // loads scene from raw SIREN weights file

SdfSBSAdaptView convert_sbs_to_adapt(SdfSBSAdapt &adapt_scene, const SdfSBSView &scene);
SdfSBSAdaptView convert_sbs_to_adapt_with_split(SdfSBSAdapt &adapt_scene, const SdfSBSView &scene);
#endif