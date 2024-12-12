#pragma once
#include <vector>
#include <functional>
#include "sdf_scene.h"
#include "utils/mesh/mesh.h"
#include "Renderer/eye_ray.h"

namespace scom
{
  struct Settings;
}

struct GridSettings
{
  GridSettings() {};
  GridSettings(unsigned _size) { size = _size; }
  unsigned size = 16;
};

struct SparseOctreeSettings
{
  SparseOctreeSettings() = default;
  explicit SparseOctreeSettings(unsigned _depth, bool _fill_all_nodes = false)
  {
    depth = _depth;
    fill_all_nodes = _fill_all_nodes;
  }
  SparseOctreeSettings(unsigned _depth, float split_thr, unsigned min_depth = 1, bool _fill_all_nodes = false)
  {
    depth = _depth;
    split_thr = split_thr;
    min_depth = min_depth;
    fill_all_nodes = _fill_all_nodes;
  }

  unsigned depth = 1;     //max depth of octree
  unsigned min_depth = 1; //if node depth is less, it will be split no matter if it reduces error or not
  float split_thr = 0.0f; //if and only if splitting the node will decrease error by this value, it will be done
  bool  fill_all_nodes = false; //if false, only leaf nodes will be filled with actual distances
};

/*
Converter is a static and unified interface for creating different 
spatial structures to represent Signed Distance Filed.

Every structure can be created from mesh or abstract distance function (std::function)
version with multithreaded function is also provided, but not every building method
will actually use multiple threads.

Converter builds structures to represent SDF in fixed region - [-1,1]^3
Thus mesh should be scaled to unit cube [-1,1]^3 (can be done with cmesh4::normalize_mesh)
Unnormalized mesh will probably be non-watertight in [-1,1]^3 and produce bad result.
*/
namespace sdf_converter
{
  using MultithreadedDistanceFunction = std::function<float(const float3 &, unsigned idx)>;

  enum class GlobalOctreeNodeType
  {
    EMPTY,      //leaf with no surface or node with an empty subtree (the latter is usually removed during build)
    LEAF,       //leaf node containing surface (i.e. both positive and negative values)
    EMPTY_NODE, //node with 8 children, but it's own values doesn't represent surface (all values have the same sign)
    NODE        //node with 8 children, containing surface (i.e. both positive and negative values)
  };
  
  struct GlobalOctreeHeader
  {
    uint32_t brick_size;      //number of voxels in each brick, (min 1)
    uint32_t brick_pad;       //how many additional voxels are stored on the borders, 0 is default
  };

  struct GlobalOctreeNode
  {
    GlobalOctreeNodeType type;
    float2 tex_coords[8]; //texture coordinates on corners
    unsigned val_off;     //offset in values_f vectors
    unsigned offset;      //offset in nodes vector for next child (0 if its leaf)
    unsigned material_id; //material
  };

  struct GlobalOctree
  {
    GlobalOctreeHeader header;
    std::vector<GlobalOctreeNode> nodes;
    std::vector<float> values_f;
  };

  SdfGrid create_sdf_grid(GridSettings settings, MultithreadedDistanceFunction sdf, unsigned max_threads);
  SdfGrid create_sdf_grid(GridSettings settings, const cmesh4::SimpleMesh &mesh);

  std::vector<SdfFrameOctreeNode> create_sdf_frame_octree(SparseOctreeSettings settings, MultithreadedDistanceFunction sdf, unsigned max_threads);
  std::vector<SdfFrameOctreeNode> create_sdf_frame_octree(SparseOctreeSettings settings, const cmesh4::SimpleMesh &mesh);

  std::vector<SdfSVSNode> create_sdf_SVS(SparseOctreeSettings settings, MultithreadedDistanceFunction sdf, unsigned max_threads);
  std::vector<SdfSVSNode> create_sdf_SVS(SparseOctreeSettings settings, const cmesh4::SimpleMesh &mesh);  

  SdfSBS create_sdf_SBS(SparseOctreeSettings settings, SdfSBSHeader header, MultithreadedDistanceFunction sdf, unsigned max_threads);
  SdfSBS create_sdf_SBS(SparseOctreeSettings settings, SdfSBSHeader header, const cmesh4::SimpleMesh &mesh);

  COctreeV2 create_COctree_v2(SparseOctreeSettings settings, const cmesh4::SimpleMesh &mesh);
  COctreeV3 create_COctree_v3(SparseOctreeSettings settings, COctreeV3Settings header, const cmesh4::SimpleMesh &mesh);
  COctreeV3 create_COctree_v3(SparseOctreeSettings settings, COctreeV3Settings header, scom::Settings scom_settings, const cmesh4::SimpleMesh &mesh);

  std::vector<SdfFrameOctreeTexNode> create_sdf_frame_octree_tex(SparseOctreeSettings settings, const cmesh4::SimpleMesh &mesh);
  SdfSBS create_sdf_SBS_tex(SparseOctreeSettings settings, SdfSBSHeader header, const cmesh4::SimpleMesh &mesh, bool noisy = false);

  //-------------------------------------------------------------------------------------------------
  //builders for specific SDF representations
  SdfSBS create_sdf_SBS_col(SparseOctreeSettings settings, SdfSBSHeader header, const cmesh4::SimpleMesh &mesh, unsigned mat_id,
                            const std::vector<MultiRendererMaterial> &materials_lib, 
                            const std::vector<std::shared_ptr<ICombinedImageSampler>> &textures_lib, bool noisy = false);

  //creates SBS with layout SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F
  SdfSBS create_sdf_SBS_indexed(SparseOctreeSettings settings, SdfSBSHeader header, const cmesh4::SimpleMesh &mesh, unsigned mat_id,
                                const std::vector<MultiRendererMaterial> &materials_lib, 
                                const std::vector<std::shared_ptr<ICombinedImageSampler>> &textures_lib, bool noisy = false);

  //creates SBS with layout SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F_IN
  SdfSBS create_sdf_SBS_indexed_with_neighbors(SparseOctreeSettings settings, SdfSBSHeader header, const cmesh4::SimpleMesh &mesh, unsigned mat_id,
                                               const std::vector<MultiRendererMaterial> &materials_lib, 
                                               const std::vector<std::shared_ptr<ICombinedImageSampler>> &textures_lib);

  
  //-------------------------------------------------------------------------------------------------
  //experimental and weird builders
  SdfSBS SBS_ind_to_SBS_ind_with_neighbors(const SdfSBS &sbs);
  std::vector<SdfFrameOctreeNode> create_sdf_frame_octree(SparseOctreeSettings settings, MultithreadedDistanceFunction sdf, float eps, bool is_smooth, bool fix_artefacts);
  std::vector<SdfFrameOctreeNode> create_psdf_frame_octree(SparseOctreeSettings settings, const cmesh4::SimpleMesh &mesh);
  std::vector<SdfFrameOctreeNode> create_vmpdf_frame_octree(SparseOctreeSettings settings, const cmesh4::SimpleMesh &mesh);
  SdfSBSAdapt greed_sbs_adapt(MultithreadedDistanceFunction sdf, uint8_t depth);
}