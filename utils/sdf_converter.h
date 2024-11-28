#pragma once
#include <vector>
#include <functional>
#include "../sdfScene/sdf_scene.h"
#include "../utils/mesh.h"
#include "../Renderer/eye_ray.h"

struct GridSettings
{
  GridSettings() {};
  GridSettings(unsigned _size) { size = _size; }
  unsigned size = 16;
};

enum class SparseOctreeBuildType
{
  DEFAULT = 0, //build from abstrace distance function, quite slow, but reliable
  MESH_TLO = 1 //works only if building from mesh, faster for detailed octrees and medium-sized meshes
};

struct SparseOctreeSettings
{
  SparseOctreeSettings() = default;
  SparseOctreeSettings(SparseOctreeBuildType type, unsigned _depth, unsigned _nodes_limit = 1 << 24)
  {
    build_type = type;
    depth = _depth;
    nodes_limit = _nodes_limit;
  }
  unsigned depth = 1;
  float remove_thr = 0.0001; //used only with SparseOctreeBuildType::DEFAULT
  unsigned nodes_limit = 1 << 24;
  SparseOctreeBuildType build_type = SparseOctreeBuildType::DEFAULT;
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
  using DistanceFunction = std::function<float(const float3 &)>;
  using MultithreadedDistanceFunction = std::function<float(const float3 &, unsigned idx)>;

  SdfGrid create_sdf_grid(GridSettings settings, DistanceFunction sdf);
  SdfGrid create_sdf_grid(GridSettings settings, MultithreadedDistanceFunction sdf, unsigned max_threads);
  SdfGrid create_sdf_grid(GridSettings settings, const cmesh4::SimpleMesh &mesh);

  std::vector<SdfFrameOctreeNode> create_sdf_frame_octree(SparseOctreeSettings settings, DistanceFunction sdf);
  std::vector<SdfFrameOctreeNode> create_sdf_frame_octree(SparseOctreeSettings settings, MultithreadedDistanceFunction sdf, unsigned max_threads);
  std::vector<SdfFrameOctreeNode> create_sdf_frame_octree(SparseOctreeSettings settings, const cmesh4::SimpleMesh &mesh);
  std::vector<SdfFrameOctreeNode> create_sdf_frame_octree(SparseOctreeSettings settings, MultithreadedDistanceFunction sdf, float eps, bool is_smooth, bool fix_artefacts);

  std::vector<SdfFrameOctreeNode> create_psdf_frame_octree(SparseOctreeSettings settings, const cmesh4::SimpleMesh &mesh);
  std::vector<SdfFrameOctreeNode> create_vmpdf_frame_octree(SparseOctreeSettings settings, const cmesh4::SimpleMesh &mesh);

  std::vector<SdfSVSNode> create_sdf_SVS(SparseOctreeSettings settings, DistanceFunction sdf);
  std::vector<SdfSVSNode> create_sdf_SVS(SparseOctreeSettings settings, MultithreadedDistanceFunction sdf, unsigned max_threads);
  std::vector<SdfSVSNode> create_sdf_SVS(SparseOctreeSettings settings, const cmesh4::SimpleMesh &mesh);  

  SdfSBS create_sdf_SBS(SparseOctreeSettings settings, SdfSBSHeader header, DistanceFunction sdf);
  SdfSBS create_sdf_SBS(SparseOctreeSettings settings, SdfSBSHeader header, MultithreadedDistanceFunction sdf, unsigned max_threads);
  SdfSBS create_sdf_SBS(SparseOctreeSettings settings, SdfSBSHeader header, const cmesh4::SimpleMesh &mesh);

  std::vector<SdfFrameOctreeTexNode> create_sdf_frame_octree_tex(SparseOctreeSettings settings, const cmesh4::SimpleMesh &mesh);
  
  SdfSBS create_sdf_SBS_tex(SparseOctreeSettings settings, SdfSBSHeader header, const cmesh4::SimpleMesh &mesh, bool noisy = false);

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

  SdfSBSAdapt greed_sbs_adapt(MultithreadedDistanceFunction sdf, uint8_t depth);

  std::vector<uint32_t> create_COctree_v3(SparseOctreeSettings settings, COctreeV3Header header, const cmesh4::SimpleMesh &mesh);

  //-------------------------------------------------------------------------------------------------

  struct GlobalOctreeHeader
  {
    uint32_t brick_size;      //number of voxels in each brick, (min 1)
    uint32_t brick_pad;       //how many additional voxels are stored on the borders, 0 is default
  };

  struct GlobalOctreeNode
  {
    float2 tex_coords[8]; //texture coordinates on corners
    unsigned val_off;     //offset in values_f vectors
    unsigned offset;      //offset in nodes vector for next child (0 if its leaf)
    bool is_not_void;     //is there any usefull data inside
    unsigned material_id; //material
  };

  struct GlobalOctree
  {
    GlobalOctreeHeader header;
    std::vector<GlobalOctreeNode> nodes;
    std::vector<float> values_f;
  };
}