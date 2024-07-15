#pragma once
#include <vector>
#include <functional>
#include "../sdfScene/sdf_scene.h"
#include "../utils/mesh.h"
#include "hp_octree.h"

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

  std::vector<SdfOctreeNode> create_sdf_octree(SparseOctreeSettings settings, DistanceFunction sdf);
  std::vector<SdfOctreeNode> create_sdf_octree(SparseOctreeSettings settings, MultithreadedDistanceFunction sdf, unsigned max_threads);
  std::vector<SdfOctreeNode> create_sdf_octree(SparseOctreeSettings settings, const cmesh4::SimpleMesh &mesh);

  std::vector<SdfFrameOctreeNode> create_sdf_frame_octree(SparseOctreeSettings settings, DistanceFunction sdf);
  std::vector<SdfFrameOctreeNode> create_sdf_frame_octree(SparseOctreeSettings settings, MultithreadedDistanceFunction sdf, unsigned max_threads);
  std::vector<SdfFrameOctreeNode> create_sdf_frame_octree(SparseOctreeSettings settings, const cmesh4::SimpleMesh &mesh);

  std::vector<SdfSVSNode> create_sdf_SVS(SparseOctreeSettings settings, DistanceFunction sdf);
  std::vector<SdfSVSNode> create_sdf_SVS(SparseOctreeSettings settings, MultithreadedDistanceFunction sdf, unsigned max_threads);
  std::vector<SdfSVSNode> create_sdf_SVS(SparseOctreeSettings settings, const cmesh4::SimpleMesh &mesh);  

  SdfSBS create_sdf_SBS(SparseOctreeSettings settings, SdfSBSHeader header, DistanceFunction sdf);
  SdfSBS create_sdf_SBS(SparseOctreeSettings settings, SdfSBSHeader header, MultithreadedDistanceFunction sdf, unsigned max_threads);
  SdfSBS create_sdf_SBS(SparseOctreeSettings settings, SdfSBSHeader header, const cmesh4::SimpleMesh &mesh);

  SdfHPOctree create_sdf_hp_octree(HPOctreeBuilder::BuildSettings settings, DistanceFunction sdf);
  SdfHPOctree create_sdf_hp_octree(HPOctreeBuilder::BuildSettings settings, MultithreadedDistanceFunction sdf, unsigned max_threads);
  SdfHPOctree create_sdf_hp_octree(HPOctreeBuilder::BuildSettings settings, const cmesh4::SimpleMesh &mesh);

  std::vector<SdfFrameOctreeTexNode> create_sdf_frame_octree_tex(SparseOctreeSettings settings, const cmesh4::SimpleMesh &mesh);
  
  SdfSBS create_sdf_SBS_tex(SparseOctreeSettings settings, SdfSBSHeader header, const cmesh4::SimpleMesh &mesh);
}