#pragma once
#include <vector>
#include <functional>
#include <fstream>
#include "../sdfScene/sdf_scene.h"
#include "../utils/mesh.h"
#include "../utils/sdf_converter.h"
/*
A class that is able to represent and arbitrary function f : R^3 -> T 
as a sparse octree, where every leaf contains value of function in it's
center. 
T - is a type that can be meaningfully interpolated (i.e. float, double, float2, float3)
It should have T+T and float*T operator and be POD of course. 
Octree always represents unit cube [-1,1]^3
*/

namespace sdf_converter
{
  SdfSBS frame_octree_to_SBS(MultithreadedDistanceFunction sdf, 
                             unsigned max_threads,
                             const std::vector<SdfFrameOctreeNode> &nodes,
                             const SdfSBSHeader &header);

  std::vector<SdfFrameOctreeNode> construct_sdf_frame_octree(SparseOctreeSettings settings, MultithreadedDistanceFunction sdf, float eps, 
                                                             unsigned max_threads, bool is_smooth, bool fix_artefacts);

  void mesh_octree_to_sdf_frame_octree(const cmesh4::SimpleMesh &mesh,
                                       const cmesh4::TriangleListOctree &tl_octree, 
                                       std::vector<SdfFrameOctreeNode> &out_frame);

  void mesh_octree_to_sdf_frame_octree_tex(const cmesh4::SimpleMesh &mesh,
                                           const cmesh4::TriangleListOctree &tl_octree, 
                                           std::vector<SdfFrameOctreeTexNode> &out_frame);

  void frame_octree_limit_nodes(std::vector<SdfFrameOctreeNode> &frame, unsigned nodes_limit,
                                bool count_only_border_nodes);
  void frame_octree_to_SVS_rec(const std::vector<SdfFrameOctreeNode> &frame,
                               std::vector<SdfSVSNode> &nodes,
                               unsigned idx, uint3 p, unsigned lod_size);

  SdfSBS frame_octree_to_SBS_tex(MultithreadedDistanceFunction sdf, 
                                 unsigned max_threads,
                                 const std::vector<SdfFrameOctreeTexNode> &nodes,
                                 const SdfSBSHeader &header);

  SdfSBS SBS_col_to_SBS_ind(const SdfSBS &sbs);

  SdfSBS SBS_ind_to_SBS_ind_with_neighbors(const SdfSBS &sbs);

  std::vector<SdfFrameOctreeNode> construct_sdf_frame_octree(SparseOctreeSettings settings, MultithreadedDistanceFunction sdf, 
                                                             unsigned max_threads);

  std::vector<SdfCompactOctreeNode> frame_octree_to_compact_octree(const std::vector<SdfFrameOctreeNode> &frame);
  std::vector<uint32_t> frame_octree_to_compact_octree_v2(const std::vector<SdfFrameOctreeNode> &frame);
}