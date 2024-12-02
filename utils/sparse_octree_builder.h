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

namespace scom
{
  struct Settings;
};

namespace sdf_converter
{
  std::vector<SdfFrameOctreeNode> construct_sdf_frame_octree(SparseOctreeSettings settings, MultithreadedDistanceFunction sdf, float eps, 
                                                             unsigned max_threads, bool is_smooth, bool fix_artefacts);

  void mesh_octree_to_vmpdf(const cmesh4::SimpleMesh &mesh,
                            const cmesh4::TriangleListOctree &tl_octree, 
                            std::vector<SdfFrameOctreeNode> &out_frame);

  void sdf_to_global_octree(SparseOctreeSettings settings, MultithreadedDistanceFunction sdf, 
                            unsigned max_threads, GlobalOctree &octree);

  void mesh_octree_to_global_octree(const cmesh4::SimpleMesh &mesh,
                                    const cmesh4::TriangleListOctree &tl_octree, 
                                    GlobalOctree &out_octree);

  void global_octree_to_frame_octree(const GlobalOctree &octree, std::vector<SdfFrameOctreeNode> &out_frame); 
  void global_octree_to_frame_octree_tex(const GlobalOctree &octree, std::vector<SdfFrameOctreeTexNode> &out_frame);
  void global_octree_to_SVS(const GlobalOctree &octree, std::vector<SdfSVSNode> &svs);
  void global_octree_to_SBS(const GlobalOctree &octree, SdfSBS &sbs);
  void global_octree_to_compact_octree_v2(const GlobalOctree &octree, COctreeV2 &compact);
  void global_octree_to_compact_octree_v3(const GlobalOctree &octree, COctreeV3 &compact_octree, unsigned max_threads);
  void global_octree_to_compact_octree_v3(const GlobalOctree &octree, COctreeV3 &compact_octree, unsigned max_threads, const scom::Settings &settings);

  void frame_octree_limit_nodes(std::vector<SdfFrameOctreeNode> &frame, unsigned nodes_limit,
                                bool count_only_border_nodes);
  void frame_octree_to_SVS_rec(const std::vector<SdfFrameOctreeNode> &frame,
                               std::vector<SdfSVSNode> &nodes,
                               unsigned idx, uint3 p, unsigned lod_size);

  std::vector<uint32_t> frame_octree_to_compact_octree_v2(const std::vector<SdfFrameOctreeNode> &frame);
}