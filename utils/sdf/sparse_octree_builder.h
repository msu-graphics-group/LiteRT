#pragma once
#include <vector>
#include <functional>
#include <fstream>
#include "sdf_scene.h"
#include "utils/mesh/mesh.h"
#include "sdf_converter.h"
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
  void sdf_to_global_octree(SparseOctreeSettings settings, MultithreadedDistanceFunction sdf, 
                            unsigned max_threads, GlobalOctree &octree);

  void mesh_octree_to_global_octree(const cmesh4::SimpleMesh &mesh, const cmesh4::TriangleListOctree &tl_octree, 
                                    GlobalOctree &out_octree, float precision, unsigned min_layer, 
                                    unsigned max_layer, bool fill_all_nodes);

  void global_octree_to_frame_octree(const GlobalOctree &octree, std::vector<SdfFrameOctreeNode> &out_frame); 
  void global_octree_to_frame_octree_tex(const GlobalOctree &octree, std::vector<SdfFrameOctreeTexNode> &out_frame);
  void global_octree_to_SVS(const GlobalOctree &octree, std::vector<SdfSVSNode> &svs);
  void global_octree_to_SBS(const GlobalOctree &octree, SdfSBS &sbs);
  void global_octree_to_COctreeV2(const GlobalOctree &octree, COctreeV2 &compact);
  void global_octree_to_COctreeV3(const GlobalOctree &octree, COctreeV3 &compact_octree, COctreeV3Settings co_settings);
  void global_octree_to_COctreeV3(const GlobalOctree &octree, COctreeV3 &compact_octree, 
                                  COctreeV3Settings co_settings, scom::Settings settings);

  void frame_octree_limit_nodes(std::vector<SdfFrameOctreeNode> &frame, unsigned nodes_limit,
                                bool count_only_border_nodes);
}