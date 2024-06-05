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
  std::vector<SdfOctreeNode> construct_sdf_octree(SparseOctreeSettings settings, MultithreadedDistanceFunction sdf, unsigned max_threads);
}

class SparseOctreeBuilder2
{
public:
  static constexpr unsigned MAX_OCTREE_DEPTH = 16;

  void construct(sdf_converter::MultithreadedDistanceFunction sdf, SparseOctreeSettings settings);
private:
  sdf_converter::MultithreadedDistanceFunction sdf;
  SparseOctreeSettings settings;
  unsigned max_threads = 16;
  unsigned min_remove_level = 4;
};