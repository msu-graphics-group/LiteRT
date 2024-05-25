#include "hp_octree.h"

void HPOctreeBuilder::construct(std::function<float(const float3 &, unsigned thread_idx)> sdf, BuildSettings settings)
{
  printf("HP Octree module is disabled. Cmake with -DMODULE_HP_OCTREE=ON\n");
}

void HPOctreeBuilder::construct(const cmesh4::SimpleMesh &mesh, BuildSettings settings)
{
  printf("HP Octree module is disabled. Cmake with -DMODULE_HP_OCTREE=ON\n");
}

  