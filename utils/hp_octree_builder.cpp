#include "hp_octree_precomputed_tables.h"
#include "hp_octree.h"
#include "../dependencies/hp_octree/Include/HP/Octree.h"

void HPOctreeBuilder::construct(std::function<float(const float3 &)> f)
{
  auto SphereFunc = [&f](const Eigen::Vector3d &pt_, const unsigned threadIdx_) -> double
  {
    return f(float3(pt_.x(), pt_.y(), pt_.z()));
  };

  SDF::Config hpConfig;
  hpConfig.targetErrorThreshold = pow(10, -8);
  hpConfig.nearnessWeighting.type = SDF::Config::NearnessWeighting::Type::Polynomial;
  hpConfig.nearnessWeighting.strength = 3.0;
  hpConfig.continuity.enforce = true;
  hpConfig.continuity.strength = 8.0;
  hpConfig.threadCount = 16;
  hpConfig.root = Eigen::AlignedBox3f(Eigen::Vector3f(-1, -1, -1), Eigen::Vector3f(1, 1, 1));

  SDF::Octree hpOctree;
  hpOctree.Create(hpConfig, SphereFunc);
  auto hpBlock = hpOctree.ToMemoryBlock();

  readLegacy((unsigned char*)hpBlock.ptr, hpBlock.size);
  coeffStore = {};
  nodes = {};
}