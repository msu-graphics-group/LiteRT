#include "hp_octree_precomputed_tables.h"
#include "hp_octree.h"
#include "../dependencies/hp_octree/Include/HP/Octree.h"
#include "mesh_bvh.h"


void HPOctreeBuilder::construct(const cmesh4::SimpleMesh &mesh, BuildSettings settings)
{
  std::vector<MeshBVH> mesh_bvhes;
  mesh_bvhes.resize(settings.threads);
  for (int i=0;i<settings.threads;i++)
  mesh_bvhes[i].init(mesh);
  
  auto sdf_func = [&mesh_bvhes](const float3 &p, unsigned thread_idx)
                    { return mesh_bvhes[thread_idx].get_signed_distance(p); };
  
  construct(sdf_func, settings);
}

void HPOctreeBuilder::construct(std::function<float(const float3 &, unsigned thread_idx)> sdf,
                                BuildSettings settings)
{
  auto SphereFunc = [&sdf](const Eigen::Vector3d &pt_, const unsigned threadIdx_) -> double
  {
    return sdf(float3(pt_.x(), pt_.y(), pt_.z()), threadIdx_);
  };

  SDF::Config hpConfig;
  hpConfig.targetErrorThreshold = 5*1e-7;
  hpConfig.nearnessWeighting.type = (SDF::Config::NearnessWeighting::Type)settings.nearness_weighting;
  hpConfig.nearnessWeighting.strength = settings.nearness_weight;
  hpConfig.continuity.enforce = settings.enforce_continuity;
  hpConfig.continuity.strength = settings.continuity_strength;
  hpConfig.threadCount = settings.threads;
  hpConfig.root = Eigen::AlignedBox3f(Eigen::Vector3f(-1, -1, -1), Eigen::Vector3f(1, 1, 1));

  SDF::Octree hpOctree;
  hpOctree.Create(hpConfig, SphereFunc);
  auto hpBlock = hpOctree.ToMemoryBlock();

  readLegacy((unsigned char*)hpBlock.ptr, hpBlock.size);
  coeffStore = {};
  nodes = {};
}