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

  //check if nodes actually have surface inside it
  //and mark nodes that are fully inside or outside object as invalid
  //we do not need them in final octree
  if (true)
  {
    float distance_thr = 1e-5f;
    unsigned partitions = 2;
    for (auto &n : nodes)
    {
      float3 step = (n.aabb.m_max - n.aabb.m_min)/float(partitions);
      float min_val = 1000;
      float max_val = -1000;
      for (int i=0;i<=partitions;i++)
      {
        for (int j=0;j<=partitions;j++)
        {
          for (int k=0;k<=partitions;k++)
          {
            float3 p = n.aabb.m_min + float3(i,j,k)*step;
            float d = sdf(p, 0);
            min_val = std::min(min_val, d);
            max_val = std::max(max_val, d);
          }          
        }        
      }

      if (min_val > distance_thr) //node is outside
        n.basis.degree = BASIS_MAX_DEGREE + 1;
      else if (max_val < -distance_thr) //node is inside
        n.basis.degree = BASIS_MAX_DEGREE + 1;
    }
  }

  readLegacy((unsigned char*)hpBlock.ptr, hpBlock.size);
  coeffStore = {};
  nodes = {};
}