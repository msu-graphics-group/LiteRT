#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>
#include <string>
#include <memory>

#include "LiteMath.h"

using LiteMath::cross;
using LiteMath::dot;
using LiteMath::float2;
using LiteMath::float3;
using LiteMath::float4;
using LiteMath::float4x4;
using LiteMath::int2;
using LiteMath::inverse4x4;
using LiteMath::normalize;
using LiteMath::sign;
using LiteMath::to_float3;
using LiteMath::uint;
using LiteMath::uint2;
using LiteMath::uint3;
using LiteMath::uint4;
using LiteMath::Box4f;

#include "../ISceneObject.h"
#include "../raytrace_common.h"
#include "cbvh.h"

// main class
//
struct BVHRT : public ISceneObject
{
  BVHRT(const char* a_buildName = nullptr, const char* a_layoutName = nullptr) : 
    m_buildName(a_buildName != nullptr ? a_buildName : ""), 
    m_layoutName(a_layoutName != nullptr ? a_layoutName : "") { }
  ~BVHRT() override {}

  const char* Name() const override { return "BVH2Fat"; }
  const char* BuildName() const override { return m_buildName.c_str(); };

  void ClearGeom() override;

  uint32_t AddGeom_Triangles3f(const float *a_vpos3f, size_t a_vertNumber, const uint32_t *a_triIndices, size_t a_indNumber, BuildQuality a_qualityLevel, size_t vByteStride) override;
  void     UpdateGeom_Triangles3f(uint32_t a_geomId, const float *a_vpos3f, size_t a_vertNumber, const uint32_t *a_triIndices, size_t a_indNumber, BuildQuality a_qualityLevel, size_t vByteStride) override;
#ifndef KERNEL_SLICER  
  uint32_t AddGeom_Sdf(const SdfScene &scene, BuildQuality a_qualityLevel = BUILD_HIGH) override;
#endif
  void ClearScene() override;
  virtual void CommitScene(BuildQuality a_qualityLevel) override;

  uint32_t AddInstance(uint32_t a_geomId, const float4x4 &a_matrix) override;
  void UpdateInstance(uint32_t a_instanceId, const float4x4 &a_matrix) override;

  CRT_Hit RayQuery_NearestHit(float4 posAndNear, float4 dirAndFar) override;
  bool    RayQuery_AnyHit(float4 posAndNear, float4 dirAndFar) override;
  
  uint32_t GetGeomNum() const override { return uint32_t(m_geomBoxes.size()); }
  uint32_t GetInstNum() const override { return uint32_t(m_instBoxes.size()); }
  const LiteMath::float4* GetGeomBoxes() const override { return (const LiteMath::float4*)m_geomBoxes.data(); }
  
//protected:

  void IntersectAllPrimitivesInLeaf(const float3 ray_pos, const float3 ray_dir,
                                    float tNear, uint32_t instId, uint32_t geomId,
                                    uint32_t a_start, uint32_t a_count,
                                    CRT_Hit *pHit);

  void IntersectAllSdfPrimitivesInLeaf(const float3 ray_pos, const float3 ray_dir,
                                       float tNear, uint32_t instId, uint32_t geomId,
                                       uint32_t a_start, uint32_t a_count,
                                       CRT_Hit *pHit);

  void IntersectAllTrianglesInLeaf(const float3 ray_pos, const float3 ray_dir,
                                   float tNear, uint32_t instId, uint32_t geomId,
                                   uint32_t a_start, uint32_t a_count,
                                   CRT_Hit *pHit);
                                   
  virtual void BVH2TraverseF32(const float3 ray_pos, const float3 ray_dir, float tNear, 
                               uint32_t instId, uint32_t geomId, uint32_t stack[STACK_SIZE], bool stopOnFirstHit,
                               CRT_Hit *pHit);

  virtual void AppendTreeData(const std::vector<BVHNodePair>& a_nodes, const std::vector<uint32_t>& a_indices, 
                              const uint32_t *a_triIndices, size_t a_indNumber);

  //helper functions to calculate SDF
  //It is a copy-past of sdfScene functions with the same names
  //Slicer is weak and can't handle calling external functions  ¯\_(ツ)_/¯
  virtual float2 box_intersects(const float3 &min_pos, const float3 &max_pos, const float3 &origin, const float3 &dir);
  virtual float eval_dist_prim(unsigned prim_id, float3 p);
  virtual float eval_dist_conjunction(unsigned conj_id, float3 p);
  virtual SdfHit sdf_conjunction_sphere_tracing(unsigned conj_id, const float3 &min_pos, const float3 &max_pos,
                                                const float3 &pos, const float3 &dir, bool need_norm);
  //for each model in scene  
  std::vector<Box4f>    m_geomBoxes;
  std::vector<uint2>    m_geomOffsets; //means different things for different types of geometry
  std::vector<uint32_t> m_bvhOffsets;
  std::vector<unsigned> m_geomTypeByGeomId;

  //SDFs data
  std::vector<float> m_SdfParameters;
  std::vector<SdfObject> m_SdfObjects;
  std::vector<SdfConjunction> m_SdfConjunctions;
  std::vector<NeuralProperties> m_SdfNeuralProperties;
  std::vector<uint32_t> m_ConjIndices; //conjunction index for each leaf node in Fat BVH related to SDF

  //for each instance in scene
  std::vector<Box4f> m_instBoxes;
  std::vector<uint32_t> m_geomIdByInstId;
  std::vector<float4x4> m_instMatricesInv; ///< inverse instance matrices
  std::vector<float4x4> m_instMatricesFwd; ///< instance matrices

  //meshes data
  std::vector<float4>   m_vertPos;
  std::vector<uint32_t> m_indices;
  std::vector<uint32_t> m_primIndices;

  //Top Level Acceleration Structure
  std::vector<BVHNode>    m_nodesTLAS;

  //Bottom Level Acceleration Structure
  std::vector<BVHNodePair> m_allNodePairs;


  // Format name the tree build from
  const std::string m_buildName;
  const std::string m_layoutName;

  bool m_firstSceneCommit = true;
};
