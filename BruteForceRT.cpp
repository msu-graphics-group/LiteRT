#include <iostream>
#include <vector>
#include <algorithm>
#include <cassert>

#include "LiteRT.h"
#include "raytrace_common.h"

using LiteMath::dot;
using LiteMath::sign;
using LiteMath::cross;
using LiteMath::float4x4;
using LiteMath::uint2;
using LiteMath::float2;
using LiteMath::float3;
using LiteMath::float4;
using LiteMath::normalize;
using LiteMath::inverse4x4;
using LiteMath::to_float3;

using LiteMath::Box4f;

struct BruteForceRT : public ISceneObject
{
  BruteForceRT(){}
  ~BruteForceRT() override {}
  
  const char* Name() const override { return "BruteForce"; }

  void ClearGeom() override;

  uint32_t AddGeom_Triangles3f(const float* a_vpos3f, size_t a_vertNumber, const uint32_t* a_triIndices, size_t a_indNumber, BuildQuality a_qualityLevel, size_t vByteStride) override;
  void     UpdateGeom_Triangles3f(uint32_t a_geomId, const float* a_vpos3f, size_t a_vertNumber, const uint32_t* a_triIndices, size_t a_indNumber, BuildQuality a_qualityLevel, size_t vByteStride) override;
  uint32_t AddGeom_Sdf(const SdfScene &scene, BuildQuality a_qualityLevel = BUILD_HIGH) override;

  void ClearScene() override; 
  void CommitScene  (BuildQuality a_qualityLevel) override; 
  
  uint32_t AddInstance(uint32_t a_geomId, const float4x4& a_matrix) override;
  void     UpdateInstance(uint32_t a_instanceId, const float4x4& a_matrix) override;

  CRT_Hit  RayQuery_NearestHit(float4 posAndNear, float4 dirAndFar) override;
  bool     RayQuery_AnyHit(float4 posAndNear, float4 dirAndFar) override;

  uint32_t GetGeomNum() const override { return uint32_t(m_geomBoxes.size()); }
  uint32_t GetInstNum() const override { return uint32_t(m_instBoxes.size()); }
  const LiteMath::float4* GetGeomBoxes() const override { return (const LiteMath::float4*)m_geomBoxes.data(); }

protected:
  
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

  //common data for both meshes and SDFs
  std::vector<Box4f> m_geomBoxes;
  std::vector<Box4f> m_instBoxes;
  std::vector<float4x4> m_instMatricesInv; ///< inverse instance matrices
  std::vector<float4x4> m_instMatricesFwd; ///< instance matrices
  std::vector<uint2> m_indStartSize;
  std::vector<uint32_t>  m_geomIdByInstId;
  std::vector<unsigned> m_geomTypeByGeomId;

  //meshes data
  std::vector<float4>   m_vertPos;
  std::vector<uint32_t> m_indices;

  //SDFs data
  std::vector<uint32_t> m_SdfSceneIdByGeomId;
  std::vector<SdfScene> m_SdfScenes;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

constexpr size_t reserveSize = 1000;

void BruteForceRT::ClearGeom()
{
  m_vertPos.reserve(std::max<size_t>(100000,   m_vertPos.capacity()));
  m_indices.reserve(std::max<size_t>(100000*3, m_indices.capacity()));

  m_vertPos.resize(0);
  m_indices.resize(0);
  
  m_indStartSize.reserve (std::max(reserveSize, m_indStartSize.capacity()));
  m_indStartSize.resize(0);

  m_geomBoxes.reserve(std::max<size_t>(reserveSize, m_geomBoxes.capacity()));
  m_geomBoxes.resize(0);

  m_geomTypeByGeomId.resize(0);
  m_SdfSceneIdByGeomId.resize(0);
  m_SdfScenes.resize(0);

  ClearScene();
}

uint32_t BruteForceRT::AddGeom_Triangles3f(const float* a_vpos3f, size_t a_vertNumber, const uint32_t* a_triIndices, size_t a_indNumber, BuildQuality a_qualityLevel, size_t vByteStride)
{
  const size_t vStride = vByteStride/4;
  assert(vByteStride%4==0);

  const size_t  oldSizeVert = m_vertPos.size();
  const size_t  oldSizeInd  = m_indices.size();

  m_indStartSize.push_back (uint2(oldSizeInd, a_indNumber));

  m_vertPos.resize(oldSizeVert + a_vertNumber);
  m_indices.resize(oldSizeInd + a_indNumber);
  
  Box4f bbox;
  for(size_t i=0;i<a_vertNumber;i++) 
  {
    const float4 v = float4(a_vpos3f[i*vStride+0], a_vpos3f[i*vStride+1], a_vpos3f[i*vStride+2], 1.0f);
    m_vertPos[oldSizeVert + i] = v;
    bbox.include(v);
  }

  m_geomBoxes.push_back(bbox);
  m_geomTypeByGeomId.push_back(TYPE_MESH_TRIANGLE);

  for(size_t i=0;i<a_indNumber;i++)
    m_indices[oldSizeInd + i] = oldSizeVert + a_triIndices[i];

  return m_geomTypeByGeomId.size()-1;
}

void BruteForceRT::UpdateGeom_Triangles3f(uint32_t a_geomId, const float* a_vpos3f, size_t a_vertNumber, const uint32_t* a_triIndices, size_t a_indNumber, BuildQuality a_qualityLevel, size_t vByteStride)
{
  std::cout << "[BruteForceRT::UpdateGeom_Triangles3f]: " << "not implemeted!" << std::endl;
}

uint32_t BruteForceRT::AddGeom_Sdf(const SdfScene &scene, BuildQuality a_qualityLevel)
{
  assert(scene.conjunctions.size() > 0);
  assert(scene.objects.size() > 0);
  assert(scene.parameters.size() > 0);
  float4 mn = scene.conjunctions[0].min_pos;
  float4 mx = scene.conjunctions[0].max_pos;
  for (auto &c : scene.conjunctions) 
  {
    mn = min(mn, c.min_pos);
    mx = max(mx, c.max_pos);
  }
  m_indStartSize.push_back (uint2(0, scene.conjunctions.size()));
  m_geomBoxes.push_back(Box4f(mn, mx));
  m_geomTypeByGeomId.push_back(TYPE_SDF_PRIMITIVE);

  m_SdfScenes.push_back(scene);
  while (m_SdfSceneIdByGeomId.size() < m_geomTypeByGeomId.size())
    m_SdfSceneIdByGeomId.push_back(0u);
  m_SdfSceneIdByGeomId.back() = m_SdfScenes.size() - 1;

  return m_geomTypeByGeomId.size()-1;
}

void BruteForceRT::ClearScene()
{
  m_instBoxes.reserve(std::max(reserveSize, m_instBoxes.capacity()));
  m_instMatricesInv.reserve(std::max(reserveSize, m_instMatricesInv.capacity()));
  m_instMatricesFwd.reserve(std::max(reserveSize, m_instMatricesFwd.capacity()));

  m_geomIdByInstId.reserve(std::max(reserveSize, m_geomIdByInstId.capacity()));

  m_instBoxes.resize(0);
  m_instMatricesInv.resize(0);
  m_instMatricesFwd.resize(0);
  m_geomIdByInstId.resize(0);
}

void BruteForceRT::CommitScene(BuildQuality a_qualityLevel)
{
  
} 

uint32_t BruteForceRT::AddInstance(uint32_t a_geomId, const float4x4& a_matrix)
{
  const auto& box = m_geomBoxes[a_geomId];

  // (1) mult mesh bounding box vertices with matrix to form new bouding box for instance
  float4 boxVertices[8]{
    a_matrix*float4{box.boxMin.x, box.boxMin.y, box.boxMin.z, 1.0f},
    
    a_matrix*float4{box.boxMax.x, box.boxMin.y, box.boxMin.z, 1.0f},
    a_matrix*float4{box.boxMin.x, box.boxMax.y, box.boxMin.z, 1.0f},
    a_matrix*float4{box.boxMin.x, box.boxMin.y, box.boxMax.z, 1.0f},

    a_matrix*float4{box.boxMax.x, box.boxMax.y, box.boxMin.z, 1.0f},
    a_matrix*float4{box.boxMax.x, box.boxMin.y, box.boxMax.z, 1.0f},
    a_matrix*float4{box.boxMin.x, box.boxMax.y, box.boxMax.z, 1.0f},
    
    a_matrix*float4{box.boxMax.x, box.boxMax.y, box.boxMax.z, 1.0f},
  };
  
  Box4f newBox;
  for(size_t i=0;i<8;i++)
    newBox.include(boxVertices[i]);
  
  // (2) append bounding box and matrices
  //
  const uint32_t oldSize = uint32_t(m_instBoxes.size());

  m_instBoxes.push_back(newBox);
  m_instMatricesFwd.push_back(a_matrix);
  m_instMatricesInv.push_back(inverse4x4(a_matrix));
  m_geomIdByInstId.push_back(a_geomId);

  return oldSize;
}

void BruteForceRT::UpdateInstance(uint32_t a_instanceId, const float4x4& a_matrix)
{
  std::cout << "[BruteForceRT::UpdateInstance]: " << "not implemeted!" << std::endl;
}

void BruteForceRT::IntersectAllTrianglesInLeaf(const float3 ray_pos, const float3 ray_dir,
                                               float tNear, uint32_t instId, uint32_t geomId,
                                               uint32_t a_start, uint32_t a_count,
                                               CRT_Hit *pHit)
{
    for (size_t triAddress = a_start; triAddress < a_start + a_count; triAddress+=3)
    { 
      const uint32_t A = m_indices[triAddress + 0];
      const uint32_t B = m_indices[triAddress + 1];
      const uint32_t C = m_indices[triAddress + 2];
    
      const float3 A_pos = to_float3(m_vertPos[A]);
      const float3 B_pos = to_float3(m_vertPos[B]);
      const float3 C_pos = to_float3(m_vertPos[C]);
    
      const float3 edge1 = B_pos - A_pos;
      const float3 edge2 = C_pos - A_pos;
      const float3 pvec  = cross(ray_dir, edge2);
      const float3 tvec  = ray_pos - A_pos;
      const float3 qvec  = cross(tvec, edge1);
      
      const float invDet = 1.0f / dot(edge1, pvec);
      const float v      = dot(tvec, pvec)*invDet;
      const float u      = dot(qvec, ray_dir)*invDet;
      const float t      = dot(edge2, qvec)*invDet;
    
      if (v >= -1e-6f && u >= -1e-6f && (u + v <= 1.0f+1e-6f) && t > tNear && t < pHit->t) // if (v > -1e-6f && u > -1e-6f && (u + v < 1.0f+1e-6f) && t > tMin && t < hit.t)
      {
        float3 n = normalize(cross(edge1, edge2));

        pHit->t         = t;
        pHit->primId    = (triAddress-a_start)/3;
        pHit->instId    = instId;
        pHit->geomId    = geomId;  
        pHit->coords[0] = u;
        pHit->coords[1] = v;
        pHit->coords[2] = n.x;
        pHit->coords[3] = n.y;
      }
    }
}

void BruteForceRT::IntersectAllSdfPrimitivesInLeaf(const float3 ray_pos, const float3 ray_dir,
                                                   float tNear, uint32_t instId, uint32_t geomId,
                                                   uint32_t a_start, uint32_t a_count,
                                                   CRT_Hit *pHit)
{
  auto &sdf = m_SdfScenes[m_SdfSceneIdByGeomId[geomId]];
  float3 p0;
  float l = LiteMath::length(ray_dir);
  float3 dir = ray_dir/l;
  float3 n = float3(1,0,0);
  if (sdf_sphere_tracing(sdf, LiteMath::to_float3(m_geomBoxes[geomId].boxMin), LiteMath::to_float3(m_geomBoxes[geomId].boxMax), ray_pos, dir, &p0))
  {
    pHit->t         = LiteMath::length(p0-ray_pos)/l;
    pHit->primId    = 0;
    pHit->instId    = instId;
    pHit->geomId    = geomId;  
    pHit->coords[0] = 0;
    pHit->coords[1] = 0;
    pHit->coords[2] = n.x;
    pHit->coords[3] = n.y;
  }
}

void BruteForceRT::IntersectAllPrimitivesInLeaf(const float3 ray_pos, const float3 ray_dir,
                                                float tNear, uint32_t instId, uint32_t geomId,
                                                uint32_t a_start, uint32_t a_count,
                                                CRT_Hit *pHit)
{
  unsigned type = m_geomTypeByGeomId[geomId];
  if (type == TYPE_MESH_TRIANGLE)
    IntersectAllTrianglesInLeaf(ray_pos, ray_dir, tNear, instId, geomId, a_start, a_count, pHit);
  else if (type == TYPE_SDF_PRIMITIVE)
    IntersectAllSdfPrimitivesInLeaf(ray_pos, ray_dir, tNear, instId, geomId, a_start, a_count, pHit);
}

CRT_Hit BruteForceRT::RayQuery_NearestHit(float4 posAndNear, float4 dirAndFar)
{
  float3 invRayDir = SafeInverse(to_float3(dirAndFar));
  float3 rayPos    = to_float3(posAndNear);
  
  std::vector<BoxHit> boxMinHits;
  boxMinHits.reserve(64);
  
  const float tMin = posAndNear.w;
  const float tMax = dirAndFar.w; 

  // (1) check all instance boxes and all triangles inside box. 
  for(uint32_t i=0;i<uint32_t(m_instBoxes.size());i++)
  {
    const float2 minMax = RayBoxIntersection2(rayPos, invRayDir, to_float3(m_instBoxes[i].boxMin), to_float3(m_instBoxes[i].boxMax));
    if( (minMax.x <= minMax.y) && (minMax.y >= tMin) && (minMax.x <= tMax) )
      boxMinHits.push_back(make_BoxHit(i, minMax.x));
  }
  
  // (2) sort all hits by hit distance to process nearest first
  //
  std::sort(boxMinHits.begin(), boxMinHits.end(), [](const BoxHit a, const BoxHit b){ return a.tHit < b.tHit; });

  // (3) process all potential hits
  //
  CRT_Hit hit;
  hit.t      = tMax;
  hit.primId = uint32_t(-1);
  hit.instId = uint32_t(-1);
  hit.geomId = uint32_t(-1);

  for(uint32_t boxId = 0; boxId < uint32_t(boxMinHits.size()); boxId++)
  {
    if(boxMinHits[boxId].tHit > hit.t) // already found hit that is closer than bounding box hit
      break;

    const uint32_t instId = boxMinHits[boxId].id;
    const uint32_t geomId = m_geomIdByInstId[instId];
    const uint2 startSize = m_indStartSize[geomId];

    // transform ray with matrix to local space
    //
    float3 ray_pos = mul4x3(m_instMatricesInv[instId], to_float3(posAndNear));
    float3 ray_dir = mul3x3(m_instMatricesInv[instId], to_float3(dirAndFar)); // DON'T NORMALIZE IT !!!! When we transform to local space of node, ray_dir must be unnormalized!!!

    // test all triangles in mesh
    //
    IntersectAllPrimitivesInLeaf(ray_pos, ray_dir, tMin, instId, geomId, startSize.x, startSize.y, &hit);
  }
 
  return hit;
}

bool BruteForceRT::RayQuery_AnyHit(float4 posAndNear, float4 dirAndFar)
{
  // (2) If any hit is found, immediately return true.
  std::cout << "[BruteForceRT::RayQuery_AnyHit]: " << "not implemeted!" << std::endl;
  return false;
}

ISceneObject* MakeBruteForceRT(const char* a_implName) { return new BruteForceRT; }