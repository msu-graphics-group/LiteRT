#pragma once

#include <iostream>
#include <vector>
#include <unordered_map>
#include <cassert>
#include <limits>

#include "CrossRT.h"
#include "LiteScene/scene_mgr.h" // RTX implementation of acceleration structures

class VulkanRTX : public ISceneObject
{
public:
  VulkanRTX(std::shared_ptr<SceneManager> a_pScnMgr);
  VulkanRTX(VkDevice a_device, VkPhysicalDevice a_physDevice, uint32_t a_graphicsQId, std::shared_ptr<vk_utils::ICopyEngine> a_pCopyHelper,
            uint32_t maxMeshes, uint32_t maxTotalVertices, uint32_t maxTotalPrimitives, uint32_t maxPrimitivesPerMesh, bool build_as_add, bool has_aabb);
  ~VulkanRTX();
  const char* Name() const override { return "VulkanRTX"; }
  
  void ClearGeom() override;
  
  uint32_t AddGeom_Triangles3f(const float* a_vpos3f, size_t a_vertNumber, const uint32_t* a_triIndices, size_t a_indNumber, 
                               uint32_t a_flags, size_t vByteStride) override;
  void     UpdateGeom_Triangles3f(uint32_t a_geomId, const float* a_vpos3f, size_t a_vertNumber, const uint32_t* a_triIndices,
                                  size_t a_indNumber, uint32_t a_flags, size_t vByteStride) override;

  uint32_t AddGeom_AABB(uint32_t a_typeId, const CRT_AABB* boxMinMaxF8, size_t a_boxNumber, void** a_customPrimPtrs, size_t a_customPrimCount) override;
  void     UpdateGeom_AABB(uint32_t a_geomId, uint32_t a_typeId, const CRT_AABB* boxMinMaxF8, size_t a_boxNumber, void** a_customPrimPtrs, size_t a_customPrimCount) override;

  void ClearScene() override; 
  void CommitScene(uint32_t options = BUILD_HIGH) override; 
  
  uint32_t AddInstance(uint32_t a_geomId, const LiteMath::float4x4& a_matrix) override;
  uint32_t AddInstanceMotion(uint32_t a_geomId, const LiteMath::float4x4* a_matrices, uint32_t a_matrixNumber) override;
  void     UpdateInstance(uint32_t a_instanceId, const LiteMath::float4x4& a_matrix) override;

  CRT_Hit  RayQuery_NearestHit(LiteMath::float4 posAndNear, LiteMath::float4 dirAndFar) override;
  bool     RayQuery_AnyHit(LiteMath::float4 posAndNear, LiteMath::float4 dirAndFar) override;
  CRT_Hit  RayQuery_NearestHitMotion(LiteMath::float4 posAndNear, LiteMath::float4 dirAndFar, float time = 0.0f) override;
  bool     RayQuery_AnyHitMotion(LiteMath::float4 posAndNear, LiteMath::float4 dirAndFar, float time= 0.0f) override;

  ////////////////////////////////////////////////////////////////////////////////////////////////
  
  void SetSceneAccelStruct(VkAccelerationStructureKHR handle) { m_accel = handle; }
  VkAccelerationStructureKHR GetSceneAccelStruct() const { return m_accel; }
  std::shared_ptr<SceneManager> GetSceneManager() const { return m_pScnMgr; }

  static constexpr size_t VERTEX_SIZE = sizeof(float) * 4;
  
  virtual void SetSBTRecordOffsets(const std::vector<uint32_t>& a_recordOffsets) { m_sbtRecordOffsets = a_recordOffsets; }

  size_t   GetBLASCount() const { return m_pScnMgr->GetBLASCount(); }

protected:
  VkAccelerationStructureKHR    m_accel = VK_NULL_HANDLE;
  std::shared_ptr<SceneManager> m_pScnMgr;
  std::vector<uint32_t>         m_sbtRecordOffsets;
};

struct RTX_Proxy : public ISceneObject
{
public:
  RTX_Proxy(std::shared_ptr<ISceneObject> a, std::shared_ptr<ISceneObject> b) { m_imps[0] = a; m_imps[1] = b; }  
  
  const char* Name() const override { return "RTX_Proxy"; }
  ISceneObject* UnderlyingImpl(uint32_t a_implId) override { return (a_implId < 2) ? m_imps[a_implId].get() : nullptr; }

  void ClearGeom() override;

  uint32_t AddGeom_Triangles3f(const float* a_vpos3f, size_t a_vertNumber, const uint32_t* a_triIndices, size_t a_indNumber, uint32_t a_flags, size_t vByteStride) override;
  void     UpdateGeom_Triangles3f(uint32_t a_geomId, const float* a_vpos3f, size_t a_vertNumber, const uint32_t* a_triIndices, size_t a_indNumber, uint32_t a_flags, size_t vByteStride) override;
  
  uint32_t AddGeom_AABB(uint32_t a_typeId, const CRT_AABB* boxMinMaxF8, size_t a_boxNumber, void** a_customPrimPtrs, size_t a_customPrimCount) override;
  void     UpdateGeom_AABB(uint32_t a_geomId, uint32_t a_typeId, const CRT_AABB* boxMinMaxF8, size_t a_boxNumber, void** a_customPrimPtrs, size_t a_customPrimCount) override;
  
  void     ClearScene() override;
  void     CommitScene(uint32_t options) override;

  uint32_t AddInstanceMotion(uint32_t a_geomId, const LiteMath::float4x4* a_matrices, uint32_t a_matrixNumber) override;
  uint32_t AddInstance(uint32_t a_geomId, const LiteMath::float4x4& a_matrix)          override;

  void    UpdateInstance(uint32_t a_instanceId, const LiteMath::float4x4& a_matrix)    override;
  CRT_Hit RayQuery_NearestHit(LiteMath::float4 posAndNear, LiteMath::float4 dirAndFar) override;
  bool    RayQuery_AnyHit(LiteMath::float4 posAndNear, LiteMath::float4 dirAndFar)     override;

  CRT_Hit RayQuery_NearestHitMotion(LiteMath::float4 posAndNear, LiteMath::float4 dirAndFar, float time) override;
  bool    RayQuery_AnyHitMotion(LiteMath::float4 posAndNear, LiteMath::float4 dirAndFar, float time)     override;

  uint32_t AddCustomGeom_FromFile(const char *geom_type_name, const char *filename, ISceneObject *fake_this) override;
  
  // internal functions further
  //
  /**
  \brief In the case when single primitive is represented with several bounding boxes, (geomId,aabbId) --> primId map is needed
  */
  struct PrimitiveRemapTable
  {
    const LiteMath::uint2* table; ///<! primId  = table[aabbId]
    const uint32_t*    geomTable; ///<! primTag = geomTable[geomId]
    uint32_t tableSize;           ///<! count of elements in table array
    uint32_t geomSize;            ///<! count of elements in geomTable array
  };

  /**
  \brief for target custom geometry type get teble which remap bounding box id to primitive id
  \param a_typeId - internal geometry typeId 
  */
  virtual PrimitiveRemapTable GetAABBToPrimTable() const; 

protected:

  std::array<std::shared_ptr<ISceneObject>, 2> m_imps = {nullptr, nullptr};

  std::vector<unsigned int>    m_geomTags;
  std::vector<LiteMath::uint2> m_remapTable;
  std::unordered_map<uint32_t, LiteMath::uint2> m_offsetByTag;

};