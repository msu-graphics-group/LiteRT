#include "VulkanRTX.h"
#include "ray_tracing/vk_rt_utils.h"
#include "vk_utils.h"

ISceneObject* CreateVulkanRTX(std::shared_ptr<SceneManager> a_pScnMgr) { return new VulkanRTX(a_pScnMgr); }

ISceneObject* CreateVulkanRTX(VkDevice a_device, VkPhysicalDevice a_physDevice, uint32_t a_graphicsQId, std::shared_ptr<vk_utils::ICopyEngine> a_pCopyHelper,
                              uint32_t a_maxMeshes, uint32_t a_maxTotalVertices, uint32_t a_maxTotalPrimitives, uint32_t a_maxPrimitivesPerMesh,
                              bool build_as_add, bool has_aabb)
{
  static constexpr uint64_t STAGING_MEM_SIZE = 16 * 16 * 1024u;
  VkQueue queue;
  vkGetDeviceQueue(a_device, a_graphicsQId, 0, &queue);

  auto copyHelper = std::make_shared<vk_utils::PingPongCopyHelper>(a_physDevice, a_device, queue, a_graphicsQId, STAGING_MEM_SIZE);

  return new VulkanRTX(a_device, a_physDevice, a_graphicsQId, copyHelper,a_maxMeshes, a_maxTotalVertices, a_maxTotalPrimitives, a_maxPrimitivesPerMesh, build_as_add, has_aabb);
}

VulkanRTX::VulkanRTX(std::shared_ptr<SceneManager> a_pScnMgr) : m_pScnMgr(a_pScnMgr)
{
}

VulkanRTX::VulkanRTX(VkDevice a_device, VkPhysicalDevice a_physDevice, uint32_t a_graphicsQId, std::shared_ptr<vk_utils::ICopyEngine> a_pCopyHelper,
                     uint32_t a_maxMeshes, uint32_t a_maxTotalVertices, uint32_t a_maxTotalPrimitives, uint32_t a_maxPrimitivesPerMesh,
                     bool build_as_add, bool has_aabb)
{
  LoaderConfig conf = {};
  conf.load_geometry = true;
  conf.load_materials = MATERIAL_LOAD_MODE::NONE;
  conf.build_acc_structs = true;
  conf.build_acc_structs_while_loading_scene = build_as_add;
  conf.builder_type    = BVH_BUILDER_TYPE::RTX;
  conf.mesh_format     = MESH_FORMATS::MESH_4F;

  m_pScnMgr = std::make_shared<SceneManager>(a_device, a_physDevice, a_graphicsQId, a_pCopyHelper, conf);
  m_pScnMgr->InitEmptyScene(a_maxMeshes, a_maxTotalVertices, a_maxTotalPrimitives, a_maxPrimitivesPerMesh, has_aabb ? a_maxPrimitivesPerMesh : 10);
}


VulkanRTX::~VulkanRTX()
{
  m_pScnMgr = nullptr;
}

void VulkanRTX::ClearGeom()
{
//  m_pScnMgr->DestroyScene();
}
  
uint32_t VulkanRTX::AddGeom_Triangles3f(const float* a_vpos3f, size_t a_vertNumber, const uint32_t* a_triIndices, size_t a_indNumber,
                                        uint32_t a_flags, size_t vByteStride)
{
  if(vByteStride == 0)
    vByteStride = sizeof(float)*3;

  if(vByteStride % 4 != 0)
  {
    std::cout << "[VulkanRTX::AddGeom_Triangles3f]: vByteStride must be multiple of sizeof(float), passed value is: " << vByteStride << std::endl;
    return uint32_t(-1);
  }

  cmesh::SimpleMesh mesh(a_vertNumber, a_indNumber);
  
  if(vByteStride == sizeof(float)*4)
    memcpy(mesh.vPos4f.data(), (float*)a_vpos3f, a_vertNumber * VERTEX_SIZE);
  else
  {
    const size_t vStride = vByteStride/sizeof(float);
    for(size_t i=0;i<a_vertNumber;i++)
    {
      mesh.vPos4f[i*4+0] = a_vpos3f[i*vStride+0];
      mesh.vPos4f[i*4+1] = a_vpos3f[i*vStride+1];
      mesh.vPos4f[i*4+2] = a_vpos3f[i*vStride+2];
      mesh.vPos4f[i*4+3] = 1.0f;
    }
  }
  memcpy(mesh.indices.data(), a_triIndices, a_indNumber * sizeof(a_triIndices[0]));

  auto idx = m_pScnMgr->AddMeshFromDataAndQueueBuildAS(mesh);
  return idx;
}

uint32_t VulkanRTX::AddGeom_AABB(uint32_t a_typeId, const CRT_AABB* boxMinMaxF8, size_t a_boxNumber, void** a_customPrimPtrs, size_t a_customPrimCount) 
{
  std::vector<VkAabbPositionsKHR> tempBuffer(a_boxNumber);
  for(size_t i=0;i<a_boxNumber;i++) 
  {
    tempBuffer[i].minX = boxMinMaxF8[i].boxMin.x;
    tempBuffer[i].minY = boxMinMaxF8[i].boxMin.y;
    tempBuffer[i].minZ = boxMinMaxF8[i].boxMin.z;
    tempBuffer[i].maxX = boxMinMaxF8[i].boxMax.x;
    tempBuffer[i].maxY = boxMinMaxF8[i].boxMax.y;
    tempBuffer[i].maxZ = boxMinMaxF8[i].boxMax.z;
  }

  return m_pScnMgr->AddGeomFromAABBAndQueueBuildAS(a_typeId, tempBuffer.data(), tempBuffer.size()) | CRT_GEOM_MASK_AABB_BIT;
}

void VulkanRTX::UpdateGeom_AABB(uint32_t a_geomId, uint32_t a_typeId, const CRT_AABB* boxMinMaxF8, size_t a_boxNumber, void** a_customPrimPtrs, size_t a_customPrimCount) 
{
  std::cout << "[VulkanRTX::UpdateGeom_AABB]: not implemented" << std::endl;
}

void VulkanRTX::UpdateGeom_Triangles3f(uint32_t a_geomId, const float* a_vpos3f, size_t a_vertNumber, const uint32_t* a_triIndices,
                                       size_t a_indNumber, uint32_t a_flags, size_t vByteStride)
{
  std::cout << "[VulkanRTX::UpdateGeom_Triangles3f]: not implemented" << std::endl;
}

void VulkanRTX::ClearScene()
{
} 

uint32_t VulkanRTX::AddInstance(uint32_t a_geomId, const LiteMath::float4x4& a_matrix)
{
  if((a_geomId & CRT_GEOM_MASK_AABB_BIT) != 0)
  {
    const uint32_t aabbGeomId = (a_geomId & CRT_GEOM_MASK_AABB_BIT_RM);
    return m_pScnMgr->InstanceAABB(aabbGeomId, a_matrix);
  }
  else
    return m_pScnMgr->InstanceMesh(a_geomId, a_matrix);
}

uint32_t VulkanRTX::AddInstanceMotion(uint32_t a_geomId, const LiteMath::float4x4* a_matrices, uint32_t a_matrixNumber)
{
  if(a_matrixNumber == 0 || a_matrices == nullptr)
  {
    std::cout << "[VulkanRTX::AddInstance] motion blur: empty matrices array" << std::endl;
    return uint32_t(-1);
  }

  if(a_matrixNumber == 1)
    return m_pScnMgr->InstanceMesh(a_geomId, a_matrices[0]);

  if(a_matrixNumber > 2)
  {
    std::cout << "[VulkanRTX::AddInstance] motion blur: only 2 matrices (start and end point) are supported on GPU" << std::endl;
  }

  return m_pScnMgr->InstanceMesh(a_geomId, a_matrices[0], true, a_matrices[1]);
}

void VulkanRTX::CommitScene(uint32_t options)
{
  if(options & MOTION_BLUR)
    m_pScnMgr->BuildTLAS_MotionBlur(m_sbtRecordOffsets.data(), m_sbtRecordOffsets.size());
  else
    m_pScnMgr->BuildTLAS(m_sbtRecordOffsets.data(), m_sbtRecordOffsets.size());
    
  m_accel = m_pScnMgr->GetTLAS();
}  

void VulkanRTX::UpdateInstance(uint32_t a_instanceId, const LiteMath::float4x4& a_matrix)
{
  std::cout << "[VulkanRTX::UpdateInstance]: not implemented" << std::endl;
}

CRT_Hit VulkanRTX::RayQuery_NearestHit(LiteMath::float4 posAndNear, LiteMath::float4 dirAndFar)
{    
  CRT_Hit result;
  result.t      = std::numeric_limits<float>::max();
  result.geomId = uint32_t(-1);
  result.instId = uint32_t(-1);
  result.primId = uint32_t(-1);
  return result;
}

bool VulkanRTX::RayQuery_AnyHit(LiteMath::float4 posAndNear, LiteMath::float4 dirAndFar)
{
  return false;
}

CRT_Hit VulkanRTX::RayQuery_NearestHitMotion(LiteMath::float4 posAndNear, LiteMath::float4 dirAndFar, float time)
{    
  CRT_Hit result;
  result.t      = std::numeric_limits<float>::max();
  result.geomId = uint32_t(-1);
  result.instId = uint32_t(-1);
  result.primId = uint32_t(-1);
  return result;
}

bool VulkanRTX::RayQuery_AnyHitMotion(LiteMath::float4 posAndNear, LiteMath::float4 dirAndFar, float time)
{
  return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RTX_Proxy::ClearGeom() 
{ 
  for(auto impl : m_imps) 
    impl->ClearGeom(); 

  m_remapTable.clear();
  m_offsetByTag.clear();
  m_geomTags.clear();

  m_remapTable.reserve(1000);
  m_geomTags.reserve(1000);
} 

uint32_t RTX_Proxy::AddGeom_Triangles3f(const float* a_vpos3f, size_t a_vertNumber, const uint32_t* a_triIndices, size_t a_indNumber, uint32_t a_flags, size_t vByteStride) 
{
  uint32_t res = 0;
  for(auto impl : m_imps) 
    res = impl->AddGeom_Triangles3f(a_vpos3f, a_vertNumber, a_triIndices, a_indNumber, a_flags, vByteStride);
  
  return res;
}
                               
void RTX_Proxy::UpdateGeom_Triangles3f(uint32_t a_geomId, const float* a_vpos3f, size_t a_vertNumber, const uint32_t* a_triIndices, size_t a_indNumber, uint32_t a_flags, size_t vByteStride)
{
  for(auto impl : m_imps) 
    impl->UpdateGeom_Triangles3f(a_geomId, a_vpos3f, a_vertNumber, a_triIndices, a_indNumber, a_flags, vByteStride);
}
  
uint32_t RTX_Proxy::AddGeom_AABB(uint32_t a_typeId, const CRT_AABB* boxMinMaxF8, size_t a_boxNumber, void** a_customPrimPtrs, size_t a_customPrimCount) 
{
  uint32_t geomId = 0;
  for(auto impl : m_imps) 
    geomId = impl->AddGeom_AABB(a_typeId, boxMinMaxF8, a_boxNumber, a_customPrimPtrs, a_customPrimCount);
  
  {
    size_t actualPrimsCount = (a_customPrimCount == 0) ? a_boxNumber : a_customPrimCount;
    uint32_t div            = uint32_t(a_boxNumber/actualPrimsCount);

    auto pRemapOffset = m_offsetByTag.find(a_typeId);
    if(pRemapOffset == m_offsetByTag.end()) 
      pRemapOffset = m_offsetByTag.insert(std::make_pair(a_typeId, LiteMath::uint2(0,div))).first;
    
    pRemapOffset->second.y = div;
    m_remapTable.push_back(pRemapOffset->second);
    pRemapOffset->second.x += uint32_t(actualPrimsCount);
  }

  const auto geomIdClean = geomId & (CRT_GEOM_MASK_AABB_BIT_RM);
  assert(m_geomTags.size() == geomIdClean); // m_geomTags[geomIdClean] = a_typeId;
  m_geomTags.push_back(a_typeId);           //

  return geomId;
}
  
void RTX_Proxy::UpdateGeom_AABB(uint32_t a_geomId, uint32_t a_typeId, const CRT_AABB* boxMinMaxF8, size_t a_boxNumber, void** a_customPrimPtrs, size_t a_customPrimCount) 
{
  for(auto impl : m_imps) 
    impl->UpdateGeom_AABB(a_geomId, a_typeId, boxMinMaxF8, a_boxNumber, a_customPrimPtrs, a_customPrimCount);
}

RTX_Proxy::PrimitiveRemapTable RTX_Proxy::GetAABBToPrimTable() const
{
  PrimitiveRemapTable res;
  res.table     = m_remapTable.data();
  res.tableSize = uint32_t(m_remapTable.size());
  res.geomTable = m_geomTags.data();
  res.geomSize  = uint32_t(m_geomTags.size());
  return res;
}

void RTX_Proxy::ClearScene()  
{ 
  for(auto impl : m_imps) 
    impl->ClearScene(); 
} 

void RTX_Proxy::CommitScene(uint32_t options)  
{ 
  for(auto impl : m_imps) 
    impl->CommitScene(options); 
}

uint32_t RTX_Proxy::AddInstanceMotion(uint32_t a_geomId, const LiteMath::float4x4* a_matrices, uint32_t a_matrixNumber)  
{ 
  uint32_t res = 0;
  for(auto impl : m_imps) 
    res = impl->AddInstanceMotion(a_geomId, a_matrices, a_matrixNumber);
  return res; 
}

uint32_t RTX_Proxy::AddInstance(uint32_t a_geomId, const LiteMath::float4x4& a_matrix)  
{ 
  uint32_t res = 0;
  for(auto impl : m_imps) 
    res = impl->AddInstance(a_geomId, a_matrix);
  return res; 
}

void RTX_Proxy::UpdateInstance(uint32_t a_instanceId, const LiteMath::float4x4& a_matrix) 
{ 
  for(auto impl : m_imps) 
    impl->UpdateInstance(a_instanceId, a_matrix); 
}

CRT_Hit RTX_Proxy::RayQuery_NearestHit(LiteMath::float4 posAndNear, LiteMath::float4 dirAndFar)  
{ 
  return m_imps[0]->RayQuery_NearestHit(posAndNear, dirAndFar); 
}

bool RTX_Proxy::RayQuery_AnyHit(LiteMath::float4 posAndNear, LiteMath::float4 dirAndFar)  
{ 
  return m_imps[0]->RayQuery_AnyHit(posAndNear, dirAndFar);
}  

CRT_Hit RTX_Proxy::RayQuery_NearestHitMotion(LiteMath::float4 posAndNear, LiteMath::float4 dirAndFar, float time)  
{ 
  return m_imps[0]->RayQuery_NearestHit(posAndNear, dirAndFar); 
}

bool RTX_Proxy::RayQuery_AnyHitMotion(LiteMath::float4 posAndNear, LiteMath::float4 dirAndFar, float time) 
{ 
  return m_imps[0]->RayQuery_AnyHit(posAndNear, dirAndFar); 
}

uint32_t RTX_Proxy::AddCustomGeom_FromFile(const char *geom_type_name, const char *filename, ISceneObject *fake_this)
{
  return m_imps[0]->AddCustomGeom_FromFile(geom_type_name, filename, fake_this);
}

