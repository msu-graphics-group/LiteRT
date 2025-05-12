#pragma once

#include <vector>
#include <memory>
#include <string>
#include <unordered_map>
#include <array>

#include "vk_pipeline.h"
#include "vk_buffers.h"
#include "vk_utils.h"
#include "vk_copy.h"
#include "vk_context.h"
#include "Image2d.h"
using LiteImage::Image2D;
using LiteImage::Sampler;
using namespace LiteMath;

#include "integrator_pt.h"

#include "include/Integrator_generated_ubo.h"
class Integrator_Generated : public Integrator
{
public:

  Integrator_Generated(int a_maxThreads, std::vector<uint32_t> a_features) : Integrator(a_maxThreads, a_features)
  {
    if(m_pAccelStruct == nullptr)
      m_pAccelStruct = std::make_shared<BVHRT>();
  }
  virtual void InitVulkanObjects(VkDevice a_device, VkPhysicalDevice a_physicalDevice, size_t a_maxThreadsCount);

  virtual void SetVulkanContext(vk_utils::VulkanContext a_ctx) { m_ctx = a_ctx; }
  virtual void SetVulkanInOutFor_RayTrace(
    VkBuffer out_colorBuffer,
    size_t   out_colorOffset,
    uint32_t dummyArgument = 0)
  {
    RayTrace_local.out_colorBuffer = out_colorBuffer;
    RayTrace_local.out_colorOffset = out_colorOffset;
    InitAllGeneratedDescriptorSets_RayTrace();
  }

  virtual void SetVulkanInOutFor_CastSingleRay(
    VkBuffer out_colorBuffer,
    size_t   out_colorOffset,
    uint32_t dummyArgument = 0)
  {
    CastSingleRay_local.out_colorBuffer = out_colorBuffer;
    CastSingleRay_local.out_colorOffset = out_colorOffset;
    InitAllGeneratedDescriptorSets_CastSingleRay();
  }

  virtual void SetVulkanInOutFor_PackXY(
    uint32_t dummyArgument = 0)
  {
    InitAllGeneratedDescriptorSets_PackXY();
  }

  virtual void SetVulkanInOutFor_PathTraceFromInputRays(
    VkBuffer in_rayPosAndNearBuffer,
    size_t   in_rayPosAndNearOffset,
    VkBuffer in_rayDirAndFarBuffer,
    size_t   in_rayDirAndFarOffset,
    VkBuffer out_colorBuffer,
    size_t   out_colorOffset,
    uint32_t dummyArgument = 0)
  {
    PathTraceFromInputRays_local.in_rayPosAndNearBuffer = in_rayPosAndNearBuffer;
    PathTraceFromInputRays_local.in_rayPosAndNearOffset = in_rayPosAndNearOffset;
    PathTraceFromInputRays_local.in_rayDirAndFarBuffer = in_rayDirAndFarBuffer;
    PathTraceFromInputRays_local.in_rayDirAndFarOffset = in_rayDirAndFarOffset;
    PathTraceFromInputRays_local.out_colorBuffer = out_colorBuffer;
    PathTraceFromInputRays_local.out_colorOffset = out_colorOffset;
    InitAllGeneratedDescriptorSets_PathTraceFromInputRays();
  }

  virtual void SetVulkanInOutFor_PathTrace(
    VkBuffer out_colorBuffer,
    size_t   out_colorOffset,
    uint32_t dummyArgument = 0)
  {
    PathTrace_local.out_colorBuffer = out_colorBuffer;
    PathTrace_local.out_colorOffset = out_colorOffset;
    InitAllGeneratedDescriptorSets_PathTrace();
  }

  virtual void SetVulkanInOutFor_NaivePathTrace(
    VkBuffer out_colorBuffer,
    size_t   out_colorOffset,
    uint32_t dummyArgument = 0)
  {
    NaivePathTrace_local.out_colorBuffer = out_colorBuffer;
    NaivePathTrace_local.out_colorOffset = out_colorOffset;
    InitAllGeneratedDescriptorSets_NaivePathTrace();
  }

  virtual ~Integrator_Generated();


  virtual void InitMemberBuffers();
  virtual void UpdateAll(std::shared_ptr<vk_utils::ICopyEngine> a_pCopyEngine)
  {
    UpdatePlainMembers(a_pCopyEngine);
    UpdateVectorMembers(a_pCopyEngine);
    UpdateTextureMembers(a_pCopyEngine);
  }
  void Update_m_lights(size_t a_first, size_t a_size) override;
  void Update_m_matIdOffsets() override;
  void Update_m_materials(size_t a_first, size_t a_size) override;

  virtual void UpdatePrefixPointers()
  {
    auto pUnderlyingImpl = dynamic_cast<BVHRT*>(m_pAccelStruct->UnderlyingImpl(0));
    if(pUnderlyingImpl != nullptr)
    {
      m_pAccelStruct_m_CatmulClarkHeaders = &pUnderlyingImpl->m_CatmulClarkHeaders;
      m_pAccelStruct_m_NURBSData = &pUnderlyingImpl->m_NURBSData;
      m_pAccelStruct_m_NURBSHeaders = &pUnderlyingImpl->m_NURBSHeaders;
      m_pAccelStruct_m_NURBS_approxes = &pUnderlyingImpl->m_NURBS_approxes;
      m_pAccelStruct_m_RibbonHeaders = &pUnderlyingImpl->m_RibbonHeaders;
      m_pAccelStruct_m_SdfCompactOctreeRotModifiers = &pUnderlyingImpl->m_SdfCompactOctreeRotModifiers;
      m_pAccelStruct_m_SdfCompactOctreeV2Data = &pUnderlyingImpl->m_SdfCompactOctreeV2Data;
      m_pAccelStruct_m_SdfCompactOctreeV3Data = &pUnderlyingImpl->m_SdfCompactOctreeV3Data;
      m_pAccelStruct_m_SdfFrameOctreeNodes = &pUnderlyingImpl->m_SdfFrameOctreeNodes;
      m_pAccelStruct_m_SdfFrameOctreeRoots = &pUnderlyingImpl->m_SdfFrameOctreeRoots;
      m_pAccelStruct_m_SdfSBSData = &pUnderlyingImpl->m_SdfSBSData;
      m_pAccelStruct_m_SdfSBSDataF = &pUnderlyingImpl->m_SdfSBSDataF;
      m_pAccelStruct_m_SdfSBSHeaders = &pUnderlyingImpl->m_SdfSBSHeaders;
      m_pAccelStruct_m_SdfSBSNodes = &pUnderlyingImpl->m_SdfSBSNodes;
      m_pAccelStruct_m_SdfSBSRoots = &pUnderlyingImpl->m_SdfSBSRoots;
      m_pAccelStruct_m_SdfSVSNodes = &pUnderlyingImpl->m_SdfSVSNodes;
      m_pAccelStruct_m_SdfSVSRoots = &pUnderlyingImpl->m_SdfSVSRoots;
      m_pAccelStruct_m_abstractObjectPtrs = &pUnderlyingImpl->m_abstractObjectPtrs;
      m_pAccelStruct_m_allNodePairs = &pUnderlyingImpl->m_allNodePairs;
      m_pAccelStruct_m_geomData = &pUnderlyingImpl->m_geomData;
      m_pAccelStruct_m_indices = &pUnderlyingImpl->m_indices;
      m_pAccelStruct_m_instanceData = &pUnderlyingImpl->m_instanceData;
      m_pAccelStruct_m_nodesTLAS = &pUnderlyingImpl->m_nodesTLAS;
      m_pAccelStruct_m_origNodes = &pUnderlyingImpl->m_origNodes;
      m_pAccelStruct_m_primIdCount = &pUnderlyingImpl->m_primIdCount;
      m_pAccelStruct_m_primIndices = &pUnderlyingImpl->m_primIndices;
      m_pAccelStruct_m_vertNorm = &pUnderlyingImpl->m_vertNorm;
      m_pAccelStruct_m_vertPos = &pUnderlyingImpl->m_vertPos;
      m_pAccelStruct_startEnd = &pUnderlyingImpl->startEnd;
    }
  }
  std::shared_ptr<vk_utils::ICopyEngine> m_pLastCopyHelper = nullptr;
  virtual void CommitDeviceData(std::shared_ptr<vk_utils::ICopyEngine> a_pCopyHelper) // you have to define this virtual function in the original imput class
  {
    UpdatePrefixPointers();
    ReserveEmptyVectors();
    InitMemberBuffers();
    UpdateAll(a_pCopyHelper);
    m_pLastCopyHelper = a_pCopyHelper;
  }
  void CommitDeviceData() override { CommitDeviceData(m_ctx.pCopyHelper); }
  void GetExecutionTime(const char* a_funcName, float a_out[4]) override;
  void UpdateMembersPlainData() override { UpdatePlainMembers(m_ctx.pCopyHelper); }
  
  virtual void ReserveEmptyVectors();
  virtual void UpdatePlainMembers(std::shared_ptr<vk_utils::ICopyEngine> a_pCopyEngine);
  virtual void UpdateVectorMembers(std::shared_ptr<vk_utils::ICopyEngine> a_pCopyEngine);
  virtual void UpdateTextureMembers(std::shared_ptr<vk_utils::ICopyEngine> a_pCopyEngine);
  virtual void ReadPlainMembers(std::shared_ptr<vk_utils::ICopyEngine> a_pCopyEngine);
  static VkPhysicalDeviceFeatures2 ListRequiredDeviceFeatures(std::vector<const char*>& deviceExtensions);

  virtual void RayTraceCmd(VkCommandBuffer a_commandBuffer, uint tid, uint channels, float* out_color);
  virtual void CastSingleRayCmd(VkCommandBuffer a_commandBuffer, uint tid, float* out_color);
  virtual void PackXYCmd(VkCommandBuffer a_commandBuffer, uint tidX, uint tidY);
  virtual void PathTraceFromInputRaysCmd(VkCommandBuffer a_commandBuffer, uint tid, uint channels, const RayPosAndW* in_rayPosAndNear, const RayDirAndT* in_rayDirAndFar, float* out_color);
  virtual void PathTraceCmd(VkCommandBuffer a_commandBuffer, uint tid, uint channels, float* out_color);
  virtual void NaivePathTraceCmd(VkCommandBuffer a_commandBuffer, uint tid, uint channels, float* out_color);

  void RayTraceBlock(uint tid, uint channels, float* out_color, uint32_t a_numPasses) override;
  void CastSingleRayBlock(uint tid, float* out_color, uint32_t a_numPasses) override;
  void PackXYBlock(uint tidX, uint tidY, uint32_t a_numPasses) override;
  void PathTraceFromInputRaysBlock(uint tid, uint channels, const RayPosAndW* in_rayPosAndNear, const RayDirAndT* in_rayDirAndFar, float* out_color, uint32_t a_numPasses) override;
  void PathTraceBlock(uint tid, uint channels, float* out_color, uint32_t a_numPasses) override;
  void NaivePathTraceBlock(uint tid, uint channels, float* out_color, uint32_t a_numPasses) override;

  inline vk_utils::ExecTime GetRayTraceExecutionTime() const { return m_exTimeRayTrace; }
  inline vk_utils::ExecTime GetCastSingleRayExecutionTime() const { return m_exTimeCastSingleRay; }
  inline vk_utils::ExecTime GetPackXYExecutionTime() const { return m_exTimePackXY; }
  inline vk_utils::ExecTime GetPathTraceFromInputRaysExecutionTime() const { return m_exTimePathTraceFromInputRays; }
  inline vk_utils::ExecTime GetPathTraceExecutionTime() const { return m_exTimePathTrace; }
  inline vk_utils::ExecTime GetNaivePathTraceExecutionTime() const { return m_exTimeNaivePathTrace; }

  vk_utils::ExecTime m_exTimeRayTrace;
  vk_utils::ExecTime m_exTimeCastSingleRay;
  vk_utils::ExecTime m_exTimePackXY;
  vk_utils::ExecTime m_exTimePathTraceFromInputRays;
  vk_utils::ExecTime m_exTimePathTrace;
  vk_utils::ExecTime m_exTimeNaivePathTrace;

  virtual void copyKernelFloatCmd(uint32_t length);
  virtual void matMulTransposeCmd(uint32_t A_offset, uint32_t B_offset, uint32_t C_offset, uint32_t A_col_len, uint32_t B_col_len, uint32_t A_row_len);

  virtual void RayTraceMegaCmd(uint tid, uint channels, float* out_color);
  virtual void CastSingleRayMegaCmd(uint tid, float* out_color);
  virtual void PackXYMegaCmd(uint tidX, uint tidY);
  virtual void PathTraceFromInputRaysMegaCmd(uint tid, uint channels, const RayPosAndW* in_rayPosAndNear, const RayDirAndT* in_rayDirAndFar, float* out_color);
  virtual void PathTraceMegaCmd(uint tid, uint channels, float* out_color);
  virtual void NaivePathTraceMegaCmd(uint tid, uint channels, float* out_color);

  struct MemLoc
  {
    VkDeviceMemory memObject = VK_NULL_HANDLE;
    size_t         memOffset = 0;
    size_t         allocId   = 0;
  };

  virtual MemLoc AllocAndBind(const std::vector<VkBuffer>& a_buffers, VkMemoryAllocateFlags a_flags = 0); ///< replace this function to apply custom allocator
  virtual MemLoc AllocAndBind(const std::vector<VkImage>& a_image,    VkMemoryAllocateFlags a_flags = 0);    ///< replace this function to apply custom allocator
  virtual void   FreeAllAllocations(std::vector<MemLoc>& a_memLoc);    ///< replace this function to apply custom allocator

protected:

  VkPhysicalDevice           physicalDevice = VK_NULL_HANDLE;
  VkDevice                   device         = VK_NULL_HANDLE;
  vk_utils::VulkanContext    m_ctx          = {};
  VkCommandBuffer            m_currCmdBuffer   = VK_NULL_HANDLE;
  uint32_t                   m_currThreadFlags = 0;
  std::vector<MemLoc>        m_allMems;
  VkPhysicalDeviceProperties m_devProps;

  VkBufferMemoryBarrier BarrierForClearFlags(VkBuffer a_buffer);
  VkBufferMemoryBarrier BarrierForSingleBuffer(VkBuffer a_buffer);
  void BarriersForSeveralBuffers(VkBuffer* a_inBuffers, VkBufferMemoryBarrier* a_outBarriers, uint32_t a_buffersNum);

  virtual void InitHelpers();
  virtual void InitBuffers(size_t a_maxThreadsCount, bool a_tempBuffersOverlay = true);
  virtual void InitKernels(const char* a_filePath);
  virtual void AllocateAllDescriptorSets();

  virtual void InitAllGeneratedDescriptorSets_RayTrace();
  virtual void InitAllGeneratedDescriptorSets_CastSingleRay();
  virtual void InitAllGeneratedDescriptorSets_PackXY();
  virtual void InitAllGeneratedDescriptorSets_PathTraceFromInputRays();
  virtual void InitAllGeneratedDescriptorSets_PathTrace();
  virtual void InitAllGeneratedDescriptorSets_NaivePathTrace();

  virtual void AssignBuffersToMemory(const std::vector<VkBuffer>& a_buffers, VkDeviceMemory a_mem);

  virtual void AllocMemoryForMemberBuffersAndImages(const std::vector<VkBuffer>& a_buffers, const std::vector<VkImage>& a_image);
  virtual std::string AlterShaderPath(const char* in_shaderPath) { return GetResourcesRootDir() + "/" + std::string("") + std::string(in_shaderPath); }
  
  

  struct RayTrace_Data
  {
    VkBuffer out_colorBuffer = VK_NULL_HANDLE;
    size_t   out_colorOffset = 0;
    bool needToClearOutput = true;
  } RayTrace_local;

  struct CastSingleRay_Data
  {
    VkBuffer out_colorBuffer = VK_NULL_HANDLE;
    size_t   out_colorOffset = 0;
    bool needToClearOutput = true;
  } CastSingleRay_local;

  struct PackXY_Data
  {
    bool needToClearOutput = true;
  } PackXY_local;

  struct PathTraceFromInputRays_Data
  {
    VkBuffer in_rayPosAndNearBuffer = VK_NULL_HANDLE;
    size_t   in_rayPosAndNearOffset = 0;
    VkBuffer in_rayDirAndFarBuffer = VK_NULL_HANDLE;
    size_t   in_rayDirAndFarOffset = 0;
    VkBuffer out_colorBuffer = VK_NULL_HANDLE;
    size_t   out_colorOffset = 0;
    bool needToClearOutput = true;
  } PathTraceFromInputRays_local;

  struct PathTrace_Data
  {
    VkBuffer out_colorBuffer = VK_NULL_HANDLE;
    size_t   out_colorOffset = 0;
    bool needToClearOutput = true;
  } PathTrace_local;

  struct NaivePathTrace_Data
  {
    VkBuffer out_colorBuffer = VK_NULL_HANDLE;
    size_t   out_colorOffset = 0;
    bool needToClearOutput = true;
  } NaivePathTrace_local;



  struct MembersDataGPU
  {
    VkBuffer all_referencesBuffer = VK_NULL_HANDLE;
    size_t   all_referencesOffset = 0;
    VkBuffer m_allRemapListsBuffer = VK_NULL_HANDLE;
    size_t   m_allRemapListsOffset = 0;
    VkBuffer m_allRemapListsOffsetsBuffer = VK_NULL_HANDLE;
    size_t   m_allRemapListsOffsetsOffset = 0;
    VkBuffer m_cie_xBuffer = VK_NULL_HANDLE;
    size_t   m_cie_xOffset = 0;
    VkBuffer m_cie_yBuffer = VK_NULL_HANDLE;
    size_t   m_cie_yOffset = 0;
    VkBuffer m_cie_zBuffer = VK_NULL_HANDLE;
    size_t   m_cie_zOffset = 0;
    VkBuffer m_films_eta_k_vecBuffer = VK_NULL_HANDLE;
    size_t   m_films_eta_k_vecOffset = 0;
    VkBuffer m_films_spec_id_vecBuffer = VK_NULL_HANDLE;
    size_t   m_films_spec_id_vecOffset = 0;
    VkBuffer m_instIdToLightInstIdBuffer = VK_NULL_HANDLE;
    size_t   m_instIdToLightInstIdOffset = 0;
    VkBuffer m_lightsBuffer = VK_NULL_HANDLE;
    size_t   m_lightsOffset = 0;
    VkBuffer m_linesBuffer = VK_NULL_HANDLE;
    size_t   m_linesOffset = 0;
    VkBuffer m_matIdByPrimIdBuffer = VK_NULL_HANDLE;
    size_t   m_matIdByPrimIdOffset = 0;
    VkBuffer m_matIdOffsetsBuffer = VK_NULL_HANDLE;
    size_t   m_matIdOffsetsOffset = 0;
    VkBuffer m_materialsBuffer = VK_NULL_HANDLE;
    size_t   m_materialsOffset = 0;
    VkBuffer m_normMatricesBuffer = VK_NULL_HANDLE;
    size_t   m_normMatricesOffset = 0;
    VkBuffer m_normMatrices2Buffer = VK_NULL_HANDLE;
    size_t   m_normMatrices2Offset = 0;
    VkBuffer m_pAccelStruct_m_CatmulClarkHeadersBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_CatmulClarkHeadersOffset = 0;
    VkBuffer m_pAccelStruct_m_NURBSDataBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_NURBSDataOffset = 0;
    VkBuffer m_pAccelStruct_m_NURBSHeadersBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_NURBSHeadersOffset = 0;
    VkBuffer m_pAccelStruct_m_NURBS_approxesBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_NURBS_approxesOffset = 0;
    VkBuffer m_pAccelStruct_m_RibbonHeadersBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_RibbonHeadersOffset = 0;
    VkBuffer m_pAccelStruct_m_SdfCompactOctreeRotModifiersBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_SdfCompactOctreeRotModifiersOffset = 0;
    VkBuffer m_pAccelStruct_m_SdfCompactOctreeV2DataBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_SdfCompactOctreeV2DataOffset = 0;
    VkBuffer m_pAccelStruct_m_SdfCompactOctreeV3DataBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_SdfCompactOctreeV3DataOffset = 0;
    VkBuffer m_pAccelStruct_m_SdfFrameOctreeNodesBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_SdfFrameOctreeNodesOffset = 0;
    VkBuffer m_pAccelStruct_m_SdfFrameOctreeRootsBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_SdfFrameOctreeRootsOffset = 0;
    VkBuffer m_pAccelStruct_m_SdfSBSDataBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_SdfSBSDataOffset = 0;
    VkBuffer m_pAccelStruct_m_SdfSBSDataFBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_SdfSBSDataFOffset = 0;
    VkBuffer m_pAccelStruct_m_SdfSBSHeadersBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_SdfSBSHeadersOffset = 0;
    VkBuffer m_pAccelStruct_m_SdfSBSNodesBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_SdfSBSNodesOffset = 0;
    VkBuffer m_pAccelStruct_m_SdfSBSRootsBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_SdfSBSRootsOffset = 0;
    VkBuffer m_pAccelStruct_m_SdfSVSNodesBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_SdfSVSNodesOffset = 0;
    VkBuffer m_pAccelStruct_m_SdfSVSRootsBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_SdfSVSRootsOffset = 0;
    VkBuffer m_pAccelStruct_m_abstractObjectPtrsBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_abstractObjectPtrsOffset = 0;
    VkBuffer m_pAccelStruct_m_abstractObjectPtrs_dataSBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_abstractObjectPtrs_dataSOffset = 0;
    VkBuffer m_pAccelStruct_m_abstractObjectPtrs_dataVBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_abstractObjectPtrs_dataVOffset = 0;
    VkBuffer m_pAccelStruct_m_allNodePairsBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_allNodePairsOffset = 0;
    VkBuffer m_pAccelStruct_m_geomDataBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_geomDataOffset = 0;
    VkBuffer m_pAccelStruct_m_indicesBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_indicesOffset = 0;
    VkBuffer m_pAccelStruct_m_instanceDataBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_instanceDataOffset = 0;
    VkBuffer m_pAccelStruct_m_nodesTLASBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_nodesTLASOffset = 0;
    VkBuffer m_pAccelStruct_m_origNodesBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_origNodesOffset = 0;
    VkBuffer m_pAccelStruct_m_primIdCountBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_primIdCountOffset = 0;
    VkBuffer m_pAccelStruct_m_primIndicesBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_primIndicesOffset = 0;
    VkBuffer m_pAccelStruct_m_vertNormBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_vertNormOffset = 0;
    VkBuffer m_pAccelStruct_m_vertPosBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_vertPosOffset = 0;
    VkBuffer m_pAccelStruct_startEndBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_startEndOffset = 0;
    VkBuffer m_packedXYBuffer = VK_NULL_HANDLE;
    size_t   m_packedXYOffset = 0;
    VkBuffer m_pdfLightDataBuffer = VK_NULL_HANDLE;
    size_t   m_pdfLightDataOffset = 0;
    VkBuffer m_precomp_coat_transmittanceBuffer = VK_NULL_HANDLE;
    size_t   m_precomp_coat_transmittanceOffset = 0;
    VkBuffer m_precomp_thin_filmsBuffer = VK_NULL_HANDLE;
    size_t   m_precomp_thin_filmsOffset = 0;
    VkBuffer m_randomGensBuffer = VK_NULL_HANDLE;
    size_t   m_randomGensOffset = 0;
    VkBuffer m_remapInstBuffer = VK_NULL_HANDLE;
    size_t   m_remapInstOffset = 0;
    VkBuffer m_spec_offset_szBuffer = VK_NULL_HANDLE;
    size_t   m_spec_offset_szOffset = 0;
    VkBuffer m_spec_tex_ids_wavelengthsBuffer = VK_NULL_HANDLE;
    size_t   m_spec_tex_ids_wavelengthsOffset = 0;
    VkBuffer m_spec_tex_offset_szBuffer = VK_NULL_HANDLE;
    size_t   m_spec_tex_offset_szOffset = 0;
    VkBuffer m_spec_valuesBuffer = VK_NULL_HANDLE;
    size_t   m_spec_valuesOffset = 0;
    VkBuffer m_triIndicesBuffer = VK_NULL_HANDLE;
    size_t   m_triIndicesOffset = 0;
    VkBuffer m_vNorm4fBuffer = VK_NULL_HANDLE;
    size_t   m_vNorm4fOffset = 0;
    VkBuffer m_vTang4fBuffer = VK_NULL_HANDLE;
    size_t   m_vTang4fOffset = 0;
    VkBuffer m_vertOffsetBuffer = VK_NULL_HANDLE;
    size_t   m_vertOffsetOffset = 0;
    std::vector<VkImage>     m_texturesArrayTexture;
    std::vector<VkImageView> m_texturesArrayView   ;
    std::vector<VkSampler>   m_texturesArraySampler; ///<! samplers for texture arrays are always used
    size_t                   m_texturesArrayMaxSize; ///<! used when texture array size is not known after constructor of base class is finished
  } m_vdata;
  std::vector<LiteMath::uint2> m_pAccelStruct_m_abstractObjectPtrs_vtable;
  std::vector<uint8_t>         m_pAccelStruct_m_abstractObjectPtrs_dataV;
  std::unordered_map<uint32_t, std::vector<uint8_t> > m_pAccelStruct_m_abstractObjectPtrs_sorted;
  std::unordered_map<uint32_t, size_t>                m_pAccelStruct_m_abstractObjectPtrs_obj_storage_offsets;
  struct AllBufferReferences 
  {
    VkDeviceAddress GeomDataTriangleAddress;
    VkDeviceAddress GeomDataSdfGridAddress;
    VkDeviceAddress GeomDataSdfNodeAddress;
    VkDeviceAddress GeomDataSdfBrickAddress;
    VkDeviceAddress GeomDataRFAddress;
    VkDeviceAddress GeomDataGSAddress;
    VkDeviceAddress GeomDataSdfAdaptBrickAddress;
    VkDeviceAddress GeomDataNURBSAddress;
    VkDeviceAddress GeomDataGraphicsPrimAddress;
    VkDeviceAddress GeomDataCOctreeSimpleAddress;
    VkDeviceAddress GeomDataCOctreeBrickedAddress;
    VkDeviceAddress GeomDataCatmulClarkAddress;
    VkDeviceAddress GeomDataRibbonAddress;
  };
  std::vector<AllBufferReferences> all_references;

  std::vector<CatmulClarkHeader>* m_pAccelStruct_m_CatmulClarkHeaders = nullptr;
  std::vector<float>* m_pAccelStruct_m_NURBSData = nullptr;
  std::vector<NURBSHeader>* m_pAccelStruct_m_NURBSHeaders = nullptr;
  std::vector<float>* m_pAccelStruct_m_NURBS_approxes = nullptr;
  std::vector<RibbonHeader>* m_pAccelStruct_m_RibbonHeaders = nullptr;
  std::vector<int4>* m_pAccelStruct_m_SdfCompactOctreeRotModifiers = nullptr;
  std::vector<uint32_t>* m_pAccelStruct_m_SdfCompactOctreeV2Data = nullptr;
  std::vector<uint32_t>* m_pAccelStruct_m_SdfCompactOctreeV3Data = nullptr;
  std::vector<SdfFrameOctreeNode>* m_pAccelStruct_m_SdfFrameOctreeNodes = nullptr;
  std::vector<uint32_t>* m_pAccelStruct_m_SdfFrameOctreeRoots = nullptr;
  std::vector<uint32_t>* m_pAccelStruct_m_SdfSBSData = nullptr;
  std::vector<float>* m_pAccelStruct_m_SdfSBSDataF = nullptr;
  std::vector<SdfSBSHeader>* m_pAccelStruct_m_SdfSBSHeaders = nullptr;
  std::vector<SdfSBSNode>* m_pAccelStruct_m_SdfSBSNodes = nullptr;
  std::vector<uint32_t>* m_pAccelStruct_m_SdfSBSRoots = nullptr;
  std::vector<SdfSVSNode>* m_pAccelStruct_m_SdfSVSNodes = nullptr;
  std::vector<uint32_t>* m_pAccelStruct_m_SdfSVSRoots = nullptr;
  std::vector<AbstractObject *>* m_pAccelStruct_m_abstractObjectPtrs = nullptr;
  std::vector<BVHNodePair>* m_pAccelStruct_m_allNodePairs = nullptr;
  std::vector<GeomData>* m_pAccelStruct_m_geomData = nullptr;
  std::vector<uint32_t>* m_pAccelStruct_m_indices = nullptr;
  std::vector<InstanceData>* m_pAccelStruct_m_instanceData = nullptr;
  std::vector<BVHNode>* m_pAccelStruct_m_nodesTLAS = nullptr;
  std::vector<BVHNode>* m_pAccelStruct_m_origNodes = nullptr;
  std::vector<uint32_t>* m_pAccelStruct_m_primIdCount = nullptr;
  std::vector<uint32_t>* m_pAccelStruct_m_primIndices = nullptr;
  std::vector<float4>* m_pAccelStruct_m_vertNorm = nullptr;
  std::vector<float4>* m_pAccelStruct_m_vertPos = nullptr;
  std::vector<uint2>* m_pAccelStruct_startEnd = nullptr;

  VkImage     CreateTexture2D(const int a_width, const int a_height, VkFormat a_format, VkImageUsageFlags a_usage);
  VkSampler   CreateSampler(const Sampler& a_sampler);
  VkImageView CreateView(VkFormat a_format, VkImage a_image);
  struct TexAccessPair
  {
    TexAccessPair() : image(VK_NULL_HANDLE), access(0) {}
    TexAccessPair(VkImage a_image, VkAccessFlags a_access) : image(a_image), access(a_access) {}
    VkImage image;
    VkAccessFlags access;
  };
  void TrackTextureAccess(const std::vector<TexAccessPair>& a_pairs, std::unordered_map<uint64_t, VkAccessFlags>& a_currImageFlags);
  VkBufferMemoryBarrier BarrierForObjCounters(VkBuffer a_buffer);
  size_t m_maxThreadCount = 0;
  VkBuffer m_classDataBuffer = VK_NULL_HANDLE;

  VkPipelineLayout      RayTraceMegaLayout   = VK_NULL_HANDLE;
  VkPipeline            RayTraceMegaPipeline = VK_NULL_HANDLE;
  VkDescriptorSetLayout RayTraceMegaDSLayout = VK_NULL_HANDLE;
  VkDescriptorSetLayout CreateRayTraceMegaDSLayout();
  virtual void InitKernel_RayTraceMega(const char* a_filePath);
  VkPipelineLayout      CastSingleRayMegaLayout   = VK_NULL_HANDLE;
  VkPipeline            CastSingleRayMegaPipeline = VK_NULL_HANDLE;
  VkDescriptorSetLayout CastSingleRayMegaDSLayout = VK_NULL_HANDLE;
  VkDescriptorSetLayout CreateCastSingleRayMegaDSLayout();
  virtual void InitKernel_CastSingleRayMega(const char* a_filePath);
  VkPipelineLayout      PackXYMegaLayout   = VK_NULL_HANDLE;
  VkPipeline            PackXYMegaPipeline = VK_NULL_HANDLE;
  VkDescriptorSetLayout PackXYMegaDSLayout = VK_NULL_HANDLE;
  VkDescriptorSetLayout CreatePackXYMegaDSLayout();
  virtual void InitKernel_PackXYMega(const char* a_filePath);
  VkPipelineLayout      PathTraceFromInputRaysMegaLayout   = VK_NULL_HANDLE;
  VkPipeline            PathTraceFromInputRaysMegaPipeline = VK_NULL_HANDLE;
  VkDescriptorSetLayout PathTraceFromInputRaysMegaDSLayout = VK_NULL_HANDLE;
  VkDescriptorSetLayout CreatePathTraceFromInputRaysMegaDSLayout();
  virtual void InitKernel_PathTraceFromInputRaysMega(const char* a_filePath);
  VkPipelineLayout      PathTraceMegaLayout   = VK_NULL_HANDLE;
  VkPipeline            PathTraceMegaPipeline = VK_NULL_HANDLE;
  VkDescriptorSetLayout PathTraceMegaDSLayout = VK_NULL_HANDLE;
  VkDescriptorSetLayout CreatePathTraceMegaDSLayout();
  virtual void InitKernel_PathTraceMega(const char* a_filePath);
  VkPipelineLayout      NaivePathTraceMegaLayout   = VK_NULL_HANDLE;
  VkPipeline            NaivePathTraceMegaPipeline = VK_NULL_HANDLE;
  VkDescriptorSetLayout NaivePathTraceMegaDSLayout = VK_NULL_HANDLE;
  VkDescriptorSetLayout CreateNaivePathTraceMegaDSLayout();
  virtual void InitKernel_NaivePathTraceMega(const char* a_filePath);


  virtual VkBufferUsageFlags GetAdditionalFlagsForUBO() const;
  virtual uint32_t           GetDefaultMaxTextures() const;

  VkPipelineLayout      copyKernelFloatLayout   = VK_NULL_HANDLE;
  VkPipeline            copyKernelFloatPipeline = VK_NULL_HANDLE;
  VkDescriptorSetLayout copyKernelFloatDSLayout = VK_NULL_HANDLE;
  VkDescriptorSetLayout CreatecopyKernelFloatDSLayout();

  VkPipelineLayout      matMulTransposeLayout   = VK_NULL_HANDLE;
  VkPipeline            matMulTransposePipeline = VK_NULL_HANDLE;
  VkDescriptorSetLayout matMulTransposeDSLayout = VK_NULL_HANDLE;
  VkDescriptorSetLayout CreatematMulTransposeDSLayout();

  VkDescriptorPool m_dsPool = VK_NULL_HANDLE;
  VkDescriptorSet  m_allGeneratedDS[6];

  Integrator_Generated_UBO_Data m_uboData;

  constexpr static uint32_t MEMCPY_BLOCK_SIZE = 256;
  constexpr static uint32_t REDUCTION_BLOCK_SIZE = 256;

  virtual void MakeComputePipelineAndLayout(const char* a_shaderPath, const char* a_mainName, const VkSpecializationInfo *a_specInfo, const VkDescriptorSetLayout a_dsLayout,
                                            VkPipelineLayout* pPipelineLayout, VkPipeline* pPipeline);
  virtual void MakeComputePipelineOnly(const char* a_shaderPath, const char* a_mainName, const VkSpecializationInfo *a_specInfo, const VkDescriptorSetLayout a_dsLayout, VkPipelineLayout pipelineLayout,
                                       VkPipeline* pPipeline);

  std::vector<VkPipelineLayout> m_allCreatedPipelineLayouts; ///<! remenber them here to delete later
  std::vector<VkPipeline>       m_allCreatedPipelines;       ///<! remenber them here to delete later
  std::vector<uint32_t>                  m_allSpecConstVals; ///<! require user to define "ListRequiredFeatures" func.
  std::vector<VkSpecializationMapEntry>  m_allSpecConstInfo;
  VkSpecializationInfo                   m_allSpecInfo;
  const VkSpecializationInfo*            GetAllSpecInfo();
public:

  struct MegaKernelIsEnabled
  {
    bool enableRayTraceMega = true;
    bool enableCastSingleRayMega = true;
    bool enablePackXYMega = true;
    bool enablePathTraceFromInputRaysMega = true;
    bool enablePathTraceMega = true;
    bool enableNaivePathTraceMega = true;
    bool dummy = 0;
  };

  static MegaKernelIsEnabled  m_megaKernelFlags;
  static MegaKernelIsEnabled& EnabledPipelines() { return m_megaKernelFlags; }
};


