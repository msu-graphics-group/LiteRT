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

#include "eye_ray.h"

#include "include/MultiRenderer_gpu_ubo.h"
class MultiRenderer_GPU : public MultiRenderer
{
public:

  MultiRenderer_GPU(uint32_t maxPrimitives) : MultiRenderer(maxPrimitives)
  {
    if(m_pAccelStruct == nullptr)
      m_pAccelStruct = std::make_shared<BVHRT>();
  }
  const char* Name() const override { return "MultiRenderer_GPU";}
  virtual void InitVulkanObjects(VkDevice a_device, VkPhysicalDevice a_physicalDevice, size_t a_maxThreadsCount);

  virtual void SetVulkanContext(vk_utils::VulkanContext a_ctx) { m_ctx = a_ctx; }
  virtual void SetVulkanInOutFor_PackXY(
    uint32_t dummyArgument = 0)
  {
    InitAllGeneratedDescriptorSets_PackXY();
  }

  virtual void SetVulkanInOutFor_CastRayFloatSingle(
    VkBuffer out_colorBuffer,
    size_t   out_colorOffset,
    uint32_t dummyArgument = 0)
  {
    CastRayFloatSingle_local.out_colorBuffer = out_colorBuffer;
    CastRayFloatSingle_local.out_colorOffset = out_colorOffset;
    InitAllGeneratedDescriptorSets_CastRayFloatSingle();
  }

  virtual void SetVulkanInOutFor_CastRaySingle(
    VkBuffer out_colorBuffer,
    size_t   out_colorOffset,
    uint32_t dummyArgument = 0)
  {
    CastRaySingle_local.out_colorBuffer = out_colorBuffer;
    CastRaySingle_local.out_colorOffset = out_colorOffset;
    InitAllGeneratedDescriptorSets_CastRaySingle();
  }

  virtual ~MultiRenderer_GPU();


  virtual void InitMemberBuffers();
  virtual void UpdateAll(std::shared_ptr<vk_utils::ICopyEngine> a_pCopyEngine)
  {
    UpdatePlainMembers(a_pCopyEngine);
    UpdateVectorMembers(a_pCopyEngine);
    UpdateTextureMembers(a_pCopyEngine);
  }

  virtual void UpdatePrefixPointers()
  {
    auto pUnderlyingImpl = dynamic_cast<BVHRT*>(m_pAccelStruct->UnderlyingImpl(0));
    if(pUnderlyingImpl != nullptr)
    {
      m_pAccelStruct_m_GraphicsPrimHeaders = &pUnderlyingImpl->m_GraphicsPrimHeaders;
      m_pAccelStruct_m_GraphicsPrimPoints = &pUnderlyingImpl->m_GraphicsPrimPoints;
      m_pAccelStruct_m_NURBSData = &pUnderlyingImpl->m_NURBSData;
      m_pAccelStruct_m_NURBSHeaders = &pUnderlyingImpl->m_NURBSHeaders;
      m_pAccelStruct_m_NURBS_approxes = &pUnderlyingImpl->m_NURBS_approxes;
      m_pAccelStruct_m_SdfCompactOctreeRotModifiers = &pUnderlyingImpl->m_SdfCompactOctreeRotModifiers;
      m_pAccelStruct_m_SdfCompactOctreeV2Data = &pUnderlyingImpl->m_SdfCompactOctreeV2Data;
      m_pAccelStruct_m_SdfCompactOctreeV3Data = &pUnderlyingImpl->m_SdfCompactOctreeV3Data;
      m_pAccelStruct_m_SdfFrameOctreeNodes = &pUnderlyingImpl->m_SdfFrameOctreeNodes;
      m_pAccelStruct_m_SdfFrameOctreeRoots = &pUnderlyingImpl->m_SdfFrameOctreeRoots;
      m_pAccelStruct_m_SdfFrameOctreeTexNodes = &pUnderlyingImpl->m_SdfFrameOctreeTexNodes;
      m_pAccelStruct_m_SdfFrameOctreeTexRoots = &pUnderlyingImpl->m_SdfFrameOctreeTexRoots;
      m_pAccelStruct_m_SdfGridData = &pUnderlyingImpl->m_SdfGridData;
      m_pAccelStruct_m_SdfGridOffsets = &pUnderlyingImpl->m_SdfGridOffsets;
      m_pAccelStruct_m_SdfGridSizes = &pUnderlyingImpl->m_SdfGridSizes;
      m_pAccelStruct_m_SdfSBSAdaptData = &pUnderlyingImpl->m_SdfSBSAdaptData;
      m_pAccelStruct_m_SdfSBSAdaptDataF = &pUnderlyingImpl->m_SdfSBSAdaptDataF;
      m_pAccelStruct_m_SdfSBSAdaptHeaders = &pUnderlyingImpl->m_SdfSBSAdaptHeaders;
      m_pAccelStruct_m_SdfSBSAdaptNodes = &pUnderlyingImpl->m_SdfSBSAdaptNodes;
      m_pAccelStruct_m_SdfSBSAdaptRoots = &pUnderlyingImpl->m_SdfSBSAdaptRoots;
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
  
  virtual void ReserveEmptyVectors();
  virtual void UpdatePlainMembers(std::shared_ptr<vk_utils::ICopyEngine> a_pCopyEngine);
  virtual void UpdateVectorMembers(std::shared_ptr<vk_utils::ICopyEngine> a_pCopyEngine);
  virtual void UpdateTextureMembers(std::shared_ptr<vk_utils::ICopyEngine> a_pCopyEngine);
  virtual void ReadPlainMembers(std::shared_ptr<vk_utils::ICopyEngine> a_pCopyEngine);
  static VkPhysicalDeviceFeatures2 ListRequiredDeviceFeatures(std::vector<const char*>& deviceExtensions);

  virtual void PackXYCmd(VkCommandBuffer a_commandBuffer, uint tidX, uint tidY);
  virtual void CastRayFloatSingleCmd(VkCommandBuffer a_commandBuffer, uint32_t tidX, float4* out_color);
  virtual void CastRaySingleCmd(VkCommandBuffer a_commandBuffer, uint32_t tidX, uint32_t* out_color);

  void PackXYBlock(uint tidX, uint tidY, uint32_t a_numPasses) override;
  void CastRayFloatSingleBlock(uint32_t tidX, float4* out_color, uint32_t a_numPasses) override;
  void CastRaySingleBlock(uint32_t tidX, uint32_t* out_color, uint32_t a_numPasses) override;

  inline vk_utils::ExecTime GetPackXYExecutionTime() const { return m_exTimePackXY; }
  inline vk_utils::ExecTime GetCastRayFloatSingleExecutionTime() const { return m_exTimeCastRayFloatSingle; }
  inline vk_utils::ExecTime GetCastRaySingleExecutionTime() const { return m_exTimeCastRaySingle; }

  vk_utils::ExecTime m_exTimePackXY;
  vk_utils::ExecTime m_exTimeCastRayFloatSingle;
  vk_utils::ExecTime m_exTimeCastRaySingle;

  virtual void copyKernelFloatCmd(uint32_t length);
  virtual void matMulTransposeCmd(uint32_t A_offset, uint32_t B_offset, uint32_t C_offset, uint32_t A_col_len, uint32_t B_col_len, uint32_t A_row_len);

  virtual void PackXYMegaCmd(uint tidX, uint tidY);
  virtual void CastRayFloatSingleMegaCmd(uint32_t tidX, float4* out_color);
  virtual void CastRaySingleMegaCmd(uint32_t tidX, uint32_t* out_color);

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

  virtual void InitAllGeneratedDescriptorSets_PackXY();
  virtual void InitAllGeneratedDescriptorSets_CastRayFloatSingle();
  virtual void InitAllGeneratedDescriptorSets_CastRaySingle();

  virtual void AssignBuffersToMemory(const std::vector<VkBuffer>& a_buffers, VkDeviceMemory a_mem);

  virtual void AllocMemoryForMemberBuffersAndImages(const std::vector<VkBuffer>& a_buffers, const std::vector<VkImage>& a_image);
  virtual std::string AlterShaderPath(const char* in_shaderPath) { return std::string("") + std::string(in_shaderPath); }
  
  

  struct PackXY_Data
  {
    bool needToClearOutput = true;
  } PackXY_local;

  struct CastRayFloatSingle_Data
  {
    VkBuffer out_colorBuffer = VK_NULL_HANDLE;
    size_t   out_colorOffset = 0;
    bool needToClearOutput = true;
  } CastRayFloatSingle_local;

  struct CastRaySingle_Data
  {
    VkBuffer out_colorBuffer = VK_NULL_HANDLE;
    size_t   out_colorOffset = 0;
    bool needToClearOutput = true;
  } CastRaySingle_local;



  struct MembersDataGPU
  {
    VkBuffer all_referencesBuffer = VK_NULL_HANDLE;
    size_t   all_referencesOffset = 0;
    VkBuffer m_geomOffsetsBuffer = VK_NULL_HANDLE;
    size_t   m_geomOffsetsOffset = 0;
    VkBuffer m_indicesBuffer = VK_NULL_HANDLE;
    size_t   m_indicesOffset = 0;
    VkBuffer m_instanceTransformInvTransposedBuffer = VK_NULL_HANDLE;
    size_t   m_instanceTransformInvTransposedOffset = 0;
    VkBuffer m_lightsBuffer = VK_NULL_HANDLE;
    size_t   m_lightsOffset = 0;
    VkBuffer m_matIdOffsetsBuffer = VK_NULL_HANDLE;
    size_t   m_matIdOffsetsOffset = 0;
    VkBuffer m_matIdbyPrimIdBuffer = VK_NULL_HANDLE;
    size_t   m_matIdbyPrimIdOffset = 0;
    VkBuffer m_materialsBuffer = VK_NULL_HANDLE;
    size_t   m_materialsOffset = 0;
    VkBuffer m_normalsBuffer = VK_NULL_HANDLE;
    size_t   m_normalsOffset = 0;
    VkBuffer m_pAccelStruct_m_GraphicsPrimHeadersBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_GraphicsPrimHeadersOffset = 0;
    VkBuffer m_pAccelStruct_m_GraphicsPrimPointsBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_GraphicsPrimPointsOffset = 0;
    VkBuffer m_pAccelStruct_m_NURBSDataBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_NURBSDataOffset = 0;
    VkBuffer m_pAccelStruct_m_NURBSHeadersBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_NURBSHeadersOffset = 0;
    VkBuffer m_pAccelStruct_m_NURBS_approxesBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_NURBS_approxesOffset = 0;
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
    VkBuffer m_pAccelStruct_m_SdfFrameOctreeTexNodesBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_SdfFrameOctreeTexNodesOffset = 0;
    VkBuffer m_pAccelStruct_m_SdfFrameOctreeTexRootsBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_SdfFrameOctreeTexRootsOffset = 0;
    VkBuffer m_pAccelStruct_m_SdfGridDataBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_SdfGridDataOffset = 0;
    VkBuffer m_pAccelStruct_m_SdfGridOffsetsBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_SdfGridOffsetsOffset = 0;
    VkBuffer m_pAccelStruct_m_SdfGridSizesBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_SdfGridSizesOffset = 0;
    VkBuffer m_pAccelStruct_m_SdfSBSAdaptDataBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_SdfSBSAdaptDataOffset = 0;
    VkBuffer m_pAccelStruct_m_SdfSBSAdaptDataFBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_SdfSBSAdaptDataFOffset = 0;
    VkBuffer m_pAccelStruct_m_SdfSBSAdaptHeadersBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_SdfSBSAdaptHeadersOffset = 0;
    VkBuffer m_pAccelStruct_m_SdfSBSAdaptNodesBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_SdfSBSAdaptNodesOffset = 0;
    VkBuffer m_pAccelStruct_m_SdfSBSAdaptRootsBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_SdfSBSAdaptRootsOffset = 0;
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
    VkBuffer m_verticesBuffer = VK_NULL_HANDLE;
    size_t   m_verticesOffset = 0;
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

  std::vector<GraphicsPrimHeader>* m_pAccelStruct_m_GraphicsPrimHeaders = nullptr;
  std::vector<float4>* m_pAccelStruct_m_GraphicsPrimPoints = nullptr;
  std::vector<float>* m_pAccelStruct_m_NURBSData = nullptr;
  std::vector<NURBSHeader>* m_pAccelStruct_m_NURBSHeaders = nullptr;
  std::vector<float>* m_pAccelStruct_m_NURBS_approxes = nullptr;
  std::vector<int4>* m_pAccelStruct_m_SdfCompactOctreeRotModifiers = nullptr;
  std::vector<uint32_t>* m_pAccelStruct_m_SdfCompactOctreeV2Data = nullptr;
  std::vector<uint32_t>* m_pAccelStruct_m_SdfCompactOctreeV3Data = nullptr;
  std::vector<SdfFrameOctreeNode>* m_pAccelStruct_m_SdfFrameOctreeNodes = nullptr;
  std::vector<uint32_t>* m_pAccelStruct_m_SdfFrameOctreeRoots = nullptr;
  std::vector<SdfFrameOctreeTexNode>* m_pAccelStruct_m_SdfFrameOctreeTexNodes = nullptr;
  std::vector<uint32_t>* m_pAccelStruct_m_SdfFrameOctreeTexRoots = nullptr;
  std::vector<float>* m_pAccelStruct_m_SdfGridData = nullptr;
  std::vector<uint32_t>* m_pAccelStruct_m_SdfGridOffsets = nullptr;
  std::vector<uint3>* m_pAccelStruct_m_SdfGridSizes = nullptr;
  std::vector<uint32_t>* m_pAccelStruct_m_SdfSBSAdaptData = nullptr;
  std::vector<float>* m_pAccelStruct_m_SdfSBSAdaptDataF = nullptr;
  std::vector<SdfSBSAdaptHeader>* m_pAccelStruct_m_SdfSBSAdaptHeaders = nullptr;
  std::vector<SdfSBSAdaptNode>* m_pAccelStruct_m_SdfSBSAdaptNodes = nullptr;
  std::vector<uint32_t>* m_pAccelStruct_m_SdfSBSAdaptRoots = nullptr;
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

  VkPipelineLayout      PackXYMegaLayout   = VK_NULL_HANDLE;
  VkPipeline            PackXYMegaPipeline = VK_NULL_HANDLE;
  VkDescriptorSetLayout PackXYMegaDSLayout = VK_NULL_HANDLE;
  VkDescriptorSetLayout CreatePackXYMegaDSLayout();
  virtual void InitKernel_PackXYMega(const char* a_filePath);
  VkPipelineLayout      CastRayFloatSingleMegaLayout   = VK_NULL_HANDLE;
  VkPipeline            CastRayFloatSingleMegaPipeline = VK_NULL_HANDLE;
  VkDescriptorSetLayout CastRayFloatSingleMegaDSLayout = VK_NULL_HANDLE;
  VkDescriptorSetLayout CreateCastRayFloatSingleMegaDSLayout();
  virtual void InitKernel_CastRayFloatSingleMega(const char* a_filePath);
  VkPipelineLayout      CastRaySingleMegaLayout   = VK_NULL_HANDLE;
  VkPipeline            CastRaySingleMegaPipeline = VK_NULL_HANDLE;
  VkDescriptorSetLayout CastRaySingleMegaDSLayout = VK_NULL_HANDLE;
  VkDescriptorSetLayout CreateCastRaySingleMegaDSLayout();
  virtual void InitKernel_CastRaySingleMega(const char* a_filePath);


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
  VkDescriptorSet  m_allGeneratedDS[3];

  MultiRenderer_GPU_UBO_Data m_uboData;

  constexpr static uint32_t MEMCPY_BLOCK_SIZE = 256;
  constexpr static uint32_t REDUCTION_BLOCK_SIZE = 256;

  virtual void MakeComputePipelineAndLayout(const char* a_shaderPath, const char* a_mainName, const VkSpecializationInfo *a_specInfo, const VkDescriptorSetLayout a_dsLayout,
                                            VkPipelineLayout* pPipelineLayout, VkPipeline* pPipeline);
  virtual void MakeComputePipelineOnly(const char* a_shaderPath, const char* a_mainName, const VkSpecializationInfo *a_specInfo, const VkDescriptorSetLayout a_dsLayout, VkPipelineLayout pipelineLayout,
                                       VkPipeline* pPipeline);

  std::vector<VkPipelineLayout> m_allCreatedPipelineLayouts; ///<! remenber them here to delete later
  std::vector<VkPipeline>       m_allCreatedPipelines;       ///<! remenber them here to delete later
public:

  struct MegaKernelIsEnabled
  {
    bool enablePackXYMega = true;
    bool enableCastRayFloatSingleMega = true;
    bool enableCastRaySingleMega = true;
    bool dummy = 0;
  };

  static MegaKernelIsEnabled  m_megaKernelFlags;
  static MegaKernelIsEnabled& EnabledPipelines() { return m_megaKernelFlags; }
  VkQueryPool m_queryPoolTimestamps = VK_NULL_HANDLE;
  uint32_t    m_timestampPoolSize = 0;
  float       m_timestampPeriod = 1.0f;
  void        ResetTimeStampMeasurements();
  void        AccumTimeStampMeasurements(uint32_t a_start, uint32_t a_size);
  struct PerKernelMeasure 
  {
    float avg = 0.0f;
    float min = 1e20f;
    float max = 0.0f;
    int   count = 0;
  };
  std::unordered_map<std::string, PerKernelMeasure> m_kernelTimings;
  std::vector<std::string>                          m_tsIdToKernelName;
};


