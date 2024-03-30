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


#include "eye_ray.h"

#include "include/EyeRayCaster_gpu_ubo.h"
class EyeRayCaster_GPU : public EyeRayCaster
{
public:

  EyeRayCaster_GPU() 
  {
    if(m_pAccelStruct == nullptr)
      m_pAccelStruct = std::make_shared<BVHRT>();
  }
  const char* Name() const override { return "EyeRayCaster_GPU";}
  virtual void InitVulkanObjects(VkDevice a_device, VkPhysicalDevice a_physicalDevice, size_t a_maxThreadsCount);
  
  virtual void SetVulkanContext(vk_utils::VulkanContext a_ctx) { m_ctx = a_ctx; }
  virtual void SetVulkanInOutFor_PackXY(
    uint32_t dummyArgument = 0)
  {
    InitAllGeneratedDescriptorSets_PackXY();
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

  virtual ~EyeRayCaster_GPU();

  
  virtual void InitMemberBuffers();
  virtual void UpdateAll(std::shared_ptr<vk_utils::ICopyEngine> a_pCopyEngine)
  {
    UpdatePlainMembers(a_pCopyEngine);
    UpdateVectorMembers(a_pCopyEngine);
    UpdateTextureMembers(a_pCopyEngine);
  }
  
  virtual void UpdatePrefixPointers()
  {
    auto pUnderlyingImpl = dynamic_cast<BVHRT*>(m_pAccelStruct.get());
    if(pUnderlyingImpl != nullptr)
    {
      m_pAccelStruct_m_ConjIndices = &pUnderlyingImpl->m_ConjIndices;
      m_pAccelStruct_m_SdfConjunctions = &pUnderlyingImpl->m_SdfConjunctions;
      m_pAccelStruct_m_SdfGridData = &pUnderlyingImpl->m_SdfGridData;
      m_pAccelStruct_m_SdfGridOffsets = &pUnderlyingImpl->m_SdfGridOffsets;
      m_pAccelStruct_m_SdfGridSizes = &pUnderlyingImpl->m_SdfGridSizes;
      m_pAccelStruct_m_SdfNeuralProperties = &pUnderlyingImpl->m_SdfNeuralProperties;
      m_pAccelStruct_m_SdfObjects = &pUnderlyingImpl->m_SdfObjects;
      m_pAccelStruct_m_SdfParameters = &pUnderlyingImpl->m_SdfParameters;
      m_pAccelStruct_m_allNodePairs = &pUnderlyingImpl->m_allNodePairs;
      m_pAccelStruct_m_bvhOffsets = &pUnderlyingImpl->m_bvhOffsets;
      m_pAccelStruct_m_geomIdByInstId = &pUnderlyingImpl->m_geomIdByInstId;
      m_pAccelStruct_m_geomOffsets = &pUnderlyingImpl->m_geomOffsets;
      m_pAccelStruct_m_geomTypeByGeomId = &pUnderlyingImpl->m_geomTypeByGeomId;
      m_pAccelStruct_m_indices = &pUnderlyingImpl->m_indices;
      m_pAccelStruct_m_instMatricesInv = &pUnderlyingImpl->m_instMatricesInv;
      m_pAccelStruct_m_nodesTLAS = &pUnderlyingImpl->m_nodesTLAS;
      m_pAccelStruct_m_primIndices = &pUnderlyingImpl->m_primIndices;
      m_pAccelStruct_m_vertPos = &pUnderlyingImpl->m_vertPos;
    }
  }
  virtual void CommitDeviceData(std::shared_ptr<vk_utils::ICopyEngine> a_pCopyHelper) // you have to define this virtual function in the original imput class
  {
    UpdatePrefixPointers(); 
    InitMemberBuffers();
    UpdateAll(a_pCopyHelper);
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
  virtual void CastRaySingleCmd(VkCommandBuffer a_commandBuffer, uint32_t tidX, uint32_t* out_color);

  void PackXYBlock(uint tidX, uint tidY, uint32_t a_numPasses) override;
  void CastRaySingleBlock(uint32_t tidX, uint32_t* out_color, uint32_t a_numPasses) override;

  inline vk_utils::ExecTime GetPackXYExecutionTime() const { return m_exTimePackXY; }
  inline vk_utils::ExecTime GetCastRaySingleExecutionTime() const { return m_exTimeCastRaySingle; }

  vk_utils::ExecTime m_exTimePackXY;
  vk_utils::ExecTime m_exTimeCastRaySingle;

  virtual void copyKernelFloatCmd(uint32_t length);
  
  virtual void PackXYMegaCmd(uint tidX, uint tidY);
  virtual void CastRaySingleMegaCmd(uint32_t tidX, uint32_t* out_color);
  
  struct MemLoc
  {
    VkDeviceMemory memObject = VK_NULL_HANDLE;
    size_t         memOffset = 0;
    size_t         allocId   = 0;
  };

  virtual MemLoc AllocAndBind(const std::vector<VkBuffer>& a_buffers); ///< replace this function to apply custom allocator
  virtual MemLoc AllocAndBind(const std::vector<VkImage>& a_image);    ///< replace this function to apply custom allocator
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
  virtual void InitAllGeneratedDescriptorSets_CastRaySingle();

  virtual void AssignBuffersToMemory(const std::vector<VkBuffer>& a_buffers, VkDeviceMemory a_mem);

  virtual void AllocMemoryForMemberBuffersAndImages(const std::vector<VkBuffer>& a_buffers, const std::vector<VkImage>& a_image);
  virtual std::string AlterShaderPath(const char* in_shaderPath) { return std::string("") + std::string(in_shaderPath); }

  
  

  struct PackXY_Data
  {
    bool needToClearOutput = true;
  } PackXY_local;

  struct CastRaySingle_Data
  {
    VkBuffer out_colorBuffer = VK_NULL_HANDLE;
    size_t   out_colorOffset = 0;
    bool needToClearOutput = true;
  } CastRaySingle_local;



  struct MembersDataGPU
  {
    VkBuffer m_pAccelStruct_m_ConjIndicesBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_ConjIndicesOffset = 0;
    VkBuffer m_pAccelStruct_m_SdfConjunctionsBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_SdfConjunctionsOffset = 0;
    VkBuffer m_pAccelStruct_m_SdfGridDataBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_SdfGridDataOffset = 0;
    VkBuffer m_pAccelStruct_m_SdfGridOffsetsBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_SdfGridOffsetsOffset = 0;
    VkBuffer m_pAccelStruct_m_SdfGridSizesBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_SdfGridSizesOffset = 0;
    VkBuffer m_pAccelStruct_m_SdfNeuralPropertiesBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_SdfNeuralPropertiesOffset = 0;
    VkBuffer m_pAccelStruct_m_SdfObjectsBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_SdfObjectsOffset = 0;
    VkBuffer m_pAccelStruct_m_SdfParametersBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_SdfParametersOffset = 0;
    VkBuffer m_pAccelStruct_m_allNodePairsBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_allNodePairsOffset = 0;
    VkBuffer m_pAccelStruct_m_bvhOffsetsBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_bvhOffsetsOffset = 0;
    VkBuffer m_pAccelStruct_m_geomIdByInstIdBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_geomIdByInstIdOffset = 0;
    VkBuffer m_pAccelStruct_m_geomOffsetsBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_geomOffsetsOffset = 0;
    VkBuffer m_pAccelStruct_m_geomTypeByGeomIdBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_geomTypeByGeomIdOffset = 0;
    VkBuffer m_pAccelStruct_m_indicesBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_indicesOffset = 0;
    VkBuffer m_pAccelStruct_m_instMatricesInvBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_instMatricesInvOffset = 0;
    VkBuffer m_pAccelStruct_m_nodesTLASBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_nodesTLASOffset = 0;
    VkBuffer m_pAccelStruct_m_primIndicesBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_primIndicesOffset = 0;
    VkBuffer m_pAccelStruct_m_vertPosBuffer = VK_NULL_HANDLE;
    size_t   m_pAccelStruct_m_vertPosOffset = 0;
    VkBuffer m_packedXYBuffer = VK_NULL_HANDLE;
    size_t   m_packedXYOffset = 0;
  } m_vdata;
  
  std::vector<uint32_t>* m_pAccelStruct_m_ConjIndices = nullptr;
  std::vector<SdfConjunction>* m_pAccelStruct_m_SdfConjunctions = nullptr;
  std::vector<float>* m_pAccelStruct_m_SdfGridData = nullptr;
  std::vector<uint32_t>* m_pAccelStruct_m_SdfGridOffsets = nullptr;
  std::vector<uint3>* m_pAccelStruct_m_SdfGridSizes = nullptr;
  std::vector<NeuralProperties>* m_pAccelStruct_m_SdfNeuralProperties = nullptr;
  std::vector<SdfObject>* m_pAccelStruct_m_SdfObjects = nullptr;
  std::vector<float>* m_pAccelStruct_m_SdfParameters = nullptr;
  std::vector<BVHNodePair>* m_pAccelStruct_m_allNodePairs = nullptr;
  std::vector<uint32_t>* m_pAccelStruct_m_bvhOffsets = nullptr;
  std::vector<uint32_t>* m_pAccelStruct_m_geomIdByInstId = nullptr;
  std::vector<uint2>* m_pAccelStruct_m_geomOffsets = nullptr;
  std::vector<uint32_t>* m_pAccelStruct_m_geomTypeByGeomId = nullptr;
  std::vector<uint32_t>* m_pAccelStruct_m_indices = nullptr;
  std::vector<float4x4>* m_pAccelStruct_m_instMatricesInv = nullptr;
  std::vector<BVHNode>* m_pAccelStruct_m_nodesTLAS = nullptr;
  std::vector<uint32_t>* m_pAccelStruct_m_primIndices = nullptr;
  std::vector<float4>* m_pAccelStruct_m_vertPos = nullptr;
  
  size_t m_maxThreadCount = 0;
  VkBuffer m_classDataBuffer = VK_NULL_HANDLE;

  VkPipelineLayout      PackXYMegaLayout   = VK_NULL_HANDLE;
  VkPipeline            PackXYMegaPipeline = VK_NULL_HANDLE; 
  VkDescriptorSetLayout PackXYMegaDSLayout = VK_NULL_HANDLE;
  VkDescriptorSetLayout CreatePackXYMegaDSLayout();
  virtual void InitKernel_PackXYMega(const char* a_filePath);
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

  VkDescriptorPool m_dsPool = VK_NULL_HANDLE;
  VkDescriptorSet  m_allGeneratedDS[2];

  EyeRayCaster_GPU_UBO_Data m_uboData;
  
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
    bool enableCastRaySingleMega = true;
    bool dummy = 0;
  };

  static MegaKernelIsEnabled  m_megaKernelFlags;
  static MegaKernelIsEnabled& EnabledPipelines() { return m_megaKernelFlags; }

};


