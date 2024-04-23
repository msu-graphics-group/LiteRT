#include <vector>
#include <array>
#include <memory>
#include <limits>
#include <cassert>
#include "vk_copy.h"
#include "vk_context.h"
#include "eye_ray_gpu.h"
#include "include/MultiRenderer_gpu_ubo.h"


std::shared_ptr<MultiRenderer> CreateMultiRenderer_GPU(vk_utils::VulkanContext a_ctx, size_t a_maxThreadsGenerated)
{
  auto pObj = std::make_shared<MultiRenderer_GPU>();
  pObj->SetVulkanContext(a_ctx);
  pObj->InitVulkanObjects(a_ctx.device, a_ctx.physicalDevice, a_maxThreadsGenerated);
  return pObj;
}

void MultiRenderer_GPU::InitVulkanObjects(VkDevice a_device, VkPhysicalDevice a_physicalDevice, size_t a_maxThreadsCount)
{
  physicalDevice = a_physicalDevice;
  device         = a_device;
  m_allCreatedPipelineLayouts.reserve(256);
  m_allCreatedPipelines.reserve(256);
  InitHelpers();
  InitBuffers(a_maxThreadsCount, true);
  InitKernels(".spv");
  AllocateAllDescriptorSets();

}

static uint32_t ComputeReductionAuxBufferElements(uint32_t whole_size, uint32_t wg_size)
{
  uint32_t sizeTotal = 0;
  while (whole_size > 1)
  {
    whole_size  = (whole_size + wg_size - 1) / wg_size;
    sizeTotal  += std::max<uint32_t>(whole_size, 1);
  }
  return sizeTotal;
}

VkBufferUsageFlags MultiRenderer_GPU::GetAdditionalFlagsForUBO() const
{
  return VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
}

uint32_t MultiRenderer_GPU::GetDefaultMaxTextures() const { return 256; }

void MultiRenderer_GPU::MakeComputePipelineAndLayout(const char* a_shaderPath, const char* a_mainName, const VkSpecializationInfo *a_specInfo, const VkDescriptorSetLayout a_dsLayout, VkPipelineLayout* pPipelineLayout, VkPipeline* pPipeline)
{
  VkPipelineShaderStageCreateInfo shaderStageInfo = {};
  shaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  shaderStageInfo.stage = VK_SHADER_STAGE_COMPUTE_BIT;

  auto shaderCode   = vk_utils::readSPVFile(a_shaderPath);
  auto shaderModule = vk_utils::createShaderModule(device, shaderCode);

  shaderStageInfo.module              = shaderModule;
  shaderStageInfo.pName               = a_mainName;
  shaderStageInfo.pSpecializationInfo = a_specInfo;

  VkPushConstantRange pcRange = {};
  pcRange.stageFlags = shaderStageInfo.stage;
  pcRange.offset     = 0;
  pcRange.size       = 128; // at least 128 bytes for push constants for all Vulkan implementations

  VkPipelineLayoutCreateInfo pipelineLayoutInfo = {};
  pipelineLayoutInfo.sType                  = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
  pipelineLayoutInfo.pushConstantRangeCount = 1;
  pipelineLayoutInfo.pPushConstantRanges    = &pcRange;
  pipelineLayoutInfo.pSetLayouts            = &a_dsLayout;
  pipelineLayoutInfo.setLayoutCount         = 1;

  VkResult res = vkCreatePipelineLayout(device, &pipelineLayoutInfo, nullptr, pPipelineLayout);
  if(res != VK_SUCCESS)
  {
    std::string errMsg = vk_utils::errorString(res);
    std::cout << "[ShaderError]: vkCreatePipelineLayout have failed for '" << a_shaderPath << "' with '" << errMsg.c_str() << "'" << std::endl;
  }
  else
    m_allCreatedPipelineLayouts.push_back(*pPipelineLayout);

  VkComputePipelineCreateInfo pipelineInfo = {};
  pipelineInfo.sType              = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
  pipelineInfo.flags              = 0;
  pipelineInfo.stage              = shaderStageInfo;
  pipelineInfo.layout             = (*pPipelineLayout);
  pipelineInfo.basePipelineHandle = VK_NULL_HANDLE;
  res = vkCreateComputePipelines(device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, pPipeline);
  if(res != VK_SUCCESS)
  {
    std::string errMsg = vk_utils::errorString(res);
    std::cout << "[ShaderError]: vkCreateComputePipelines have failed for '" << a_shaderPath << "' with '" << errMsg.c_str() << "'" << std::endl;
  }
  else
    m_allCreatedPipelines.push_back(*pPipeline);

  if (shaderModule != VK_NULL_HANDLE)
    vkDestroyShaderModule(device, shaderModule, VK_NULL_HANDLE);
}

void MultiRenderer_GPU::MakeComputePipelineOnly(const char* a_shaderPath, const char* a_mainName, const VkSpecializationInfo *a_specInfo, const VkDescriptorSetLayout a_dsLayout, VkPipelineLayout pipelineLayout, VkPipeline* pPipeline)
{
  VkPipelineShaderStageCreateInfo shaderStageInfo = {};
  shaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  shaderStageInfo.stage = VK_SHADER_STAGE_COMPUTE_BIT;

  auto shaderCode   = vk_utils::readSPVFile(a_shaderPath);
  auto shaderModule = vk_utils::createShaderModule(device, shaderCode);

  shaderStageInfo.module              = shaderModule;
  shaderStageInfo.pName               = a_mainName;
  shaderStageInfo.pSpecializationInfo = a_specInfo;

  VkComputePipelineCreateInfo pipelineInfo = {};
  pipelineInfo.sType              = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
  pipelineInfo.flags              = 0;
  pipelineInfo.stage              = shaderStageInfo;
  pipelineInfo.layout             = pipelineLayout;
  pipelineInfo.basePipelineHandle = VK_NULL_HANDLE;
  VkResult res = vkCreateComputePipelines(device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, pPipeline);
  if(res != VK_SUCCESS)
  {
    std::string errMsg = vk_utils::errorString(res);
    std::cout << "[ShaderError]: vkCreateComputePipelines have failed for '" << a_shaderPath << "' with '" << errMsg.c_str() << "'" << std::endl;
  }
  else
    m_allCreatedPipelines.push_back(*pPipeline);

  if (shaderModule != VK_NULL_HANDLE)
    vkDestroyShaderModule(device, shaderModule, VK_NULL_HANDLE);
}

MultiRenderer_GPU::~MultiRenderer_GPU()
{
  for(size_t i=0;i<m_allCreatedPipelines.size();i++)
    vkDestroyPipeline(device, m_allCreatedPipelines[i], nullptr);
  for(size_t i=0;i<m_allCreatedPipelineLayouts.size();i++)
    vkDestroyPipelineLayout(device, m_allCreatedPipelineLayouts[i], nullptr);

  vkDestroyDescriptorSetLayout(device, PackXYMegaDSLayout, nullptr);
  PackXYMegaDSLayout = VK_NULL_HANDLE;
  vkDestroyDescriptorSetLayout(device, CastRaySingleMegaDSLayout, nullptr);
  CastRaySingleMegaDSLayout = VK_NULL_HANDLE;
  vkDestroyDescriptorPool(device, m_dsPool, NULL); m_dsPool = VK_NULL_HANDLE;


  vkDestroyBuffer(device, m_classDataBuffer, nullptr);

  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_ConjIndicesBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_RFGridDataBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_RFGridScalesBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_RFGridSizesBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfConjunctionsBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfFrameOctreeNodesBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfFrameOctreeRootsBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfGridDataBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfGridOffsetsBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfGridSizesBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfNeuralPropertiesBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfObjectsBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfOctreeNodesBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfOctreeRootsBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfParametersBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfSBSDataBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfSBSHeadersBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfSBSNodesBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfSBSRemapBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfSVSNodesBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfSVSRootsBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_allNodePairsBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_bvhOffsetsBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_geomIdByInstIdBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_geomOffsetsBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_geomTypeByGeomIdBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_indicesBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_instMatricesInvBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_nodesTLASBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_origNodesBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_primIndicesBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_vertPosBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_packedXYBuffer, nullptr);
  FreeAllAllocations(m_allMems);
}

void MultiRenderer_GPU::InitHelpers()
{
  vkGetPhysicalDeviceProperties(physicalDevice, &m_devProps);
}


void MultiRenderer_GPU::InitKernel_PackXYMega(const char* a_filePath)
{
  std::string shaderPath = AlterShaderPath("shaders_gpu/PackXYMega.comp.spv");
  const VkSpecializationInfo* kspec = nullptr;
  PackXYMegaDSLayout = CreatePackXYMegaDSLayout();
  if(m_megaKernelFlags.enablePackXYMega)
  {
    MakeComputePipelineAndLayout(shaderPath.c_str(), "main", kspec, PackXYMegaDSLayout, &PackXYMegaLayout, &PackXYMegaPipeline);
  }
  else
  {
    PackXYMegaLayout   = nullptr;
    PackXYMegaPipeline = nullptr;
  }
}

void MultiRenderer_GPU::InitKernel_CastRaySingleMega(const char* a_filePath)
{
  std::string shaderPath = AlterShaderPath("shaders_gpu/CastRaySingleMega.comp.spv");
  const VkSpecializationInfo* kspec = nullptr;
  CastRaySingleMegaDSLayout = CreateCastRaySingleMegaDSLayout();
  if(m_megaKernelFlags.enableCastRaySingleMega)
  {
    MakeComputePipelineAndLayout(shaderPath.c_str(), "main", kspec, CastRaySingleMegaDSLayout, &CastRaySingleMegaLayout, &CastRaySingleMegaPipeline);
  }
  else
  {
    CastRaySingleMegaLayout   = nullptr;
    CastRaySingleMegaPipeline = nullptr;
  }
}


void MultiRenderer_GPU::InitKernels(const char* a_filePath)
{
  InitKernel_PackXYMega(a_filePath);
  InitKernel_CastRaySingleMega(a_filePath);
}

void MultiRenderer_GPU::InitBuffers(size_t a_maxThreadsCount, bool a_tempBuffersOverlay)
{
  ReserveEmptyVectors();

  m_maxThreadCount = a_maxThreadsCount;
  std::vector<VkBuffer> allBuffers;
  allBuffers.reserve(64);

  struct BufferReqPair
  {
    BufferReqPair() {  }
    BufferReqPair(VkBuffer a_buff, VkDevice a_dev) : buf(a_buff) { vkGetBufferMemoryRequirements(a_dev, a_buff, &req); }
    VkBuffer             buf = VK_NULL_HANDLE;
    VkMemoryRequirements req = {};
  };

  struct LocalBuffers
  {
    std::vector<BufferReqPair> bufs;
    size_t                     size = 0;
    std::vector<VkBuffer>      bufsClean;
  };

  std::vector<LocalBuffers> groups;
  groups.reserve(16);


  size_t largestIndex = 0;
  size_t largestSize  = 0;
  for(size_t i=0;i<groups.size();i++)
  {
    if(groups[i].size > largestSize)
    {
      largestIndex = i;
      largestSize  = groups[i].size;
    }
    groups[i].bufsClean.resize(groups[i].bufs.size());
    for(size_t j=0;j<groups[i].bufsClean.size();j++)
      groups[i].bufsClean[j] = groups[i].bufs[j].buf;
  }

  auto& allBuffersRef = allBuffers;

  m_classDataBuffer = vk_utils::createBuffer(device, sizeof(m_uboData),  VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | GetAdditionalFlagsForUBO());
  allBuffersRef.push_back(m_classDataBuffer);


  auto internalBuffersMem = AllocAndBind(allBuffersRef);
  if(a_tempBuffersOverlay)
  {
    for(size_t i=0;i<groups.size();i++)
      if(i != largestIndex)
        AssignBuffersToMemory(groups[i].bufsClean, internalBuffersMem.memObject);
  }
}

void MultiRenderer_GPU::ReserveEmptyVectors()
{
  if(m_pAccelStruct_m_ConjIndices != nullptr && m_pAccelStruct_m_ConjIndices->capacity() == 0)
    m_pAccelStruct_m_ConjIndices->reserve(4);
  if(m_pAccelStruct_m_RFGridData != nullptr && m_pAccelStruct_m_RFGridData->capacity() == 0)
    m_pAccelStruct_m_RFGridData->reserve(4);
  if(m_pAccelStruct_m_RFGridScales != nullptr && m_pAccelStruct_m_RFGridScales->capacity() == 0)
    m_pAccelStruct_m_RFGridScales->reserve(4);
  if(m_pAccelStruct_m_RFGridSizes != nullptr && m_pAccelStruct_m_RFGridSizes->capacity() == 0)
    m_pAccelStruct_m_RFGridSizes->reserve(4);
  if(m_pAccelStruct_m_SdfConjunctions != nullptr && m_pAccelStruct_m_SdfConjunctions->capacity() == 0)
    m_pAccelStruct_m_SdfConjunctions->reserve(4);
  if(m_pAccelStruct_m_SdfFrameOctreeNodes != nullptr && m_pAccelStruct_m_SdfFrameOctreeNodes->capacity() == 0)
    m_pAccelStruct_m_SdfFrameOctreeNodes->reserve(4);
  if(m_pAccelStruct_m_SdfFrameOctreeRoots != nullptr && m_pAccelStruct_m_SdfFrameOctreeRoots->capacity() == 0)
    m_pAccelStruct_m_SdfFrameOctreeRoots->reserve(4);
  if(m_pAccelStruct_m_SdfGridData != nullptr && m_pAccelStruct_m_SdfGridData->capacity() == 0)
    m_pAccelStruct_m_SdfGridData->reserve(4);
  if(m_pAccelStruct_m_SdfGridOffsets != nullptr && m_pAccelStruct_m_SdfGridOffsets->capacity() == 0)
    m_pAccelStruct_m_SdfGridOffsets->reserve(4);
  if(m_pAccelStruct_m_SdfGridSizes != nullptr && m_pAccelStruct_m_SdfGridSizes->capacity() == 0)
    m_pAccelStruct_m_SdfGridSizes->reserve(4);
  if(m_pAccelStruct_m_SdfNeuralProperties != nullptr && m_pAccelStruct_m_SdfNeuralProperties->capacity() == 0)
    m_pAccelStruct_m_SdfNeuralProperties->reserve(4);
  if(m_pAccelStruct_m_SdfObjects != nullptr && m_pAccelStruct_m_SdfObjects->capacity() == 0)
    m_pAccelStruct_m_SdfObjects->reserve(4);
  if(m_pAccelStruct_m_SdfOctreeNodes != nullptr && m_pAccelStruct_m_SdfOctreeNodes->capacity() == 0)
    m_pAccelStruct_m_SdfOctreeNodes->reserve(4);
  if(m_pAccelStruct_m_SdfOctreeRoots != nullptr && m_pAccelStruct_m_SdfOctreeRoots->capacity() == 0)
    m_pAccelStruct_m_SdfOctreeRoots->reserve(4);
  if(m_pAccelStruct_m_SdfParameters != nullptr && m_pAccelStruct_m_SdfParameters->capacity() == 0)
    m_pAccelStruct_m_SdfParameters->reserve(4);
  if(m_pAccelStruct_m_SdfSBSData != nullptr && m_pAccelStruct_m_SdfSBSData->capacity() == 0)
    m_pAccelStruct_m_SdfSBSData->reserve(4);
  if(m_pAccelStruct_m_SdfSBSHeaders != nullptr && m_pAccelStruct_m_SdfSBSHeaders->capacity() == 0)
    m_pAccelStruct_m_SdfSBSHeaders->reserve(4);
  if(m_pAccelStruct_m_SdfSBSNodes != nullptr && m_pAccelStruct_m_SdfSBSNodes->capacity() == 0)
    m_pAccelStruct_m_SdfSBSNodes->reserve(4);
  if(m_pAccelStruct_m_SdfSBSRemap != nullptr && m_pAccelStruct_m_SdfSBSRemap->capacity() == 0)
    m_pAccelStruct_m_SdfSBSRemap->reserve(4);
  if(m_pAccelStruct_m_SdfSVSNodes != nullptr && m_pAccelStruct_m_SdfSVSNodes->capacity() == 0)
    m_pAccelStruct_m_SdfSVSNodes->reserve(4);
  if(m_pAccelStruct_m_SdfSVSRoots != nullptr && m_pAccelStruct_m_SdfSVSRoots->capacity() == 0)
    m_pAccelStruct_m_SdfSVSRoots->reserve(4);
  if(m_pAccelStruct_m_allNodePairs != nullptr && m_pAccelStruct_m_allNodePairs->capacity() == 0)
    m_pAccelStruct_m_allNodePairs->reserve(4);
  if(m_pAccelStruct_m_bvhOffsets != nullptr && m_pAccelStruct_m_bvhOffsets->capacity() == 0)
    m_pAccelStruct_m_bvhOffsets->reserve(4);
  if(m_pAccelStruct_m_geomIdByInstId != nullptr && m_pAccelStruct_m_geomIdByInstId->capacity() == 0)
    m_pAccelStruct_m_geomIdByInstId->reserve(4);
  if(m_pAccelStruct_m_geomOffsets != nullptr && m_pAccelStruct_m_geomOffsets->capacity() == 0)
    m_pAccelStruct_m_geomOffsets->reserve(4);
  if(m_pAccelStruct_m_geomTypeByGeomId != nullptr && m_pAccelStruct_m_geomTypeByGeomId->capacity() == 0)
    m_pAccelStruct_m_geomTypeByGeomId->reserve(4);
  if(m_pAccelStruct_m_indices != nullptr && m_pAccelStruct_m_indices->capacity() == 0)
    m_pAccelStruct_m_indices->reserve(4);
  if(m_pAccelStruct_m_instMatricesInv != nullptr && m_pAccelStruct_m_instMatricesInv->capacity() == 0)
    m_pAccelStruct_m_instMatricesInv->reserve(4);
  if(m_pAccelStruct_m_nodesTLAS != nullptr && m_pAccelStruct_m_nodesTLAS->capacity() == 0)
    m_pAccelStruct_m_nodesTLAS->reserve(4);
  if(m_pAccelStruct_m_origNodes != nullptr && m_pAccelStruct_m_origNodes->capacity() == 0)
    m_pAccelStruct_m_origNodes->reserve(4);
  if(m_pAccelStruct_m_primIndices != nullptr && m_pAccelStruct_m_primIndices->capacity() == 0)
    m_pAccelStruct_m_primIndices->reserve(4);
  if(m_pAccelStruct_m_vertPos != nullptr && m_pAccelStruct_m_vertPos->capacity() == 0)
    m_pAccelStruct_m_vertPos->reserve(4);
  if(m_packedXY.capacity() == 0)
    m_packedXY.reserve(4);
}

void MultiRenderer_GPU::InitMemberBuffers()
{
  std::vector<VkBuffer> memberVectors;
  std::vector<VkImage>  memberTextures;

  m_vdata.m_pAccelStruct_m_ConjIndicesBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_ConjIndices->capacity()*sizeof(unsigned int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_ConjIndicesBuffer);
  m_vdata.m_pAccelStruct_m_RFGridDataBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_RFGridData->capacity()*sizeof(float), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_RFGridDataBuffer);
  m_vdata.m_pAccelStruct_m_RFGridScalesBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_RFGridScales->capacity()*sizeof(float), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_RFGridScalesBuffer);
  m_vdata.m_pAccelStruct_m_RFGridSizesBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_RFGridSizes->capacity()*sizeof(unsigned long), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_RFGridSizesBuffer);
  m_vdata.m_pAccelStruct_m_SdfConjunctionsBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfConjunctions->capacity()*sizeof(struct SdfConjunction), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfConjunctionsBuffer);
  m_vdata.m_pAccelStruct_m_SdfFrameOctreeNodesBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfFrameOctreeNodes->capacity()*sizeof(struct SdfFrameOctreeNode), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfFrameOctreeNodesBuffer);
  m_vdata.m_pAccelStruct_m_SdfFrameOctreeRootsBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfFrameOctreeRoots->capacity()*sizeof(unsigned int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfFrameOctreeRootsBuffer);
  m_vdata.m_pAccelStruct_m_SdfGridDataBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfGridData->capacity()*sizeof(float), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfGridDataBuffer);
  m_vdata.m_pAccelStruct_m_SdfGridOffsetsBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfGridOffsets->capacity()*sizeof(unsigned int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfGridOffsetsBuffer);
  m_vdata.m_pAccelStruct_m_SdfGridSizesBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfGridSizes->capacity()*sizeof(struct LiteMath::uint3), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfGridSizesBuffer);
  m_vdata.m_pAccelStruct_m_SdfNeuralPropertiesBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfNeuralProperties->capacity()*sizeof(struct NeuralProperties), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfNeuralPropertiesBuffer);
  m_vdata.m_pAccelStruct_m_SdfObjectsBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfObjects->capacity()*sizeof(struct SdfObject), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfObjectsBuffer);
  m_vdata.m_pAccelStruct_m_SdfOctreeNodesBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfOctreeNodes->capacity()*sizeof(struct SdfOctreeNode), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfOctreeNodesBuffer);
  m_vdata.m_pAccelStruct_m_SdfOctreeRootsBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfOctreeRoots->capacity()*sizeof(unsigned int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfOctreeRootsBuffer);
  m_vdata.m_pAccelStruct_m_SdfParametersBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfParameters->capacity()*sizeof(float), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfParametersBuffer);
  m_vdata.m_pAccelStruct_m_SdfSBSDataBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfSBSData->capacity()*sizeof(unsigned int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfSBSDataBuffer);
  m_vdata.m_pAccelStruct_m_SdfSBSHeadersBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfSBSHeaders->capacity()*sizeof(struct SdfSBSHeader), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfSBSHeadersBuffer);
  m_vdata.m_pAccelStruct_m_SdfSBSNodesBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfSBSNodes->capacity()*sizeof(struct SdfSBSNode), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfSBSNodesBuffer);
  m_vdata.m_pAccelStruct_m_SdfSBSRemapBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfSBSRemap->capacity()*sizeof(struct LiteMath::uint2), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfSBSRemapBuffer);
  m_vdata.m_pAccelStruct_m_SdfSVSNodesBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfSVSNodes->capacity()*sizeof(struct SdfSVSNode), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfSVSNodesBuffer);
  m_vdata.m_pAccelStruct_m_SdfSVSRootsBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfSVSRoots->capacity()*sizeof(unsigned int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfSVSRootsBuffer);
  m_vdata.m_pAccelStruct_m_allNodePairsBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_allNodePairs->capacity()*sizeof(struct BVHNodePair), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_allNodePairsBuffer);
  m_vdata.m_pAccelStruct_m_bvhOffsetsBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_bvhOffsets->capacity()*sizeof(unsigned int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_bvhOffsetsBuffer);
  m_vdata.m_pAccelStruct_m_geomIdByInstIdBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_geomIdByInstId->capacity()*sizeof(unsigned int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_geomIdByInstIdBuffer);
  m_vdata.m_pAccelStruct_m_geomOffsetsBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_geomOffsets->capacity()*sizeof(struct LiteMath::uint2), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_geomOffsetsBuffer);
  m_vdata.m_pAccelStruct_m_geomTypeByGeomIdBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_geomTypeByGeomId->capacity()*sizeof(unsigned int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_geomTypeByGeomIdBuffer);
  m_vdata.m_pAccelStruct_m_indicesBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_indices->capacity()*sizeof(unsigned int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_indicesBuffer);
  m_vdata.m_pAccelStruct_m_instMatricesInvBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_instMatricesInv->capacity()*sizeof(struct LiteMath::float4x4), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_instMatricesInvBuffer);
  m_vdata.m_pAccelStruct_m_nodesTLASBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_nodesTLAS->capacity()*sizeof(struct BVHNode), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_nodesTLASBuffer);
  m_vdata.m_pAccelStruct_m_origNodesBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_origNodes->capacity()*sizeof(struct BVHNode), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_origNodesBuffer);
  m_vdata.m_pAccelStruct_m_primIndicesBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_primIndices->capacity()*sizeof(unsigned int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_primIndicesBuffer);
  m_vdata.m_pAccelStruct_m_vertPosBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_vertPos->capacity()*sizeof(struct LiteMath::float4), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_vertPosBuffer);
  m_vdata.m_packedXYBuffer = vk_utils::createBuffer(device, m_packedXY.capacity()*sizeof(unsigned int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_packedXYBuffer);


  AllocMemoryForMemberBuffersAndImages(memberVectors, memberTextures);
}




void MultiRenderer_GPU::AssignBuffersToMemory(const std::vector<VkBuffer>& a_buffers, VkDeviceMemory a_mem)
{
  if(a_buffers.size() == 0 || a_mem == VK_NULL_HANDLE)
    return;

  std::vector<VkMemoryRequirements> memInfos(a_buffers.size());
  for(size_t i=0;i<memInfos.size();i++)
  {
    if(a_buffers[i] != VK_NULL_HANDLE)
      vkGetBufferMemoryRequirements(device, a_buffers[i], &memInfos[i]);
    else
    {
      memInfos[i] = memInfos[0];
      memInfos[i].size = 0;
    }
  }

  for(size_t i=1;i<memInfos.size();i++)
  {
    if(memInfos[i].memoryTypeBits != memInfos[0].memoryTypeBits)
    {
      std::cout << "[MultiRenderer_GPU::AssignBuffersToMemory]: error, input buffers has different 'memReq.memoryTypeBits'" << std::endl;
      return;
    }
  }

  auto offsets = vk_utils::calculateMemOffsets(memInfos);
  for (size_t i = 0; i < memInfos.size(); i++)
  {
    if(a_buffers[i] != VK_NULL_HANDLE)
      vkBindBufferMemory(device, a_buffers[i], a_mem, offsets[i]);
  }
}

MultiRenderer_GPU::MemLoc MultiRenderer_GPU::AllocAndBind(const std::vector<VkBuffer>& a_buffers)
{
  MemLoc currLoc;
  if(a_buffers.size() > 0)
  {
    currLoc.memObject = vk_utils::allocateAndBindWithPadding(device, physicalDevice, a_buffers);
    currLoc.allocId   = m_allMems.size();
    m_allMems.push_back(currLoc);
  }
  return currLoc;
}

MultiRenderer_GPU::MemLoc MultiRenderer_GPU::AllocAndBind(const std::vector<VkImage>& a_images)
{
  MemLoc currLoc;
  if(a_images.size() > 0)
  {
    std::vector<VkMemoryRequirements> reqs(a_images.size());
    for(size_t i=0; i<reqs.size(); i++)
      vkGetImageMemoryRequirements(device, a_images[i], &reqs[i]);

    for(size_t i=0; i<reqs.size(); i++)
    {
      if(reqs[i].memoryTypeBits != reqs[0].memoryTypeBits)
      {
        std::cout << "MultiRenderer_GPU::AllocAndBind(textures): memoryTypeBits warning, need to split mem allocation (override me)" << std::endl;
        break;
      }
    }

    auto offsets  = vk_utils::calculateMemOffsets(reqs);
    auto memTotal = offsets[offsets.size() - 1];

    VkMemoryAllocateInfo allocateInfo = {};
    allocateInfo.sType           = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    allocateInfo.pNext           = nullptr;
    allocateInfo.allocationSize  = memTotal;
    allocateInfo.memoryTypeIndex = vk_utils::findMemoryType(reqs[0].memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, physicalDevice);
    VK_CHECK_RESULT(vkAllocateMemory(device, &allocateInfo, NULL, &currLoc.memObject));

    for(size_t i=0;i<a_images.size();i++) {
      VK_CHECK_RESULT(vkBindImageMemory(device, a_images[i], currLoc.memObject, offsets[i]));
    }

    currLoc.allocId = m_allMems.size();
    m_allMems.push_back(currLoc);
  }
  return currLoc;
}

void MultiRenderer_GPU::FreeAllAllocations(std::vector<MemLoc>& a_memLoc)
{
  // in general you may check 'mem.allocId' for unique to be sure you dont free mem twice
  // for default implementation this is not needed
  for(auto mem : a_memLoc)
    vkFreeMemory(device, mem.memObject, nullptr);
  a_memLoc.resize(0);
}

void MultiRenderer_GPU::AllocMemoryForMemberBuffersAndImages(const std::vector<VkBuffer>& a_buffers, const std::vector<VkImage>& a_images)
{
  std::vector<VkMemoryRequirements> bufMemReqs(a_buffers.size()); // we must check that all buffers have same memoryTypeBits;
  for(size_t i = 0; i < a_buffers.size(); ++i)                    // if not, split to multiple allocations
  {
    if(a_buffers[i] != VK_NULL_HANDLE)
      vkGetBufferMemoryRequirements(device, a_buffers[i], &bufMemReqs[i]);
    else
    {
      bufMemReqs[i] = bufMemReqs[0];
      bufMemReqs[i].size = 0;
    }
  }

  bool needSplit = false;
  for(size_t i = 1; i < bufMemReqs.size(); ++i)
  {
    if(bufMemReqs[i].memoryTypeBits != bufMemReqs[0].memoryTypeBits)
    {
      needSplit = true;
      break;
    }
  }

  if(needSplit)
  {
    std::unordered_map<uint32_t, std::vector<uint32_t> > bufferSets;
    for(uint32_t j = 0; j < uint32_t(bufMemReqs.size()); ++j)
    {
      uint32_t key = uint32_t(bufMemReqs[j].memoryTypeBits);
      bufferSets[key].push_back(j);
    }

    for(const auto& buffGroup : bufferSets)
    {
      std::vector<VkBuffer> currGroup;
      for(auto id : buffGroup.second)
        currGroup.push_back(a_buffers[id]);
      AllocAndBind(currGroup);
    }
  }
  else
    AllocAndBind(a_buffers);

}

VkPhysicalDeviceFeatures2 MultiRenderer_GPU::ListRequiredDeviceFeatures(std::vector<const char*>& deviceExtensions)
{
  static VkPhysicalDeviceFeatures2 features2 = {};
  features2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
  features2.pNext = nullptr;
  features2.features.shaderInt64   = false;
  features2.features.shaderFloat64 = false;
  features2.features.shaderInt16   = false;
  void** ppNext = &features2.pNext;
  return features2;
}

MultiRenderer_GPU::MegaKernelIsEnabled MultiRenderer_GPU::m_megaKernelFlags;

