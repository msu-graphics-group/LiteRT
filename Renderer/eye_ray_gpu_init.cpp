#include <cstdint>
#include <vector>
#include <array>
#include <memory>
#include <limits>
#include <cassert>
#include "vk_copy.h"
#include "vk_context.h"
#include "ray_tracing/vk_rt_funcs.h"
#include "ray_tracing/vk_rt_utils.h"
#include "eye_ray_gpu.h"
#include "include/MultiRenderer_gpu_ubo.h"


std::shared_ptr<MultiRenderer> CreateMultiRenderer_GPU(uint32_t maxPrimitives, vk_utils::VulkanContext a_ctx, size_t a_maxThreadsGenerated)
{
  auto pObj = std::make_shared<MultiRenderer_GPU>(maxPrimitives);
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
  {
    m_timestampPoolSize = uint32_t(3*2); // 2 for each kernel call
    VkQueryPoolCreateInfo query_pool_info{};
    query_pool_info.sType      = VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO;
    query_pool_info.queryType  = VK_QUERY_TYPE_TIMESTAMP;
    query_pool_info.queryCount = m_timestampPoolSize; 
    VkResult res = vkCreateQueryPool(device, &query_pool_info, nullptr, &m_queryPoolTimestamps);
    if(res != VK_SUCCESS)
      std::cout << "[InitVulkanObjects]: ALERT! can't create timestamp pool " << std::endl;
    ResetTimeStampMeasurements();
    // get timestampPeriod from device props
    //
    VkPhysicalDeviceProperties2 physicalDeviceProperties;
    physicalDeviceProperties.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2;
    physicalDeviceProperties.pNext = nullptr;
    vkGetPhysicalDeviceProperties2(physicalDevice, &physicalDeviceProperties);
    m_timestampPeriod = float(physicalDeviceProperties.properties.limits.timestampPeriod);
  }
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
  if(m_queryPoolTimestamps != VK_NULL_HANDLE)
    vkDestroyQueryPool(device, m_queryPoolTimestamps, nullptr);
  for(size_t i=0;i<m_allCreatedPipelines.size();i++)
    vkDestroyPipeline(device, m_allCreatedPipelines[i], nullptr);
  for(size_t i=0;i<m_allCreatedPipelineLayouts.size();i++)
    vkDestroyPipelineLayout(device, m_allCreatedPipelineLayouts[i], nullptr);

  vkDestroyDescriptorSetLayout(device, PackXYMegaDSLayout, nullptr);
  PackXYMegaDSLayout = VK_NULL_HANDLE;
  vkDestroyDescriptorSetLayout(device, CastRayFloatSingleMegaDSLayout, nullptr);
  CastRayFloatSingleMegaDSLayout = VK_NULL_HANDLE;
  vkDestroyDescriptorSetLayout(device, CastRaySingleMegaDSLayout, nullptr);
  CastRaySingleMegaDSLayout = VK_NULL_HANDLE;
  vkDestroyDescriptorPool(device, m_dsPool, NULL); m_dsPool = VK_NULL_HANDLE;


  vkDestroyBuffer(device, m_classDataBuffer, nullptr);

  vkDestroyBuffer(device, m_vdata.all_referencesBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_geomOffsetsBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_indicesBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_instanceTransformInvTransposedBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_lightsBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_matIdOffsetsBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_matIdbyPrimIdBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_materialsBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_normalsBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_GraphicsPrimHeadersBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_GraphicsPrimPointsBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_NURBSDataBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_NURBSHeadersBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_NURBS_approxesBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfCompactOctreeRotModifiersBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfCompactOctreeV2DataBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfCompactOctreeV3DataBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfFrameOctreeNodesBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfFrameOctreeRootsBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfFrameOctreeTexNodesBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfFrameOctreeTexRootsBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfGridDataBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfGridOffsetsBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfGridSizesBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfSBSAdaptDataBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfSBSAdaptDataFBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfSBSAdaptHeadersBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfSBSAdaptNodesBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfSBSAdaptRootsBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfSBSDataBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfSBSDataFBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfSBSHeadersBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfSBSNodesBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfSBSRootsBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfSVSNodesBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfSVSRootsBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_abstractObjectPtrsBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_allNodePairsBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_geomDataBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_indicesBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_instanceDataBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_nodesTLASBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_origNodesBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_primIdCountBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_primIndicesBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_vertNormBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_vertPosBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_startEndBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_packedXYBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_verticesBuffer, nullptr);
  for(auto obj : m_vdata.m_texturesArrayTexture)
    vkDestroyImage(device, obj, nullptr);
  for(auto obj : m_vdata.m_texturesArrayView)
    vkDestroyImageView(device, obj, nullptr);
  for(auto obj : m_vdata.m_texturesArraySampler)
  vkDestroySampler(device, obj, nullptr);
  FreeAllAllocations(m_allMems);
}

void MultiRenderer_GPU::InitHelpers()
{
  vkGetPhysicalDeviceProperties(physicalDevice, &m_devProps);
}

VkBufferMemoryBarrier MultiRenderer_GPU::BarrierForObjCounters(VkBuffer a_buffer)
{
  VkBufferMemoryBarrier bar = {};
  bar.sType               = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
  bar.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  bar.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  bar.buffer              = a_buffer;
  bar.offset              = 0;
  bar.size                = VK_WHOLE_SIZE; // TODO: count offset and size carefully, actually we can do this!
  return bar;
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

void MultiRenderer_GPU::InitKernel_CastRayFloatSingleMega(const char* a_filePath)
{
  std::string shaderPath = AlterShaderPath("shaders_gpu/CastRayFloatSingleMega.comp.spv");
  const VkSpecializationInfo* kspec = nullptr;
  CastRayFloatSingleMegaDSLayout = CreateCastRayFloatSingleMegaDSLayout();
  if(m_megaKernelFlags.enableCastRayFloatSingleMega)
  {
    MakeComputePipelineAndLayout(shaderPath.c_str(), "main", kspec, CastRayFloatSingleMegaDSLayout, &CastRayFloatSingleMegaLayout, &CastRayFloatSingleMegaPipeline);
  }
  else
  {
    CastRayFloatSingleMegaLayout   = nullptr;
    CastRayFloatSingleMegaPipeline = nullptr;
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
  InitKernel_CastRayFloatSingleMega(a_filePath);
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
  if(all_references.capacity() == 0)
    all_references.reserve(4);
  if(m_geomOffsets.capacity() == 0)
    m_geomOffsets.reserve(4);
  if(m_indices.capacity() == 0)
    m_indices.reserve(4);
  if(m_instanceTransformInvTransposed.capacity() == 0)
    m_instanceTransformInvTransposed.reserve(4);
  if(m_lights.capacity() == 0)
    m_lights.reserve(4);
  if(m_matIdOffsets.capacity() == 0)
    m_matIdOffsets.reserve(4);
  if(m_matIdbyPrimId.capacity() == 0)
    m_matIdbyPrimId.reserve(4);
  if(m_materials.capacity() == 0)
    m_materials.reserve(4);
  if(m_normals.capacity() == 0)
    m_normals.reserve(4);
  if(m_pAccelStruct_m_GraphicsPrimHeaders != nullptr && m_pAccelStruct_m_GraphicsPrimHeaders->capacity() == 0)
    m_pAccelStruct_m_GraphicsPrimHeaders->reserve(4);
  if(m_pAccelStruct_m_GraphicsPrimPoints != nullptr && m_pAccelStruct_m_GraphicsPrimPoints->capacity() == 0)
    m_pAccelStruct_m_GraphicsPrimPoints->reserve(4);
  if(m_pAccelStruct_m_NURBSData != nullptr && m_pAccelStruct_m_NURBSData->capacity() == 0)
    m_pAccelStruct_m_NURBSData->reserve(4);
  if(m_pAccelStruct_m_NURBSHeaders != nullptr && m_pAccelStruct_m_NURBSHeaders->capacity() == 0)
    m_pAccelStruct_m_NURBSHeaders->reserve(4);
  if(m_pAccelStruct_m_NURBS_approxes != nullptr && m_pAccelStruct_m_NURBS_approxes->capacity() == 0)
    m_pAccelStruct_m_NURBS_approxes->reserve(4);
  if(m_pAccelStruct_m_SdfCompactOctreeRotModifiers != nullptr && m_pAccelStruct_m_SdfCompactOctreeRotModifiers->capacity() == 0)
    m_pAccelStruct_m_SdfCompactOctreeRotModifiers->reserve(4);
  if(m_pAccelStruct_m_SdfCompactOctreeV2Data != nullptr && m_pAccelStruct_m_SdfCompactOctreeV2Data->capacity() == 0)
    m_pAccelStruct_m_SdfCompactOctreeV2Data->reserve(4);
  if(m_pAccelStruct_m_SdfCompactOctreeV3Data != nullptr && m_pAccelStruct_m_SdfCompactOctreeV3Data->capacity() == 0)
    m_pAccelStruct_m_SdfCompactOctreeV3Data->reserve(4);
  if(m_pAccelStruct_m_SdfFrameOctreeNodes != nullptr && m_pAccelStruct_m_SdfFrameOctreeNodes->capacity() == 0)
    m_pAccelStruct_m_SdfFrameOctreeNodes->reserve(4);
  if(m_pAccelStruct_m_SdfFrameOctreeRoots != nullptr && m_pAccelStruct_m_SdfFrameOctreeRoots->capacity() == 0)
    m_pAccelStruct_m_SdfFrameOctreeRoots->reserve(4);
  if(m_pAccelStruct_m_SdfFrameOctreeTexNodes != nullptr && m_pAccelStruct_m_SdfFrameOctreeTexNodes->capacity() == 0)
    m_pAccelStruct_m_SdfFrameOctreeTexNodes->reserve(4);
  if(m_pAccelStruct_m_SdfFrameOctreeTexRoots != nullptr && m_pAccelStruct_m_SdfFrameOctreeTexRoots->capacity() == 0)
    m_pAccelStruct_m_SdfFrameOctreeTexRoots->reserve(4);
  if(m_pAccelStruct_m_SdfGridData != nullptr && m_pAccelStruct_m_SdfGridData->capacity() == 0)
    m_pAccelStruct_m_SdfGridData->reserve(4);
  if(m_pAccelStruct_m_SdfGridOffsets != nullptr && m_pAccelStruct_m_SdfGridOffsets->capacity() == 0)
    m_pAccelStruct_m_SdfGridOffsets->reserve(4);
  if(m_pAccelStruct_m_SdfGridSizes != nullptr && m_pAccelStruct_m_SdfGridSizes->capacity() == 0)
    m_pAccelStruct_m_SdfGridSizes->reserve(4);
  if(m_pAccelStruct_m_SdfSBSAdaptData != nullptr && m_pAccelStruct_m_SdfSBSAdaptData->capacity() == 0)
    m_pAccelStruct_m_SdfSBSAdaptData->reserve(4);
  if(m_pAccelStruct_m_SdfSBSAdaptDataF != nullptr && m_pAccelStruct_m_SdfSBSAdaptDataF->capacity() == 0)
    m_pAccelStruct_m_SdfSBSAdaptDataF->reserve(4);
  if(m_pAccelStruct_m_SdfSBSAdaptHeaders != nullptr && m_pAccelStruct_m_SdfSBSAdaptHeaders->capacity() == 0)
    m_pAccelStruct_m_SdfSBSAdaptHeaders->reserve(4);
  if(m_pAccelStruct_m_SdfSBSAdaptNodes != nullptr && m_pAccelStruct_m_SdfSBSAdaptNodes->capacity() == 0)
    m_pAccelStruct_m_SdfSBSAdaptNodes->reserve(4);
  if(m_pAccelStruct_m_SdfSBSAdaptRoots != nullptr && m_pAccelStruct_m_SdfSBSAdaptRoots->capacity() == 0)
    m_pAccelStruct_m_SdfSBSAdaptRoots->reserve(4);
  if(m_pAccelStruct_m_SdfSBSData != nullptr && m_pAccelStruct_m_SdfSBSData->capacity() == 0)
    m_pAccelStruct_m_SdfSBSData->reserve(4);
  if(m_pAccelStruct_m_SdfSBSDataF != nullptr && m_pAccelStruct_m_SdfSBSDataF->capacity() == 0)
    m_pAccelStruct_m_SdfSBSDataF->reserve(4);
  if(m_pAccelStruct_m_SdfSBSHeaders != nullptr && m_pAccelStruct_m_SdfSBSHeaders->capacity() == 0)
    m_pAccelStruct_m_SdfSBSHeaders->reserve(4);
  if(m_pAccelStruct_m_SdfSBSNodes != nullptr && m_pAccelStruct_m_SdfSBSNodes->capacity() == 0)
    m_pAccelStruct_m_SdfSBSNodes->reserve(4);
  if(m_pAccelStruct_m_SdfSBSRoots != nullptr && m_pAccelStruct_m_SdfSBSRoots->capacity() == 0)
    m_pAccelStruct_m_SdfSBSRoots->reserve(4);
  if(m_pAccelStruct_m_SdfSVSNodes != nullptr && m_pAccelStruct_m_SdfSVSNodes->capacity() == 0)
    m_pAccelStruct_m_SdfSVSNodes->reserve(4);
  if(m_pAccelStruct_m_SdfSVSRoots != nullptr && m_pAccelStruct_m_SdfSVSRoots->capacity() == 0)
    m_pAccelStruct_m_SdfSVSRoots->reserve(4);
  if(m_pAccelStruct_m_abstractObjectPtrs != nullptr && m_pAccelStruct_m_abstractObjectPtrs->capacity() == 0)
    m_pAccelStruct_m_abstractObjectPtrs->reserve(4);
  if(m_pAccelStruct_m_allNodePairs != nullptr && m_pAccelStruct_m_allNodePairs->capacity() == 0)
    m_pAccelStruct_m_allNodePairs->reserve(4);
  if(m_pAccelStruct_m_geomData != nullptr && m_pAccelStruct_m_geomData->capacity() == 0)
    m_pAccelStruct_m_geomData->reserve(4);
  if(m_pAccelStruct_m_indices != nullptr && m_pAccelStruct_m_indices->capacity() == 0)
    m_pAccelStruct_m_indices->reserve(4);
  if(m_pAccelStruct_m_instanceData != nullptr && m_pAccelStruct_m_instanceData->capacity() == 0)
    m_pAccelStruct_m_instanceData->reserve(4);
  if(m_pAccelStruct_m_nodesTLAS != nullptr && m_pAccelStruct_m_nodesTLAS->capacity() == 0)
    m_pAccelStruct_m_nodesTLAS->reserve(4);
  if(m_pAccelStruct_m_origNodes != nullptr && m_pAccelStruct_m_origNodes->capacity() == 0)
    m_pAccelStruct_m_origNodes->reserve(4);
  if(m_pAccelStruct_m_primIdCount != nullptr && m_pAccelStruct_m_primIdCount->capacity() == 0)
    m_pAccelStruct_m_primIdCount->reserve(4);
  if(m_pAccelStruct_m_primIndices != nullptr && m_pAccelStruct_m_primIndices->capacity() == 0)
    m_pAccelStruct_m_primIndices->reserve(4);
  if(m_pAccelStruct_m_vertNorm != nullptr && m_pAccelStruct_m_vertNorm->capacity() == 0)
    m_pAccelStruct_m_vertNorm->reserve(4);
  if(m_pAccelStruct_m_vertPos != nullptr && m_pAccelStruct_m_vertPos->capacity() == 0)
    m_pAccelStruct_m_vertPos->reserve(4);
  if(m_pAccelStruct_startEnd != nullptr && m_pAccelStruct_startEnd->capacity() == 0)
    m_pAccelStruct_startEnd->reserve(4);
  if(m_packedXY.capacity() == 0)
    m_packedXY.reserve(4);
  if(m_vertices.capacity() == 0)
    m_vertices.reserve(4);
}
static size_t GetSizeByTag_AbstractObject(uint32_t a_tag)
{
  switch(a_tag)
  {
    case AbstractObject::TAG_TRIANGLE: return sizeof(GeomDataTriangle);
    case AbstractObject::TAG_SDF_GRID: return sizeof(GeomDataSdfGrid);
    case AbstractObject::TAG_SDF_NODE: return sizeof(GeomDataSdfNode);
    case AbstractObject::TAG_SDF_BRICK: return sizeof(GeomDataSdfBrick);
    case AbstractObject::TAG_RF: return sizeof(GeomDataRF);
    case AbstractObject::TAG_GS: return sizeof(GeomDataGS);
    case AbstractObject::TAG_SDF_ADAPT_BRICK: return sizeof(GeomDataSdfAdaptBrick);
    case AbstractObject::TAG_NURBS: return sizeof(GeomDataNURBS);
    case AbstractObject::TAG_GRAPHICS_PRIM: return sizeof(GeomDataGraphicsPrim);
    case AbstractObject::TAG_COCTREE_SIMPLE: return sizeof(GeomDataCOctreeSimple);
    case AbstractObject::TAG_COCTREE_BRICKED: return sizeof(GeomDataCOctreeBricked);
    case AbstractObject::TAG_CATMUL_CLARK: return sizeof(GeomDataCatmulClark);
    case AbstractObject::TAG_RIBBON: return sizeof(GeomDataRibbon);
    default : return sizeof(EmptyGeomData);
  }
};

static size_t PackObject_AbstractObject(std::vector<uint8_t>& buffer, const AbstractObject* a_ptr) // todo: generate implementation via dynamic_cast or static_cast (can be used because we know the type)
{
  const size_t objSize  = GetSizeByTag_AbstractObject(a_ptr->GetTag());
  const size_t currSize = buffer.size();
  const size_t nextSize = buffer.size() + objSize - sizeof(void*); // minus vptr size
  buffer.resize(nextSize);
  const char* objData = ((const char*)a_ptr) + sizeof(void*);      // do not account for vptr
  memcpy(buffer.data() + currSize, objData, objSize - sizeof(void*)); 
  return objSize;
}


void MultiRenderer_GPU::InitMemberBuffers()
{
  std::vector<VkBuffer> memberVectorsWithDevAddr;
  std::vector<VkBuffer> memberVectors;
  std::vector<VkImage>  memberTextures;
  
  if(m_pAccelStruct_m_abstractObjectPtrs_sorted.empty()) // Pack all objects of 'AbstractObject'
  {
    auto& bufferV = m_pAccelStruct_m_abstractObjectPtrs_dataV;
    auto& sorted  = m_pAccelStruct_m_abstractObjectPtrs_sorted;
    auto& vtable  = m_pAccelStruct_m_abstractObjectPtrs_vtable;
    vtable.resize(m_pAccelStruct_m_abstractObjectPtrs->size());
    //sorted.resize(13 + 1);
    bufferV.resize(16*4); // (m_pAccelStruct_m_abstractObjectPtrs.size()*sizeof(m_pAccelStruct_m_abstractObjectPtrs)); actual reserve may not be needed due to implementation don't have vectors. TODO: you may cvheck this explicitly in kslicer
    for(size_t arrId=0;arrId<sorted.size(); arrId++) {
      sorted[arrId].reserve(m_pAccelStruct_m_abstractObjectPtrs->size()*sizeof(AbstractObject));
      sorted[arrId].resize(0);
    }
    
    std::unordered_map<uint32_t, uint32_t> objCount;

    for(size_t i=0;i<m_pAccelStruct_m_abstractObjectPtrs->size();i++) 
    {
      const auto tag = m_pAccelStruct_m_abstractObjectPtrs->at(i)->GetTag(); 
      PackObject_AbstractObject(sorted[tag], m_pAccelStruct_m_abstractObjectPtrs->at(i));

      auto p = objCount.find(tag);
      if(p == objCount.end())
        p = objCount.insert(std::make_pair(tag,0)).first;

      vtable[i] = LiteMath::uint2(tag, uint32_t(p->second));
      p->second++;
    }

    const size_t buffReferenceAlign = 16; // from EXT_buffer_reference spec: "If the layout qualifier is not specified, it defaults to 16 bytes"
    size_t objDataBufferSize = 0;
    m_pAccelStruct_m_abstractObjectPtrs_obj_storage_offsets.reserve(sorted.size());
    for(auto it = sorted.begin(); it != sorted.end(); ++it)
    {
      m_pAccelStruct_m_abstractObjectPtrs_obj_storage_offsets[it->first] = objDataBufferSize;
      objDataBufferSize += vk_utils::getPaddedSize(it->second.size(), buffReferenceAlign);
    }

    m_vdata.m_pAccelStruct_m_abstractObjectPtrs_dataSBuffer = vk_utils::createBuffer(device, objDataBufferSize, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT);
    m_vdata.m_pAccelStruct_m_abstractObjectPtrs_dataVBuffer = vk_utils::createBuffer(device, bufferV.size(), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
    m_vdata.m_pAccelStruct_m_abstractObjectPtrs_dataSOffset = 0;
    m_vdata.m_pAccelStruct_m_abstractObjectPtrs_dataVOffset = 0;
    memberVectorsWithDevAddr.push_back(m_vdata.m_pAccelStruct_m_abstractObjectPtrs_dataSBuffer);
    memberVectors.push_back(m_vdata.m_pAccelStruct_m_abstractObjectPtrs_dataVBuffer);
  }
  
  all_references.resize(1); // need just single element to store all references

  m_vdata.all_referencesBuffer = vk_utils::createBuffer(device, all_references.capacity()*sizeof(AllBufferReferences), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.all_referencesBuffer);
  m_vdata.m_geomOffsetsBuffer = vk_utils::createBuffer(device, m_geomOffsets.capacity()*sizeof(struct LiteMath::uint2), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_geomOffsetsBuffer);
  m_vdata.m_indicesBuffer = vk_utils::createBuffer(device, m_indices.capacity()*sizeof(unsigned int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_indicesBuffer);
  m_vdata.m_instanceTransformInvTransposedBuffer = vk_utils::createBuffer(device, m_instanceTransformInvTransposed.capacity()*sizeof(struct LiteMath::float4x4), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_instanceTransformInvTransposedBuffer);
  m_vdata.m_lightsBuffer = vk_utils::createBuffer(device, m_lights.capacity()*sizeof(struct Light), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_lightsBuffer);
  m_vdata.m_matIdOffsetsBuffer = vk_utils::createBuffer(device, m_matIdOffsets.capacity()*sizeof(struct LiteMath::uint2), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_matIdOffsetsBuffer);
  m_vdata.m_matIdbyPrimIdBuffer = vk_utils::createBuffer(device, m_matIdbyPrimId.capacity()*sizeof(unsigned int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_matIdbyPrimIdBuffer);
  m_vdata.m_materialsBuffer = vk_utils::createBuffer(device, m_materials.capacity()*sizeof(struct MultiRendererMaterial), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_materialsBuffer);
  m_vdata.m_normalsBuffer = vk_utils::createBuffer(device, m_normals.capacity()*sizeof(struct LiteMath::float4), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_normalsBuffer);
  m_vdata.m_pAccelStruct_m_GraphicsPrimHeadersBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_GraphicsPrimHeaders->capacity()*sizeof(struct GraphicsPrimHeader), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_GraphicsPrimHeadersBuffer);
  m_vdata.m_pAccelStruct_m_GraphicsPrimPointsBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_GraphicsPrimPoints->capacity()*sizeof(struct LiteMath::float4), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_GraphicsPrimPointsBuffer);
  m_vdata.m_pAccelStruct_m_NURBSDataBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_NURBSData->capacity()*sizeof(float), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_NURBSDataBuffer);
  m_vdata.m_pAccelStruct_m_NURBSHeadersBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_NURBSHeaders->capacity()*sizeof(struct NURBSHeader), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_NURBSHeadersBuffer);
  m_vdata.m_pAccelStruct_m_NURBS_approxesBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_NURBS_approxes->capacity()*sizeof(float), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_NURBS_approxesBuffer);
  m_vdata.m_pAccelStruct_m_SdfCompactOctreeRotModifiersBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfCompactOctreeRotModifiers->capacity()*sizeof(struct LiteMath::int4), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfCompactOctreeRotModifiersBuffer);
  m_vdata.m_pAccelStruct_m_SdfCompactOctreeV2DataBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfCompactOctreeV2Data->capacity()*sizeof(unsigned int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfCompactOctreeV2DataBuffer);
  m_vdata.m_pAccelStruct_m_SdfCompactOctreeV3DataBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfCompactOctreeV3Data->capacity()*sizeof(unsigned int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfCompactOctreeV3DataBuffer);
  m_vdata.m_pAccelStruct_m_SdfFrameOctreeNodesBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfFrameOctreeNodes->capacity()*sizeof(struct SdfFrameOctreeNode), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfFrameOctreeNodesBuffer);
  m_vdata.m_pAccelStruct_m_SdfFrameOctreeRootsBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfFrameOctreeRoots->capacity()*sizeof(unsigned int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfFrameOctreeRootsBuffer);
  m_vdata.m_pAccelStruct_m_SdfFrameOctreeTexNodesBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfFrameOctreeTexNodes->capacity()*sizeof(struct SdfFrameOctreeTexNode), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfFrameOctreeTexNodesBuffer);
  m_vdata.m_pAccelStruct_m_SdfFrameOctreeTexRootsBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfFrameOctreeTexRoots->capacity()*sizeof(unsigned int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfFrameOctreeTexRootsBuffer);
  m_vdata.m_pAccelStruct_m_SdfGridDataBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfGridData->capacity()*sizeof(float), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfGridDataBuffer);
  m_vdata.m_pAccelStruct_m_SdfGridOffsetsBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfGridOffsets->capacity()*sizeof(unsigned int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfGridOffsetsBuffer);
  m_vdata.m_pAccelStruct_m_SdfGridSizesBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfGridSizes->capacity()*sizeof(struct LiteMath::uint3), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfGridSizesBuffer);
  m_vdata.m_pAccelStruct_m_SdfSBSAdaptDataBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfSBSAdaptData->capacity()*sizeof(unsigned int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfSBSAdaptDataBuffer);
  m_vdata.m_pAccelStruct_m_SdfSBSAdaptDataFBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfSBSAdaptDataF->capacity()*sizeof(float), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfSBSAdaptDataFBuffer);
  m_vdata.m_pAccelStruct_m_SdfSBSAdaptHeadersBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfSBSAdaptHeaders->capacity()*sizeof(struct SdfSBSAdaptHeader), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfSBSAdaptHeadersBuffer);
  m_vdata.m_pAccelStruct_m_SdfSBSAdaptNodesBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfSBSAdaptNodes->capacity()*sizeof(struct SdfSBSAdaptNode), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfSBSAdaptNodesBuffer);
  m_vdata.m_pAccelStruct_m_SdfSBSAdaptRootsBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfSBSAdaptRoots->capacity()*sizeof(unsigned int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfSBSAdaptRootsBuffer);
  m_vdata.m_pAccelStruct_m_SdfSBSDataBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfSBSData->capacity()*sizeof(unsigned int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfSBSDataBuffer);
  m_vdata.m_pAccelStruct_m_SdfSBSDataFBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfSBSDataF->capacity()*sizeof(float), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfSBSDataFBuffer);
  m_vdata.m_pAccelStruct_m_SdfSBSHeadersBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfSBSHeaders->capacity()*sizeof(struct SdfSBSHeader), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfSBSHeadersBuffer);
  m_vdata.m_pAccelStruct_m_SdfSBSNodesBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfSBSNodes->capacity()*sizeof(struct SdfSBSNode), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfSBSNodesBuffer);
  m_vdata.m_pAccelStruct_m_SdfSBSRootsBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfSBSRoots->capacity()*sizeof(unsigned int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfSBSRootsBuffer);
  m_vdata.m_pAccelStruct_m_SdfSVSNodesBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfSVSNodes->capacity()*sizeof(struct SdfSVSNode), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfSVSNodesBuffer);
  m_vdata.m_pAccelStruct_m_SdfSVSRootsBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_SdfSVSRoots->capacity()*sizeof(unsigned int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_SdfSVSRootsBuffer);
  m_vdata.m_pAccelStruct_m_abstractObjectPtrsBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_abstractObjectPtrs->capacity()*sizeof(struct AbstractObject *), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_abstractObjectPtrsBuffer);
  m_vdata.m_pAccelStruct_m_allNodePairsBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_allNodePairs->capacity()*sizeof(struct BVHNodePair), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_allNodePairsBuffer);
  m_vdata.m_pAccelStruct_m_geomDataBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_geomData->capacity()*sizeof(struct GeomData), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_geomDataBuffer);
  m_vdata.m_pAccelStruct_m_indicesBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_indices->capacity()*sizeof(unsigned int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_indicesBuffer);
  m_vdata.m_pAccelStruct_m_instanceDataBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_instanceData->capacity()*sizeof(struct InstanceData), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_instanceDataBuffer);
  m_vdata.m_pAccelStruct_m_nodesTLASBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_nodesTLAS->capacity()*sizeof(struct BVHNode), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_nodesTLASBuffer);
  m_vdata.m_pAccelStruct_m_origNodesBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_origNodes->capacity()*sizeof(struct BVHNode), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_origNodesBuffer);
  m_vdata.m_pAccelStruct_m_primIdCountBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_primIdCount->capacity()*sizeof(unsigned int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_primIdCountBuffer);
  m_vdata.m_pAccelStruct_m_primIndicesBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_primIndices->capacity()*sizeof(unsigned int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_primIndicesBuffer);
  m_vdata.m_pAccelStruct_m_vertNormBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_vertNorm->capacity()*sizeof(struct LiteMath::float4), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_vertNormBuffer);
  m_vdata.m_pAccelStruct_m_vertPosBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_vertPos->capacity()*sizeof(struct LiteMath::float4), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_vertPosBuffer);
  m_vdata.m_pAccelStruct_startEndBuffer = vk_utils::createBuffer(device, m_pAccelStruct_startEnd->capacity()*sizeof(struct LiteMath::uint2), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_startEndBuffer);
  m_vdata.m_packedXYBuffer = vk_utils::createBuffer(device, m_packedXY.capacity()*sizeof(unsigned int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_packedXYBuffer);
  m_vdata.m_verticesBuffer = vk_utils::createBuffer(device, m_vertices.capacity()*sizeof(struct LiteMath::float4), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_verticesBuffer);

  
  m_vdata.m_texturesArrayTexture.resize(0);
  m_vdata.m_texturesArrayView.resize(0);
  m_vdata.m_texturesArraySampler.resize(0);
  m_vdata.m_texturesArrayTexture.reserve(64);
  m_vdata.m_texturesArrayView.reserve(64);
  m_vdata.m_texturesArraySampler.reserve(64);
  for(auto imageObj : m_textures)
  {
    auto tex = CreateTexture2D(imageObj->width(), imageObj->height(), VkFormat(imageObj->format()), VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT);
    auto sam = CreateSampler(imageObj->sampler());
    m_vdata.m_texturesArrayTexture.push_back(tex);
    m_vdata.m_texturesArrayView.push_back(VK_NULL_HANDLE);
    m_vdata.m_texturesArraySampler.push_back(sam);
    memberTextures.push_back(tex);
  }


  AllocMemoryForMemberBuffersAndImages(memberVectors, memberTextures);
  if(memberVectorsWithDevAddr.size() != 0)
    AllocAndBind(memberVectorsWithDevAddr, VK_MEMORY_ALLOCATE_DEVICE_ADDRESS_BIT);
  {
    all_references[0].GeomDataTriangleAddress = vk_rt_utils::getBufferDeviceAddress(device, m_vdata.m_pAccelStruct_m_abstractObjectPtrs_dataSBuffer) + m_pAccelStruct_m_abstractObjectPtrs_obj_storage_offsets[AbstractObject::TAG_TRIANGLE];
    all_references[0].GeomDataSdfGridAddress = vk_rt_utils::getBufferDeviceAddress(device, m_vdata.m_pAccelStruct_m_abstractObjectPtrs_dataSBuffer) + m_pAccelStruct_m_abstractObjectPtrs_obj_storage_offsets[AbstractObject::TAG_SDF_GRID];
    all_references[0].GeomDataSdfNodeAddress = vk_rt_utils::getBufferDeviceAddress(device, m_vdata.m_pAccelStruct_m_abstractObjectPtrs_dataSBuffer) + m_pAccelStruct_m_abstractObjectPtrs_obj_storage_offsets[AbstractObject::TAG_SDF_NODE];
    all_references[0].GeomDataSdfBrickAddress = vk_rt_utils::getBufferDeviceAddress(device, m_vdata.m_pAccelStruct_m_abstractObjectPtrs_dataSBuffer) + m_pAccelStruct_m_abstractObjectPtrs_obj_storage_offsets[AbstractObject::TAG_SDF_BRICK];
    all_references[0].GeomDataRFAddress = vk_rt_utils::getBufferDeviceAddress(device, m_vdata.m_pAccelStruct_m_abstractObjectPtrs_dataSBuffer) + m_pAccelStruct_m_abstractObjectPtrs_obj_storage_offsets[AbstractObject::TAG_RF];
    all_references[0].GeomDataGSAddress = vk_rt_utils::getBufferDeviceAddress(device, m_vdata.m_pAccelStruct_m_abstractObjectPtrs_dataSBuffer) + m_pAccelStruct_m_abstractObjectPtrs_obj_storage_offsets[AbstractObject::TAG_GS];
    all_references[0].GeomDataSdfAdaptBrickAddress = vk_rt_utils::getBufferDeviceAddress(device, m_vdata.m_pAccelStruct_m_abstractObjectPtrs_dataSBuffer) + m_pAccelStruct_m_abstractObjectPtrs_obj_storage_offsets[AbstractObject::TAG_SDF_ADAPT_BRICK];
    all_references[0].GeomDataNURBSAddress = vk_rt_utils::getBufferDeviceAddress(device, m_vdata.m_pAccelStruct_m_abstractObjectPtrs_dataSBuffer) + m_pAccelStruct_m_abstractObjectPtrs_obj_storage_offsets[AbstractObject::TAG_NURBS];
    all_references[0].GeomDataGraphicsPrimAddress = vk_rt_utils::getBufferDeviceAddress(device, m_vdata.m_pAccelStruct_m_abstractObjectPtrs_dataSBuffer) + m_pAccelStruct_m_abstractObjectPtrs_obj_storage_offsets[AbstractObject::TAG_GRAPHICS_PRIM];
    all_references[0].GeomDataCOctreeSimpleAddress = vk_rt_utils::getBufferDeviceAddress(device, m_vdata.m_pAccelStruct_m_abstractObjectPtrs_dataSBuffer) + m_pAccelStruct_m_abstractObjectPtrs_obj_storage_offsets[AbstractObject::TAG_COCTREE_SIMPLE];
    all_references[0].GeomDataCOctreeBrickedAddress = vk_rt_utils::getBufferDeviceAddress(device, m_vdata.m_pAccelStruct_m_abstractObjectPtrs_dataSBuffer) + m_pAccelStruct_m_abstractObjectPtrs_obj_storage_offsets[AbstractObject::TAG_COCTREE_BRICKED];
    all_references[0].GeomDataCatmulClarkAddress = vk_rt_utils::getBufferDeviceAddress(device, m_vdata.m_pAccelStruct_m_abstractObjectPtrs_dataSBuffer) + m_pAccelStruct_m_abstractObjectPtrs_obj_storage_offsets[AbstractObject::TAG_CATMUL_CLARK];
    all_references[0].GeomDataRibbonAddress = vk_rt_utils::getBufferDeviceAddress(device, m_vdata.m_pAccelStruct_m_abstractObjectPtrs_dataSBuffer) + m_pAccelStruct_m_abstractObjectPtrs_obj_storage_offsets[AbstractObject::TAG_RIBBON];
  }
  for(size_t i = 0; i < m_textures.size(); i++)
    m_vdata.m_texturesArrayView[i] = CreateView(VkFormat(m_textures[i]->format()), m_vdata.m_texturesArrayTexture[i]);
}



VkImage MultiRenderer_GPU::CreateTexture2D(const int a_width, const int a_height, VkFormat a_format, VkImageUsageFlags a_usage)
{
  VkImage result = VK_NULL_HANDLE;
  VkImageCreateInfo imgCreateInfo = {};
  imgCreateInfo.sType         = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
  imgCreateInfo.pNext         = nullptr;
  imgCreateInfo.flags         = 0; // not sure about this ...
  imgCreateInfo.imageType     = VK_IMAGE_TYPE_2D;
  imgCreateInfo.format        = a_format;
  imgCreateInfo.extent        = VkExtent3D{uint32_t(a_width), uint32_t(a_height), 1};
  imgCreateInfo.mipLevels     = 1;
  imgCreateInfo.samples       = VK_SAMPLE_COUNT_1_BIT;
  imgCreateInfo.tiling        = VK_IMAGE_TILING_OPTIMAL;
  imgCreateInfo.usage         = a_usage;
  imgCreateInfo.sharingMode   = VK_SHARING_MODE_EXCLUSIVE;
  imgCreateInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  imgCreateInfo.arrayLayers   = 1;
  VK_CHECK_RESULT(vkCreateImage(device, &imgCreateInfo, nullptr, &result));
  return result;
}

VkSampler MultiRenderer_GPU::CreateSampler(const Sampler& a_sampler) // TODO: implement this function correctly
{
  VkSampler result = VK_NULL_HANDLE;
  VkSamplerCreateInfo samplerInfo = {};
  samplerInfo.sType        = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
  samplerInfo.pNext        = nullptr;
  samplerInfo.flags        = 0;
  samplerInfo.magFilter    = VkFilter(int(a_sampler.filter));
  samplerInfo.minFilter    = VkFilter(int(a_sampler.filter));
  samplerInfo.mipmapMode   = (samplerInfo.magFilter == VK_FILTER_LINEAR ) ? VK_SAMPLER_MIPMAP_MODE_LINEAR : VK_SAMPLER_MIPMAP_MODE_NEAREST;
  samplerInfo.addressModeU = VkSamplerAddressMode(int(a_sampler.addressU));
  samplerInfo.addressModeV = VkSamplerAddressMode(int(a_sampler.addressV));
  samplerInfo.addressModeW = VkSamplerAddressMode(int(a_sampler.addressW));
  samplerInfo.mipLodBias   = a_sampler.mipLODBias;
  samplerInfo.compareOp    = VK_COMPARE_OP_NEVER;
  samplerInfo.minLod           = a_sampler.minLOD;
  samplerInfo.maxLod           = a_sampler.maxLOD;
  samplerInfo.maxAnisotropy    = a_sampler.maxAnisotropy;
  samplerInfo.anisotropyEnable = (a_sampler.maxAnisotropy > 1) ? VK_TRUE : VK_FALSE;
  samplerInfo.borderColor      = VK_BORDER_COLOR_FLOAT_OPAQUE_BLACK;
  samplerInfo.unnormalizedCoordinates = VK_FALSE;
  VK_CHECK_RESULT(vkCreateSampler(device, &samplerInfo, nullptr, &result));
  return result;
}

VkImageView MultiRenderer_GPU::CreateView(VkFormat a_format, VkImage a_image)
{
  VkImageView result = VK_NULL_HANDLE;
  VkImageViewCreateInfo createInfo{};
  createInfo.sType    = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
  createInfo.image    = a_image;
  createInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
  createInfo.format   = a_format;

  if(a_format == VK_FORMAT_R32_SFLOAT || a_format == VK_FORMAT_R8_UNORM  || a_format == VK_FORMAT_R8_SNORM ||
     a_format == VK_FORMAT_R16_SFLOAT || a_format == VK_FORMAT_R16_UNORM || a_format == VK_FORMAT_R16_SNORM)
  {
    createInfo.components.r = VK_COMPONENT_SWIZZLE_R;
    createInfo.components.g = VK_COMPONENT_SWIZZLE_R;
    createInfo.components.b = VK_COMPONENT_SWIZZLE_R;
    createInfo.components.a = VK_COMPONENT_SWIZZLE_R;
  }
  else
  {
    createInfo.components.r = VK_COMPONENT_SWIZZLE_IDENTITY;
    createInfo.components.g = VK_COMPONENT_SWIZZLE_IDENTITY;
    createInfo.components.b = VK_COMPONENT_SWIZZLE_IDENTITY;
    createInfo.components.a = VK_COMPONENT_SWIZZLE_IDENTITY;
  }

  createInfo.subresourceRange.aspectMask     = VK_IMAGE_ASPECT_COLOR_BIT;
  createInfo.subresourceRange.baseMipLevel   = 0;
  createInfo.subresourceRange.levelCount     = 1;
  createInfo.subresourceRange.baseArrayLayer = 0;
  createInfo.subresourceRange.layerCount     = 1;

  VK_CHECK_RESULT(vkCreateImageView(device, &createInfo, nullptr, &result));
  return result;
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

MultiRenderer_GPU::MemLoc MultiRenderer_GPU::AllocAndBind(const std::vector<VkBuffer>& a_buffers, VkMemoryAllocateFlags a_flags)
{
  MemLoc currLoc;
  if(a_buffers.size() > 0)
  {
    currLoc.memObject = vk_utils::allocateAndBindWithPadding(device, physicalDevice, a_buffers, a_flags);
    currLoc.allocId   = m_allMems.size();
    m_allMems.push_back(currLoc);
  }
  return currLoc;
}

MultiRenderer_GPU::MemLoc MultiRenderer_GPU::AllocAndBind(const std::vector<VkImage>& a_images, VkMemoryAllocateFlags a_flags)
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

  std::vector<VkFormat>             formats;  formats.reserve(0);
  std::vector<VkImageView*>         views;    views.reserve(0);
  std::vector<VkImage>              textures; textures.reserve(0);
  VkMemoryRequirements memoryRequirements;

  for(size_t i=0;i< m_vdata.m_texturesArrayTexture.size(); i++)
  {
    formats.push_back (VkFormat(m_textures[i]->format()));
    views.push_back   (&m_vdata.m_texturesArrayView[i]);
    textures.push_back(m_vdata.m_texturesArrayTexture[i]);
  }

  AllocAndBind(textures);
  for(size_t i=0;i<textures.size();i++)
  {
    VkImageViewCreateInfo imageViewInfo = {};
    imageViewInfo.sType                           = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    imageViewInfo.flags                           = 0;
    imageViewInfo.viewType                        = VK_IMAGE_VIEW_TYPE_2D;
    imageViewInfo.format                          = formats[i];
    imageViewInfo.components                      = { VK_COMPONENT_SWIZZLE_R, VK_COMPONENT_SWIZZLE_G, VK_COMPONENT_SWIZZLE_B, VK_COMPONENT_SWIZZLE_A };
    imageViewInfo.subresourceRange.aspectMask     = VK_IMAGE_ASPECT_COLOR_BIT;
    imageViewInfo.subresourceRange.baseMipLevel   = 0;
    imageViewInfo.subresourceRange.baseArrayLayer = 0;
    imageViewInfo.subresourceRange.layerCount     = 1;
    imageViewInfo.subresourceRange.levelCount     = 1;
    imageViewInfo.image                           = textures[i];     // The view will be based on the texture's image
    VK_CHECK_RESULT(vkCreateImageView(device, &imageViewInfo, nullptr, views[i]));
  }
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
  static VkPhysicalDeviceBufferDeviceAddressFeaturesKHR bufferDeviceAddressFeatures = {};
  bufferDeviceAddressFeatures.sType               = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_BUFFER_DEVICE_ADDRESS_FEATURES_KHR;
  bufferDeviceAddressFeatures.bufferDeviceAddress = VK_TRUE;
  (*ppNext) = &bufferDeviceAddressFeatures; ppNext = &bufferDeviceAddressFeatures.pNext;
  deviceExtensions.push_back(VK_KHR_BUFFER_DEVICE_ADDRESS_EXTENSION_NAME);
  static VkPhysicalDeviceDescriptorIndexingFeatures indexingFeatures{};
  indexingFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_DESCRIPTOR_INDEXING_FEATURES;
  indexingFeatures.shaderSampledImageArrayNonUniformIndexing = VK_TRUE;
  indexingFeatures.runtimeDescriptorArray                    = VK_TRUE;
  (*ppNext) = &indexingFeatures; ppNext = &indexingFeatures.pNext;
  deviceExtensions.push_back(VK_EXT_DESCRIPTOR_INDEXING_EXTENSION_NAME);
  return features2;
}

MultiRenderer_GPU::MegaKernelIsEnabled MultiRenderer_GPU::m_megaKernelFlags;

