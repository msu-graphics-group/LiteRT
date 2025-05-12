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
#include "integrator_pt_generated.h"
#include "include/Integrator_generated_ubo.h"


std::shared_ptr<Integrator> CreateIntegrator_Generated(int a_maxThreads, std::vector<uint32_t> a_features, vk_utils::VulkanContext a_ctx, size_t a_maxThreadsGenerated)
{
  auto pObj = std::make_shared<Integrator_Generated>(a_maxThreads, a_features);
  pObj->SetVulkanContext(a_ctx);
  pObj->InitVulkanObjects(a_ctx.device, a_ctx.physicalDevice, a_maxThreadsGenerated);
  return pObj;
}

void Integrator_Generated::InitVulkanObjects(VkDevice a_device, VkPhysicalDevice a_physicalDevice, size_t a_maxThreadsCount)
{
  physicalDevice = a_physicalDevice;
  device         = a_device;
  m_allCreatedPipelineLayouts.reserve(256);
  m_allCreatedPipelines.reserve(256);
  m_allSpecConstVals = ListRequiredFeatures();
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

VkBufferUsageFlags Integrator_Generated::GetAdditionalFlagsForUBO() const
{
  return VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
}

uint32_t Integrator_Generated::GetDefaultMaxTextures() const { return 256; }

void Integrator_Generated::MakeComputePipelineAndLayout(const char* a_shaderPath, const char* a_mainName, const VkSpecializationInfo *a_specInfo, const VkDescriptorSetLayout a_dsLayout, VkPipelineLayout* pPipelineLayout, VkPipeline* pPipeline)
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

void Integrator_Generated::MakeComputePipelineOnly(const char* a_shaderPath, const char* a_mainName, const VkSpecializationInfo *a_specInfo, const VkDescriptorSetLayout a_dsLayout, VkPipelineLayout pipelineLayout, VkPipeline* pPipeline)
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

Integrator_Generated::~Integrator_Generated()
{
  for(size_t i=0;i<m_allCreatedPipelines.size();i++)
    vkDestroyPipeline(device, m_allCreatedPipelines[i], nullptr);
  for(size_t i=0;i<m_allCreatedPipelineLayouts.size();i++)
    vkDestroyPipelineLayout(device, m_allCreatedPipelineLayouts[i], nullptr);

  vkDestroyDescriptorSetLayout(device, RayTraceMegaDSLayout, nullptr);
  RayTraceMegaDSLayout = VK_NULL_HANDLE;
  vkDestroyDescriptorSetLayout(device, CastSingleRayMegaDSLayout, nullptr);
  CastSingleRayMegaDSLayout = VK_NULL_HANDLE;
  vkDestroyDescriptorSetLayout(device, PackXYMegaDSLayout, nullptr);
  PackXYMegaDSLayout = VK_NULL_HANDLE;
  vkDestroyDescriptorSetLayout(device, PathTraceFromInputRaysMegaDSLayout, nullptr);
  PathTraceFromInputRaysMegaDSLayout = VK_NULL_HANDLE;
  vkDestroyDescriptorSetLayout(device, PathTraceMegaDSLayout, nullptr);
  PathTraceMegaDSLayout = VK_NULL_HANDLE;
  vkDestroyDescriptorSetLayout(device, NaivePathTraceMegaDSLayout, nullptr);
  NaivePathTraceMegaDSLayout = VK_NULL_HANDLE;
  vkDestroyDescriptorPool(device, m_dsPool, NULL); m_dsPool = VK_NULL_HANDLE;


  vkDestroyBuffer(device, m_classDataBuffer, nullptr);

  vkDestroyBuffer(device, m_vdata.all_referencesBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_allRemapListsBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_allRemapListsOffsetsBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_cie_xBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_cie_yBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_cie_zBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_films_eta_k_vecBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_films_spec_id_vecBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_instIdToLightInstIdBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_lightsBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_linesBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_matIdByPrimIdBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_matIdOffsetsBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_materialsBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_normMatricesBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_normMatrices2Buffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_CatmulClarkHeadersBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_NURBSDataBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_NURBSHeadersBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_NURBS_approxesBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_RibbonHeadersBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfCompactOctreeRotModifiersBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfCompactOctreeV2DataBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfCompactOctreeV3DataBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfFrameOctreeNodesBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_pAccelStruct_m_SdfFrameOctreeRootsBuffer, nullptr);
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
  vkDestroyBuffer(device, m_vdata.m_pdfLightDataBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_precomp_coat_transmittanceBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_precomp_thin_filmsBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_randomGensBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_remapInstBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_spec_offset_szBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_spec_tex_ids_wavelengthsBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_spec_tex_offset_szBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_spec_valuesBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_triIndicesBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_vNorm4fBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_vTang4fBuffer, nullptr);
  vkDestroyBuffer(device, m_vdata.m_vertOffsetBuffer, nullptr);
  for(auto obj : m_vdata.m_texturesArrayTexture)
    vkDestroyImage(device, obj, nullptr);
  for(auto obj : m_vdata.m_texturesArrayView)
    vkDestroyImageView(device, obj, nullptr);
  for(auto obj : m_vdata.m_texturesArraySampler)
  vkDestroySampler(device, obj, nullptr);
  FreeAllAllocations(m_allMems);
}

void Integrator_Generated::InitHelpers()
{
  vkGetPhysicalDeviceProperties(physicalDevice, &m_devProps);
}

VkBufferMemoryBarrier Integrator_Generated::BarrierForObjCounters(VkBuffer a_buffer)
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
const VkSpecializationInfo* Integrator_Generated::GetAllSpecInfo()
{
  if(m_allSpecConstInfo.size() == m_allSpecConstVals.size()) // already processed
    return &m_allSpecInfo;
  m_allSpecConstInfo.resize(m_allSpecConstVals.size());
  m_allSpecConstInfo[0].constantID = 0;
  m_allSpecConstInfo[0].size       = sizeof(uint32_t);
  m_allSpecConstInfo[0].offset     = 0;
  m_allSpecConstInfo[1].constantID = 1;
  m_allSpecConstInfo[1].size       = sizeof(uint32_t);
  m_allSpecConstInfo[1].offset     = 1*sizeof(uint32_t);
  m_allSpecConstInfo[2].constantID = 2;
  m_allSpecConstInfo[2].size       = sizeof(uint32_t);
  m_allSpecConstInfo[2].offset     = 2*sizeof(uint32_t);
  m_allSpecConstInfo[3].constantID = 3;
  m_allSpecConstInfo[3].size       = sizeof(uint32_t);
  m_allSpecConstInfo[3].offset     = 3*sizeof(uint32_t);
  m_allSpecConstInfo[4].constantID = 4;
  m_allSpecConstInfo[4].size       = sizeof(uint32_t);
  m_allSpecConstInfo[4].offset     = 4*sizeof(uint32_t);
  m_allSpecConstInfo[5].constantID = 5;
  m_allSpecConstInfo[5].size       = sizeof(uint32_t);
  m_allSpecConstInfo[5].offset     = 5*sizeof(uint32_t);
  m_allSpecConstInfo[6].constantID = 6;
  m_allSpecConstInfo[6].size       = sizeof(uint32_t);
  m_allSpecConstInfo[6].offset     = 6*sizeof(uint32_t);
  m_allSpecConstInfo[7].constantID = 7;
  m_allSpecConstInfo[7].size       = sizeof(uint32_t);
  m_allSpecConstInfo[7].offset     = 7*sizeof(uint32_t);
  m_allSpecConstInfo[8].constantID = 8;
  m_allSpecConstInfo[8].size       = sizeof(uint32_t);
  m_allSpecConstInfo[8].offset     = 8*sizeof(uint32_t);
  m_allSpecConstInfo[9].constantID = 9;
  m_allSpecConstInfo[9].size       = sizeof(uint32_t);
  m_allSpecConstInfo[9].offset     = 9*sizeof(uint32_t);
  m_allSpecConstInfo[10].constantID = 10;
  m_allSpecConstInfo[10].size       = sizeof(uint32_t);
  m_allSpecConstInfo[10].offset     = 10*sizeof(uint32_t);
  m_allSpecConstInfo[11].constantID = 11;
  m_allSpecConstInfo[11].size       = sizeof(uint32_t);
  m_allSpecConstInfo[11].offset     = 11*sizeof(uint32_t);
  m_allSpecConstInfo[12].constantID = 12;
  m_allSpecConstInfo[12].size       = sizeof(uint32_t);
  m_allSpecConstInfo[12].offset     = 12*sizeof(uint32_t);
  m_allSpecConstInfo[13].constantID = 13;
  m_allSpecConstInfo[13].size       = sizeof(uint32_t);
  m_allSpecConstInfo[13].offset     = 13*sizeof(uint32_t);
  m_allSpecConstInfo[14].constantID = 14;
  m_allSpecConstInfo[14].size       = sizeof(uint32_t);
  m_allSpecConstInfo[14].offset     = 14*sizeof(uint32_t);
  m_allSpecConstInfo[15].constantID = 15;
  m_allSpecConstInfo[15].size       = sizeof(uint32_t);
  m_allSpecConstInfo[15].offset     = 15*sizeof(uint32_t);
  m_allSpecConstInfo[16].constantID = 16;
  m_allSpecConstInfo[16].size       = sizeof(uint32_t);
  m_allSpecConstInfo[16].offset     = 16*sizeof(uint32_t);
  m_allSpecConstInfo[17].constantID = 17;
  m_allSpecConstInfo[17].size       = sizeof(uint32_t);
  m_allSpecConstInfo[17].offset     = 17*sizeof(uint32_t);
  m_allSpecConstInfo[18].constantID = 18;
  m_allSpecConstInfo[18].size       = sizeof(uint32_t);
  m_allSpecConstInfo[18].offset     = 18*sizeof(uint32_t);
  m_allSpecConstInfo[19].constantID = 19;
  m_allSpecConstInfo[19].size       = sizeof(uint32_t);
  m_allSpecConstInfo[19].offset     = 19*sizeof(uint32_t);
  m_allSpecInfo.dataSize      = m_allSpecConstVals.size()*sizeof(uint32_t);
  m_allSpecInfo.mapEntryCount = static_cast<uint32_t>(m_allSpecConstInfo.size());
  m_allSpecInfo.pMapEntries   = m_allSpecConstInfo.data();
  m_allSpecInfo.pData         = m_allSpecConstVals.data();
  return &m_allSpecInfo;
}

void Integrator_Generated::InitKernel_RayTraceMega(const char* a_filePath)
{
  std::string shaderPath = AlterShaderPath("shaders_generated/RayTraceMega.comp.spv");
  const VkSpecializationInfo* kspec = GetAllSpecInfo();
  RayTraceMegaDSLayout = CreateRayTraceMegaDSLayout();
  if(m_megaKernelFlags.enableRayTraceMega)
  {
    MakeComputePipelineAndLayout(shaderPath.c_str(), "main", kspec, RayTraceMegaDSLayout, &RayTraceMegaLayout, &RayTraceMegaPipeline);
  }
  else
  {
    RayTraceMegaLayout   = nullptr;
    RayTraceMegaPipeline = nullptr;
  }
}

void Integrator_Generated::InitKernel_CastSingleRayMega(const char* a_filePath)
{
  std::string shaderPath = AlterShaderPath("shaders_generated/CastSingleRayMega.comp.spv");
  const VkSpecializationInfo* kspec = GetAllSpecInfo();
  CastSingleRayMegaDSLayout = CreateCastSingleRayMegaDSLayout();
  if(m_megaKernelFlags.enableCastSingleRayMega)
  {
    MakeComputePipelineAndLayout(shaderPath.c_str(), "main", kspec, CastSingleRayMegaDSLayout, &CastSingleRayMegaLayout, &CastSingleRayMegaPipeline);
  }
  else
  {
    CastSingleRayMegaLayout   = nullptr;
    CastSingleRayMegaPipeline = nullptr;
  }
}

void Integrator_Generated::InitKernel_PackXYMega(const char* a_filePath)
{
  std::string shaderPath = AlterShaderPath("shaders_generated/PackXYMega.comp.spv");
  const VkSpecializationInfo* kspec = GetAllSpecInfo();
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

void Integrator_Generated::InitKernel_PathTraceFromInputRaysMega(const char* a_filePath)
{
  std::string shaderPath = AlterShaderPath("shaders_generated/PathTraceFromInputRaysMega.comp.spv");
  const VkSpecializationInfo* kspec = GetAllSpecInfo();
  PathTraceFromInputRaysMegaDSLayout = CreatePathTraceFromInputRaysMegaDSLayout();
  if(m_megaKernelFlags.enablePathTraceFromInputRaysMega)
  {
    MakeComputePipelineAndLayout(shaderPath.c_str(), "main", kspec, PathTraceFromInputRaysMegaDSLayout, &PathTraceFromInputRaysMegaLayout, &PathTraceFromInputRaysMegaPipeline);
  }
  else
  {
    PathTraceFromInputRaysMegaLayout   = nullptr;
    PathTraceFromInputRaysMegaPipeline = nullptr;
  }
}

void Integrator_Generated::InitKernel_PathTraceMega(const char* a_filePath)
{
  std::string shaderPath = AlterShaderPath("shaders_generated/PathTraceMega.comp.spv");
  const VkSpecializationInfo* kspec = GetAllSpecInfo();
  PathTraceMegaDSLayout = CreatePathTraceMegaDSLayout();
  if(m_megaKernelFlags.enablePathTraceMega)
  {
    MakeComputePipelineAndLayout(shaderPath.c_str(), "main", kspec, PathTraceMegaDSLayout, &PathTraceMegaLayout, &PathTraceMegaPipeline);
  }
  else
  {
    PathTraceMegaLayout   = nullptr;
    PathTraceMegaPipeline = nullptr;
  }
}

void Integrator_Generated::InitKernel_NaivePathTraceMega(const char* a_filePath)
{
  std::string shaderPath = AlterShaderPath("shaders_generated/NaivePathTraceMega.comp.spv");
  const VkSpecializationInfo* kspec = GetAllSpecInfo();
  NaivePathTraceMegaDSLayout = CreateNaivePathTraceMegaDSLayout();
  if(m_megaKernelFlags.enableNaivePathTraceMega)
  {
    MakeComputePipelineAndLayout(shaderPath.c_str(), "main", kspec, NaivePathTraceMegaDSLayout, &NaivePathTraceMegaLayout, &NaivePathTraceMegaPipeline);
  }
  else
  {
    NaivePathTraceMegaLayout   = nullptr;
    NaivePathTraceMegaPipeline = nullptr;
  }
}


void Integrator_Generated::InitKernels(const char* a_filePath)
{
  InitKernel_RayTraceMega(a_filePath);
  InitKernel_CastSingleRayMega(a_filePath);
  InitKernel_PackXYMega(a_filePath);
  InitKernel_PathTraceFromInputRaysMega(a_filePath);
  InitKernel_PathTraceMega(a_filePath);
  InitKernel_NaivePathTraceMega(a_filePath);
}

void Integrator_Generated::InitBuffers(size_t a_maxThreadsCount, bool a_tempBuffersOverlay)
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

void Integrator_Generated::ReserveEmptyVectors()
{
  if(all_references.capacity() == 0)
    all_references.reserve(4);
  if(m_allRemapLists.capacity() == 0)
    m_allRemapLists.reserve(4);
  if(m_allRemapListsOffsets.capacity() == 0)
    m_allRemapListsOffsets.reserve(4);
  if(m_cie_x.capacity() == 0)
    m_cie_x.reserve(4);
  if(m_cie_y.capacity() == 0)
    m_cie_y.reserve(4);
  if(m_cie_z.capacity() == 0)
    m_cie_z.reserve(4);
  if(m_films_eta_k_vec.capacity() == 0)
    m_films_eta_k_vec.reserve(4);
  if(m_films_spec_id_vec.capacity() == 0)
    m_films_spec_id_vec.reserve(4);
  if(m_instIdToLightInstId.capacity() == 0)
    m_instIdToLightInstId.reserve(4);
  if(m_lights.capacity() == 0)
    m_lights.reserve(4);
  if(m_lines.capacity() == 0)
    m_lines.reserve(4);
  if(m_matIdByPrimId.capacity() == 0)
    m_matIdByPrimId.reserve(4);
  if(m_matIdOffsets.capacity() == 0)
    m_matIdOffsets.reserve(4);
  if(m_materials.capacity() == 0)
    m_materials.reserve(4);
  if(m_normMatrices.capacity() == 0)
    m_normMatrices.reserve(4);
  if(m_normMatrices2.capacity() == 0)
    m_normMatrices2.reserve(4);
  if(m_pAccelStruct_m_CatmulClarkHeaders != nullptr && m_pAccelStruct_m_CatmulClarkHeaders->capacity() == 0)
    m_pAccelStruct_m_CatmulClarkHeaders->reserve(4);
  if(m_pAccelStruct_m_NURBSData != nullptr && m_pAccelStruct_m_NURBSData->capacity() == 0)
    m_pAccelStruct_m_NURBSData->reserve(4);
  if(m_pAccelStruct_m_NURBSHeaders != nullptr && m_pAccelStruct_m_NURBSHeaders->capacity() == 0)
    m_pAccelStruct_m_NURBSHeaders->reserve(4);
  if(m_pAccelStruct_m_NURBS_approxes != nullptr && m_pAccelStruct_m_NURBS_approxes->capacity() == 0)
    m_pAccelStruct_m_NURBS_approxes->reserve(4);
  if(m_pAccelStruct_m_RibbonHeaders != nullptr && m_pAccelStruct_m_RibbonHeaders->capacity() == 0)
    m_pAccelStruct_m_RibbonHeaders->reserve(4);
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
  if(m_pdfLightData.capacity() == 0)
    m_pdfLightData.reserve(4);
  if(m_precomp_coat_transmittance.capacity() == 0)
    m_precomp_coat_transmittance.reserve(4);
  if(m_precomp_thin_films.capacity() == 0)
    m_precomp_thin_films.reserve(4);
  if(m_randomGens.capacity() == 0)
    m_randomGens.reserve(4);
  if(m_remapInst.capacity() == 0)
    m_remapInst.reserve(4);
  if(m_spec_offset_sz.capacity() == 0)
    m_spec_offset_sz.reserve(4);
  if(m_spec_tex_ids_wavelengths.capacity() == 0)
    m_spec_tex_ids_wavelengths.reserve(4);
  if(m_spec_tex_offset_sz.capacity() == 0)
    m_spec_tex_offset_sz.reserve(4);
  if(m_spec_values.capacity() == 0)
    m_spec_values.reserve(4);
  if(m_triIndices.capacity() == 0)
    m_triIndices.reserve(4);
  if(m_vNorm4f.capacity() == 0)
    m_vNorm4f.reserve(4);
  if(m_vTang4f.capacity() == 0)
    m_vTang4f.reserve(4);
  if(m_vertOffset.capacity() == 0)
    m_vertOffset.reserve(4);
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


void Integrator_Generated::InitMemberBuffers()
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
  m_vdata.m_allRemapListsBuffer = vk_utils::createBuffer(device, m_allRemapLists.capacity()*sizeof(int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_allRemapListsBuffer);
  m_vdata.m_allRemapListsOffsetsBuffer = vk_utils::createBuffer(device, m_allRemapListsOffsets.capacity()*sizeof(int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_allRemapListsOffsetsBuffer);
  m_vdata.m_cie_xBuffer = vk_utils::createBuffer(device, m_cie_x.capacity()*sizeof(float), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_cie_xBuffer);
  m_vdata.m_cie_yBuffer = vk_utils::createBuffer(device, m_cie_y.capacity()*sizeof(float), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_cie_yBuffer);
  m_vdata.m_cie_zBuffer = vk_utils::createBuffer(device, m_cie_z.capacity()*sizeof(float), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_cie_zBuffer);
  m_vdata.m_films_eta_k_vecBuffer = vk_utils::createBuffer(device, m_films_eta_k_vec.capacity()*sizeof(float), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_films_eta_k_vecBuffer);
  m_vdata.m_films_spec_id_vecBuffer = vk_utils::createBuffer(device, m_films_spec_id_vec.capacity()*sizeof(unsigned int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_films_spec_id_vecBuffer);
  m_vdata.m_instIdToLightInstIdBuffer = vk_utils::createBuffer(device, m_instIdToLightInstId.capacity()*sizeof(unsigned int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_instIdToLightInstIdBuffer);
  m_vdata.m_lightsBuffer = vk_utils::createBuffer(device, m_lights.capacity()*sizeof(struct LightSource), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_lightsBuffer);
  m_vdata.m_linesBuffer = vk_utils::createBuffer(device, m_lines.capacity()*sizeof(struct Integrator::LensElementInterface), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_linesBuffer);
  m_vdata.m_matIdByPrimIdBuffer = vk_utils::createBuffer(device, m_matIdByPrimId.capacity()*sizeof(unsigned int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_matIdByPrimIdBuffer);
  m_vdata.m_matIdOffsetsBuffer = vk_utils::createBuffer(device, m_matIdOffsets.capacity()*sizeof(unsigned int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_matIdOffsetsBuffer);
  m_vdata.m_materialsBuffer = vk_utils::createBuffer(device, m_materials.capacity()*sizeof(struct Material), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_materialsBuffer);
  m_vdata.m_normMatricesBuffer = vk_utils::createBuffer(device, m_normMatrices.capacity()*sizeof(struct LiteMath::float4x4), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_normMatricesBuffer);
  m_vdata.m_normMatrices2Buffer = vk_utils::createBuffer(device, m_normMatrices2.capacity()*sizeof(struct LiteMath::float4x4), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_normMatrices2Buffer);
  m_vdata.m_pAccelStruct_m_CatmulClarkHeadersBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_CatmulClarkHeaders->capacity()*sizeof(struct CatmulClarkHeader), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_CatmulClarkHeadersBuffer);
  m_vdata.m_pAccelStruct_m_NURBSDataBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_NURBSData->capacity()*sizeof(float), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_NURBSDataBuffer);
  m_vdata.m_pAccelStruct_m_NURBSHeadersBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_NURBSHeaders->capacity()*sizeof(struct NURBSHeader), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_NURBSHeadersBuffer);
  m_vdata.m_pAccelStruct_m_NURBS_approxesBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_NURBS_approxes->capacity()*sizeof(float), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_NURBS_approxesBuffer);
  m_vdata.m_pAccelStruct_m_RibbonHeadersBuffer = vk_utils::createBuffer(device, m_pAccelStruct_m_RibbonHeaders->capacity()*sizeof(struct RibbonHeader), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pAccelStruct_m_RibbonHeadersBuffer);
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
  m_vdata.m_pdfLightDataBuffer = vk_utils::createBuffer(device, m_pdfLightData.capacity()*sizeof(float), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_pdfLightDataBuffer);
  m_vdata.m_precomp_coat_transmittanceBuffer = vk_utils::createBuffer(device, m_precomp_coat_transmittance.capacity()*sizeof(float), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_precomp_coat_transmittanceBuffer);
  m_vdata.m_precomp_thin_filmsBuffer = vk_utils::createBuffer(device, m_precomp_thin_films.capacity()*sizeof(float), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_precomp_thin_filmsBuffer);
  m_vdata.m_randomGensBuffer = vk_utils::createBuffer(device, m_randomGens.capacity()*sizeof(struct RandomGenT), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_randomGensBuffer);
  m_vdata.m_remapInstBuffer = vk_utils::createBuffer(device, m_remapInst.capacity()*sizeof(int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_remapInstBuffer);
  m_vdata.m_spec_offset_szBuffer = vk_utils::createBuffer(device, m_spec_offset_sz.capacity()*sizeof(struct LiteMath::uint2), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_spec_offset_szBuffer);
  m_vdata.m_spec_tex_ids_wavelengthsBuffer = vk_utils::createBuffer(device, m_spec_tex_ids_wavelengths.capacity()*sizeof(struct LiteMath::uint2), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_spec_tex_ids_wavelengthsBuffer);
  m_vdata.m_spec_tex_offset_szBuffer = vk_utils::createBuffer(device, m_spec_tex_offset_sz.capacity()*sizeof(struct LiteMath::uint2), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_spec_tex_offset_szBuffer);
  m_vdata.m_spec_valuesBuffer = vk_utils::createBuffer(device, m_spec_values.capacity()*sizeof(float), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_spec_valuesBuffer);
  m_vdata.m_triIndicesBuffer = vk_utils::createBuffer(device, m_triIndices.capacity()*sizeof(unsigned int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_triIndicesBuffer);
  m_vdata.m_vNorm4fBuffer = vk_utils::createBuffer(device, m_vNorm4f.capacity()*sizeof(struct LiteMath::float4), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_vNorm4fBuffer);
  m_vdata.m_vTang4fBuffer = vk_utils::createBuffer(device, m_vTang4f.capacity()*sizeof(struct LiteMath::float4), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_vTang4fBuffer);
  m_vdata.m_vertOffsetBuffer = vk_utils::createBuffer(device, m_vertOffset.capacity()*sizeof(unsigned int), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  memberVectors.push_back(m_vdata.m_vertOffsetBuffer);

  
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



VkImage Integrator_Generated::CreateTexture2D(const int a_width, const int a_height, VkFormat a_format, VkImageUsageFlags a_usage)
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

VkSampler Integrator_Generated::CreateSampler(const Sampler& a_sampler) // TODO: implement this function correctly
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

VkImageView Integrator_Generated::CreateView(VkFormat a_format, VkImage a_image)
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




void Integrator_Generated::AssignBuffersToMemory(const std::vector<VkBuffer>& a_buffers, VkDeviceMemory a_mem)
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
      std::cout << "[Integrator_Generated::AssignBuffersToMemory]: error, input buffers has different 'memReq.memoryTypeBits'" << std::endl;
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

Integrator_Generated::MemLoc Integrator_Generated::AllocAndBind(const std::vector<VkBuffer>& a_buffers, VkMemoryAllocateFlags a_flags)
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

Integrator_Generated::MemLoc Integrator_Generated::AllocAndBind(const std::vector<VkImage>& a_images, VkMemoryAllocateFlags a_flags)
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
        std::cout << "Integrator_Generated::AllocAndBind(textures): memoryTypeBits warning, need to split mem allocation (override me)" << std::endl;
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

void Integrator_Generated::FreeAllAllocations(std::vector<MemLoc>& a_memLoc)
{
  // in general you may check 'mem.allocId' for unique to be sure you dont free mem twice
  // for default implementation this is not needed
  for(auto mem : a_memLoc)
    vkFreeMemory(device, mem.memObject, nullptr);
  a_memLoc.resize(0);
}

void Integrator_Generated::AllocMemoryForMemberBuffersAndImages(const std::vector<VkBuffer>& a_buffers, const std::vector<VkImage>& a_images)
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

VkPhysicalDeviceFeatures2 Integrator_Generated::ListRequiredDeviceFeatures(std::vector<const char*>& deviceExtensions)
{
  static VkPhysicalDeviceFeatures2 features2 = {};
  features2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
  features2.pNext = nullptr;
  features2.features.shaderInt64   = false;
  features2.features.shaderFloat64 = true;
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

Integrator_Generated::MegaKernelIsEnabled Integrator_Generated::m_megaKernelFlags;

