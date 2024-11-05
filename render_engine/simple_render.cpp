#include "simple_render.h"
#include "input_definitions.h"
#include "scene_mgr.h"

#include <geom/vk_mesh.h>
#include <vk_pipeline.h>
#include <vk_buffers.h>

SimpleRender::SimpleRender(uint32_t a_width, uint32_t a_height) : m_width(a_width), m_height(a_height)
{
#ifdef NDEBUG
  m_enableValidation = false;
#else
  m_enableValidation = true;
#endif

  //m_force_rayrace_cpu = true;
  m_raytracedImageData.resize(m_width * m_height);
}

VkPhysicalDeviceFeatures2 SimpleRender::SetupDeviceFeatures()
{
  static VkPhysicalDeviceAccelerationStructureFeaturesKHR m_enabledAccelStructFeatures{};
  static VkPhysicalDeviceBufferDeviceAddressFeatures m_enabledDeviceAddressFeatures{};
  static VkPhysicalDeviceRayQueryFeaturesKHR m_enabledRayQueryFeatures{};
  static VkPhysicalDeviceFeatures2 features2{};

  if (false)
  {
    m_enabledRayQueryFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_QUERY_FEATURES_KHR;
    m_enabledRayQueryFeatures.rayQuery = VK_TRUE;
    m_enabledRayQueryFeatures.pNext = nullptr;
    
    m_enabledDeviceAddressFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_BUFFER_DEVICE_ADDRESS_FEATURES;
    m_enabledDeviceAddressFeatures.bufferDeviceAddress = VK_TRUE;
    m_enabledDeviceAddressFeatures.pNext = &m_enabledRayQueryFeatures;
    
    m_enabledAccelStructFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_FEATURES_KHR;
    m_enabledAccelStructFeatures.accelerationStructure = VK_TRUE;
    m_enabledAccelStructFeatures.pNext = &m_enabledDeviceAddressFeatures;
    
    features2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
    features2.pNext = &m_enabledAccelStructFeatures;  
  }
  else
  {
    features2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
    features2.pNext = nullptr;      
  } 

    return features2;
}

std::vector<const char*> SimpleRender::SetupDeviceExtensions()
{
  std::vector<const char*> deviceExtensions;
  deviceExtensions.push_back(VK_KHR_SWAPCHAIN_EXTENSION_NAME);
  
  if(false)
  {
    deviceExtensions.push_back(VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME);
    deviceExtensions.push_back(VK_KHR_RAY_QUERY_EXTENSION_NAME);
    // Required by VK_KHR_acceleration_structure
    deviceExtensions.push_back(VK_KHR_BUFFER_DEVICE_ADDRESS_EXTENSION_NAME);
    deviceExtensions.push_back(VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME);
    deviceExtensions.push_back(VK_EXT_DESCRIPTOR_INDEXING_EXTENSION_NAME);
    // Required by VK_KHR_ray_tracing_pipeline
    deviceExtensions.push_back(VK_KHR_SPIRV_1_4_EXTENSION_NAME);
    // Required by VK_KHR_spirv_1_4
    deviceExtensions.push_back(VK_KHR_SHADER_FLOAT_CONTROLS_EXTENSION_NAME);
  }

  return deviceExtensions;
}

void SimpleRender::SetupValidationLayers()
{
  m_enableValidation = false;
  m_validationLayers.push_back("VK_LAYER_KHRONOS_validation");
  m_validationLayers.push_back("VK_LAYER_LUNARG_monitor");
}

void SimpleRender::InitVulkan(const char** a_instanceExtensions, uint32_t a_instanceExtensionsCount, uint32_t a_deviceId)
{
  for(size_t i = 0; i < a_instanceExtensionsCount; ++i)
  {
    m_instanceExtensions.push_back(a_instanceExtensions[i]);
  }

  SetupValidationLayers();
  VK_CHECK_RESULT(volkInitialize());
  CreateInstance();
  volkLoadInstance(m_instance);

  CreateDevice(a_deviceId);
  volkLoadDevice(m_device);

  m_commandPool = vk_utils::createCommandPool(m_device, m_queueFamilyIDXs.graphics,
                                              VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT);

  m_cmdBuffersDrawMain.reserve(m_framesInFlight);
  m_cmdBuffersDrawMain = vk_utils::createCommandBuffers(m_device, m_commandPool, m_framesInFlight);

  m_frameFences.resize(m_framesInFlight);
  VkFenceCreateInfo fenceInfo = {};
  fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
  fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;
  for (size_t i = 0; i < m_framesInFlight; i++)
  {
    VK_CHECK_RESULT(vkCreateFence(m_device, &fenceInfo, nullptr, &m_frameFences[i]));
  }

  m_pCopyHelper = std::make_shared<vk_utils::PingPongCopyHelper>(m_physicalDevice, m_device, m_transferQueue,
    m_queueFamilyIDXs.transfer, STAGING_MEM_SIZE);

  m_pAllocatorSpecial = vk_utils::CreateMemoryAlloc_Special(m_device, m_physicalDevice);

  LoaderConfig conf = {};
  conf.load_geometry = true;
  conf.load_materials = MATERIAL_LOAD_MODE::NONE;
  m_pScnMgr = std::make_shared<SceneManager>(m_device, m_physicalDevice, m_queueFamilyIDXs.graphics, m_pCopyHelper, conf);
}

void SimpleRender::InitPresentation(VkSurfaceKHR &a_surface)
{
  m_surface = a_surface;

  m_presentationResources.queue = m_swapchain.CreateSwapChain(m_physicalDevice, m_device, m_surface,
                                                              m_width, m_height, m_framesInFlight, m_vsync);
  m_presentationResources.currentFrame = 0;

  std::vector<VkFormat> depthFormats = {
      VK_FORMAT_D32_SFLOAT,
      VK_FORMAT_D32_SFLOAT_S8_UINT,
      VK_FORMAT_D24_UNORM_S8_UINT,
      VK_FORMAT_D16_UNORM_S8_UINT,
      VK_FORMAT_D16_UNORM
  };
  vk_utils::getSupportedDepthFormat(m_physicalDevice, depthFormats, &m_depthBuffer.format);

  VkSemaphoreCreateInfo semaphoreInfo = {};
  semaphoreInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
  VK_CHECK_RESULT(vkCreateSemaphore(m_device, &semaphoreInfo, nullptr, &m_presentationResources.imageAvailable));
  VK_CHECK_RESULT(vkCreateSemaphore(m_device, &semaphoreInfo, nullptr, &m_presentationResources.renderingFinished));
  m_screenRenderPass = vk_utils::createDefaultRenderPass(m_device, m_swapchain.GetFormat(), m_depthBuffer.format);


  m_depthBuffer  = vk_utils::createDepthTexture(m_device, m_physicalDevice, m_width, m_height, m_depthBuffer.format);
  m_frameBuffers = vk_utils::createFrameBuffers(m_device, m_swapchain, m_screenRenderPass, m_depthBuffer.view);

  m_pGUIRender = std::make_shared<ImGuiRender>(m_instance, m_device, m_physicalDevice, m_queueFamilyIDXs.graphics, m_graphicsQueue, m_swapchain);

  SetupQuadRenderer();
}

void SimpleRender::CreateInstance()
{
  VkApplicationInfo appInfo = {};
  appInfo.sType              = VK_STRUCTURE_TYPE_APPLICATION_INFO;
  appInfo.pNext              = nullptr;
  appInfo.pApplicationName   = "VkRender";
  appInfo.applicationVersion = VK_MAKE_VERSION(0, 1, 0);
  appInfo.pEngineName        = "RayTracingSample";
  appInfo.engineVersion      = VK_MAKE_VERSION(0, 1, 0);
  appInfo.apiVersion         = VK_MAKE_VERSION(1, 1, 0);

  m_instance = vk_utils::createInstance(m_enableValidation, m_validationLayers, m_instanceExtensions, &appInfo);

  if (m_enableValidation)
    vk_utils::initDebugReportCallback(m_instance, &debugReportCallbackFn, &m_debugReportCallback);
}

std::vector<const char *> merge_extensions(const std::vector<const char *> &e1, const std::vector<const char *> &e2)
{
  std::vector<const char *> result = e1;
  //for (const char *r : result)
  // printf("extension %s\n", r);
  for (const char *e : e2)
  {
    bool found = false;
    for (const char *r : result)
    {
      if (strcmp(r, e) == 0)
      {
        found = true;
        break;
      }
    }
    if (!found)
    {
      result.push_back(e);
      //printf("add extension %s\n", e);
    }
  }
  return result;
}

VkPhysicalDeviceFeatures2 merge_device_features(VkPhysicalDeviceFeatures2 &f1, VkPhysicalDeviceFeatures2 &f2)
{
  std::vector<VkStructureType> all_types = {f1.sType};

  VkPhysicalDeviceFeatures2 *p_feature = &f1;
  while (p_feature->pNext)
  {
    p_feature = (VkPhysicalDeviceFeatures2 *)p_feature->pNext;
    all_types.push_back(p_feature->sType);
    //printf("new type %u\n", (unsigned)p_feature->sType);
  }

  VkPhysicalDeviceFeatures2 *p_feature2 = &f2;
  while (p_feature2)
  {
    bool found = false;
    for (const VkStructureType &t : all_types)
    {
      if (p_feature2->sType == t)
      {
        found = true;
        break;
      }
    }
    if (!found)
    {
      p_feature->pNext = p_feature2;
      p_feature = p_feature2;
      //printf("new type %u\n", (unsigned)p_feature->sType);
    }
    else
    {
      //printf("existing type %u\n", (unsigned)p_feature2->sType);
    }
    p_feature2 = (VkPhysicalDeviceFeatures2 *)p_feature2->pNext;
  }

  return f1;
}

void SimpleRender::CreateDevice(uint32_t a_deviceId)
{
  std::vector<const char *> extensions_self = SetupDeviceExtensions();
  VkPhysicalDeviceFeatures2 features_self  = SetupDeviceFeatures();

  std::vector<const char *> extensions_raytracer;
  VkPhysicalDeviceFeatures2 features_raytracer = MultiRendererGPUImpl::ListRequiredDeviceFeatures(extensions_raytracer);

  std::vector<const char *> extensions = merge_extensions(extensions_self, extensions_raytracer);
  VkPhysicalDeviceFeatures2 features = merge_device_features(features_self, features_raytracer);

  m_physicalDevice = vk_utils::findPhysicalDevice(m_instance, true, a_deviceId, extensions);

  m_device = vk_utils::createLogicalDevice(m_physicalDevice, m_validationLayers, extensions,
                                           m_enabledDeviceFeatures, m_queueFamilyIDXs,
                                           VK_QUEUE_GRAPHICS_BIT | VK_QUEUE_TRANSFER_BIT, features.pNext);

  vkGetDeviceQueue(m_device, m_queueFamilyIDXs.graphics, 0, &m_graphicsQueue);
  vkGetDeviceQueue(m_device, m_queueFamilyIDXs.transfer, 0, &m_transferQueue);
  vkGetDeviceQueue(m_device, m_queueFamilyIDXs.compute,  0, &m_computeQueue);
}

void SimpleRender::SetupSimplePipeline()
{
  m_pBindings->BindBegin(VK_SHADER_STAGE_FRAGMENT_BIT);
  m_pBindings->BindBuffer(0, m_ubo, VK_NULL_HANDLE, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
  m_pBindings->BindEnd(&m_dSet, &m_dSetLayout);

  // if we are recreating pipeline (for example, to reload shaders)
  // we need to cleanup old pipeline
  if(m_basicForwardPipeline.layout != VK_NULL_HANDLE)
  {
    vkDestroyPipelineLayout(m_device, m_basicForwardPipeline.layout, nullptr);
    m_basicForwardPipeline.layout = VK_NULL_HANDLE;
  }
  if(m_basicForwardPipeline.pipeline != VK_NULL_HANDLE)
  {
    vkDestroyPipeline(m_device, m_basicForwardPipeline.pipeline, nullptr);
    m_basicForwardPipeline.pipeline = VK_NULL_HANDLE;
  }

  vk_utils::GraphicsPipelineMaker maker;

  std::unordered_map<VkShaderStageFlagBits, std::string> shader_paths;
  shader_paths[VK_SHADER_STAGE_FRAGMENT_BIT] = FRAGMENT_SHADER_PATH + ".spv";
  shader_paths[VK_SHADER_STAGE_VERTEX_BIT]   = VERTEX_SHADER_PATH + ".spv";

  maker.LoadShaders(m_device, shader_paths);

  m_basicForwardPipeline.layout = maker.MakeLayout(m_device, {m_dSetLayout}, sizeof(pushConst2M));
  maker.SetDefaultState(m_width, m_height);

  m_basicForwardPipeline.pipeline = maker.MakePipeline(m_device, m_pScnMgr->GetPipelineVertexInputStateCreateInfo(),
                                                       m_screenRenderPass, {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR});
}

void SimpleRender::CreateUniformBuffer()
{
  VkMemoryRequirements memReq;
  m_ubo = vk_utils::createBuffer(m_device, sizeof(UniformParams), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, &memReq);

  VkMemoryAllocateInfo allocateInfo = {};
  allocateInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
  allocateInfo.pNext = nullptr;
  allocateInfo.allocationSize = memReq.size;
  allocateInfo.memoryTypeIndex = vk_utils::findMemoryType(memReq.memoryTypeBits,
                                                          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                                          VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                                                          m_physicalDevice);
  VK_CHECK_RESULT(vkAllocateMemory(m_device, &allocateInfo, nullptr, &m_uboAlloc));

  VK_CHECK_RESULT(vkBindBufferMemory(m_device, m_ubo, m_uboAlloc, 0));

  vkMapMemory(m_device, m_uboAlloc, 0, sizeof(m_uniforms), 0, &m_uboMappedMem);

  m_uniforms.lightPos  = LiteMath::float4(0.0f, 1.0f,  1.0f, 1.0f);
  m_uniforms.baseColor = LiteMath::float4(0.9f, 0.92f, 1.0f, 0.0f);
  m_uniforms.animateLightColor = true;

  UpdateUniformBuffer(0.0f);
}

void SimpleRender::UpdateUniformBuffer(float a_time)
{
// most uniforms are updated in GUI -> SetupGUIElements()
  m_uniforms.time = a_time;
  memcpy(m_uboMappedMem, &m_uniforms, sizeof(m_uniforms));
}

void SimpleRender::BuildCommandBufferSimple(VkCommandBuffer a_cmdBuff, VkFramebuffer a_frameBuff,
                                            VkImageView, VkPipeline a_pipeline)
{
  vkResetCommandBuffer(a_cmdBuff, 0);

  VkCommandBufferBeginInfo beginInfo = {};
  beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
  beginInfo.flags = VK_COMMAND_BUFFER_USAGE_SIMULTANEOUS_USE_BIT;

  VK_CHECK_RESULT(vkBeginCommandBuffer(a_cmdBuff, &beginInfo));

  vk_utils::setDefaultViewport(a_cmdBuff, static_cast<float>(m_width), static_cast<float>(m_height));
  vk_utils::setDefaultScissor(a_cmdBuff, m_width, m_height);

  ///// draw final scene to screen
  {
    VkRenderPassBeginInfo renderPassInfo = {};
    renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    renderPassInfo.renderPass = m_screenRenderPass;
    renderPassInfo.framebuffer = a_frameBuff;
    renderPassInfo.renderArea.offset = {0, 0};
    renderPassInfo.renderArea.extent = m_swapchain.GetExtent();

    VkClearValue clearValues[2] = {};
    clearValues[0].color = {0.0f, 0.0f, 0.0f, 1.0f};
    clearValues[1].depthStencil = {1.0f, 0};
    renderPassInfo.clearValueCount = 2;
    renderPassInfo.pClearValues = &clearValues[0];

    vkCmdBeginRenderPass(a_cmdBuff, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdBindPipeline(a_cmdBuff, VK_PIPELINE_BIND_POINT_GRAPHICS, a_pipeline);

    vkCmdBindDescriptorSets(a_cmdBuff, VK_PIPELINE_BIND_POINT_GRAPHICS, m_basicForwardPipeline.layout, 0, 1,
                            &m_dSet, 0, VK_NULL_HANDLE);

    VkShaderStageFlags stageFlags = (VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT);

    VkDeviceSize zero_offset = 0u;
    VkBuffer vertexBuf = m_pScnMgr->GetVertexBuffer();
    VkBuffer indexBuf = m_pScnMgr->GetIndexBuffer();

    vkCmdBindVertexBuffers(a_cmdBuff, 0, 1, &vertexBuf, &zero_offset);
    vkCmdBindIndexBuffer(a_cmdBuff, indexBuf, 0, VK_INDEX_TYPE_UINT32);

    for (uint32_t i = 0; i < m_pScnMgr->InstancesNum(); ++i)
    {
      auto inst = m_pScnMgr->GetInstanceInfo(i);

      pushConst2M.model = m_pScnMgr->GetInstanceMatrix(i);
      vkCmdPushConstants(a_cmdBuff, m_basicForwardPipeline.layout, stageFlags, 0,
                         sizeof(pushConst2M), &pushConst2M);

      auto mesh_info = m_pScnMgr->GetMeshInfo(inst.mesh_id);
      vkCmdDrawIndexed(a_cmdBuff, mesh_info.m_indNum, 1, mesh_info.m_indexOffset, mesh_info.m_vertexOffset, 0);
    }

    vkCmdEndRenderPass(a_cmdBuff);
  }

  VK_CHECK_RESULT(vkEndCommandBuffer(a_cmdBuff));
}

void SimpleRender::BuildCommandBufferQuad(VkCommandBuffer a_cmdBuff, VkImageView a_targetImageView)
{
  vkResetCommandBuffer(a_cmdBuff, 0);

  VkCommandBufferBeginInfo beginInfo = {};
  beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
  beginInfo.flags = VK_COMMAND_BUFFER_USAGE_SIMULTANEOUS_USE_BIT;

  VK_CHECK_RESULT(vkBeginCommandBuffer(a_cmdBuff, &beginInfo));
  {
    float scaleAndOffset[4] = { 0.5f, 0.5f, -0.5f, +0.5f };
    m_pFSQuad->SetRenderTarget(a_targetImageView);
    m_pFSQuad->DrawCmd(a_cmdBuff, m_quadDS, scaleAndOffset);
  }

  VK_CHECK_RESULT(vkEndCommandBuffer(a_cmdBuff));
}

void SimpleRender::CleanupPipelineAndSwapchain()
{
  if (!m_cmdBuffersDrawMain.empty())
  {
    vkFreeCommandBuffers(m_device, m_commandPool, static_cast<uint32_t>(m_cmdBuffersDrawMain.size()),
                         m_cmdBuffersDrawMain.data());
    m_cmdBuffersDrawMain.clear();
  }

  for (size_t i = 0; i < m_frameFences.size(); i++)
  {
    vkDestroyFence(m_device, m_frameFences[i], nullptr);
  }
  m_frameFences.clear();

  vk_utils::deleteImg(m_device, &m_depthBuffer);

  for (size_t i = 0; i < m_frameBuffers.size(); i++)
  {
    vkDestroyFramebuffer(m_device, m_frameBuffers[i], nullptr);
  }
  m_frameBuffers.clear();

  if(m_screenRenderPass != VK_NULL_HANDLE)
  {
    vkDestroyRenderPass(m_device, m_screenRenderPass, nullptr);
    m_screenRenderPass = VK_NULL_HANDLE;
  }

  m_swapchain.Cleanup();
}

void SimpleRender::RecreateSwapChain()
{
  vkDeviceWaitIdle(m_device);

  CleanupPipelineAndSwapchain();
  auto oldImagesNum = m_swapchain.GetImageCount();
  m_presentationResources.queue = m_swapchain.CreateSwapChain(m_physicalDevice, m_device, m_surface, m_width, m_height,
    oldImagesNum, m_vsync);

  std::vector<VkFormat> depthFormats = {
      VK_FORMAT_D32_SFLOAT,
      VK_FORMAT_D32_SFLOAT_S8_UINT,
      VK_FORMAT_D24_UNORM_S8_UINT,
      VK_FORMAT_D16_UNORM_S8_UINT,
      VK_FORMAT_D16_UNORM
  };                                                            
  vk_utils::getSupportedDepthFormat(m_physicalDevice, depthFormats, &m_depthBuffer.format);
  
  m_screenRenderPass = vk_utils::createDefaultRenderPass(m_device, m_swapchain.GetFormat(),m_depthBuffer.format);
  m_depthBuffer      = vk_utils::createDepthTexture(m_device, m_physicalDevice, m_width, m_height, m_depthBuffer.format);
  m_frameBuffers     = vk_utils::createFrameBuffers(m_device, m_swapchain, m_screenRenderPass, m_depthBuffer.view);

  m_frameFences.resize(m_framesInFlight);
  VkFenceCreateInfo fenceInfo = {};
  fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
  fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;
  for (size_t i = 0; i < m_framesInFlight; i++)
  {
    VK_CHECK_RESULT(vkCreateFence(m_device, &fenceInfo, nullptr, &m_frameFences[i]));
  }

  m_cmdBuffersDrawMain = vk_utils::createCommandBuffers(m_device, m_commandPool, m_framesInFlight);

  // *** ray tracing resources
  m_raytracedImageData.resize(m_width * m_height);
  // change screen resolution

  SetupRTImage();
  SetupQuadRenderer();
  SetupQuadDescriptors();
  //

  if (m_pRayTracer)
    OnScreenResolutionChangeRT();

  m_pGUIRender->OnSwapchainChanged(m_swapchain);
}

void SimpleRender::ProcessInput(const AppInput &input)
{
  // add keyboard controls here
  // camera movement is processed separately

  // recreate pipeline to reload shaders
  // disabled as it is not so trivial with ray tracing
  //   if(input.keyPressed[GLFW_KEY_B])
  //   {
  // #ifdef WIN32
  //     std::system("cd ./shaders && python compile_simple_render_shaders.py");
  // #else
  //     std::system("cd ./shaders && python3 compile_simple_render_shaders.py");
  // #endif

  //     SetupSimplePipeline();

  //     for (uint32_t i = 0; i < m_framesInFlight; ++i)
  //     {
  //       BuildCommandBufferSimple(m_cmdBuffersDrawMain[i], m_frameBuffers[i],
  //                                m_swapchain.GetAttachment(i).view, m_basicForwardPipeline.pipeline);
  //     }
  //   }

  if(input.keyPressed[GLFW_KEY_1])
  {
    m_currentRenderMode = RenderMode::RASTERIZATION;
  }
  else if(input.keyPressed[GLFW_KEY_2])
  {
    m_currentRenderMode = RenderMode::RAYTRACING;
  }

}

void SimpleRender::UpdateCamera(const Camera* cams, uint32_t a_camsCount)
{
  assert(a_camsCount > 0);
  m_cam = cams[0];
  UpdateView();
}

void SimpleRender::UpdateView()
{
  const float aspect   = float(m_width) / float(m_height);
  auto mProjFix        = OpenglToVulkanProjectionMatrixFix();
  m_projectionMatrix   = projectionMatrix(m_cam.fov, aspect, 0.1f, 1000.0f);
  m_worldViewMatrix    = LiteMath::lookAt(m_cam.pos, m_cam.lookAt, m_cam.up);
  auto mWorldViewProj  = mProjFix * m_projectionMatrix * m_worldViewMatrix;
  pushConst2M.projView = mWorldViewProj;

  m_inverseProjViewMatrix = LiteMath::inverse4x4(m_projectionMatrix * transpose(inverse4x4(m_worldViewMatrix)));
}


void SimpleRender::LoadScene(const char* path)
{
  m_scenePath = path;

  std::vector<std::pair<VkDescriptorType, uint32_t> > dtypes = {
    {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,             1},
    {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,     1}
  };

  // set large a_maxSets, because every window resize will cause the descriptor set for quad being to be recreated
  m_pBindings = std::make_shared<vk_utils::DescriptorMaker>(m_device, dtypes, 1000);

  SetupRTImage();
  CreateUniformBuffer();

  SetupQuadDescriptors();

  UpdateView();
}

void SimpleRender::DrawFrameSimple()
{
  vkWaitForFences(m_device, 1, &m_frameFences[m_presentationResources.currentFrame], VK_TRUE, UINT64_MAX);
  vkResetFences(m_device, 1, &m_frameFences[m_presentationResources.currentFrame]);

  uint32_t imageIdx;
  m_swapchain.AcquireNextImage(m_presentationResources.imageAvailable, &imageIdx);

  auto currentCmdBuf = m_cmdBuffersDrawMain[m_presentationResources.currentFrame];

  VkSemaphore waitSemaphores[] = {m_presentationResources.imageAvailable};
  VkPipelineStageFlags waitStages[] = {VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT};

  if(m_currentRenderMode == RenderMode::RASTERIZATION)
  {
    BuildCommandBufferSimple(currentCmdBuf, m_frameBuffers[imageIdx], m_swapchain.GetAttachment(imageIdx).view, m_basicForwardPipeline.pipeline);
  }
  else if(m_currentRenderMode == RenderMode::RAYTRACING)
  {
    if (m_pRayTracerGPU)
      RayTraceGPU();
    else
      RayTraceCPU();

    BuildCommandBufferQuad(currentCmdBuf, m_swapchain.GetAttachment(imageIdx).view);
  }

  VkSubmitInfo submitInfo = {};
  submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
  submitInfo.waitSemaphoreCount = 1;
  submitInfo.pWaitSemaphores = waitSemaphores;
  submitInfo.pWaitDstStageMask = waitStages;
  submitInfo.commandBufferCount = 1;
  submitInfo.pCommandBuffers = &currentCmdBuf;

  VkSemaphore signalSemaphores[] = {m_presentationResources.renderingFinished};
  submitInfo.signalSemaphoreCount = 1;
  submitInfo.pSignalSemaphores = signalSemaphores;

  VK_CHECK_RESULT(vkQueueSubmit(m_graphicsQueue, 1, &submitInfo, m_frameFences[m_presentationResources.currentFrame]));

  VkResult presentRes = m_swapchain.QueuePresent(m_presentationResources.queue, imageIdx,
                                                 m_presentationResources.renderingFinished);

  if (presentRes == VK_ERROR_OUT_OF_DATE_KHR || presentRes == VK_SUBOPTIMAL_KHR)
  {
    RecreateSwapChain();
  }
  else if (presentRes != VK_SUCCESS)
  {
    RUN_TIME_ERROR("Failed to present swapchain image");
  }

  m_presentationResources.currentFrame = (m_presentationResources.currentFrame + 1) % m_framesInFlight;

  vkQueueWaitIdle(m_presentationResources.queue);
}

void SimpleRender::DrawFrame(float a_time, DrawMode a_mode)
{
  if (!m_RasterSceneSetUp && m_currentRenderMode == RenderMode::RASTERIZATION)
  {
    m_pScnMgr->LoadSceneXML(m_scenePath.c_str(), false);
    SetupSimplePipeline();

    m_RasterSceneSetUp = true;
  }

  if (!m_RTSceneSetUp && m_currentRenderMode == RenderMode::RAYTRACING)
  {
    SetupRTScene(m_scenePath.c_str());
    m_RTSceneSetUp = true;
  }

  UpdateUniformBuffer(a_time);

  switch (a_mode)
  {
  case DrawMode::WITH_GUI:
    SetupGUIElements();
    DrawFrameWithGUI();
    break;
  case DrawMode::NO_GUI:
    DrawFrameSimple();
    break;
  default:
    DrawFrameSimple();
  }
}

void SimpleRender::Cleanup()
{
  m_pGUIRender = nullptr;
  ImGui::DestroyContext();
  CleanupPipelineAndSwapchain();

  if (m_pRayTracer)
  {
    m_pRayTracer.reset();
    m_pRayTracer = nullptr;
    m_pRayTracerGPU = nullptr;
  }

  if(m_surface != VK_NULL_HANDLE)
  {
    vkDestroySurfaceKHR(m_instance, m_surface, nullptr);
    m_surface = VK_NULL_HANDLE;
  }

  if(m_rtImageSampler != VK_NULL_HANDLE)
  {
    vkDestroySampler(m_device, m_rtImageSampler, nullptr);
    m_rtImageSampler = VK_NULL_HANDLE;
  }
  vk_utils::deleteImg(m_device, &m_rtImage);

  m_pFSQuad = nullptr;

  if (m_basicForwardPipeline.pipeline != VK_NULL_HANDLE)
  {
    vkDestroyPipeline(m_device, m_basicForwardPipeline.pipeline, nullptr);
    m_basicForwardPipeline.pipeline = VK_NULL_HANDLE;
  }
  if (m_basicForwardPipeline.layout != VK_NULL_HANDLE)
  {
    vkDestroyPipelineLayout(m_device, m_basicForwardPipeline.layout, nullptr);
    m_basicForwardPipeline.layout = VK_NULL_HANDLE;
  }

  if (m_presentationResources.imageAvailable != VK_NULL_HANDLE)
  {
    vkDestroySemaphore(m_device, m_presentationResources.imageAvailable, nullptr);
    m_presentationResources.imageAvailable = VK_NULL_HANDLE;
  }
  if (m_presentationResources.renderingFinished != VK_NULL_HANDLE)
  {
    vkDestroySemaphore(m_device, m_presentationResources.renderingFinished, nullptr);
    m_presentationResources.renderingFinished = VK_NULL_HANDLE;
  }

  if (m_commandPool != VK_NULL_HANDLE)
  {
    vkDestroyCommandPool(m_device, m_commandPool, nullptr);
    m_commandPool = VK_NULL_HANDLE;
  }

  if(m_ubo != VK_NULL_HANDLE)
  {
    vkDestroyBuffer(m_device, m_ubo, nullptr);
    m_ubo = VK_NULL_HANDLE;
  }

  if(m_uboAlloc != VK_NULL_HANDLE)
  {
    vkFreeMemory(m_device, m_uboAlloc, nullptr);
    m_uboAlloc = VK_NULL_HANDLE;
  }

  if(m_genColorBuffer != VK_NULL_HANDLE)
  {
    vkDestroyBuffer(m_device, m_genColorBuffer, nullptr);
    m_genColorBuffer = VK_NULL_HANDLE;
  }

  if(m_colorMem != VK_NULL_HANDLE)
  {
    vkFreeMemory(m_device, m_colorMem, nullptr);
    m_colorMem = VK_NULL_HANDLE;
  }

  m_pBindings = nullptr;
  m_pScnMgr   = nullptr;
  m_pCopyHelper = nullptr;

  if(m_device != VK_NULL_HANDLE)
  {
    vkDestroyDevice(m_device, nullptr);
    m_device = VK_NULL_HANDLE;
  }

  if(m_debugReportCallback != VK_NULL_HANDLE)
  {
    vkDestroyDebugReportCallbackEXT(m_instance, m_debugReportCallback, nullptr);
    m_debugReportCallback = VK_NULL_HANDLE;
  }

  if(m_instance != VK_NULL_HANDLE)
  {
    vkDestroyInstance(m_instance, nullptr);
    m_instance = VK_NULL_HANDLE;
  }
}

/////////////////////////////////

//MultiRenderMode item list
static constexpr const char* const multi_render_mode_items[] = { "Mask", "Lambert (no tex)", "Depth", "Depth (linear)", "Depth (inverse linear)", "Primitive",
                                                                 "Object type", "Geom", "Normals", "Barycentric", "ST iterations", "Radiance fields",
                                                                 "Phong (no tex)", "Gaussian splats", "Radiance fields (density)", "Tex coords", "Diffuse",
                                                                 "Lambert", "Phong", "Depth (HSV)" };

void SimpleRender::SetupGUIElements()
{
  ImGui_ImplVulkan_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();

  MultiRenderPreset preset = this->m_pRayTracer->GetPreset();
  int render_mode_int = preset.render_mode;
  {
//    ImGui::ShowDemoWindow();
    ImGui::Begin("Your render settings here");
    ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f),"Press '1' for rasterization mode");
    ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "Press '2' for raytracing mode");
    ImGui::NewLine();

    ImGui::ColorEdit3("Meshes base color", m_uniforms.baseColor.M, ImGuiColorEditFlags_PickerHueWheel | ImGuiColorEditFlags_NoInputs);
    ImGui::SliderFloat3("Light source position", m_uniforms.lightPos.M, -10.f, 10.f);

    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

    ImGui::NewLine();

    ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f),"Press 'B' to recompile and reload shaders");
    ImGui::Text("Changing bindings is not supported.");
    ImGui::Text("Vertex shader path: %s", VERTEX_SHADER_PATH.c_str());
    ImGui::Text("Fragment shader path: %s", FRAGMENT_SHADER_PATH.c_str());
    ImGui::Text("Render mode (RT):");
    ImGui::ListBox("", &render_mode_int, multi_render_mode_items, sizeof(multi_render_mode_items) / sizeof(char*));
    ImGui::End();
  }

  // Update preset
  preset.render_mode = render_mode_int;
  this->m_pRayTracer->SetPreset(preset);

  // Rendering
  ImGui::Render();
}

void SimpleRender::DrawFrameWithGUI()
{
  vkWaitForFences(m_device, 1, &m_frameFences[m_presentationResources.currentFrame], VK_TRUE, UINT64_MAX);
  vkResetFences(m_device, 1, &m_frameFences[m_presentationResources.currentFrame]);

  uint32_t imageIdx;
  auto result = m_swapchain.AcquireNextImage(m_presentationResources.imageAvailable, &imageIdx);
  if (result == VK_ERROR_OUT_OF_DATE_KHR)
  {
    RecreateSwapChain();
    return;
  }
  else if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR)
  {
    RUN_TIME_ERROR("Failed to acquire the next swapchain image!");
  }

  auto currentCmdBuf = m_cmdBuffersDrawMain[m_presentationResources.currentFrame];

  VkSemaphore waitSemaphores[] = {m_presentationResources.imageAvailable};
  VkPipelineStageFlags waitStages[] = {VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT};

  if(m_currentRenderMode == RenderMode::RASTERIZATION)
  {
    BuildCommandBufferSimple(currentCmdBuf, m_frameBuffers[imageIdx], m_swapchain.GetAttachment(imageIdx).view, m_basicForwardPipeline.pipeline);
  }
  else if(m_currentRenderMode == RenderMode::RAYTRACING)
  {
    if (m_pRayTracerGPU)
      RayTraceGPU();
    else
      RayTraceCPU();

    BuildCommandBufferQuad(currentCmdBuf, m_swapchain.GetAttachment(imageIdx).view);
  }

  ImDrawData* pDrawData = ImGui::GetDrawData();
  auto currentGUICmdBuf = m_pGUIRender->BuildGUIRenderCommand(imageIdx, pDrawData);

  std::vector<VkCommandBuffer> submitCmdBufs = { currentCmdBuf, currentGUICmdBuf};

  VkSubmitInfo submitInfo = {};
  submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
  submitInfo.waitSemaphoreCount = 1;
  submitInfo.pWaitSemaphores = waitSemaphores;
  submitInfo.pWaitDstStageMask = waitStages;
  submitInfo.commandBufferCount = (uint32_t)submitCmdBufs.size();
  submitInfo.pCommandBuffers = submitCmdBufs.data();

  VkSemaphore signalSemaphores[] = {m_presentationResources.renderingFinished};
  submitInfo.signalSemaphoreCount = 1;
  submitInfo.pSignalSemaphores = signalSemaphores;

  VK_CHECK_RESULT(vkQueueSubmit(m_graphicsQueue, 1, &submitInfo, m_frameFences[m_presentationResources.currentFrame]));

  VkResult presentRes = m_swapchain.QueuePresent(m_presentationResources.queue, imageIdx,
    m_presentationResources.renderingFinished);

  if (presentRes == VK_ERROR_OUT_OF_DATE_KHR || presentRes == VK_SUBOPTIMAL_KHR)
  {
    RecreateSwapChain();
  }
  else if (presentRes != VK_SUCCESS)
  {
    RUN_TIME_ERROR("Failed to present swapchain image");
  }

  m_presentationResources.currentFrame = (m_presentationResources.currentFrame + 1) % m_framesInFlight;

  vkQueueWaitIdle(m_presentationResources.queue);
}
