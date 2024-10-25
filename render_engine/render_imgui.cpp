#include "backends/imgui_impl_vulkan.h"
#include "render_gui.h"
#include <vk_utils.h>
#include <vk_descriptor_sets.h>


ImGuiRender::ImGuiRender(VkInstance a_instance, VkDevice a_device, VkPhysicalDevice a_physDevice, uint32_t a_queueFID, VkQueue a_queue,
  const VulkanSwapChain &a_swapchain) : m_instance(a_instance), m_device(a_device), m_physDevice(a_physDevice),
                                        m_queue_FID(a_queueFID), m_queue(a_queue), m_swapchain(&a_swapchain)
{
  InitImGui();
}

static VkInstance g_instance = VK_NULL_HANDLE;
PFN_vkVoidFunction vulkanLoaderFunction(const char* function_name, void*) { return vkGetInstanceProcAddr(g_instance, function_name); }

void ImGuiRender::InitImGui()
{
  vk_utils::DescriptorTypesVec descrTypes = {{ VK_DESCRIPTOR_TYPE_SAMPLER, 1000 },
    { VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1000 },
    { VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1000 },
    { VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1000 },
    { VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER, 1000 },
    { VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER, 1000 },
    { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1000 },
    { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1000 },
    { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, 1000 },
    { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC, 1000 },
    { VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 1000 }};

  m_descriptorPool = vk_utils::createDescriptorPool(m_device, descrTypes, (uint32_t)descrTypes.size() * 1000);

  vk_utils::RenderTargetInfo2D rtInfo = {};
  rtInfo.format = m_swapchain->GetFormat();
  rtInfo.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
  rtInfo.initialLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
  rtInfo.finalLayout   = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

  m_renderpass   = vk_utils::createRenderPass(m_device, rtInfo);
  m_framebuffers = vk_utils::createFrameBuffers(m_device, *m_swapchain, m_renderpass);
  m_commandPool  = vk_utils::createCommandPool(m_device, m_queue_FID, VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT);

  m_drawGUICmdBuffers.reserve(m_swapchain->GetImageCount());
  m_drawGUICmdBuffers = vk_utils::createCommandBuffers(m_device, m_commandPool, m_swapchain->GetImageCount());

  g_instance = m_instance;

  ImGui_ImplVulkan_InitInfo init_info {};

  init_info.Instance       = m_instance;
  init_info.PhysicalDevice = m_physDevice;
  init_info.Device         = m_device;
  init_info.QueueFamily    = m_queue_FID;
  init_info.Queue          = m_queue;
  init_info.RenderPass     = m_renderpass;
  init_info.PipelineCache  = VK_NULL_HANDLE;
  init_info.DescriptorPool = m_descriptorPool;
  init_info.Allocator      = VK_NULL_HANDLE;
  init_info.MinImageCount  = m_swapchain->GetMinImageCount();
  init_info.ImageCount     = m_swapchain->GetImageCount();
  init_info.CheckVkResultFn = nullptr;

  ImGui_ImplVulkan_LoadFunctions(vulkanLoaderFunction);
  ImGui_ImplVulkan_Init(&init_info);

  ImGui_ImplVulkan_CreateFontsTexture();
}

VkCommandBuffer ImGuiRender::BuildGUIRenderCommand(uint32_t a_swapchainFrameIdx, void* a_userData)
{
  auto currentCmdBuf = m_drawGUICmdBuffers[a_swapchainFrameIdx];

  VkCommandBufferBeginInfo cmdBeginInfo = {};
  cmdBeginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
  cmdBeginInfo.flags |= VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
  VK_CHECK_RESULT(vkBeginCommandBuffer(currentCmdBuf, &cmdBeginInfo));

  VkRenderPassBeginInfo rpassBeginInfo = {};
  rpassBeginInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
  rpassBeginInfo.renderPass = m_renderpass;
  rpassBeginInfo.framebuffer = m_framebuffers[a_swapchainFrameIdx];
  rpassBeginInfo.renderArea.extent = m_swapchain->GetExtent();
  rpassBeginInfo.clearValueCount = 0;
  rpassBeginInfo.pClearValues = nullptr;
  vkCmdBeginRenderPass(currentCmdBuf, &rpassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);

  ImGui_ImplVulkan_RenderDrawData(static_cast<ImDrawData*>(a_userData), currentCmdBuf);

  vkCmdEndRenderPass(currentCmdBuf);

  vkEndCommandBuffer(currentCmdBuf);

  return currentCmdBuf;
}

void ImGuiRender::OnSwapchainChanged(const VulkanSwapChain &a_swapchain)
{
  // If swapchain size changed, we are doomed, but that generaly does not happen. I think.
  m_swapchain = &a_swapchain;

  ClearFrameBuffers();
  m_framebuffers = vk_utils::createFrameBuffers(m_device, *m_swapchain, m_renderpass);
}

void ImGuiRender::CleanupImGui()
{
  ImGui_ImplVulkan_Shutdown();
  ImGui_ImplGlfw_Shutdown();

  ClearFrameBuffers();

  if(m_renderpass)
  {
    vkDestroyRenderPass(m_device, m_renderpass, VK_NULL_HANDLE);
    m_renderpass = VK_NULL_HANDLE;
  }
  
  if(m_commandPool)
  {
    vkDestroyCommandPool(m_device, m_commandPool, VK_NULL_HANDLE);
    m_commandPool = VK_NULL_HANDLE;  
  }

  if(m_descriptorPool)
  {
    vkDestroyDescriptorPool(m_device, m_descriptorPool, VK_NULL_HANDLE);
    m_descriptorPool = VK_NULL_HANDLE;   
  }
}

ImGuiRender::~ImGuiRender()
{
  CleanupImGui();
}

void ImGuiRender::ClearFrameBuffers()
{
  for(auto fbuf : m_framebuffers)
    vkDestroyFramebuffer(m_device, fbuf, VK_NULL_HANDLE);
  m_framebuffers.clear();
}


