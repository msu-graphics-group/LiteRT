#ifndef VK_GRAPHICS_BASIC_RENDER_GUI_H
#define VK_GRAPHICS_BASIC_RENDER_GUI_H

#include "volk.h"
#include "imgui/backends/imgui_impl_vulkan.h"
#include "imgui/backends/imgui_impl_glfw.h"
#include "GLFW/glfw3.h"

#include <vk_swapchain.h>
#include <memory>

class IRenderGUI
{
public:
  virtual VkCommandBuffer BuildGUIRenderCommand(uint32_t a_swapchainFrameIdx, void* a_userData) = 0;
  virtual void OnSwapchainChanged(const VulkanSwapChain &a_swapchain) = 0;
  virtual ~IRenderGUI() = default;
};

/////////////////////////////////////////
/////////////////////////////////////////

class ImGuiRender : public IRenderGUI
{
public:
  ImGuiRender(VkInstance a_instance, VkDevice a_device, VkPhysicalDevice a_physDevice, uint32_t a_queueFID, VkQueue a_queue,
    const VulkanSwapChain &a_swapchain);

  VkCommandBuffer BuildGUIRenderCommand(uint32_t a_swapchainFrameIdx, void* a_userData) override;
  void OnSwapchainChanged(const VulkanSwapChain &a_swapchain) override;

  ~ImGuiRender() override;

private:
  void ClearFrameBuffers();

private:
  VkInstance m_instance = VK_NULL_HANDLE;
  VkDevice m_device = VK_NULL_HANDLE;
  VkPhysicalDevice m_physDevice = VK_NULL_HANDLE;
  uint32_t m_queue_FID = UINT32_MAX;
  VkQueue m_queue = VK_NULL_HANDLE;
  const VulkanSwapChain* m_swapchain;

  // Owned objects
  VkRenderPass m_renderpass = VK_NULL_HANDLE;
  std::vector<VkFramebuffer> m_framebuffers;
  VkCommandPool m_commandPool = VK_NULL_HANDLE;
  std::vector<VkCommandBuffer> m_drawGUICmdBuffers;
  VkDescriptorPool m_descriptorPool = VK_NULL_HANDLE;

  void InitImGui();
  void CleanupImGui();

};

#endif// VK_GRAPHICS_BASIC_RENDER_GUI_H
