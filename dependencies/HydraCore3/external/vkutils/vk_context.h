#ifndef VULKAN_CONTEXT_H
#define VULKAN_CONTEXT_H

#include "vk_include.h"
#include "vk_copy.h"
#include "vk_alloc.h"

#include <vector>
#include <memory>

#include <stdexcept>
#include <sstream>

namespace vk_utils
{
  //// global vulkan context for simple applications and generated code
  //
  struct VulkanContext
  {
    VkInstance       instance       = VK_NULL_HANDLE;
    VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
    VkDevice         device         = VK_NULL_HANDLE;
    VkCommandPool    commandPool    = VK_NULL_HANDLE; 
    VkQueue          computeQueue   = VK_NULL_HANDLE;
    VkQueue          transferQueue  = VK_NULL_HANDLE;
    
    std::shared_ptr<ICopyEngine>  pCopyHelper       = nullptr;
    std::shared_ptr<IMemoryAlloc> pAllocatorCommon  = nullptr;
    std::shared_ptr<IMemoryAlloc> pAllocatorSpecial = nullptr;

    VkPhysicalDeviceSubgroupProperties subgroupProps;
  };

  bool          globalContextIsInitialized(const std::vector<const char*>& requiredExtensions = std::vector<const char*>());
  VulkanContext globalContextInit(const std::vector<const char*>& requiredExtensions = std::vector<const char*>(), 
                                  bool enableValidationLayers = false, 
                                  unsigned int a_preferredDeviceId = 0,
                                  VkPhysicalDeviceFeatures2* a_pKnownFeatures = nullptr,
                                  size_t memForBuffers = size_t(-1), 
                                  size_t memForTextures = size_t(-1));
  VulkanContext globalContextGet(bool enableValidationLayers = false, unsigned int a_preferredDeviceId = 0);
  void          globalContextDestroy();

  struct ExecTime
  {
    float msCopyToGPU    = 0.0f;
    float msCopyFromGPU  = 0.0f;
    float msExecuteOnGPU = 0.0f;
    float msAPIOverhead  = 0.0f;
    float msLayoutChange = 0.0f;
  };
};

// test change 2

#endif
