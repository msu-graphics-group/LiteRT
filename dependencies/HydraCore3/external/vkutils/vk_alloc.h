#ifndef VKUTILS_VK_RESOURCE_ALLOC_H
#define VKUTILS_VK_RESOURCE_ALLOC_H

#include "vk_include.h"
#include "vk_copy.h"
#include "external/samplers_vk.h"
#include <memory>
#include <unordered_map>

namespace vk_utils
{
  struct MemoryBlock
  {
    VkDeviceMemory memory = VK_NULL_HANDLE;
    VkDeviceSize   offset = 0u;
    VkDeviceSize   size   = 0u;
  };

  struct MemAllocInfo
  {
    VkMemoryRequirements  memReq {0, 0, 0};
    VkFlags               memUsage {0};
    VkMemoryAllocateFlags allocateFlags {0};
    VkImage               dedicated_image  {VK_NULL_HANDLE};
    VkBuffer              dedicated_buffer {VK_NULL_HANDLE};
  };

  struct IMemoryAlloc
  {
    virtual uint32_t Allocate(const MemAllocInfo& a_allocInfo) = 0;
    virtual uint32_t Allocate(const MemAllocInfo& a_allocInfoBuffers, const std::vector<VkBuffer> &a_buffers) = 0;
    virtual uint32_t Allocate(const MemAllocInfo& a_allocInfoImages, const std::vector<VkImage> &a_images) = 0;

    virtual void Free(uint32_t a_memBlockId) = 0;

    virtual void FreeAllMemory() = 0;

    virtual MemoryBlock GetMemoryBlock(uint32_t a_memBlockId) const = 0;

    virtual void* Map(uint32_t a_memBlockId, VkDeviceSize a_offset, VkDeviceSize a_size) = 0;

    virtual void Unmap(uint32_t a_memBlockId) = 0;

    virtual VkDevice GetDevice() const = 0;

    virtual VkPhysicalDevice GetPhysicalDevice() const = 0;

    virtual ~IMemoryAlloc() = default;
  };


  std::shared_ptr<IMemoryAlloc> CreateMemoryAlloc_VMA(VkInstance a_instance, VkDevice a_device, VkPhysicalDevice a_physicalDevice,
    VkFlags a_flags = 0, uint32_t a_vkAPIVersion = VK_API_VERSION_1_1);
  std::shared_ptr<IMemoryAlloc> CreateMemoryAlloc_Simple(VkDevice a_device, VkPhysicalDevice a_physicalDevice);
  std::shared_ptr<IMemoryAlloc> CreateMemoryAlloc_Special(VkDevice a_device, VkPhysicalDevice a_physicalDevice);
}

#endif// VKUTILS_VK_RESOURCE_ALLOC_H
