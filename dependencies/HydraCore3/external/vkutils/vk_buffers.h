#ifndef VK_UTILS_VK_BUFFERS_H
#define VK_UTILS_VK_BUFFERS_H

#include "vk_include.h"

#include <string>
#include <vector>
#include <cassert>

namespace vk_utils
{
  struct BufferOffsetPair
  {
    VkBuffer buffer = VK_NULL_HANDLE;
    size_t   offset = 0u;

    BufferOffsetPair(VkBuffer a_buff, size_t a_offset) : buffer(a_buff), offset(a_offset) {}
  };


  VkBuffer createBuffer(VkDevice a_dev, VkDeviceSize a_size, VkBufferUsageFlags a_usageFlags, VkMemoryRequirements* a_pMemReq = nullptr);

  void createBufferStaging(VkDevice a_device, VkPhysicalDevice a_physDevice, size_t a_bufferSize,
                           VkBuffer &a_buf, VkDeviceMemory& a_mem);

  VkDeviceMemory allocateAndBindWithPadding(VkDevice a_dev, VkPhysicalDevice a_physDev, const std::vector<VkBuffer> &a_buffers,
                                            VkMemoryAllocateFlags flags = {});

  std::vector<size_t> assignMemOffsetsWithPadding(const std::vector<VkMemoryRequirements> &a_memInfos);
  std::vector<size_t> calculateMemOffsets(const std::vector<VkMemoryRequirements> &a_memReqs, size_t a_buffImageGranularity = 0);
}

#endif //VK_UTILS_VK_BUFFERS_H
