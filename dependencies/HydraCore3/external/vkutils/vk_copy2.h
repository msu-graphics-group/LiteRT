#ifndef VK_UTILS_COPY2_H
#define VK_UTILS_COPY2_H

#include "vk_copy.h"
#include "vk_alloc.h"

namespace vk_utils
{
  // same as PingPongCopyHelper but uses allocator to create staging buffers
  struct PingPongCopyHelper2 : PingPongCopyHelper
  {

    PingPongCopyHelper2(VkDevice a_device, VkPhysicalDevice a_physDevice,
                        std::shared_ptr<IMemoryAlloc> a_pAlloc,
                        uint32_t a_queueIDX, size_t a_stagingBuffSize);


    ~PingPongCopyHelper2() override;

    void UpdateBuffer(VkBuffer a_dst, size_t a_dstOffset, const void* a_src, size_t a_size) override;


  protected:
    uint32_t allocIds[2] = {UINT32_MAX, UINT32_MAX};
    std::shared_ptr<IMemoryAlloc> pAlloc;
  };
}

#endif //VK_UTILS_COPY2_H
