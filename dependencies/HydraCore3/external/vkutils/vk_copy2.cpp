#include "vk_copy2.h"
#include "vk_alloc.h"
#include "vk_utils.h"
#include "vk_buffers.h"


namespace vk_utils
{
  PingPongCopyHelper2::PingPongCopyHelper2(VkDevice a_device, VkPhysicalDevice a_physDevice,
                                           std::shared_ptr<IMemoryAlloc> a_pAlloc,
                                           uint32_t a_queueIDX, size_t a_stagingBuffSize)

  {
    physDev  = a_physDevice;
    dev      = a_device;
    pAlloc   = a_pAlloc;
    vkGetDeviceQueue(a_device, a_queueIDX, 0, &queue);

    VkCommandPoolCreateInfo poolInfo = {};
    poolInfo.sType            = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    poolInfo.flags            = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    poolInfo.queueFamilyIndex = a_queueIDX;
    VK_CHECK_RESULT(vkCreateCommandPool(a_device, &poolInfo, nullptr, &cmdPool));

    VkCommandBufferAllocateInfo cmdBufAllocInfo = {};
    cmdBufAllocInfo.sType              = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    cmdBufAllocInfo.commandPool        = cmdPool;
    cmdBufAllocInfo.level              = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cmdBufAllocInfo.commandBufferCount = 1;
    VK_CHECK_RESULT(vkAllocateCommandBuffers(a_device, &cmdBufAllocInfo, &cmdBuff));

    stagingSize     = a_stagingBuffSize;
    stagingSizeHalf = a_stagingBuffSize/2;

    VkBufferUsageFlags    usage    = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
    VkMemoryPropertyFlags memProps = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT| VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;

    staging[0] =  vk_utils::createBuffer(dev, stagingSizeHalf, usage);
    staging[1] =  vk_utils::createBuffer(dev, stagingSizeHalf, usage);

    MemAllocInfo allocInfo{};
    allocInfo.memUsage = memProps;

    // since VMA maps entire allocation when MapMemory is called,
    // we need to create separate allocations for staging buffer halfs
    allocIds[0] = a_pAlloc->Allocate(allocInfo, {staging[0]});
    allocIds[1] = a_pAlloc->Allocate(allocInfo, {staging[1]});

    VkFenceCreateInfo fenceCreateInfo = {};
    fenceCreateInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    fenceCreateInfo.flags = 0;
    VK_CHECK_RESULT(vkCreateFence(a_device, &fenceCreateInfo, NULL, &fence));
  }

  PingPongCopyHelper2::~PingPongCopyHelper2()
  {
    pAlloc->Free(allocIds[0]);
    pAlloc->Free(allocIds[1]);
  }


  void PingPongCopyHelper2::UpdateBuffer(VkBuffer a_dst, size_t a_dstOffset, const void* a_src, size_t a_size)
  {
    assert(a_dstOffset % 4 == 0);
    assert(a_size      % 4 == 0);

    VkMemoryRequirements memInfo = {};
    vkGetBufferMemoryRequirements(dev, a_dst, &memInfo);

    if (a_size <= SMALL_BUFF)
    {
      VkCommandBufferBeginInfo beginInfo = {};
      beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;

      vkResetCommandBuffer(cmdBuff, 0);
      vkBeginCommandBuffer(cmdBuff, &beginInfo);
      vkCmdUpdateBuffer   (cmdBuff, a_dst, a_dstOffset, a_size, a_src);
      vkEndCommandBuffer  (cmdBuff);
      
      vkResetFences(dev, 1, &fence);
      VkSubmitInfo submitInfo       = {};
      submitInfo.sType              = VK_STRUCTURE_TYPE_SUBMIT_INFO;
      submitInfo.commandBufferCount = 1;        
      submitInfo.pCommandBuffers    = &cmdBuff; 
      VK_CHECK_RESULT(vkQueueSubmit(queue, 1, &submitInfo, fence));
      VK_CHECK_RESULT(vkWaitForFences(dev, 1, &fence, VK_TRUE, vk_utils::DEFAULT_TIMEOUT));

      return;
    }

    uint32_t currStaging = 0;
    size_t currPos  = 0;
    size_t prevCopySize = 0;

    for(; currPos < a_size; currPos += stagingSizeHalf) // use ping-pong shceme
    {
      size_t currCopySize = std::min(a_size - currPos, stagingSizeHalf);

      // (0) begin (copy staging[prev] ==> result) in parallel with further vkMapMemory/memcpy/vkUnmapMemory
      //
      if(currPos != 0) 
        SubmitCopy(a_dst, a_dstOffset + currPos - stagingSizeHalf, prevCopySize, 1 - currStaging);
      
      // (1) (copy src ==> staging[curr])
      //
     
      void* mappedMemory = pAlloc->Map(allocIds[currStaging], 0, currCopySize);
      memcpy(mappedMemory, ((char*)(a_src)) + currPos, currCopySize);
      pAlloc->Unmap(allocIds[currStaging]);
      
      // (3) end (staging[prev] ==> result)
      //
      if(currPos != 0) 
        vkWaitForFences(dev, 1, &fence, VK_TRUE, vk_utils::DEFAULT_TIMEOUT);

      currStaging  = 1 - currStaging;
      prevCopySize = currCopySize;
    }

    // last iter copy: (staging[prev] ==> result)
    //
    SubmitCopy(a_dst, a_dstOffset + currPos - stagingSizeHalf, prevCopySize, 1 - currStaging);
    VK_CHECK_RESULT(vkWaitForFences(dev, 1, &fence, VK_TRUE, vk_utils::DEFAULT_TIMEOUT));
  }
}