#include "vk_alloc_simple.h"
#include "vk_utils.h"
#include "vk_buffers.h"
#include "vk_images.h"
#include <unordered_map>

namespace vk_utils
{
  std::shared_ptr<IMemoryAlloc> CreateMemoryAlloc_Simple(VkDevice a_device, VkPhysicalDevice a_physicalDevice)
  {
    return std::make_shared<MemoryAlloc_Simple>(a_device, a_physicalDevice);
  }

  std::shared_ptr<IMemoryAlloc> CreateMemoryAlloc_Special(VkDevice a_device, VkPhysicalDevice a_physicalDevice)
  {
    return std::make_shared<MemoryAlloc_Special>(a_device, a_physicalDevice);
  }


  MemoryAlloc_Simple::MemoryAlloc_Simple(VkDevice a_device, VkPhysicalDevice a_physicalDevice) :
    m_device(a_device), m_physicalDevice(a_physicalDevice)
  {
    vkGetPhysicalDeviceMemoryProperties(m_physicalDevice, &m_physicalMemoryProps);
  }

  MemoryAlloc_Simple::~MemoryAlloc_Simple()
  {
    Cleanup();
  }

  void MemoryAlloc_Simple::Cleanup()
  {
    FreeAllMemory();
  }

  uint32_t MemoryAlloc_Simple::Allocate(const MemAllocInfo& a_allocInfo)
  {
    VkMemoryAllocateInfo memAllocInfo {};
    memAllocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    memAllocInfo.allocationSize = a_allocInfo.memReq.size;
    memAllocInfo.memoryTypeIndex = vk_utils::findMemoryType(a_allocInfo.memReq.memoryTypeBits, a_allocInfo.memUsage,
      m_physicalDevice);

    VkMemoryDedicatedAllocateInfo dedicatedInfo {};
    dedicatedInfo.sType = VK_STRUCTURE_TYPE_MEMORY_DEDICATED_ALLOCATE_INFO;
    if(a_allocInfo.dedicated_buffer || a_allocInfo.dedicated_image)
    {
      dedicatedInfo.pNext = memAllocInfo.pNext;
      memAllocInfo.pNext  = &dedicatedInfo;

      dedicatedInfo.buffer = a_allocInfo.dedicated_buffer;
      dedicatedInfo.image = a_allocInfo.dedicated_image;
    }

    VkMemoryAllocateFlagsInfo flagsInfo {};
    flagsInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_FLAGS_INFO;
    if(a_allocInfo.allocateFlags)
    {
      flagsInfo.pNext    = memAllocInfo.pNext;
      memAllocInfo.pNext = &flagsInfo;

      flagsInfo.flags = a_allocInfo.allocateFlags;
    }


    VkDeviceMemory memory = VK_NULL_HANDLE;
    VkResult result = vkAllocateMemory(m_device, &memAllocInfo, nullptr, &memory);
    VK_CHECK_RESULT(result);

    MemoryBlock block {};
    block.memory = memory;
    block.size   = memAllocInfo.allocationSize;
    block.offset = 0;

    auto allocId = nextAllocIdx;
    m_allocations[allocId] = block;
    nextAllocIdx++;

    return allocId;
  }

  uint32_t MemoryAlloc_Simple::Allocate(const MemAllocInfo& a_allocInfoBuffers, const std::vector<VkBuffer> &a_buffers)
  {
    MemAllocInfo allocInfo = a_allocInfoBuffers;
    std::vector<VkMemoryRequirements> bufMemReqs(a_buffers.size());
    for(size_t i = 0; i < a_buffers.size(); ++i)
    {
      if(a_buffers[i] != VK_NULL_HANDLE)
        vkGetBufferMemoryRequirements(m_device, a_buffers[i], &bufMemReqs[i]);
      else
      {
        bufMemReqs[i] = bufMemReqs[0];
        bufMemReqs[i].size = 0;
      }
    }
    for(size_t i = 1; i < bufMemReqs.size(); ++i)
    {
      if(bufMemReqs[i].memoryTypeBits != bufMemReqs[0].memoryTypeBits)
      {
        VK_UTILS_LOG_WARNING("[MemoryAlloc_Simple::Allocate]: input buffers have different memReq.memoryTypeBits");
        return UINT32_MAX;
      }
    }

    auto bufOffsets  = calculateMemOffsets(bufMemReqs);
    auto bufMemTotal = bufOffsets[bufOffsets.size() - 1];

    allocInfo.memReq      = bufMemReqs[0];
    allocInfo.memReq.size = bufMemTotal;

    auto allocId = Allocate(allocInfo);

#if defined(VK_VERSION_1_1)
    std::vector<VkBindBufferMemoryInfo> bindInfos;
    bindInfos.reserve(bufMemReqs.size());
    for(size_t i = 0; i < bufMemReqs.size(); ++i)
    {
      if(a_buffers[i] != VK_NULL_HANDLE)
      {
        VkBindBufferMemoryInfo info {};
        info.sType        = VK_STRUCTURE_TYPE_BIND_BUFFER_MEMORY_INFO;
        info.buffer       = a_buffers[i];
        info.memory       = GetMemoryBlock(allocId).memory;
        info.memoryOffset = GetMemoryBlock(allocId).offset + bufOffsets[i];
        bindInfos.emplace_back(info);
      }
    }
    vkBindBufferMemory2(m_device, static_cast<uint32_t>(bindInfos.size()), bindInfos.data());
#else
    for(size_t i = 0; i < bufMemReqs.size(); i++)
    {
      if(a_buffers[i] != VK_NULL_HANDLE)
        vkBindBufferMemory(m_device, a_buffers[i], GetMemoryBlock(allocId).memory, GetMemoryBlock(allocId).offset + bufOffsets[i]);
    }
#endif

    return allocId;
  }

  uint32_t MemoryAlloc_Simple::Allocate(const MemAllocInfo& a_allocInfoImages, const std::vector<VkImage> &a_images)
  {
    MemAllocInfo allocInfo = a_allocInfoImages;
    std::vector<VkMemoryRequirements> imgMemReqs(a_images.size());
    for(size_t i = 0; i < a_images.size(); ++i)
    {
      if(a_images[i] != VK_NULL_HANDLE)
        vkGetImageMemoryRequirements(m_device, a_images[i], &imgMemReqs[i]);
      else
      {
        imgMemReqs[i] = imgMemReqs[0];
        imgMemReqs[i].size = 0;
      }
    }
    for(size_t i = 1; i < imgMemReqs.size(); ++i)
    {
      if(imgMemReqs[i].memoryTypeBits != imgMemReqs[0].memoryTypeBits)
      {
        VK_UTILS_LOG_WARNING("[MemoryAlloc_Simple::Allocate]: input images have different memReq.memoryTypeBits");
        return UINT32_MAX;
      }
    }

    auto imgOffsets  = calculateMemOffsets(imgMemReqs);
    auto imgMemTotal = imgOffsets[imgOffsets.size() - 1];

    allocInfo.memReq      = imgMemReqs[0];
    allocInfo.memReq.size = imgMemTotal;

    auto allocId = Allocate(allocInfo);

#if defined(VK_VERSION_1_1)
    std::vector<VkBindImageMemoryInfo> bindInfos;
    bindInfos.reserve(imgMemReqs.size());
    for(size_t i = 0; i < imgMemReqs.size(); ++i)
    {
      if(a_images[i] != VK_NULL_HANDLE)
      {
        VkBindImageMemoryInfo info {};
        info.sType        = VK_STRUCTURE_TYPE_BIND_IMAGE_MEMORY_INFO;
        info.image        = a_images[i];
        info.memory       = GetMemoryBlock(allocId).memory;
        info.memoryOffset = GetMemoryBlock(allocId).offset + imgOffsets[i];
        bindInfos.emplace_back(info);
      }
    }
    vkBindImageMemory2(m_device, static_cast<uint32_t>(bindInfos.size()), bindInfos.data());
#else
    for(size_t i = 0; i < imgMemReqs.size(); i++)
    {
      if(a_images[i] != VK_NULL_HANDLE)// unnecessary check ?
        vkBindBufferMemory(m_device, a_images[i], GetMemoryBlock(allocId).memory, GetMemoryBlock(allocId).offset + imgOffsets[i]);
    }
#endif

    return allocId;
  }


  void MemoryAlloc_Simple::Free(uint32_t a_memBlockId)
  {
    if(!m_allocations.count(a_memBlockId)|| m_allocations[a_memBlockId].memory == VK_NULL_HANDLE)
      return;

    vkFreeMemory(m_device, m_allocations[a_memBlockId].memory, nullptr);
    m_allocations.erase(a_memBlockId);
  }

  void MemoryAlloc_Simple::FreeAllMemory()
  {
    for(auto& [alloc_id, alloc] : m_allocations)
    {
      Free(alloc_id);
    }
  }

  MemoryBlock MemoryAlloc_Simple::GetMemoryBlock(uint32_t a_memBlockId) const
  {
    if(!m_allocations.count(a_memBlockId))
      return {};

    return m_allocations.at(a_memBlockId);
  }

  void* MemoryAlloc_Simple::Map(uint32_t a_memBlockId, VkDeviceSize a_offset, VkDeviceSize a_size)
  {
    if(!m_allocations.count(a_memBlockId))
      return nullptr;

    void* ptr = nullptr;
    VkResult result = vkMapMemory(m_device, m_allocations[a_memBlockId].memory, a_offset, a_size, 0, &ptr);
    VK_CHECK_RESULT(result);

    return ptr;
  }

  void MemoryAlloc_Simple::Unmap(uint32_t a_memBlockId)
  {
    if(!m_allocations.count(a_memBlockId))
      return;

    vkUnmapMemory(m_device, m_allocations[a_memBlockId].memory);
  }

  // **************************************************

  MemoryAlloc_Special::MemoryAlloc_Special(VkDevice a_device, VkPhysicalDevice a_physicalDevice) :
    m_device(a_device), m_physicalDevice(a_physicalDevice)
  {
    vkGetPhysicalDeviceMemoryProperties(m_physicalDevice, &m_physicalMemoryProps);
  }

  MemoryAlloc_Special::~MemoryAlloc_Special()
  {
    Cleanup();
  }

  void MemoryAlloc_Special::Cleanup()
  {
    FreeAllMemory();
  }

  uint32_t MemoryAlloc_Special::Allocate(const MemAllocInfo& a_allocInfo)
  {
    (void)a_allocInfo;
    VK_UTILS_LOG_WARNING("[MemoryAlloc_Special::Allocate] general allocation not supported");
    return UINT32_MAX;
  }

  MemoryBlock MemoryAlloc_Special::AllocateInternal(const MemAllocInfo& a_allocInfo)
  {
    VkMemoryAllocateInfo memAllocInfo {};
    memAllocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    memAllocInfo.allocationSize = a_allocInfo.memReq.size;
    memAllocInfo.memoryTypeIndex = vk_utils::findMemoryType(a_allocInfo.memReq.memoryTypeBits, a_allocInfo.memUsage,
      m_physicalDevice);

    VkMemoryDedicatedAllocateInfo dedicatedInfo {};
    dedicatedInfo.sType = VK_STRUCTURE_TYPE_MEMORY_DEDICATED_ALLOCATE_INFO;
    if(a_allocInfo.dedicated_buffer || a_allocInfo.dedicated_image)
    {
      dedicatedInfo.pNext = memAllocInfo.pNext;
      memAllocInfo.pNext  = &dedicatedInfo;

      dedicatedInfo.buffer = a_allocInfo.dedicated_buffer;
      dedicatedInfo.image = a_allocInfo.dedicated_image;
    }

    VkMemoryAllocateFlagsInfo flagsInfo {};
    flagsInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_FLAGS_INFO;
    if(a_allocInfo.allocateFlags)
    {
      flagsInfo.pNext    = memAllocInfo.pNext;
      memAllocInfo.pNext = &flagsInfo;

      flagsInfo.flags = a_allocInfo.allocateFlags;
    }

    VkDeviceMemory memory = VK_NULL_HANDLE;
    VkResult result = vkAllocateMemory(m_device, &memAllocInfo, nullptr, &memory);
    VK_CHECK_RESULT(result);

    MemoryBlock block {};
    block.memory = memory;
    block.size   = memAllocInfo.allocationSize;
    block.offset = 0;

    return block;
  }

uint32_t MemoryAlloc_Special::AllocateHidden(const MemAllocInfo& a_allocInfoBuffers, const std::vector<VkBuffer> &a_buffers, size_t a_offset, size_t* a_pAllocatedSize)
{
  MemAllocInfo allocInfo = a_allocInfoBuffers;
  std::vector<VkMemoryRequirements> bufMemReqs(a_buffers.size());
  for(size_t i = 0; i < a_buffers.size(); ++i)
  {
    if(a_buffers[i] != VK_NULL_HANDLE)
      vkGetBufferMemoryRequirements(m_device, a_buffers[i], &bufMemReqs[i]);
    else
    {
      bufMemReqs[i] = bufMemReqs[0];
      bufMemReqs[i].size = 0;
    }
  }
  for(size_t i = 1; i < bufMemReqs.size(); ++i)
  {
    if(bufMemReqs[i].memoryTypeBits != bufMemReqs[0].memoryTypeBits)
    {
      std::unordered_map<uint32_t, std::vector<uint32_t> > bufferSets;
      for(uint32_t j = 0; j < uint32_t(bufMemReqs.size()); ++j)
      {
        uint32_t key = bufMemReqs[j].memoryTypeBits;
        bufferSets[key].push_back(j);
      }

      size_t currOffset = 0, currSize = 0;
      for(const auto& buffGroup : bufferSets)
      {
        std::vector<VkBuffer> currGroup;
        for(auto id : buffGroup.second)
          currGroup.push_back(a_buffers[id]);
        AllocateHidden(a_allocInfoBuffers, currGroup, currOffset, &currSize);
        currOffset += currSize;
      }

      return BUF_ALLOC_ID;
    }
  }

  auto bufOffsets  = calculateMemOffsets(bufMemReqs);
  auto bufMemTotal = bufOffsets[bufOffsets.size() - 1];

  if(a_pAllocatedSize != nullptr)
    (*a_pAllocatedSize) = bufMemTotal;

  if(m_bufAlloc.size < a_offset + bufMemTotal)
  {
    VK_UTILS_LOG_INFO("[MemoryAlloc_Special::Allocate] Buffers REALLOC : old_size = " + std::to_string(m_bufAlloc.size) + ", new_size = " + std::to_string(bufMemTotal));

    allocInfo.memReq      = bufMemReqs[0];
    allocInfo.memReq.size = bufMemTotal;

    Free(BUF_ALLOC_ID);
    m_bufAlloc = AllocateInternal(allocInfo);
  }

#if defined(VK_VERSION_1_1)
  std::vector<VkBindBufferMemoryInfo> bindInfos;
  bindInfos.reserve(bufMemReqs.size());
  for(size_t i = 0; i < bufMemReqs.size(); ++i)
  {
    if(a_buffers[i] != VK_NULL_HANDLE)
    {
      VkBindBufferMemoryInfo info {};
      info.sType        = VK_STRUCTURE_TYPE_BIND_BUFFER_MEMORY_INFO;
      info.buffer       = a_buffers[i];
      info.memory       = m_bufAlloc.memory;
      info.memoryOffset = m_bufAlloc.offset + bufOffsets[i] + a_offset;
      bindInfos.emplace_back(info);
    }
  }
  vkBindBufferMemory2(m_device, static_cast<uint32_t>(bindInfos.size()), bindInfos.data());
#else
  for(size_t i = 0; i < bufMemReqs.size(); i++)
    {
      if(a_buffers[i] != VK_NULL_HANDLE)
        vkBindBufferMemory(m_device, a_buffers[i], m_bufAlloc.memory, m_bufAlloc.offset + bufOffsets[i] + a_offset);
    }
#endif

  return BUF_ALLOC_ID;
}

  uint32_t MemoryAlloc_Special::Allocate(const MemAllocInfo& a_allocInfoBuffers, const std::vector<VkBuffer> &a_buffers)
  {
    return AllocateHidden(a_allocInfoBuffers, a_buffers);
  }

  uint32_t MemoryAlloc_Special::Allocate(const MemAllocInfo& a_allocInfoImages, const std::vector<VkImage> &a_images)
  {
    MemAllocInfo allocInfo = a_allocInfoImages;
    std::vector<VkMemoryRequirements> imgMemReqs(a_images.size());
    for(size_t i = 0; i < a_images.size(); ++i)
    {
      if(a_images[i] != VK_NULL_HANDLE)
        vkGetImageMemoryRequirements(m_device, a_images[i], &imgMemReqs[i]);
      else
      {
        imgMemReqs[i] = imgMemReqs[0];
        imgMemReqs[i].size = 0;
      }
    }
    for(size_t i = 1; i < imgMemReqs.size(); ++i)
    {
      if(imgMemReqs[i].memoryTypeBits != imgMemReqs[0].memoryTypeBits)
      {
        VK_UTILS_LOG_WARNING("[MemoryAlloc_Simple::Allocate]: input images have different memReq.memoryTypeBits");
        return UINT32_MAX;
      }
    }

    auto imgOffsets  = calculateMemOffsets(imgMemReqs);
    auto imgMemTotal = imgOffsets[imgOffsets.size() - 1];

    allocInfo.memReq      = imgMemReqs[0];
    allocInfo.memReq.size = imgMemTotal;

    if(m_imgAlloc.size < imgMemTotal)
    {
      VK_UTILS_LOG_INFO("[MemoryAlloc_Special::Allocate] Textures REALLOC : old_size = "
                        + std::to_string(m_imgAlloc.size)
                        + ", new_size = "
                        + std::to_string(imgMemTotal));

      allocInfo.memReq      = imgMemReqs[0];
      allocInfo.memReq.size = imgMemTotal;

      Free(IMG_ALLOC_ID);
      m_imgAlloc = AllocateInternal(allocInfo);
    }

#if defined(VK_VERSION_1_1)
    std::vector<VkBindImageMemoryInfo> bindInfos;
    bindInfos.reserve(imgMemReqs.size());
    for(size_t i = 0; i < imgMemReqs.size(); ++i)
    {
      if(a_images[i] != VK_NULL_HANDLE)
      {
        VkBindImageMemoryInfo info {};
        info.sType        = VK_STRUCTURE_TYPE_BIND_IMAGE_MEMORY_INFO;
        info.image        = a_images[i];
        info.memory       = m_imgAlloc.memory;
        info.memoryOffset = m_imgAlloc.offset + imgOffsets[i];
        bindInfos.emplace_back(info);
      }
    }
    vkBindImageMemory2(m_device, static_cast<uint32_t>(bindInfos.size()), bindInfos.data());
#else
    for(size_t i = 0; i < imgMemReqs.size(); i++)
    {
      if(a_images[i] != VK_NULL_HANDLE)// unnecessary check ?
        vkBindImageMemory(m_device, a_images[i], m_imgAlloc.memory, m_imgAlloc.offset + imgOffsets[i]);
    }
#endif

    return IMG_ALLOC_ID;
  }

  void MemoryAlloc_Special::Free(uint32_t a_memBlockId)
  {
    assert(a_memBlockId == BUF_ALLOC_ID || a_memBlockId == IMG_ALLOC_ID);

    switch(a_memBlockId)
    {
    case BUF_ALLOC_ID:
      if(m_bufAlloc.memory != VK_NULL_HANDLE)
      {
        vkFreeMemory(m_device, m_bufAlloc.memory, nullptr);
        m_bufAlloc.memory = VK_NULL_HANDLE;
        m_bufAlloc.size   = 0;
        m_bufAlloc.offset = 0;
      }
      break;
    case IMG_ALLOC_ID:
      if(m_imgAlloc.memory != VK_NULL_HANDLE)
      {
        vkFreeMemory(m_device, m_imgAlloc.memory, nullptr);
        m_imgAlloc.memory = VK_NULL_HANDLE;
        m_imgAlloc.size   = 0;
        m_imgAlloc.offset = 0;
      }
      break;
    default:
      break;
    }
  }

  void MemoryAlloc_Special::FreeAllMemory()
  {
    Free(BUF_ALLOC_ID);
    Free(IMG_ALLOC_ID);
  }

  MemoryBlock MemoryAlloc_Special::GetMemoryBlock(uint32_t a_memBlockId) const
  {
    assert(a_memBlockId == BUF_ALLOC_ID || a_memBlockId == IMG_ALLOC_ID);

    switch(a_memBlockId)
    {
    case BUF_ALLOC_ID:
      return m_bufAlloc;
    case IMG_ALLOC_ID:
      return m_imgAlloc;
    default:
      return {};
    }
  }

  void* MemoryAlloc_Special::Map(uint32_t a_memBlockId, VkDeviceSize a_offset, VkDeviceSize a_size)
  {
    assert(a_memBlockId == BUF_ALLOC_ID || a_memBlockId == IMG_ALLOC_ID);

    void* ptr = nullptr;

    switch(a_memBlockId)
    {
    case BUF_ALLOC_ID:
      VK_CHECK_RESULT(vkMapMemory(m_device, m_bufAlloc.memory, a_offset, a_size, 0, &ptr));
      break;
    case IMG_ALLOC_ID:
      VK_CHECK_RESULT(vkMapMemory(m_device, m_imgAlloc.memory, a_offset, a_size, 0, &ptr));
      break;
    default:
      break;
    }
    return ptr;
  }

  void MemoryAlloc_Special::Unmap(uint32_t a_memBlockId)
  {
    assert(a_memBlockId == BUF_ALLOC_ID || a_memBlockId == IMG_ALLOC_ID);

    switch(a_memBlockId)
    {
    case BUF_ALLOC_ID:
      vkUnmapMemory(m_device, m_bufAlloc.memory);
      break;
    case IMG_ALLOC_ID:
      vkUnmapMemory(m_device, m_imgAlloc.memory);
      break;
    default:
      break;
    }
  }
}

