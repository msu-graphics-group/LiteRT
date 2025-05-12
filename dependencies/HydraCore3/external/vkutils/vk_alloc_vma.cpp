#define VMA_IMPLEMENTATION
#define VK_NO_PROTOTYPES
#define VMA_STATIC_VULKAN_FUNCTIONS  0
#define VMA_DYNAMIC_VULKAN_FUNCTIONS 0
#include "external/vk_mem_alloc.h"

#include "vk_alloc_vma.h"
#include "vk_utils.h"
#include "vk_images.h"
#include "vk_buffers.h"

namespace vk_utils
{
  std::shared_ptr<IMemoryAlloc> CreateMemoryAlloc_VMA(VkInstance a_instance,
                                                      VkDevice a_device, VkPhysicalDevice a_physicalDevice,
                                                      VkFlags a_flags, uint32_t a_vkAPIVersion)
  {
    return std::make_shared<MemoryAlloc_VMA>(a_instance, a_device, a_physicalDevice, a_flags, a_vkAPIVersion);
  }

  std::shared_ptr<IMemoryAlloc> CreateMemoryAlloc_VMA(VkDevice a_device, VkPhysicalDevice a_physicalDevice,
                                                      VmaAllocator a_allocator)
  {
    return std::make_shared<MemoryAlloc_VMA>(a_device, a_physicalDevice, a_allocator);
  }

  static inline VmaMemoryUsage getVMAMemoryUsage(VkMemoryPropertyFlags flags)
  {
    if((flags & VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT)       == VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT)
      return VMA_MEMORY_USAGE_GPU_ONLY;
    else if((flags & VK_MEMORY_PROPERTY_HOST_COHERENT_BIT) == VK_MEMORY_PROPERTY_HOST_COHERENT_BIT)
      return VMA_MEMORY_USAGE_CPU_ONLY;
    else if((flags & VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT)  == VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT)
      return VMA_MEMORY_USAGE_CPU_TO_GPU;

    return VMA_MEMORY_USAGE_UNKNOWN;
  }

  static inline VmaMemoryUsage getVMAMemoryUsage2(VkMemoryPropertyFlags flags)
  {
    if((flags & VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT)       == VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT)
      return VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
//    else if((flags & VK_MEMORY_PROPERTY_HOST_COHERENT_BIT) == VK_MEMORY_PROPERTY_HOST_COHERENT_BIT)
//      return VMA_MEMORY_USAGE_AUTO_PREFER_HOST;
//    else if((flags & VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT)  == VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT)
//      return VMA_MEMORY_USAGE_CPU_TO_GPU;

    return VMA_MEMORY_USAGE_AUTO;
  }

  static inline VmaAllocationCreateFlags getVMAFlags(VkMemoryPropertyFlags flags)
  {
    if((flags & VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT)       == VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT)
      return 0;
    else if((flags & VK_MEMORY_PROPERTY_HOST_COHERENT_BIT) == VK_MEMORY_PROPERTY_HOST_COHERENT_BIT)
      return VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT;
    else if((flags & VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT)  == VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT)
      return VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT;

    //TODO: VMA_ALLOCATION_CREATE_HOST_ACCESS_ALLOW_TRANSFER_INSTEAD_BIT

    return VMA_MEMORY_USAGE_UNKNOWN;
  }

  VmaAllocator initVMA(VkInstance a_instance, VkDevice a_device, VkPhysicalDevice a_physicalDevice,
                       VkFlags a_flags, uint32_t a_vkAPIVersion)
  {
    VmaVulkanFunctions functions = {};
    functions.vkAllocateMemory                    = vkAllocateMemory;
    functions.vkGetInstanceProcAddr               = vkGetInstanceProcAddr;
    functions.vkGetDeviceProcAddr                 = vkGetDeviceProcAddr;
    functions.vkGetPhysicalDeviceProperties       = vkGetPhysicalDeviceProperties;
    functions.vkGetPhysicalDeviceMemoryProperties = vkGetPhysicalDeviceMemoryProperties;
    functions.vkFreeMemory                        = vkFreeMemory;
    functions.vkMapMemory                         = vkMapMemory;
    functions.vkUnmapMemory                       = vkUnmapMemory;
    functions.vkFlushMappedMemoryRanges           = vkFlushMappedMemoryRanges;
    functions.vkInvalidateMappedMemoryRanges      = vkInvalidateMappedMemoryRanges;
    functions.vkBindBufferMemory                  = vkBindBufferMemory;
    functions.vkBindImageMemory                   = vkBindImageMemory;
    functions.vkGetBufferMemoryRequirements       = vkGetBufferMemoryRequirements;
    functions.vkGetImageMemoryRequirements        = vkGetImageMemoryRequirements;
    functions.vkCreateBuffer                      = vkCreateBuffer;
    functions.vkDestroyBuffer                     = vkDestroyBuffer;
    functions.vkCreateImage                       = vkCreateImage;
    functions.vkDestroyImage                      = vkDestroyImage;
    functions.vkCmdCopyBuffer                     = vkCmdCopyBuffer;
    if(a_vkAPIVersion >= VK_API_VERSION_1_1)
    {
      functions.vkGetBufferMemoryRequirements2KHR       = vkGetBufferMemoryRequirements2;
      functions.vkGetImageMemoryRequirements2KHR        = vkGetImageMemoryRequirements2;
      functions.vkBindBufferMemory2KHR                  = vkBindBufferMemory2;
      functions.vkBindImageMemory2KHR                   = vkBindImageMemory2;
      functions.vkGetPhysicalDeviceMemoryProperties2KHR = vkGetPhysicalDeviceMemoryProperties2;
    }

    if(a_vkAPIVersion >= VK_API_VERSION_1_3)
    {
      functions.vkGetDeviceBufferMemoryRequirements = vkGetDeviceBufferMemoryRequirements;
      functions.vkGetDeviceImageMemoryRequirements  = vkGetDeviceImageMemoryRequirements;
    }

    VmaAllocatorCreateInfo allocatorInfo = {};
    allocatorInfo.vulkanApiVersion = a_vkAPIVersion;
    allocatorInfo.instance         = a_instance;
    allocatorInfo.physicalDevice   = a_physicalDevice;
    allocatorInfo.device           = a_device;
    allocatorInfo.flags            = a_flags;  // VMA_ALLOCATOR_CREATE_BUFFER_DEVICE_ADDRESS_BIT;
    allocatorInfo.pVulkanFunctions = &functions;

    VmaAllocator allocator;
    vmaCreateAllocator(&allocatorInfo, &allocator);

    return allocator;
  }

  MemoryAlloc_VMA::MemoryAlloc_VMA(VkDevice a_device, VkPhysicalDevice a_physicalDevice, VmaAllocator a_allocator)
    : m_device(a_device), m_physicalDevice(a_physicalDevice), m_vma(a_allocator)
  {

  }

  MemoryAlloc_VMA::MemoryAlloc_VMA(VkInstance a_instance, VkDevice a_device, VkPhysicalDevice a_physicalDevice,
                                   VkFlags a_flags, uint32_t a_vkAPIVersion) :
                                   m_device(a_device), m_physicalDevice(a_physicalDevice)
  {
    // vma created internally and will be destroyed internally
    m_destroyVma = true;
    m_vma = initVMA(a_instance, a_device, a_physicalDevice, a_flags, a_vkAPIVersion);
  }

  VmaAllocator MemoryAlloc_VMA::GetVMA()
  {
    // don't return vma if it was created internally
    if(m_destroyVma)
      return nullptr;
    else
      return m_vma;
  }

  MemoryAlloc_VMA::~MemoryAlloc_VMA()
  {
    Cleanup();
    if(m_destroyVma)
      vmaDestroyAllocator(m_vma);
  }

  void MemoryAlloc_VMA::Cleanup()
  {
    FreeAllMemory();
  }

  uint32_t MemoryAlloc_VMA::Allocate(const MemAllocInfo& a_allocInfo)
  {
    VmaAllocationCreateInfo vmaAllocCreateInfo = {};
    vmaAllocCreateInfo.usage = getVMAMemoryUsage(a_allocInfo.memUsage);
    if(a_allocInfo.dedicated_image || a_allocInfo.dedicated_buffer)
    {
      vmaAllocCreateInfo.flags |= VMA_ALLOCATION_CREATE_DEDICATED_MEMORY_BIT;
    }

    VmaAllocationInfo vmaAllocInfo;
    VmaAllocation     alloc = nullptr;

    VkResult result = vmaAllocateMemory(m_vma, &a_allocInfo.memReq, &vmaAllocCreateInfo, &alloc, &vmaAllocInfo);

    VK_CHECK_RESULT(result);

    auto allocId = nextAllocIdx;
    m_allocations[allocId] = alloc;
    nextAllocIdx++;

    return allocId;
  }

  uint32_t MemoryAlloc_VMA::Allocate(const MemAllocInfo& a_allocInfoBuffers, const std::vector<VkBuffer> &a_buffers)
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
        VK_UTILS_LOG_WARNING("[MemoryAlloc_VMA::Allocate]: input buffers have different memReq.memoryTypeBits");
        return UINT32_MAX;
      }
    }

    auto bufOffsets  = calculateMemOffsets(bufMemReqs);
    auto bufMemTotal = bufOffsets[bufOffsets.size() - 1];

    allocInfo.memReq      = bufMemReqs[0];
    allocInfo.memReq.size = bufMemTotal;

//    vmaAllocateMemoryForBuffer(m_vma, a_buffers[i], vmaAllocCreateInfo, alloc, vmaAllocInfo);
    auto allocId = Allocate(allocInfo);

    for(size_t i = 0; i < bufMemReqs.size(); i++)
    {
      if(a_buffers[i] != VK_NULL_HANDLE)// unnecessary check ?
        vmaBindBufferMemory2(m_vma, m_allocations[allocId], bufOffsets[i], a_buffers[i], nullptr);
    }

    return allocId;
  }

  uint32_t MemoryAlloc_VMA::Allocate(const MemAllocInfo& a_allocInfoImages, const std::vector<VkImage> &a_images)
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
        VK_UTILS_LOG_WARNING("[MemoryAlloc_VMA::Allocate]: input images have different memReq.memoryTypeBits");
        return UINT32_MAX;
      }
    }

    auto imgOffsets  = calculateMemOffsets(imgMemReqs);
    auto imgMemTotal = imgOffsets[imgOffsets.size() - 1];

    allocInfo.memReq      = imgMemReqs[0];
    allocInfo.memReq.size = imgMemTotal;

    auto allocId = Allocate(allocInfo);

    for(size_t i = 0; i < imgMemReqs.size(); i++)
    {
      if(a_images[i] != VK_NULL_HANDLE)// unnecessary check ?
        vmaBindImageMemory2(m_vma, m_allocations[allocId], imgOffsets[i], a_images[i], nullptr);
    }

    return allocId;
  }

  VkBuffer MemoryAlloc_VMA::AllocateBuffer(const VkBufferCreateInfo& a_bufCreateInfo, VkMemoryPropertyFlags a_memProps)
  {
    VmaAllocationCreateInfo allocInfo = {};
    allocInfo.usage = getVMAMemoryUsage2(a_memProps);
    allocInfo.flags |= getVMAFlags(a_memProps);

    VmaAllocation allocation;
    VkBuffer buffer;
    auto result = vmaCreateBuffer(m_vma, &a_bufCreateInfo, &allocInfo, &buffer, &allocation, nullptr);
    VK_CHECK_RESULT(result);

    auto allocId = nextAllocIdx;
    m_allocations[allocId] = allocation;
    nextAllocIdx++;

    return buffer;
  }

  VkImage MemoryAlloc_VMA::AllocateImage(const VkImageCreateInfo& a_imgCreateInfo, VkMemoryPropertyFlags a_memProps)
  {
    VmaAllocationCreateInfo allocInfo = {};
    allocInfo.usage = getVMAMemoryUsage2(a_memProps);
    allocInfo.flags |= getVMAFlags(a_memProps);

    VmaAllocation allocation;
    VkImage image;
    auto result = vmaCreateImage(m_vma, &a_imgCreateInfo, &allocInfo, &image, &allocation, nullptr);
    VK_CHECK_RESULT(result);

    auto allocId = nextAllocIdx;
    m_allocations[allocId] = allocation;
    nextAllocIdx++;

    return image;
  }

  void MemoryAlloc_VMA::Free(uint32_t a_memBlockId)
  {
    if(!m_allocations.count(a_memBlockId))
      return;

    vmaFreeMemory(m_vma, m_allocations[a_memBlockId]);
    m_allocations.erase(a_memBlockId);
  }

  void MemoryAlloc_VMA::FreeAllMemory()
  {
    for(auto& [idx, _] : m_allocations)
    {
      Free(idx);
    }
  }

  MemoryBlock MemoryAlloc_VMA::GetMemoryBlock(uint32_t a_memBlockId) const
  {
    if(!m_allocations.count(a_memBlockId))
      return {};

    VmaAllocationInfo allocInfo;
    vmaGetAllocationInfo(m_vma, m_allocations.at(a_memBlockId), &allocInfo);

    MemoryBlock memInfo = {};
    memInfo.memory = allocInfo.deviceMemory;
    memInfo.offset = allocInfo.offset;
    memInfo.size   = allocInfo.size;

    return memInfo;
  }

  void* MemoryAlloc_VMA::Map(uint32_t a_memBlockId, VkDeviceSize a_offset, VkDeviceSize a_size)
  {
    if(!m_allocations.count(a_memBlockId))
      return nullptr;

    (void)a_offset;
    (void)a_size;

    void* ptr;
    VkResult result = vmaMapMemory(m_vma, m_allocations[a_memBlockId], &ptr);
    VK_CHECK_RESULT(result);

    return ptr;
  }

  void MemoryAlloc_VMA::Unmap(uint32_t a_memBlockId)
  {
    if(!m_allocations.count(a_memBlockId))
      return;

    vmaUnmapMemory(m_vma, m_allocations[a_memBlockId]);
  }

  //*********************************************************************************

  ResourceManager_VMA::ResourceManager_VMA(VkDevice a_device, VkPhysicalDevice a_physicalDevice,
                                           VmaAllocator a_allocator, std::shared_ptr<ICopyEngine> a_pCopy) :
    m_device(a_device), m_physicalDevice(a_physicalDevice), m_vma(a_allocator), m_pCopy(a_pCopy)
  {
    m_samplerPool.init(m_device);
  }

  ResourceManager_VMA::~ResourceManager_VMA()
  {
    Cleanup();
    vmaDestroyAllocator(m_vma);
  }

  void ResourceManager_VMA::Cleanup()
  {
    for(auto& [buf, _] : m_bufAllocs)
    {
      vmaDestroyBuffer(m_vma, buf, m_bufAllocs[buf]);
    }
    m_bufAllocs.clear();

    for(auto& [img, _] : m_imgAllocs)
    {
      vmaDestroyImage(m_vma, img, m_imgAllocs[img]);
    }
    m_imgAllocs.clear();

    m_samplerPool.deinit();
  }

  VkBuffer ResourceManager_VMA::CreateBuffer(VkDeviceSize a_size, VkBufferUsageFlags a_usage,
                                             VkMemoryPropertyFlags a_memProps, VkMemoryAllocateFlags)
  {
    VkBuffer buffer;

    VkBufferCreateInfo bufferInfo {};
    bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    bufferInfo.size  = a_size;
    bufferInfo.usage = a_usage;

    VmaAllocationCreateInfo allocInfo = {};
    allocInfo.usage = getVMAMemoryUsage2(a_memProps);
    allocInfo.flags |= getVMAFlags(a_memProps);

    VmaAllocation allocation;
    vmaCreateBuffer(m_vma, &bufferInfo, &allocInfo, &buffer,
      &allocation, nullptr);

    m_bufAllocs[buffer] = allocation;

    return buffer;
  }

  VkBuffer ResourceManager_VMA::CreateBuffer(const void* a_data, VkDeviceSize a_size, VkBufferUsageFlags a_usage,
                                             VkMemoryPropertyFlags a_memProps, VkMemoryAllocateFlags flags)
  {
    auto buf = CreateBuffer(a_size, a_usage, a_memProps, flags);

    m_pCopy->UpdateBuffer(buf, 0, a_data, a_size);

    return buf;
  }

  std::vector<VkBuffer> ResourceManager_VMA::CreateBuffers(const std::vector<void*> &a_dataPointers,
                                                           const std::vector<VkDeviceSize> &a_sizes,
                                                           const std::vector<VkBufferUsageFlags> &a_usages,
                                                           VkMemoryPropertyFlags a_memProps, VkMemoryAllocateFlags flags)
  {
    std::vector<VkBuffer> buffers = CreateBuffers(a_sizes, a_usages, a_memProps, flags);

    for (size_t i = 0; i < buffers.size(); i++)
    {
      m_pCopy->UpdateBuffer(buffers[i], 0, a_dataPointers[i], a_sizes[i]);
    }

    return buffers;
  }

  std::vector<VkBuffer> ResourceManager_VMA::CreateBuffers(const std::vector<VkDeviceSize> &a_sizes,
                                                           const std::vector<VkBufferUsageFlags> &a_usages,
                                                           VkMemoryPropertyFlags a_memProps, VkMemoryAllocateFlags flags)
  {
    std::vector<VkBuffer> buffers(a_usages.size());
    for(size_t i = 0; i < buffers.size(); ++i)
    {
      VkBufferCreateInfo bufferInfo {};
      bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
      bufferInfo.size  = a_sizes[i];
      bufferInfo.usage = a_usages[i];

      VmaAllocationCreateInfo allocInfo = {};
      allocInfo.usage = getVMAMemoryUsage2(a_memProps);
      allocInfo.flags |= getVMAFlags(a_memProps);

      VmaAllocation allocation;
      vmaCreateBuffer(m_vma, &bufferInfo, &allocInfo, &buffers[i],
        &allocation, nullptr);

      m_bufAllocs[buffers[i]] = allocation;
    }

    return buffers;
  }


  void* ResourceManager_VMA::MapBufferToHostMemory(VkBuffer a_buf, VkDeviceSize a_offset, VkDeviceSize a_size)
  {
    void* pRes = nullptr;
    if(m_bufAllocs.count(a_buf) > 0)
    {
      (void)a_offset;
      (void)a_size;

      VkResult result = vmaMapMemory(m_vma, m_bufAllocs[a_buf], &pRes);
      VK_CHECK_RESULT(result);
    }
    return pRes;
  }

  void ResourceManager_VMA::UnmapBuffer(VkBuffer a_buf)
  {
    if(m_bufAllocs.count(a_buf) > 0)
    {
      vmaUnmapMemory(m_vma, m_bufAllocs[a_buf]);
    }
  }

  VkImage ResourceManager_VMA::CreateImage(const VkImageCreateInfo& a_createInfo)
  {
    VmaAllocationCreateInfo allocInfo = {};
    allocInfo.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
//    allocInfo.flags = VMA_ALLOCATION_CREATE_USER_DATA_COPY_STRING_BIT;
//    allocInfo.pUserData = imageName.c_str();

    VkImage image;
    VmaAllocation allocation;
    vmaCreateImage(m_vma, &a_createInfo, &allocInfo, &image, &allocation, nullptr);

    m_imgAllocs[image] = allocation;

    return image;
  }

  VkImage ResourceManager_VMA::CreateImage(uint32_t a_width, uint32_t a_height, VkFormat a_format,
                                           VkImageUsageFlags a_usage, uint32_t a_mipLvls)
  {
    VkImageCreateInfo imageCreateInfo{};
    imageCreateInfo.sType         = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    imageCreateInfo.imageType     = VK_IMAGE_TYPE_2D;
    imageCreateInfo.usage         = a_usage;
    imageCreateInfo.format        = a_format;
    imageCreateInfo.extent.width  = a_width;
    imageCreateInfo.extent.height = a_height;
    imageCreateInfo.extent.depth  = 1;
    imageCreateInfo.mipLevels     = a_mipLvls;
    imageCreateInfo.arrayLayers   = 1;
    imageCreateInfo.samples       = VK_SAMPLE_COUNT_1_BIT;
    imageCreateInfo.tiling        = VK_IMAGE_TILING_OPTIMAL;

    auto img = CreateImage(imageCreateInfo);

    return img;
  }

  VkImage ResourceManager_VMA::CreateImage(const void* a_data, uint32_t a_width, uint32_t a_height, VkFormat a_format,
                                           VkImageUsageFlags a_usage, VkImageLayout a_layout, uint32_t a_mipLvls)
  {
    auto img = CreateImage(a_width, a_height, a_format, a_usage, a_mipLvls);

    m_pCopy->UpdateImage(img, a_data, a_width, a_height, vk_utils::bppFromVkFormat(a_format), a_layout);

    return VK_NULL_HANDLE;
  }

  std::vector<VkImage> ResourceManager_VMA::CreateImages(const std::vector<VkImageCreateInfo>& a_createInfos)
  {
    std::vector<VkImage> res(a_createInfos.size());
    for(size_t i = 0; i < a_createInfos.size(); ++i) // no need to manually create single alloc in vma (?)
    {
      res[i] = CreateImage(a_createInfos[i]);
    }

    return res;
  }

  VulkanTexture ResourceManager_VMA::CreateTexture(const VkImageCreateInfo& a_createInfo,
                                                   VkImageViewCreateInfo& a_imgViewCreateInfo)
  {
    VulkanTexture res{};
    res.image = CreateImage(a_createInfo);

    a_imgViewCreateInfo.image = res.image;
    VK_CHECK_RESULT(vkCreateImageView(m_device, &a_imgViewCreateInfo, nullptr, &res.descriptor.imageView));

    return res;
  }

  VulkanTexture ResourceManager_VMA::CreateTexture(const VkImageCreateInfo& a_createInfo,
                                                   VkImageViewCreateInfo& a_imgViewCreateInfo,
                                                   const VkSamplerCreateInfo& a_samplerCreateInfo)
  {
    VulkanTexture res{};
    res.image = CreateImage(a_createInfo);

    a_imgViewCreateInfo.image = res.image;
    VK_CHECK_RESULT(vkCreateImageView(m_device, &a_imgViewCreateInfo, nullptr, &res.descriptor.imageView));

    res.descriptor.sampler = m_samplerPool.acquireSampler(a_samplerCreateInfo);

    return res;
  }

  std::vector<VulkanTexture> ResourceManager_VMA::CreateTextures(const std::vector<VkImageCreateInfo>& a_createInfos,
                                                                 std::vector<VkImageViewCreateInfo>& a_imgViewCreateInfos)
  {
    std::vector<VulkanTexture> res(a_createInfos.size());
    for(size_t i = 0; i < a_createInfos.size(); ++i)
    {
      res[i] = CreateTexture(a_createInfos[i], a_imgViewCreateInfos[i]);
    }

    return res;
  }

  std::vector<VulkanTexture> ResourceManager_VMA::CreateTextures(const std::vector<VkImageCreateInfo>& a_createInfos,
                                                                 std::vector<VkImageViewCreateInfo>& a_imgViewCreateInfos,
                                                                 const std::vector<VkSamplerCreateInfo>& a_samplerCreateInfos)
  {
    std::vector<VulkanTexture> res(a_createInfos.size());
    for(size_t i = 0; i < a_createInfos.size(); ++i)
    {
      res[i] = CreateTexture(a_createInfos[i], a_imgViewCreateInfos[i], a_samplerCreateInfos[i]);
    }

    return res;
  }

  VkSampler ResourceManager_VMA::CreateSampler(const VkSamplerCreateInfo& a_samplerCreateInfo)
  {
    return m_samplerPool.acquireSampler(a_samplerCreateInfo);
  }

  void ResourceManager_VMA::DestroyBuffer(VkBuffer &a_buffer)
  {
    if(a_buffer == VK_NULL_HANDLE)
      return;

    if(!m_bufAllocs.count(a_buffer))
    {
      VK_UTILS_LOG_WARNING("[ResourceManager_VMA::DestroyBuffer] trying to destroy unknown buffer");
      return;
    }

    vmaDestroyBuffer(m_vma, a_buffer, m_bufAllocs[a_buffer]);
    m_bufAllocs.erase(a_buffer);
    a_buffer = VK_NULL_HANDLE;
  }

  void ResourceManager_VMA::DestroyImage(VkImage &a_image)
  {
    if(a_image == VK_NULL_HANDLE)
      return;
    if(!m_imgAllocs.count(a_image))
    {
      VK_UTILS_LOG_WARNING("[ResourceManager_VMA::DestroyImage] trying to destroy unknown image");
      return;
    }

    vmaDestroyImage(m_vma, a_image, m_imgAllocs[a_image]);
    m_imgAllocs.erase(a_image);
    a_image = VK_NULL_HANDLE;
  }

  void ResourceManager_VMA::DestroyTexture(VulkanTexture &a_texture)
  {
    DestroyImage(a_texture.image);

    if(a_texture.descriptor.imageView != VK_NULL_HANDLE)
    {
      vkDestroyImageView(m_device, a_texture.descriptor.imageView, nullptr);
      a_texture.descriptor.imageView = VK_NULL_HANDLE;
    }

    if(a_texture.descriptor.sampler != VK_NULL_HANDLE)
    {
      m_samplerPool.releaseSampler(a_texture.descriptor.sampler);
      a_texture.descriptor.sampler = VK_NULL_HANDLE;
    }
  }

  void ResourceManager_VMA::DestroySampler(VkSampler &a_sampler)
  {
    if(a_sampler != VK_NULL_HANDLE)
    {
      m_samplerPool.releaseSampler(a_sampler);
      a_sampler = VK_NULL_HANDLE;
    }
  }

}
