#include "vk_resource_manager.h"
#include "vk_alloc.h"
#include "vk_utils.h"
#include "vk_buffers.h"
#include "vk_images.h"
#include <unordered_set>

namespace vk_utils
{

  ResourceManager::ResourceManager(VkDevice a_device, VkPhysicalDevice a_physicalDevice,
                                   std::shared_ptr<IMemoryAlloc> a_pAlloc, std::shared_ptr<ICopyEngine> a_pCopy) :
      m_device(a_device), m_physicalDevice(a_physicalDevice), m_pAlloc(a_pAlloc), m_pCopy(a_pCopy)
  {
    m_samplerPool.init(m_device);
  }

  void ResourceManager::Cleanup()
  {
    std::unordered_set<uint32_t> allocIds;
    allocIds.reserve(m_bufAllocs.size() + m_imgAllocs.size());
    for(auto& [buf, _] : m_bufAllocs)
    {
      auto id = m_bufAllocs[buf];
      vkDestroyBuffer(m_device, buf, nullptr);

      allocIds.insert(id);
    }
    m_bufAllocs.clear();

    for(auto& [img, _] : m_imgAllocs)
    {
      auto id = m_imgAllocs[img];
      vkDestroyImage(m_device, img, nullptr);

      allocIds.insert(id);
    }
    m_imgAllocs.clear();

    for(auto id : allocIds)
      m_pAlloc->Free(id);

    m_allocRefCount.clear();

    m_samplerPool.deinit();
  }

  VkBuffer ResourceManager::CreateBuffer(VkDeviceSize a_size, VkBufferUsageFlags a_usage, VkMemoryPropertyFlags a_memProps,
                                         VkMemoryAllocateFlags flags)
  {
    VkMemoryRequirements memreq = {};
    auto buf = vk_utils::createBuffer(m_device, a_size, a_usage, &memreq);

    MemAllocInfo allocInfo  = {};
    allocInfo.allocateFlags = flags;
    allocInfo.memUsage      = a_memProps;
    allocInfo.memReq        = memreq;

    uint32_t allocId = m_pAlloc->Allocate(allocInfo);
    vkBindBufferMemory(m_device, buf, m_pAlloc->GetMemoryBlock(allocId).memory, m_pAlloc->GetMemoryBlock(allocId).offset);

    m_bufAllocs[buf] = allocId;
    if(m_allocRefCount.count(allocId))
      m_allocRefCount[allocId] += 1;
    else
      m_allocRefCount[allocId] = 1;

    return buf;
  }

  VkBuffer ResourceManager::CreateBuffer(const void* a_data, VkDeviceSize a_size, VkBufferUsageFlags a_usage,
                                         VkMemoryPropertyFlags a_memProps, VkMemoryAllocateFlags flags)
  {
    auto buf = CreateBuffer(a_size, a_usage, a_memProps, flags);

    m_pCopy->UpdateBuffer(buf, 0, a_data, a_size);

    return buf;
  }

  std::vector<VkBuffer> ResourceManager::CreateBuffers(const std::vector<void*> &a_dataPointers, const std::vector<VkDeviceSize> &a_sizes,
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

  std::vector<VkBuffer> ResourceManager::CreateBuffers(const std::vector<VkDeviceSize> &a_sizes,
                                                       const std::vector<VkBufferUsageFlags> &a_usages,
                                                       VkMemoryPropertyFlags a_memProps, VkMemoryAllocateFlags flags)
  {
    std::vector<VkBuffer> buffers(a_sizes.size());
    for(size_t i = 0; i < buffers.size(); ++i)
    {
      VkMemoryRequirements memreq = {};
      auto buf = vk_utils::createBuffer(m_device, a_sizes[i], a_usages[i], &memreq);
      buffers[i]  = buf;
    }

    MemAllocInfo allocInfo  = {};
    allocInfo.allocateFlags = flags;
    allocInfo.memUsage      = a_memProps;

    uint32_t allocId = m_pAlloc->Allocate(allocInfo, buffers);

    for (size_t i = 0; i < buffers.size(); i++)
    {
      m_bufAllocs[buffers[i]] = allocId;
    }

    if(m_allocRefCount.count(allocId))
      m_allocRefCount[allocId] += static_cast<uint32_t>(buffers.size());
    else
      m_allocRefCount[allocId]  = static_cast<uint32_t>(buffers.size());

    return buffers;
  }

  void* ResourceManager::MapBufferToHostMemory(VkBuffer a_buf, VkDeviceSize a_offset, VkDeviceSize a_size)
  {
    void* pRes = nullptr;
    if(m_bufAllocs.count(a_buf) > 0)
    {
      auto allocId = m_bufAllocs[a_buf];
      pRes = m_pAlloc->Map(allocId, a_offset, a_size);

      if(pRes)
        m_allocsMapped.insert(allocId);
    }
    return pRes;
  }

  void ResourceManager::UnmapBuffer(VkBuffer a_buf)
  {
    if(m_bufAllocs.count(a_buf) > 0)
    {
      auto allocId = m_bufAllocs[a_buf];
      m_pAlloc->Unmap(allocId);
      
      m_allocsMapped.erase(allocId);
    }
  }

  VkImage ResourceManager::CreateImage(const VkImageCreateInfo& a_createInfo)
  {
    VkImage image;
    VK_CHECK_RESULT(vkCreateImage(m_device, &a_createInfo, nullptr, &image));

    MemAllocInfo allocInfo{};
    allocInfo.memUsage = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;
    vkGetImageMemoryRequirements(m_device, image, &allocInfo.memReq);

    auto allocId = m_pAlloc->Allocate(allocInfo);
    m_imgAllocs[image] = allocId;

    if(m_allocRefCount.count(allocId))
      m_allocRefCount[allocId] += 1;
    else
      m_allocRefCount[allocId] = 1;

    vkBindImageMemory(m_device, image, m_pAlloc->GetMemoryBlock(allocId).memory, m_pAlloc->GetMemoryBlock(allocId).offset);

    return image;
  }

  VkImage ResourceManager::CreateImage(uint32_t a_width, uint32_t a_height, VkFormat a_format, VkImageUsageFlags a_usage,
                                       uint32_t a_mipLvls)
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

  VkImage ResourceManager::CreateImage(const void* a_data, uint32_t a_width, uint32_t a_height, VkFormat a_format,
                                       VkImageUsageFlags a_usage, VkImageLayout a_layout, uint32_t a_mipLvls)
  {
    auto img = CreateImage(a_width, a_height, a_format, a_usage, a_mipLvls);

    m_pCopy->UpdateImage(img, a_data, a_width, a_height, vk_utils::bppFromVkFormat(a_format), a_layout);

    return VK_NULL_HANDLE;
  }

  std::vector<VkImage> ResourceManager::CreateImages(const std::vector<VkImageCreateInfo>& a_createInfos)
  {
    std::vector<VkImage> images(a_createInfos.size());
    for(size_t i = 0; i < images.size(); ++i)
    {
      VK_CHECK_RESULT(vkCreateImage(m_device, &a_createInfos[i], nullptr, &images[i]));
    }

    MemAllocInfo allocInfo = {};
    allocInfo.memUsage     = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;

    uint32_t allocId = m_pAlloc->Allocate(allocInfo, images);

    for (size_t i = 0; i < images.size(); i++)
    {
      m_imgAllocs[images[i]] = allocId;
    }

    if(m_allocRefCount.count(allocId))
      m_allocRefCount[allocId] += static_cast<uint32_t>(images.size());
    else
      m_allocRefCount[allocId]  = static_cast<uint32_t>(images.size());

    return images;
  }

  VulkanTexture ResourceManager::CreateTexture(const VkImageCreateInfo& a_createInfo, VkImageViewCreateInfo& a_imgViewCreateInfo)
  {
    VulkanTexture res{};
    res.image = CreateImage(a_createInfo);
    res.resource_id = m_imgAllocs[res.image];

    a_imgViewCreateInfo.image = res.image;
    VK_CHECK_RESULT(vkCreateImageView(m_device, &a_imgViewCreateInfo, nullptr, &res.descriptor.imageView));

    return res;
  }

  VulkanTexture ResourceManager::CreateTexture(const VkImageCreateInfo& a_createInfo, VkImageViewCreateInfo& a_imgViewCreateInfo,
                                               const VkSamplerCreateInfo& a_samplerCreateInfo)
  {
    VulkanTexture res{};
    res.image = CreateImage(a_createInfo);
    res.resource_id = m_imgAllocs[res.image];

    a_imgViewCreateInfo.image = res.image;
    VK_CHECK_RESULT(vkCreateImageView(m_device, &a_imgViewCreateInfo, nullptr, &res.descriptor.imageView));

    res.descriptor.sampler = m_samplerPool.acquireSampler(a_samplerCreateInfo);

    return res;
  }

  std::vector<VulkanTexture> ResourceManager::CreateTextures(const std::vector<VkImageCreateInfo>& a_createInfos,
                                                             std::vector<VkImageViewCreateInfo>& a_imgViewCreateInfos)
  {
    std::vector<VulkanTexture> res(a_createInfos.size());

    auto imgs = CreateImages(a_createInfos);
    for(size_t i = 0; i < a_createInfos.size(); ++i)
    {
      res[i].image = imgs[i];
      res[i].resource_id = m_imgAllocs[res[i].image];

      a_imgViewCreateInfos[i].image = res[i].image;
      VK_CHECK_RESULT(vkCreateImageView(m_device, &a_imgViewCreateInfos[i], nullptr, &res[i].descriptor.imageView));
    }

    return res;
  }

  std::vector<VulkanTexture> ResourceManager::CreateTextures(const std::vector<VkImageCreateInfo>& a_createInfos,
                                                             std::vector<VkImageViewCreateInfo>& a_imgViewCreateInfos,
                                                             const std::vector<VkSamplerCreateInfo>& a_samplerCreateInfos)
  {
    std::vector<VulkanTexture> res(a_createInfos.size());

    auto imgs = CreateImages(a_createInfos);
    for(size_t i = 0; i < a_createInfos.size(); ++i)
    {
      res[i].image = imgs[i];
      res[i].resource_id = m_imgAllocs[res[i].image];

      a_imgViewCreateInfos[i].image = res[i].image;
      VK_CHECK_RESULT(vkCreateImageView(m_device, &a_imgViewCreateInfos[i], nullptr, &res[i].descriptor.imageView));

      res[i].descriptor.sampler = CreateSampler(a_samplerCreateInfos[i]);
    }

    return res;
  }

  VkSampler ResourceManager::CreateSampler(const VkSamplerCreateInfo& a_samplerCreateInfo)
  {
    return m_samplerPool.acquireSampler(a_samplerCreateInfo);
  }

  void ResourceManager::DestroyBuffer(VkBuffer &a_buffer)
  {
    if(a_buffer == VK_NULL_HANDLE)
      return;
    if(!m_bufAllocs.count(a_buffer))
    {
      VK_UTILS_LOG_WARNING("[ResourceManager::DestroyBuffer] trying to destroy unknown buffer");
      return;
    }

    auto id = m_bufAllocs[a_buffer];
    vkDestroyBuffer(m_device, a_buffer, nullptr);

    m_allocRefCount[id] -= 1;
    if(m_allocRefCount[id] == 0)
    {
      if(m_allocsMapped.count(id) > 0)
      {
        m_pAlloc->Unmap(id);
        m_allocsMapped.erase(id);
      }
      m_pAlloc->Free(id);
    }

    m_bufAllocs.erase(a_buffer);
    a_buffer = VK_NULL_HANDLE;
  }

  void ResourceManager::DestroyImage(VkImage &a_image)
  {
    if(a_image == VK_NULL_HANDLE)
      return;
    if(!m_imgAllocs.count(a_image))
    {
      VK_UTILS_LOG_WARNING("[ResourceManager::DestroyImage] trying to destroy unknown image");
      return;
    }

    auto id = m_imgAllocs[a_image];
    vkDestroyImage(m_device, a_image, nullptr);

    m_allocRefCount[id] -= 1;
    if(m_allocRefCount[id] == 0)
      m_pAlloc->Free(id);

    m_imgAllocs.erase(a_image);
    a_image = VK_NULL_HANDLE;
  }

  void ResourceManager::DestroyTexture(VulkanTexture &a_texture)
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

  void ResourceManager::DestroySampler(VkSampler &a_sampler)
  {
    if(a_sampler != VK_NULL_HANDLE)
    {
      m_samplerPool.releaseSampler(a_sampler);
      a_sampler = VK_NULL_HANDLE;
    }
  }

}
