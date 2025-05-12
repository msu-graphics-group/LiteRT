#ifndef VK_UTILS_RESOURCE_MANAGER_H
#define VK_UTILS_RESOURCE_MANAGER_H

#include "vk_alloc.h"
#include <unordered_set>

namespace vk_utils
{
  struct VulkanTexture
  {
    uint32_t resource_id = UINT32_MAX;
    VkImage image        = VK_NULL_HANDLE;
    VkDescriptorImageInfo descriptor{VK_NULL_HANDLE, VK_NULL_HANDLE, VK_IMAGE_LAYOUT_UNDEFINED};
  };

  struct IResourceManager
  {
    virtual ~IResourceManager() = default;

    virtual std::shared_ptr<IMemoryAlloc> GetAllocator() { return nullptr; }
    virtual std::shared_ptr<ICopyEngine>  GetCopyEngine() {return nullptr; }

    virtual VkBuffer CreateBuffer(VkDeviceSize a_size, VkBufferUsageFlags a_usage,
                                  VkMemoryPropertyFlags a_memProps, VkMemoryAllocateFlags flags) = 0;

    virtual VkBuffer CreateBuffer(const void* a_data, VkDeviceSize a_size, VkBufferUsageFlags a_usage,
                                  VkMemoryPropertyFlags a_memProps, VkMemoryAllocateFlags flags) = 0;

    virtual std::vector<VkBuffer> CreateBuffers(const std::vector<VkDeviceSize> &a_sizes, const std::vector<VkBufferUsageFlags> &a_usages,
                                                VkMemoryPropertyFlags a_memProps, VkMemoryAllocateFlags flags) = 0;

    virtual std::vector<VkBuffer> CreateBuffers(const std::vector<void*> &a_data, const std::vector<VkDeviceSize> &a_sizes,
                                                const std::vector<VkBufferUsageFlags> &a_usages,
                                                VkMemoryPropertyFlags a_memProps, VkMemoryAllocateFlags flags)  = 0;

    virtual void* MapBufferToHostMemory(VkBuffer a_buf, VkDeviceSize a_offset, VkDeviceSize a_size) = 0;
    virtual void UnmapBuffer(VkBuffer a_buf) = 0;

    virtual VkImage CreateImage(const VkImageCreateInfo& a_createInfo) = 0;

    virtual VkImage CreateImage(uint32_t a_width, uint32_t a_height, VkFormat a_format, VkImageUsageFlags a_usage, uint32_t a_mipLvls) = 0;

    virtual VkImage CreateImage(const void* a_data, uint32_t a_width, uint32_t a_height, VkFormat a_format, VkImageUsageFlags a_usage,
                                VkImageLayout a_finalLayout, uint32_t a_mipLvls) = 0;

    virtual std::vector<VkImage> CreateImages(const std::vector<VkImageCreateInfo>& a_createInfos) = 0;

    virtual VulkanTexture CreateTexture(const VkImageCreateInfo& a_createInfo, VkImageViewCreateInfo& a_imgViewCreateInfo) = 0;

    virtual VulkanTexture CreateTexture(const VkImageCreateInfo& a_createInfo, VkImageViewCreateInfo& a_imgViewCreateInfo,
                                        const VkSamplerCreateInfo& a_samplerCreateInfo) = 0;

    virtual std::vector<VulkanTexture> CreateTextures(const std::vector<VkImageCreateInfo>& a_createInfos,
                                                      std::vector<VkImageViewCreateInfo>& a_imgViewCreateInfos) = 0;

    virtual std::vector<VulkanTexture> CreateTextures(const std::vector<VkImageCreateInfo>& a_createInfos,
                                                      std::vector<VkImageViewCreateInfo>& a_imgViewCreateInfos,
                                                      const std::vector<VkSamplerCreateInfo>& a_samplerCreateInfos) = 0;

    virtual VkSampler CreateSampler(const VkSamplerCreateInfo& a_samplerCreateInfo) = 0;

    virtual void DestroyBuffer(VkBuffer &a_buffer) = 0;
    virtual void DestroyImage(VkImage &a_image) = 0;
    virtual void DestroyTexture(VulkanTexture &a_texture) = 0;
    virtual void DestroySampler(VkSampler &a_sampler) = 0;

    // create accel struct ?
    // map, unmap
  };

  struct ResourceManager : IResourceManager
  {
    ResourceManager(VkDevice a_device, VkPhysicalDevice a_physicalDevice, std::shared_ptr<IMemoryAlloc> a_pAlloc,
                    std::shared_ptr<ICopyEngine> a_pCopy);

    ResourceManager(ResourceManager const&) = delete;
    ResourceManager& operator=(ResourceManager const&) = delete;

    virtual ~ResourceManager() { Cleanup(); };

    void Cleanup();

    std::shared_ptr<IMemoryAlloc> GetAllocator()  override { return m_pAlloc; }
    std::shared_ptr<ICopyEngine>  GetCopyEngine() override {return m_pCopy; }

    VkBuffer CreateBuffer(VkDeviceSize a_size, VkBufferUsageFlags a_usage,  VkMemoryPropertyFlags a_memProps,
                          VkMemoryAllocateFlags flags) override;

    VkBuffer CreateBuffer(const void* a_data, VkDeviceSize a_size, VkBufferUsageFlags a_usage,
                          VkMemoryPropertyFlags a_memProps, VkMemoryAllocateFlags flags) override;

    std::vector<VkBuffer> CreateBuffers(const std::vector<VkDeviceSize> &a_sizes, const std::vector<VkBufferUsageFlags> &a_usages,
                                        VkMemoryPropertyFlags a_memProps, VkMemoryAllocateFlags flags) override;

    std::vector<VkBuffer> CreateBuffers(const std::vector<void*> &a_data, const std::vector<VkDeviceSize> &a_sizes,
                                        const std::vector<VkBufferUsageFlags> &a_usages,
                                        VkMemoryPropertyFlags a_memProps, VkMemoryAllocateFlags flags) override;

    void* MapBufferToHostMemory(VkBuffer a_buf, VkDeviceSize a_offset, VkDeviceSize a_size) override;
    void UnmapBuffer(VkBuffer a_buf) override;

    VkImage CreateImage(const VkImageCreateInfo& a_createInfo) override;
    VkImage CreateImage(uint32_t a_width, uint32_t a_height, VkFormat a_format, VkImageUsageFlags a_usage, uint32_t a_mipLvls) override;
    VkImage CreateImage(const void* a_data, uint32_t a_width, uint32_t a_height, VkFormat a_format, VkImageUsageFlags a_usage,
                        VkImageLayout a_finalLayout, uint32_t a_mipLvls) override;

    std::vector<VkImage> CreateImages(const std::vector<VkImageCreateInfo>& a_createInfos) override;

    VulkanTexture CreateTexture(const VkImageCreateInfo& a_createInfo, VkImageViewCreateInfo& a_imgViewCreateInfo) override;
    VulkanTexture CreateTexture(const VkImageCreateInfo& a_createInfo, VkImageViewCreateInfo& a_imgViewCreateInfo,
                                const VkSamplerCreateInfo& a_samplerCreateInfo) override;

    std::vector<VulkanTexture> CreateTextures(const std::vector<VkImageCreateInfo>& a_createInfos,
                                              std::vector<VkImageViewCreateInfo>& a_imgViewCreateInfos) override;
    std::vector<VulkanTexture> CreateTextures(const std::vector<VkImageCreateInfo>& a_createInfos,
                                              std::vector<VkImageViewCreateInfo>& a_imgViewCreateInfos,
                                              const std::vector<VkSamplerCreateInfo>& a_samplerCreateInfos) override;

    VkSampler CreateSampler(const VkSamplerCreateInfo& a_samplerCreateInfo) override;

    void DestroyBuffer(VkBuffer &a_buffer) override;
    void DestroyImage(VkImage &a_image) override;
    void DestroyTexture(VulkanTexture &a_texture) override;
    void DestroySampler(VkSampler &a_sampler) override;

    // create accel struct ?
    // map, unmap

  private:
    VkDevice         m_device         = VK_NULL_HANDLE;
    VkPhysicalDevice m_physicalDevice = VK_NULL_HANDLE;

    std::shared_ptr<IMemoryAlloc> m_pAlloc;
    std::shared_ptr<ICopyEngine>  m_pCopy;
    vk_utils::SamplerPool m_samplerPool;

    std::unordered_map<VkBuffer, uint32_t> m_bufAllocs;
    std::unordered_map<VkImage,  uint32_t> m_imgAllocs;

    std::unordered_map<uint32_t, uint32_t> m_allocRefCount;
    std::unordered_set<uint32_t> m_allocsMapped;
  };

}

#endif //VK_UTILS_RESOURCE_MANAGER_H
