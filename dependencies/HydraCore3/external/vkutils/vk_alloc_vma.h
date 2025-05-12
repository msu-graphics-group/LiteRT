#ifndef VKUTILS_VK_ALLOC_VMA_H
#define VKUTILS_VK_ALLOC_VMA_H

#include "vk_alloc.h"
#include "external/vk_mem_alloc.h"
#include "vk_resource_manager.h"
#include <vector>
#include <unordered_map>

namespace vk_utils
{
  VmaAllocator initVMA(VkInstance a_instance, VkDevice a_device, VkPhysicalDevice a_physicalDevice,
    VkFlags a_flags = 0, uint32_t a_vkAPIVersion = VK_API_VERSION_1_1);

  struct MemoryAlloc_VMA : IMemoryAlloc
  {
    MemoryAlloc_VMA(VkDevice a_device, VkPhysicalDevice a_physicalDevice, VmaAllocator a_allocator);

    MemoryAlloc_VMA(VkInstance a_instance, VkDevice a_device, VkPhysicalDevice a_physicalDevice,
                    VkFlags a_flags = 0, uint32_t a_vkAPIVersion = VK_API_VERSION_1_1);

    ~MemoryAlloc_VMA() override;

    void Cleanup();

    uint32_t Allocate(const MemAllocInfo &a_allocInfo) override;

    uint32_t Allocate(const MemAllocInfo &a_allocInfoBuffers, const std::vector<VkBuffer> &a_buffers) override;

    uint32_t Allocate(const MemAllocInfo &a_allocInfoImages, const std::vector<VkImage> &a_images) override;

    void Free(uint32_t a_memBlockId) override;

    void FreeAllMemory() override;

    MemoryBlock GetMemoryBlock(uint32_t a_memBlockId) const override;

    void *Map(uint32_t a_memBlockId, VkDeviceSize a_offset, VkDeviceSize a_size) override;

    void Unmap(uint32_t a_memBlockId) override;

    VkDevice GetDevice() const override { return m_device; }

    VkPhysicalDevice GetPhysicalDevice() const override { return m_physicalDevice; }

    // not part of IMemoryAlloc interface
    //
    VkBuffer AllocateBuffer(const VkBufferCreateInfo &a_bufCreateInfo, VkMemoryPropertyFlags a_memProps);

    VkImage AllocateImage(const VkImageCreateInfo &a_imgCreateInfo, VkMemoryPropertyFlags a_memProps);

    VmaAllocator GetVMA();
    void SetDestroyVMA(bool doDestroy) { m_destroyVma = doDestroy; }

  private:
    bool m_destroyVma = false;
    VkDevice m_device = VK_NULL_HANDLE;
    VkPhysicalDevice m_physicalDevice = VK_NULL_HANDLE;

    VmaAllocator m_vma = VK_NULL_HANDLE;

    uint32_t nextAllocIdx = 0;
    std::unordered_map<uint32_t, VmaAllocation> m_allocations;
  };

  struct ResourceManager_VMA : IResourceManager
  {
    ResourceManager_VMA(VkDevice a_device, VkPhysicalDevice a_physicalDevice, VmaAllocator a_allocator,
                        std::shared_ptr<ICopyEngine> a_pCopy);

    ResourceManager_VMA(ResourceManager_VMA const &) = delete;

    ResourceManager_VMA &operator=(ResourceManager_VMA const &) = delete;

    virtual ~ResourceManager_VMA();

    void Cleanup();

    void SetCopyEngine(std::shared_ptr<ICopyEngine> a_pCopy) { m_pCopy = a_pCopy; }

    std::shared_ptr<ICopyEngine>  GetCopyEngine() override {return m_pCopy; }

    VkBuffer CreateBuffer(VkDeviceSize a_size, VkBufferUsageFlags a_usage, VkMemoryPropertyFlags a_memProps,
                          VkMemoryAllocateFlags flags) override;

    VkBuffer CreateBuffer(const void *a_data, VkDeviceSize a_size, VkBufferUsageFlags a_usage, VkMemoryPropertyFlags a_memProps,
                          VkMemoryAllocateFlags flags) override;

    std::vector<VkBuffer> CreateBuffers(const std::vector<VkDeviceSize> &a_sizes, const std::vector<VkBufferUsageFlags> &a_usages,
                                        VkMemoryPropertyFlags a_memProps, VkMemoryAllocateFlags flags) override;

    std::vector<VkBuffer> CreateBuffers(const std::vector<void *> &a_data, const std::vector<VkDeviceSize> &a_sizes,
                                        const std::vector<VkBufferUsageFlags> &a_usages,
                                        VkMemoryPropertyFlags a_memProps, VkMemoryAllocateFlags flags) override;

    void* MapBufferToHostMemory(VkBuffer a_buf, VkDeviceSize a_offset, VkDeviceSize a_size) override;
    void UnmapBuffer(VkBuffer a_buf) override;

    VkImage CreateImage(const VkImageCreateInfo &a_createInfo) override;

    VkImage CreateImage(uint32_t a_width, uint32_t a_height, VkFormat a_format, VkImageUsageFlags a_usage,
                        uint32_t a_mipLvls) override;

    VkImage CreateImage(const void *a_data, uint32_t a_width, uint32_t a_height, VkFormat a_format, VkImageUsageFlags a_usage,
                        VkImageLayout a_finalLayout, uint32_t a_mipLvls) override;

    std::vector<VkImage> CreateImages(const std::vector<VkImageCreateInfo> &a_createInfos) override;

    VulkanTexture CreateTexture(const VkImageCreateInfo &a_createInfo, VkImageViewCreateInfo &a_imgViewCreateInfo) override;

    VulkanTexture CreateTexture(const VkImageCreateInfo &a_createInfo, VkImageViewCreateInfo &a_imgViewCreateInfo,
                                const VkSamplerCreateInfo &a_samplerCreateInfo) override;

    std::vector<VulkanTexture> CreateTextures(const std::vector<VkImageCreateInfo> &a_createInfos,
                                              std::vector<VkImageViewCreateInfo> &a_imgViewCreateInfos) override;

    std::vector<VulkanTexture> CreateTextures(const std::vector<VkImageCreateInfo> &a_createInfos,
                                              std::vector<VkImageViewCreateInfo> &a_imgViewCreateInfos,
                                              const std::vector<VkSamplerCreateInfo> &a_samplerCreateInfos) override;

    VkSampler CreateSampler(const VkSamplerCreateInfo &a_samplerCreateInfo) override;

    // create accel struct ?
    // map, unmap

    void DestroyBuffer(VkBuffer &a_buffer) override;

    void DestroyImage(VkImage &a_image) override;

    void DestroyTexture(VulkanTexture &a_texture) override;

    void DestroySampler(VkSampler &a_sampler) override;

  private:
    VkDevice m_device = VK_NULL_HANDLE;
    VkPhysicalDevice m_physicalDevice = VK_NULL_HANDLE;

    VmaAllocator m_vma = VK_NULL_HANDLE;
    std::shared_ptr<ICopyEngine> m_pCopy;
    vk_utils::SamplerPool m_samplerPool;

    std::unordered_map<VkBuffer, VmaAllocation> m_bufAllocs;
    std::unordered_map<VkImage, VmaAllocation> m_imgAllocs;
  };
}

#endif// VKUTILS_VK_ALLOC_VMA_H
