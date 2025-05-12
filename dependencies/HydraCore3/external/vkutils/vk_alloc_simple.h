#ifndef VKUTILS_VK_ALLOC_SIMPLE_H
#define VKUTILS_VK_ALLOC_SIMPLE_H


#include "vk_alloc.h"
#include "external/vk_mem_alloc.h"
#include <vector>

namespace vk_utils
{
  struct MemoryAlloc_Simple : IMemoryAlloc
  {
    MemoryAlloc_Simple(VkDevice a_device, VkPhysicalDevice a_physicalDevice);
    ~MemoryAlloc_Simple() override;

    uint32_t Allocate(const MemAllocInfo& a_allocInfo) override;
    uint32_t Allocate(const MemAllocInfo& a_allocInfoBuffers, const std::vector<VkBuffer> &a_buffers) override;
    uint32_t Allocate(const MemAllocInfo& a_allocInfoImages, const std::vector<VkImage> &a_images) override;

    void Cleanup();
    void Free(uint32_t a_memBlockId) override;
    void FreeAllMemory() override;
    MemoryBlock GetMemoryBlock(uint32_t a_memBlockId) const override;
    void* Map(uint32_t a_memBlockId, VkDeviceSize a_offset, VkDeviceSize a_size) override;
    void Unmap(uint32_t a_memBlockId) override;
    VkDevice GetDevice() const override { return m_device; }
    VkPhysicalDevice GetPhysicalDevice() const override { return m_physicalDevice; }

  private:

    VkDevice m_device = VK_NULL_HANDLE;
    VkPhysicalDevice m_physicalDevice = VK_NULL_HANDLE;
    VkPhysicalDeviceMemoryProperties m_physicalMemoryProps = {};

    uint32_t nextAllocIdx = 0;
    std::unordered_map<uint32_t, MemoryBlock> m_allocations;
  };

  struct MemoryAlloc_Special : IMemoryAlloc
  {
    MemoryAlloc_Special(VkDevice a_device, VkPhysicalDevice a_physicalDevice);
    ~MemoryAlloc_Special() override;

    uint32_t Allocate(const MemAllocInfo& a_allocInfo) override;
    uint32_t Allocate(const MemAllocInfo& a_allocInfoBuffers, const std::vector<VkBuffer> &a_buffers) override;
    uint32_t Allocate(const MemAllocInfo& a_allocInfoImages, const std::vector<VkImage> &a_images) override;

    void Cleanup();
    void Free(uint32_t a_memBlockId) override;
    void FreeAllMemory() override;
    MemoryBlock GetMemoryBlock(uint32_t a_memBlockId) const override;
    void* Map(uint32_t a_memBlockId, VkDeviceSize a_offset, VkDeviceSize a_size) override;
    void Unmap(uint32_t a_memBlockId) override;
    VkDevice GetDevice() const override { return m_device; }
    VkPhysicalDevice GetPhysicalDevice() const override { return m_physicalDevice; }

  private:
    static constexpr uint8_t BUF_ALLOC_ID = 0;
    static constexpr uint8_t IMG_ALLOC_ID = 1;

    MemoryBlock AllocateInternal(const MemAllocInfo& a_allocInfo);
    uint32_t AllocateHidden(const MemAllocInfo& a_allocInfoBuffers, const std::vector<VkBuffer> &a_buffers, size_t a_offset = 0, size_t* a_pAllocatedSize = nullptr);

    VkDevice m_device = VK_NULL_HANDLE;
    VkPhysicalDevice m_physicalDevice = VK_NULL_HANDLE;
    VkPhysicalDeviceMemoryProperties m_physicalMemoryProps = {};

    MemoryBlock m_bufAlloc = {};
    MemoryBlock m_imgAlloc = {};
  };
}

#endif// VKUTILS_VK_ALLOC_SIMPLE_H
