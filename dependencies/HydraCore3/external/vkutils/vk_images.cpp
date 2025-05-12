#include "vk_images.h"
#include "vk_utils.h"
#include "vk_buffers.h"

#include <array>
#include <algorithm>


namespace vk_utils
{

  VkImageCreateInfo defaultImageCreateInfo(uint32_t a_width, uint32_t a_height, VkFormat a_format, VkImageUsageFlags a_usage, uint32_t a_mipLvls)
  {
    VkImageCreateInfo imageInfo{};
    imageInfo.sType         = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    imageInfo.imageType     = VK_IMAGE_TYPE_2D;
    imageInfo.format        = a_format;
    imageInfo.extent.width  = a_width;
    imageInfo.extent.height = a_height;
    imageInfo.extent.depth  = 1;
    imageInfo.mipLevels     = a_mipLvls;
    imageInfo.arrayLayers   = 1;
    imageInfo.samples       = VK_SAMPLE_COUNT_1_BIT;
    imageInfo.tiling        = VK_IMAGE_TILING_OPTIMAL;
    imageInfo.usage         = a_usage;
    imageInfo.sharingMode   = VK_SHARING_MODE_EXCLUSIVE;
    imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

    return imageInfo;
  }

  VkImageViewCreateInfo defaultImageViewCreateInfo(VkImage a_image, VkFormat a_format, uint32_t a_mipLvls, VkImageAspectFlags a_aspect)
  {
    VkImageViewCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    info.viewType = VK_IMAGE_VIEW_TYPE_2D;
    info.format = a_format;
    info.subresourceRange = {};
    info.subresourceRange.aspectMask = a_aspect;
    info.subresourceRange.baseMipLevel = 0;
    info.subresourceRange.levelCount = a_mipLvls;
    info.subresourceRange.baseArrayLayer = 0;
    info.subresourceRange.layerCount = 1;
    info.image = a_image;

    return info;
  }


  VkImage createVkImage(VkDevice a_device, uint32_t a_width, uint32_t a_height, VkFormat a_format, VkImageUsageFlags a_usage, uint32_t a_mipLvls)
  {
    VkImageCreateInfo imageInfo = defaultImageCreateInfo(a_width, a_height, a_format, a_usage, a_mipLvls);

    VkImage image;
    VK_CHECK_RESULT(vkCreateImage(a_device, &imageInfo, nullptr, &image));

    return image;
  }

  VulkanImageMem createImg(VkDevice a_device, uint32_t a_width, uint32_t a_height, VkFormat a_format, VkImageUsageFlags a_usage,
    VkImageAspectFlags a_aspectFlags, uint32_t a_mipLvls)
  {
    VulkanImageMem result = {};
    result.format     = a_format;
    result.aspectMask = a_aspectFlags;
    result.mipLvls    = a_mipLvls;

    VkImageCreateInfo imageInfo = defaultImageCreateInfo(a_width, a_height, result.format, a_usage, result.mipLvls);

    VK_CHECK_RESULT(vkCreateImage(a_device, &imageInfo, nullptr, &result.image));
    vkGetImageMemoryRequirements(a_device, result.image, &result.memReq);

    return result;
  }

  void deleteImg(VkDevice a_device, VulkanImageMem *a_pImgMem)
  {
    if(a_pImgMem->view != VK_NULL_HANDLE)
    {
      vkDestroyImageView(a_device, a_pImgMem->view, nullptr);
      a_pImgMem->view = VK_NULL_HANDLE;
    }

    if(a_pImgMem->image != VK_NULL_HANDLE)
    {
      vkDestroyImage(a_device, a_pImgMem->image, nullptr);
      a_pImgMem->image = VK_NULL_HANDLE;
    }

    if(a_pImgMem->mem != VK_NULL_HANDLE)
    {
      vkFreeMemory(a_device, a_pImgMem->mem, nullptr);
      a_pImgMem->mem = VK_NULL_HANDLE;
    }
  }

  VkDeviceMemory allocateImgsBindCreateView(VkDevice a_device, VkPhysicalDevice a_physDevice, std::vector<VulkanImageMem> &a_images)
  {
    std::vector<VkMemoryRequirements> memInfos(a_images.size());
    for(size_t i = 0; i < memInfos.size(); ++i)
    {
      memInfos[i] = a_images[i].memReq;
    }

    for(size_t i = 1; i < memInfos.size(); i++)
    {
      if(memInfos[i].memoryTypeBits != memInfos[0].memoryTypeBits)
      {
        VK_UTILS_LOG_WARNING("[allocateAndBindWithPadding]: input buffers have different memReq.memoryTypeBits");
        return VK_NULL_HANDLE;
      }
    }

    auto offsets  = vk_utils::calculateMemOffsets(memInfos);
    auto memTotal = offsets[offsets.size() - 1];

    VkDeviceMemory res;
    VkMemoryAllocateInfo allocateInfo = {};
    allocateInfo.sType           = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    allocateInfo.pNext           = nullptr;
    allocateInfo.allocationSize  = memTotal;
    allocateInfo.memoryTypeIndex = vk_utils::findMemoryType(memInfos[0].memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, a_physDevice);

//    VkMemoryAllocateFlagsInfo memoryAllocateFlagsInfo{};
//    if(flags)
//    {
//      memoryAllocateFlagsInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_FLAGS_INFO;
//      memoryAllocateFlagsInfo.flags = flags;
//
//      allocateInfo.pNext = &memoryAllocateFlagsInfo;
//    }

    VK_CHECK_RESULT(vkAllocateMemory(a_device, &allocateInfo, NULL, &res));

    for (size_t i = 0; i < a_images.size(); i++)
    {
      a_images[i].mem = res;
      a_images[i].mem_offset = offsets[i];
      createImageViewAndBindMem(a_device, &a_images[i]);
    }

    return res;
  }

  VkImageView createImageViewAndBindMem(VkDevice a_device, VulkanImageMem *a_pImgMem, const VkImageViewCreateInfo *a_pViewCreateInfo)
  {
    VK_CHECK_RESULT(vkBindImageMemory(a_device, a_pImgMem->image, a_pImgMem->mem, a_pImgMem->mem_offset));

    VkImageViewCreateInfo imageView{};
    if(a_pViewCreateInfo != nullptr)
    {
      imageView = *a_pViewCreateInfo;
      imageView.image = a_pImgMem->image;
    }
    else
    {
      imageView = defaultImageViewCreateInfo(a_pImgMem->image, a_pImgMem->format, a_pImgMem->mipLvls, a_pImgMem->aspectMask);
    }

    VK_CHECK_RESULT(vkCreateImageView(a_device, &imageView, nullptr, &a_pImgMem->view));
    return a_pImgMem->view;
  }

  VkImageView createVkImageView(VkDevice a_device, VkImage a_image, VkFormat a_format, uint32_t a_mipLvls, VkImageAspectFlags a_aspect)
  {
    VkImageViewCreateInfo info = defaultImageViewCreateInfo(a_image, a_format, a_mipLvls, a_aspect);

    VkImageView view;
    VK_CHECK_RESULT(vkCreateImageView(a_device, &info, nullptr, &view));

    return view;
  }

  VulkanImageMem allocateColorTextureFromDataLDR(VkDevice a_device, VkPhysicalDevice a_physDevice, const unsigned char *pixels,
                                                 uint32_t w, uint32_t h, uint32_t a_mipLevels, VkFormat a_format,
                                                 std::shared_ptr<vk_utils::ICopyEngine> a_pCopy, VkImageUsageFlags a_usageFlags)
  {
    VulkanImageMem result = {};

    result.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    result.format     = a_format;
    result.mipLvls    = a_mipLevels;

    VkImageCreateInfo imageInfo = defaultImageCreateInfo(w, h, result.format, a_usageFlags | VK_IMAGE_USAGE_SAMPLED_BIT, result.mipLvls);

    VK_CHECK_RESULT(vkCreateImage(a_device, &imageInfo, nullptr, &result.image));
    vkGetImageMemoryRequirements(a_device, result.image, &result.memReq);

    VkMemoryAllocateInfo memAlloc{};
    memAlloc.sType           = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    memAlloc.allocationSize  = result.memReq.size;
    memAlloc.memoryTypeIndex = findMemoryType(result.memReq.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, a_physDevice);
    VK_CHECK_RESULT(vkAllocateMemory(a_device, &memAlloc, nullptr, &result.mem));

    createImageViewAndBindMem(a_device, &result);

    a_pCopy->UpdateImage(result.image, pixels, w, h, 4, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

    return result;
  }

  void createImgAllocAndBind(VkDevice a_device, VkPhysicalDevice a_physicalDevice,
                             uint32_t a_width, uint32_t a_height, VkFormat a_format,  VkImageUsageFlags a_usage,
                             VulkanImageMem *a_pImgMem,
                             const VkImageCreateInfo *a_pImageCreateInfo, const VkImageViewCreateInfo *a_pViewCreateInfo)
  {

    a_pImgMem->format = a_format;
    // aspect mask ?
    VkImageCreateInfo image{};
    if(a_pImageCreateInfo != nullptr)
    {
      image = *a_pImageCreateInfo;
    }
    else
    {
      a_pImgMem->mipLvls = 1;
      image= defaultImageCreateInfo(a_width, a_height, a_format, a_usage | VK_IMAGE_USAGE_SAMPLED_BIT, a_pImgMem->mipLvls);
    }

    VK_CHECK_RESULT(vkCreateImage(a_device, &image, nullptr, &a_pImgMem->image));
    vkGetImageMemoryRequirements(a_device, a_pImgMem->image, &a_pImgMem->memReq);

    VkMemoryAllocateInfo memAlloc{};
    memAlloc.sType           = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    memAlloc.allocationSize  = a_pImgMem->memReq.size;
    memAlloc.memoryTypeIndex = findMemoryType(a_pImgMem->memReq.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, a_physicalDevice);
    VK_CHECK_RESULT(vkAllocateMemory(a_device, &memAlloc, nullptr, &a_pImgMem->mem));

    createImageViewAndBindMem(a_device, a_pImgMem, a_pViewCreateInfo);
  }

  VkBool32 getSupportedDepthFormat(VkPhysicalDevice physicalDevice, const std::vector<VkFormat> &depthFormats, VkFormat *depthFormat)
  {
    for (auto &format : depthFormats)
    {
      VkFormatProperties formatProps;
      vkGetPhysicalDeviceFormatProperties(physicalDevice, format, &formatProps);

      // Format must support depth stencil attachment for optimal tiling
      if (formatProps.optimalTilingFeatures & VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT)
      {
        *depthFormat = format;
        return true;
      }
    }

    return false;
  }

  bool isDepthFormat(VkFormat a_format)
  {
    static constexpr std::array<VkFormat, 6> depth_formats =
        {
            VK_FORMAT_D16_UNORM,
            VK_FORMAT_X8_D24_UNORM_PACK32,
            VK_FORMAT_D32_SFLOAT,
            VK_FORMAT_D16_UNORM_S8_UINT,
            VK_FORMAT_D24_UNORM_S8_UINT,
            VK_FORMAT_D32_SFLOAT_S8_UINT,
        };

    return std::find(depth_formats.begin(), depth_formats.end(), a_format) != std::end(depth_formats);
  }

  bool isStencilFormat(VkFormat a_format)
  {
    static constexpr std::array<VkFormat, 4> stencil_formats =
        {
            VK_FORMAT_S8_UINT,
            VK_FORMAT_D16_UNORM_S8_UINT,
            VK_FORMAT_D24_UNORM_S8_UINT,
            VK_FORMAT_D32_SFLOAT_S8_UINT,
        };
    return std::find(stencil_formats.begin(), stencil_formats.end(), a_format) != std::end(stencil_formats);
  }

  bool isDepthOrStencil(VkFormat a_format)
  {
    return(isDepthFormat(a_format) || isStencilFormat(a_format));
  }

  VulkanImageMem createDepthTexture(VkDevice a_device, VkPhysicalDevice a_physDevice,
    const uint32_t a_width, const uint32_t a_height, VkFormat a_format)
  {
    VulkanImageMem result = {};
    result.format = a_format;

    VkImageCreateInfo imgCreateInfo = defaultImageCreateInfo(a_width, a_height, a_format, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT, 1);
    VkImageViewCreateInfo imageViewInfo = defaultImageViewCreateInfo(VK_NULL_HANDLE, a_format, 1, VK_IMAGE_ASPECT_DEPTH_BIT);

    createImgAllocAndBind(a_device, a_physDevice, a_width, a_height, a_format, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT,
                          &result,
                          &imgCreateInfo, &imageViewInfo);

    return result;
  }


  void setImageLayout(VkCommandBuffer cmdBuffer,
                      VkImage image,
                      VkImageLayout oldImageLayout,
                      VkImageLayout newImageLayout,
                      VkImageSubresourceRange subresourceRange,
                      VkPipelineStageFlags srcStageMask,
                      VkPipelineStageFlags dstStageMask)
  {
    VkImageMemoryBarrier imageMemoryBarrier = {};
    imageMemoryBarrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    imageMemoryBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    imageMemoryBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    imageMemoryBarrier.oldLayout = oldImageLayout;
    imageMemoryBarrier.newLayout = newImageLayout;
    imageMemoryBarrier.image = image;
    imageMemoryBarrier.subresourceRange = subresourceRange;

    // Source layouts (old)
    // Source access mask controls actions that have to be finished on the old layout
    // before it will be transitioned to the new layout
    switch (oldImageLayout)
    {
      case VK_IMAGE_LAYOUT_UNDEFINED:
        // Image layout is undefined (or does not matter)
        // Only valid as initial layout
        // No flags required, listed only for completeness
        imageMemoryBarrier.srcAccessMask = 0;
        break;

      case VK_IMAGE_LAYOUT_PREINITIALIZED:
        // Image is pre-initialized
        // Only valid as initial layout for linear images, preserves memory contents
        // Make sure host writes have been finished
        imageMemoryBarrier.srcAccessMask = VK_ACCESS_HOST_WRITE_BIT;
        break;

      case VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL:
        // Image is a color attachment
        // Make sure any writes to the color buffer have been finished
        imageMemoryBarrier.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        break;

      case VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL:
        // Image is a depth/stencil attachment
        // Make sure any writes to the depth/stencil buffer have been finished
        imageMemoryBarrier.srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
        break;

      case VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL:
        // Image is a transfer source
        // Make sure any reads from the image have been finished
        imageMemoryBarrier.srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
        break;

      case VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL:
        // Image is a transfer destination
        // Make sure any writes to the image have been finished
        imageMemoryBarrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        break;

      case VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL:
        // Image is read by a shader
        // Make sure any shader reads from the image have been finished
        imageMemoryBarrier.srcAccessMask = VK_ACCESS_SHADER_READ_BIT;
        break;
      default:
        // Other source layouts aren't handled (yet)
        break;
    }

    // Target layouts
    // Destination access mask controls the dependency for the new image layout
    switch (newImageLayout)
    {
      case VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL:
        // Image will be used as a transfer destination
        // Make sure any writes to the image have been finished
        imageMemoryBarrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        break;

      case VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL:
        // Image will be used as a transfer source
        // Make sure any reads from the image have been finished
        imageMemoryBarrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
        break;

      case VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL:
        // Image will be used as a color attachment
        // Make sure any writes to the color buffer have been finished
        imageMemoryBarrier.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        break;

      case VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL:
        // Image layout will be used as a depth/stencil attachment
        // Make sure any writes to depth/stencil buffer have been finished
        imageMemoryBarrier.dstAccessMask = imageMemoryBarrier.dstAccessMask | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
        break;

      case VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL:
        // Image will be read in a shader (sampler, input attachment)
        // Make sure any writes to the image have been finished
        if (imageMemoryBarrier.srcAccessMask == 0)
        {
          imageMemoryBarrier.srcAccessMask = VK_ACCESS_HOST_WRITE_BIT | VK_ACCESS_TRANSFER_WRITE_BIT;
        }
        imageMemoryBarrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        break;
      default:
        // Other source layouts aren't handled (yet)
        break;
    }

    vkCmdPipelineBarrier(
        cmdBuffer,
        srcStageMask,
        dstStageMask,
        0,
        0, nullptr,
        0, nullptr,
        1, &imageMemoryBarrier);
  }

  // Fixed sub resource on first mip level and layer
  void setImageLayout(
      VkCommandBuffer cmdBuffer,
      VkImage image,
      VkImageAspectFlags aspectMask,
      VkImageLayout oldImageLayout,
      VkImageLayout newImageLayout,
      VkPipelineStageFlags srcStageMask,
      VkPipelineStageFlags dstStageMask)
  {
    VkImageSubresourceRange subresourceRange = {};
    subresourceRange.aspectMask = aspectMask;
    subresourceRange.baseMipLevel = 0;
    subresourceRange.levelCount = 1;
    subresourceRange.layerCount = 1;
    setImageLayout(cmdBuffer, image, oldImageLayout, newImageLayout, subresourceRange, srcStageMask, dstStageMask);
  }

  void insertImageMemoryBarrier(
      VkCommandBuffer cmdBuffer,
      VkImage image,
      VkAccessFlags srcAccessMask,
      VkAccessFlags dstAccessMask,
      VkImageLayout oldImageLayout,
      VkImageLayout newImageLayout,
      VkPipelineStageFlags srcStageMask,
      VkPipelineStageFlags dstStageMask,
      VkImageSubresourceRange subresourceRange)
  {
    VkImageMemoryBarrier imageMemoryBarrier {};
    imageMemoryBarrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    imageMemoryBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    imageMemoryBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    imageMemoryBarrier.srcAccessMask = srcAccessMask;
    imageMemoryBarrier.dstAccessMask = dstAccessMask;
    imageMemoryBarrier.oldLayout = oldImageLayout;
    imageMemoryBarrier.newLayout = newImageLayout;
    imageMemoryBarrier.image = image;
    imageMemoryBarrier.subresourceRange = subresourceRange;

    vkCmdPipelineBarrier(
        cmdBuffer,
        srcStageMask,
        dstStageMask,
        0,
        0, nullptr,
        0, nullptr,
        1, &imageMemoryBarrier);
  }

  VkSamplerCreateInfo defaultSamplerCreateInfo(VkFilter a_filterMode, VkSamplerAddressMode a_addressMode, VkBorderColor a_border_color,
    uint32_t a_mipLevels)
  {
    VkSamplerCreateInfo samplerCreateInfo = {};
    samplerCreateInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    samplerCreateInfo.maxAnisotropy = 1.0f;
    samplerCreateInfo.magFilter = a_filterMode;
    samplerCreateInfo.minFilter = a_filterMode;
    samplerCreateInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
    samplerCreateInfo.addressModeU = a_addressMode;
    samplerCreateInfo.addressModeV = a_addressMode;
    samplerCreateInfo.addressModeW = a_addressMode;
    samplerCreateInfo.mipLodBias = 0.0f;
    samplerCreateInfo.compareOp = VK_COMPARE_OP_NEVER;
    samplerCreateInfo.minLod = 0.0f;
    samplerCreateInfo.maxLod = 0.0f;
    samplerCreateInfo.borderColor = a_border_color;
    samplerCreateInfo.maxAnisotropy = 1.0;
    samplerCreateInfo.anisotropyEnable = VK_FALSE;
    samplerCreateInfo.maxLod = (float)a_mipLevels;

    return samplerCreateInfo;
  }

  VkSampler createSampler(VkDevice a_device, VkFilter a_filterMode, VkSamplerAddressMode a_addressMode,
                          VkBorderColor a_border_color, uint32_t a_mipLevels)
  {
    VkSampler result;
    VkSamplerCreateInfo samplerCreateInfo = defaultSamplerCreateInfo(a_filterMode, a_addressMode, a_border_color, a_mipLevels);

    VK_CHECK_RESULT(vkCreateSampler(a_device, &samplerCreateInfo, nullptr, &result));
    return result;
  }

  //@TODO: Software mip chain generation with good interpolation filters (can use stb_image_resize.h)

  void generateMipChainCmd(VkCommandBuffer a_cmdBuf, VkImage a_image,
                           uint32_t a_width, uint32_t a_height, uint32_t a_mipLevels, VkImageLayout a_targetLayout)
  {
    VkCommandBufferBeginInfo beginInfo = {};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;

    vkBeginCommandBuffer(a_cmdBuf, &beginInfo);

    auto mip_w = int32_t(a_width);
    auto mip_h = int32_t(a_height);
    for (uint32_t i = 1; i < a_mipLevels; i++)
    {
      VkImageSubresourceRange currSubRange = {};
      currSubRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
      currSubRange.baseMipLevel = i;
      currSubRange.levelCount = 1;
      currSubRange.layerCount = 1;

      // Transition mip level to be generated to transfer dst
      vk_utils::setImageLayout(
        a_cmdBuf,
        a_image,
        VK_IMAGE_LAYOUT_UNDEFINED,
        VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
        currSubRange,
        VK_PIPELINE_STAGE_TRANSFER_BIT,
        VK_PIPELINE_STAGE_TRANSFER_BIT);

      VkImageBlit imageBlit{};
      imageBlit.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
      imageBlit.srcSubresource.layerCount = 1;
      imageBlit.srcSubresource.mipLevel = i - 1;
      imageBlit.srcOffsets[1].x = mip_w;
      imageBlit.srcOffsets[1].y = mip_h;
      imageBlit.srcOffsets[1].z = 1;

      imageBlit.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
      imageBlit.dstSubresource.layerCount = 1;
      imageBlit.dstSubresource.mipLevel = i;
      imageBlit.dstOffsets[1].x = mip_w > 1 ? mip_w / 2 : 1;
      imageBlit.dstOffsets[1].y = mip_h > 1 ? mip_h / 2 : 1;
      imageBlit.dstOffsets[1].z = 1;

      // Blit from upper (i - 1) level to lower (i) level
      vkCmdBlitImage(
          a_cmdBuf,
          a_image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
          a_image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
          1, &imageBlit,
          VK_FILTER_LINEAR);

//       Transition generated mip level to transfer source for read in next iteration
      vk_utils::setImageLayout(
          a_cmdBuf,
          a_image,
        VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
        VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
        currSubRange,
          VK_PIPELINE_STAGE_TRANSFER_BIT,
        VK_PIPELINE_STAGE_TRANSFER_BIT);

      if(mip_w > 1) mip_w /= 2;
      if(mip_h > 1) mip_h /= 2;
    }

    for(uint32_t i = 0; i < a_mipLevels; i++)
    {
      VkImageSubresourceRange subresourceRange = {};
      subresourceRange.baseMipLevel            = i;
      subresourceRange.aspectMask              = VK_IMAGE_ASPECT_COLOR_BIT;
      subresourceRange.levelCount              = 1;
      subresourceRange.layerCount              = 1;
      vk_utils::setImageLayout(
        a_cmdBuf,
        a_image,
        VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
        a_targetLayout,
        subresourceRange);
    }

    vkEndCommandBuffer(a_cmdBuf);
  }

  // bytes per pixel from commonly supported image formats
  uint8_t bppFromVkFormat(VkFormat a_format)
  {
    switch(a_format)
    {
      case VK_FORMAT_R8_SINT:
      case VK_FORMAT_R8_SNORM:
      case VK_FORMAT_R8_UINT:
      case VK_FORMAT_R8_UNORM:
      case VK_FORMAT_R8_SRGB:
        return 1;

      case VK_FORMAT_R8G8_SINT:
      case VK_FORMAT_R8G8_SNORM:
      case VK_FORMAT_R8G8_UINT:
      case VK_FORMAT_R8G8_UNORM:
      case VK_FORMAT_R16_SFLOAT:
      case VK_FORMAT_R16_SINT:
      case VK_FORMAT_R16_UINT:
      case VK_FORMAT_D16_UNORM:
      case VK_FORMAT_R16_SNORM:
      case VK_FORMAT_R16_UNORM:
      case VK_FORMAT_A1R5G5B5_UNORM_PACK16:
      case VK_FORMAT_R5G6B5_UNORM_PACK16:
      case VK_FORMAT_B4G4R4A4_UNORM_PACK16:
      case VK_FORMAT_R4G4B4A4_UNORM_PACK16:
      case VK_FORMAT_R5G5B5A1_UNORM_PACK16:
      case VK_FORMAT_B5G5R5A1_UNORM_PACK16:
        return 2;

      case VK_FORMAT_B8G8R8A8_SRGB:
      case VK_FORMAT_B8G8R8A8_UNORM:
      case VK_FORMAT_R8G8B8A8_SINT:
      case VK_FORMAT_R8G8B8A8_SNORM:
      case VK_FORMAT_R8G8B8A8_SRGB:
      case VK_FORMAT_R8G8B8A8_UINT:
      case VK_FORMAT_R8G8B8A8_UNORM:
      case VK_FORMAT_D32_SFLOAT:
      case VK_FORMAT_R16G16_SFLOAT:
      case VK_FORMAT_R16G16_SINT:
      case VK_FORMAT_R16G16_UINT:
      case VK_FORMAT_R32_SFLOAT:
      case VK_FORMAT_R32_SINT:
      case VK_FORMAT_R32_UINT:
      case VK_FORMAT_R16G16_SNORM:
      case VK_FORMAT_R16G16_UNORM:
      case VK_FORMAT_A2B10G10R10_UINT_PACK32:
      case VK_FORMAT_A2B10G10R10_UNORM_PACK32:
      case VK_FORMAT_A8B8G8R8_SINT_PACK32:
      case VK_FORMAT_A8B8G8R8_SNORM_PACK32:
      case VK_FORMAT_A8B8G8R8_SRGB_PACK32:
      case VK_FORMAT_A8B8G8R8_UINT_PACK32:
      case VK_FORMAT_A8B8G8R8_UNORM_PACK32:
      case VK_FORMAT_B10G11R11_UFLOAT_PACK32:
      case VK_FORMAT_E5B9G9R9_UFLOAT_PACK32:
      case VK_FORMAT_A2R10G10B10_UINT_PACK32:
      case VK_FORMAT_A2R10G10B10_UNORM_PACK32:
        return 4;

      case VK_FORMAT_R16G16B16A16_SFLOAT:
      case VK_FORMAT_R16G16B16A16_SINT:
      case VK_FORMAT_R16G16B16A16_UINT:
      case VK_FORMAT_R32G32_SFLOAT:
      case VK_FORMAT_R32G32_SINT:
      case VK_FORMAT_R32G32_UINT:
      case VK_FORMAT_R16G16B16A16_SNORM:
      case VK_FORMAT_R16G16B16A16_UNORM:
        return 8;

      case VK_FORMAT_R32G32B32A32_SFLOAT:
      case VK_FORMAT_R32G32B32A32_SINT:
      case VK_FORMAT_R32G32B32A32_UINT:
        return 16;

      default:
        VK_UTILS_LOG_WARNING("[bppFromVkFormat]: unknown format");
        return 0;
    }
  }

  VkDeviceMemory allocateAndBindWithPadding(VkDevice a_dev, VkPhysicalDevice a_physDev, const std::vector<VkBuffer> &a_buffers, std::vector<VulkanImageMem>& a_images, VkMemoryAllocateFlags flags)
  {
    if(a_buffers.empty() && a_images.empty())
    {
      logWarning("[allocateAndBindWithPadding]: both buffers and images vector is empty");
      return VK_NULL_HANDLE;
    }

    VkPhysicalDeviceProperties props;
    vkGetPhysicalDeviceProperties(a_physDev, &props);

    std::vector<VkMemoryRequirements> memInfos(a_buffers.size() + a_images.size());

    for(size_t i = 0; i < a_buffers.size(); ++i)
    {
      if(a_buffers[i] != VK_NULL_HANDLE)
        vkGetBufferMemoryRequirements(a_dev, a_buffers[i], &memInfos[i]);
      else
      {
        memInfos[i] = memInfos[0];
        memInfos[i].size = 0;
      }
    }

    for(size_t i = 0; i < a_images.size(); ++i)
    {
      size_t j = a_buffers.size() + i;
      if(a_images[i].image != VK_NULL_HANDLE)
        memInfos[j] = a_images[i].memReq;
      else
      {
        memInfos[j] = memInfos[0];
        memInfos[j].size = 0;
      }
    }

    for(size_t i=1;i<memInfos.size();i++)
    {
      if(memInfos[i].memoryTypeBits != memInfos[0].memoryTypeBits)
      {
        //logWarning("[allocateAndBindWithPadding]: input objects(buffer or texture) has different memReq.memoryTypeBits");
        return VK_NULL_HANDLE;
      }
    }

    auto offsets  = calculateMemOffsets(memInfos, props.limits.bufferImageGranularity);
    auto memTotal = offsets[offsets.size() - 1];

    VkDeviceMemory res;
    VkMemoryAllocateInfo allocateInfo = {};
    allocateInfo.sType           = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    allocateInfo.pNext           = nullptr;
    allocateInfo.allocationSize  = memTotal;
    allocateInfo.memoryTypeIndex = vk_utils::findMemoryType(memInfos[0].memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, a_physDev);

    VkMemoryAllocateFlagsInfo memoryAllocateFlagsInfo{};
    if(flags)
    {
      memoryAllocateFlagsInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_FLAGS_INFO;
      memoryAllocateFlagsInfo.flags = flags;

      allocateInfo.pNext = &memoryAllocateFlagsInfo;
    }

    VK_CHECK_RESULT(vkAllocateMemory(a_dev, &allocateInfo, NULL, &res));

    for (size_t i = 0; i < a_buffers.size(); i++)
    {
      if(a_buffers[i] != VK_NULL_HANDLE)
        vkBindBufferMemory(a_dev, a_buffers[i], res, offsets[i]);
    }

    for(size_t i = 0; i < a_images.size(); ++i)
    {
      size_t j = a_buffers.size() + i;
      a_images[i].mem        = res;
      a_images[i].mem_offset = offsets[j];
      a_images[i].memReq     = memInfos[j];
      vk_utils::createImageViewAndBindMem(a_dev, &a_images[i]);
    }

    return res;
  }
}