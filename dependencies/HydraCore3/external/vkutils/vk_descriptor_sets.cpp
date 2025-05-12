#include "vk_descriptor_sets.h"
#include "vk_utils.h"

namespace vk_utils
{
  VkDescriptorSetLayout createDescriptorSetLayout(VkDevice a_device, const DescriptorTypesMap &a_descrTypes,
                                                  VkShaderStageFlags a_stage)
  {
    VkDescriptorSetLayout layout;

    std::vector<VkDescriptorSetLayoutBinding> bindings;
    bindings.reserve(a_descrTypes.size());
    for (auto& [location, descriptor] : a_descrTypes)
    {
      VkDescriptorSetLayoutBinding descriptorSetLayoutBinding = {};
      descriptorSetLayoutBinding.binding                      = location;
      descriptorSetLayoutBinding.descriptorType               = descriptor.first;
      descriptorSetLayoutBinding.descriptorCount              = descriptor.second;
      descriptorSetLayoutBinding.stageFlags                   = a_stage;

      bindings.push_back(descriptorSetLayoutBinding);
    }

    VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCreateInfo = {};
    descriptorSetLayoutCreateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    descriptorSetLayoutCreateInfo.bindingCount = (uint32_t)bindings.size();
    descriptorSetLayoutCreateInfo.pBindings = bindings.data();
    VK_CHECK_RESULT(vkCreateDescriptorSetLayout(a_device, &descriptorSetLayoutCreateInfo, nullptr, &layout));

    return layout;
  }

  VkDescriptorPool createDescriptorPool(VkDevice a_device, const DescriptorTypesVec &a_descrTypes, unsigned a_maxSets,
                                        VkDescriptorPoolCreateFlags a_flags)
  {
    VkDescriptorPool pool;

    std::vector<VkDescriptorPoolSize> poolSizes(a_descrTypes.size());
    for(size_t i = 0; i < a_descrTypes.size(); ++i)
    {
      VkDescriptorPoolSize descriptorPoolSize = {};
      descriptorPoolSize.type = a_descrTypes[i].first;
      descriptorPoolSize.descriptorCount = a_descrTypes[i].second;

      poolSizes[i] = descriptorPoolSize;
    }

    VkDescriptorPoolCreateInfo descriptorPoolCreateInfo = {};
    descriptorPoolCreateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    descriptorPoolCreateInfo.maxSets = a_maxSets;
    descriptorPoolCreateInfo.poolSizeCount = (uint32_t)poolSizes.size();
    descriptorPoolCreateInfo.pPoolSizes = poolSizes.data();
    descriptorPoolCreateInfo.flags = a_flags;

    VK_CHECK_RESULT(vkCreateDescriptorPool(a_device, &descriptorPoolCreateInfo, nullptr, &pool));

    return pool;
  }

  VkDescriptorSet createDescriptorSet(VkDevice a_device, VkDescriptorSetLayout a_pDSLayout, VkDescriptorPool a_pDSPool, const std::vector<VkDescriptorBufferInfo> &bufInfos)
  {
    VkDescriptorSet set;

    VkDescriptorSetAllocateInfo descriptorSetAllocateInfo = {};
    descriptorSetAllocateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    descriptorSetAllocateInfo.descriptorPool = a_pDSPool;
    descriptorSetAllocateInfo.descriptorSetCount = 1;
    descriptorSetAllocateInfo.pSetLayouts = &a_pDSLayout;

    VK_CHECK_RESULT(vkAllocateDescriptorSets(a_device, &descriptorSetAllocateInfo, &set));

    std::vector<VkWriteDescriptorSet> writeSets(bufInfos.size());
    for (uint32_t i = 0; i < bufInfos.size(); ++i)
    {
      VkWriteDescriptorSet writeDescriptorSet = {};
      writeDescriptorSet.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
      writeDescriptorSet.dstSet = set;
      writeDescriptorSet.dstBinding = i;
      writeDescriptorSet.descriptorCount = 1;
      writeDescriptorSet.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
      writeDescriptorSet.pBufferInfo = &bufInfos[i];

      writeSets[i] = writeDescriptorSet;
    }

    vkUpdateDescriptorSets(a_device, (uint32_t)writeSets.size(), writeSets.data(), 0, nullptr);

    return set;
  }


  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  DescriptorMaker::DescriptorMaker(VkDevice a_device, const DescriptorTypesVec &a_maxDescrTypes, uint32_t a_maxSets) : m_device(a_device)
  {
    assert(m_device != VK_NULL_HANDLE);
    m_pool = createDescriptorPool(m_device, a_maxDescrTypes, a_maxSets);
  }

  DescriptorMaker::~DescriptorMaker()
  {
    assert(m_device != VK_NULL_HANDLE);

    for (auto& l : m_layoutDict)
    {
      if(l.second != VK_NULL_HANDLE)
      {
        vkDestroyDescriptorSetLayout(m_device, l.second, nullptr);
      }
    }

    vkDestroyDescriptorPool(m_device, m_pool, nullptr);
  }

  void DescriptorMaker::BindBegin(VkShaderStageFlags a_shaderStage)
  {
    m_currentStageFlags = a_shaderStage;
    m_bindings.clear();
  }

  void DescriptorMaker::BindBuffer(uint32_t a_loc, VkBuffer a_buffer, VkBufferView a_buffView, VkDescriptorType a_bindType)
  {
    DescriptorHandles h{};
    h.buffer = a_buffer;
    h.buffView = a_buffView;
    h.type = a_bindType;

    if (m_bindings.count(a_loc))
      VK_UTILS_LOG_WARNING("[DescriptorMaker::BindBuffer] binding to the same location!");

    m_bindings[a_loc] = h;
  }

  void DescriptorMaker::BindImage(uint32_t a_loc, VkImageView  a_imageView, VkSampler a_sampler, VkDescriptorType a_bindType,
                                  VkImageLayout a_imageLayout)
  {
    DescriptorHandles h{};
    VkDescriptorImageInfo info {a_sampler, a_imageView, a_imageLayout};
    h.imageDescriptor.push_back(info);
    h.type = a_bindType;

    if (m_bindings.count(a_loc))
      VK_UTILS_LOG_WARNING("[DescriptorMaker::BindImage] binding to the same location!");

    m_bindings[a_loc] = h;
  }

  void DescriptorMaker::BindImageArray(uint32_t a_loc, const std::vector<VkImageView> &a_imageView,
    const std::vector<VkSampler> &a_sampler, VkDescriptorType a_bindType, VkImageLayout a_imageLayout)
  {
    if(a_imageView.empty())
    {
      VK_UTILS_LOG_WARNING("[DescriptorMaker::BindImageArray] binding ignored - empty image views array");
      return;
    }

    if(a_bindType == VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER && a_sampler.empty())
    {
      VK_UTILS_LOG_WARNING("[DescriptorMaker::BindImageArray] binding ignored - empty samplers array for combined image sampler");
      return;
    }

    std::vector<VkDescriptorImageInfo> tmp;
    tmp.reserve(a_imageView.size());
    for(size_t i = 0; i < a_imageView.size(); ++i)
    {
      VkDescriptorImageInfo info {VK_NULL_HANDLE, a_imageView[i], a_imageLayout};
      if(!a_sampler.empty())
        info.sampler = a_sampler[i];
      tmp.push_back(info);
    }

    DescriptorHandles h{};
    h.imageDescriptor = std::move(tmp);
    h.type = a_bindType;

    if (m_bindings.count(a_loc))
      VK_UTILS_LOG_WARNING("[DescriptorMaker::BindImageArray] binding to the same location!");

    m_bindings[a_loc] = h;
  }

  void DescriptorMaker::BindImageArray(uint32_t a_loc, const std::vector<VkDescriptorImageInfo> &a_imageDesc,
    VkDescriptorType a_bindType)
  {
    if(a_imageDesc.empty())
    {
      VK_UTILS_LOG_WARNING("[DescriptorMaker::BindImageArray] binding ignored - empty VkDescriptorImageInfo array");
      return;
    }

    DescriptorHandles h{};
    h.type            = a_bindType;
    h.imageDescriptor = a_imageDesc;

    if (m_bindings.count(a_loc))
      VK_UTILS_LOG_WARNING("[DescriptorMaker::BindImageArray] binding to the same location!");

    m_bindings[a_loc] = h;
  }

  void DescriptorMaker::BindAccelStruct(uint32_t a_loc, VkAccelerationStructureKHR a_accStruct, VkDescriptorType a_bindType)
  {
    DescriptorHandles h{};
    h.accelStruct = a_accStruct;
    h.type = a_bindType;

    if (m_bindings.count(a_loc))
      VK_UTILS_LOG_WARNING("[DescriptorMaker::BindAccelStruct] binding to the same location!");

    m_bindings[a_loc] = h;
  }

  void DescriptorMaker::BindEnd(VkDescriptorSet *a_pSet, VkDescriptorSetLayout *a_pLayout)
  {
    VkDescriptorSetLayout layout = VK_NULL_HANDLE;
    VkDescriptorSet set = VK_NULL_HANDLE;
    DescriptorTypesMap descrTypes;

    size_t totalImageInfos = 0;
    size_t totalBufferInfos = 0;
    size_t totalAccStructsInfos = 0;
    for (const auto &[location, handle] : m_bindings)
    {
      uint32_t count = 1;
      switch (handle.type)
      {
      case VK_DESCRIPTOR_TYPE_SAMPLER:
      case VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER:
      case VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE:
      case VK_DESCRIPTOR_TYPE_STORAGE_IMAGE:
      case VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT:
        count = (uint32_t)handle.imageDescriptor.size();
        totalImageInfos += handle.imageDescriptor.size();
        break;
      case VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER: //TODO: test and fix
      case VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER: //TODO: test and fix
      case VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER:
      case VK_DESCRIPTOR_TYPE_STORAGE_BUFFER:
      case VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC:
      case VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC:
        totalBufferInfos++;
        break;
      case VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR:
        totalAccStructsInfos++;
        break;
      //case VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_NV:
      //case VK_DESCRIPTOR_TYPE_INLINE_UNIFORM_BLOCK_EXT: //TODO
      default:
        count = 0;
        break;
      }

      descrTypes[location] = { handle.type, count };
    }

    if(hashingMode == HASHING_MODE::LAYOUTS_ONLY || hashingMode == HASHING_MODE::LAYOUTS_AND_SETS)
    {
      // Check if we already had created such layout
      LayoutKey layout_key = { m_currentStageFlags, descrTypes };
      auto found_layout = m_layoutDict.find(layout_key);
      if (found_layout != m_layoutDict.end())
      {
        layout = found_layout->second;
      }
      else
      {
        layout = createDescriptorSetLayout(m_device, descrTypes, m_currentStageFlags);
        m_layoutDict[layout_key] = layout;
      }
    }
    else
    {
      layout = createDescriptorSetLayout(m_device, descrTypes, m_currentStageFlags);
    }

    SetKey set_key = { layout, m_bindings };
    if(hashingMode == HASHING_MODE::LAYOUTS_AND_SETS)
    {
      // Check if we already had created such set
      auto found_set = m_setDict.find(set_key);
      if (found_set != m_setDict.end())
      {
        set = found_set->second;
        *a_pSet = set;
        *a_pLayout = layout;
        return;
      }
    }

    ///////////////////

    VkDescriptorSetAllocateInfo descriptorSetAllocateInfo = {};
    descriptorSetAllocateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    descriptorSetAllocateInfo.descriptorPool = m_pool;
    descriptorSetAllocateInfo.descriptorSetCount = 1;
    descriptorSetAllocateInfo.pSetLayouts = &layout;

    VK_CHECK_RESULT(vkAllocateDescriptorSets(m_device, &descriptorSetAllocateInfo, &set));

    const size_t descriptorsInSet = descrTypes.size();
    std::vector<VkWriteDescriptorSet> writeSets;
    writeSets.reserve(descriptorsInSet);
    std::vector<VkDescriptorBufferInfo> dBufferInfos(totalBufferInfos);
    std::vector<VkDescriptorImageInfo> dImageInfos(totalImageInfos);
    std::vector<VkWriteDescriptorSetAccelerationStructureKHR> dAccStructInfos(totalAccStructsInfos);

    size_t imgInfoIdx = 0;
    size_t bufInfoIdx = 0;
    size_t accStructInfoIdx = 0;
    for(auto& [location, descriptor] : descrTypes)
    {
      VkWriteDescriptorSet writeDescriptorSet = {};
      writeDescriptorSet.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
      writeDescriptorSet.dstSet = set;
      writeDescriptorSet.dstBinding = location;
      writeDescriptorSet.descriptorCount = descriptor.second;
      writeDescriptorSet.descriptorType = descriptor.first;

      switch (writeDescriptorSet.descriptorType)
      {
      case VK_DESCRIPTOR_TYPE_SAMPLER:
      case VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE:
      case VK_DESCRIPTOR_TYPE_STORAGE_IMAGE:
      case VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT:
      case VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER:
        writeDescriptorSet.pImageInfo = &dImageInfos[imgInfoIdx];
        for(size_t j = 0; j < writeDescriptorSet.descriptorCount; ++j)
          dImageInfos[imgInfoIdx++] = m_bindings[location].imageDescriptor[j];
        break;

      case VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER:
      case VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER:
        writeDescriptorSet.pTexelBufferView = &m_bindings[location].buffView; //TODO: test and fix if needed
        [[fallthrough]];
      case VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER:
      case VK_DESCRIPTOR_TYPE_STORAGE_BUFFER:
      case VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC:
      case VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC:
        dBufferInfos[bufInfoIdx] = {m_bindings[location].buffer, 0, VK_WHOLE_SIZE}; //TODO: buffer range
        writeDescriptorSet.pBufferInfo = &dBufferInfos[bufInfoIdx];
        bufInfoIdx++;
        break;
      case VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR:
        dAccStructInfos[accStructInfoIdx] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET_ACCELERATION_STRUCTURE_KHR,
          VK_NULL_HANDLE,1,&m_bindings[location].accelStruct}; //TODO: support accStruct arrays
        writeDescriptorSet.pNext = &dAccStructInfos[accStructInfoIdx];
        accStructInfoIdx++;
        break;
      case VK_DESCRIPTOR_TYPE_INLINE_UNIFORM_BLOCK_EXT:
        VK_UTILS_LOG_WARNING("[DescriptorMaker::BindEnd] VK_DESCRIPTOR_TYPE_INLINE_UNIFORM_BLOCK_EXT not yet supported");
        break;
        //case VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_NV:
      default:
        break;
      }

      writeSets.push_back(writeDescriptorSet);
    }

    vkUpdateDescriptorSets(m_device, (uint32_t)writeSets.size(), writeSets.data(), 0, nullptr);
    if(hashingMode == HASHING_MODE::LAYOUTS_AND_SETS)
    {
        m_setDict[set_key] = set;
    }

    *a_pSet = set;
    *a_pLayout = layout;
  }
}// namespace vk_utils
