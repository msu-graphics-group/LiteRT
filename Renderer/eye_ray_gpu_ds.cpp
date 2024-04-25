#include <vector>
#include <array>
#include <memory>
#include <limits>

#include <cassert>
#include "vk_copy.h"
#include "vk_context.h"

#include "eye_ray_gpu.h"


void MultiRenderer_GPU::AllocateAllDescriptorSets()
{
  // allocate pool
  //
  VkDescriptorPoolSize buffersSize, combinedImageSamSize, imageStorageSize;
  buffersSize.type                     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  buffersSize.descriptorCount          = 67 + 64; // + 64 for reserve

  std::vector<VkDescriptorPoolSize> poolSizes = {buffersSize};

  combinedImageSamSize.type            = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
  combinedImageSamSize.descriptorCount = 0*GetDefaultMaxTextures() + 0;

  imageStorageSize.type                = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
  imageStorageSize.descriptorCount     = 0;

  if(combinedImageSamSize.descriptorCount > 0)
    poolSizes.push_back(combinedImageSamSize);
  if(imageStorageSize.descriptorCount > 0)
    poolSizes.push_back(imageStorageSize);

  VkDescriptorPoolCreateInfo descriptorPoolCreateInfo = {};
  descriptorPoolCreateInfo.sType         = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
  descriptorPoolCreateInfo.maxSets       = 2 + 2; // add 1 to prevent zero case and one more for internal needs
  descriptorPoolCreateInfo.poolSizeCount = poolSizes.size();
  descriptorPoolCreateInfo.pPoolSizes    = poolSizes.data();

  VK_CHECK_RESULT(vkCreateDescriptorPool(device, &descriptorPoolCreateInfo, NULL, &m_dsPool));

  // allocate all descriptor sets
  //
  VkDescriptorSetLayout layouts[2] = {};
  layouts[0] = PackXYMegaDSLayout;
  layouts[1] = CastRaySingleMegaDSLayout;

  VkDescriptorSetAllocateInfo descriptorSetAllocateInfo = {};
  descriptorSetAllocateInfo.sType              = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
  descriptorSetAllocateInfo.descriptorPool     = m_dsPool;
  descriptorSetAllocateInfo.descriptorSetCount = 2;
  descriptorSetAllocateInfo.pSetLayouts        = layouts;

  auto tmpRes = vkAllocateDescriptorSets(device, &descriptorSetAllocateInfo, m_allGeneratedDS);
  VK_CHECK_RESULT(tmpRes);
}

VkDescriptorSetLayout MultiRenderer_GPU::CreatePackXYMegaDSLayout()
{
  std::array<VkDescriptorSetLayoutBinding, 33+1> dsBindings;

  const auto stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  // binding for m_packedXY
  dsBindings[0].binding            = 0;
  dsBindings[0].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[0].descriptorCount    = 1;
  dsBindings[0].stageFlags         = stageFlags;
  dsBindings[0].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSBSNodes
  dsBindings[1].binding            = 1;
  dsBindings[1].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[1].descriptorCount    = 1;
  dsBindings[1].stageFlags         = stageFlags;
  dsBindings[1].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfGridData
  dsBindings[2].binding            = 2;
  dsBindings[2].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[2].descriptorCount    = 1;
  dsBindings[2].stageFlags         = stageFlags;
  dsBindings[2].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfOctreeNodes
  dsBindings[3].binding            = 3;
  dsBindings[3].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[3].descriptorCount    = 1;
  dsBindings[3].stageFlags         = stageFlags;
  dsBindings[3].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_geomOffsets
  dsBindings[4].binding            = 4;
  dsBindings[4].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[4].descriptorCount    = 1;
  dsBindings[4].stageFlags         = stageFlags;
  dsBindings[4].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_geomIdByInstId
  dsBindings[5].binding            = 5;
  dsBindings[5].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[5].descriptorCount    = 1;
  dsBindings[5].stageFlags         = stageFlags;
  dsBindings[5].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfFrameOctreeRoots
  dsBindings[6].binding            = 6;
  dsBindings[6].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[6].descriptorCount    = 1;
  dsBindings[6].stageFlags         = stageFlags;
  dsBindings[6].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_bvhOffsets
  dsBindings[7].binding            = 7;
  dsBindings[7].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[7].descriptorCount    = 1;
  dsBindings[7].stageFlags         = stageFlags;
  dsBindings[7].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_RFGridScales
  dsBindings[8].binding            = 8;
  dsBindings[8].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[8].descriptorCount    = 1;
  dsBindings[8].stageFlags         = stageFlags;
  dsBindings[8].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfFrameOctreeNodes
  dsBindings[9].binding            = 9;
  dsBindings[9].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[9].descriptorCount    = 1;
  dsBindings[9].stageFlags         = stageFlags;
  dsBindings[9].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_vertPos
  dsBindings[10].binding            = 10;
  dsBindings[10].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[10].descriptorCount    = 1;
  dsBindings[10].stageFlags         = stageFlags;
  dsBindings[10].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_RFGridSizes
  dsBindings[11].binding            = 11;
  dsBindings[11].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[11].descriptorCount    = 1;
  dsBindings[11].stageFlags         = stageFlags;
  dsBindings[11].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSBSData
  dsBindings[12].binding            = 12;
  dsBindings[12].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[12].descriptorCount    = 1;
  dsBindings[12].stageFlags         = stageFlags;
  dsBindings[12].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfConjunctions
  dsBindings[13].binding            = 13;
  dsBindings[13].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[13].descriptorCount    = 1;
  dsBindings[13].stageFlags         = stageFlags;
  dsBindings[13].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_instMatricesInv
  dsBindings[14].binding            = 14;
  dsBindings[14].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[14].descriptorCount    = 1;
  dsBindings[14].stageFlags         = stageFlags;
  dsBindings[14].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfParameters
  dsBindings[15].binding            = 15;
  dsBindings[15].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[15].descriptorCount    = 1;
  dsBindings[15].stageFlags         = stageFlags;
  dsBindings[15].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_primIndices
  dsBindings[16].binding            = 16;
  dsBindings[16].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[16].descriptorCount    = 1;
  dsBindings[16].stageFlags         = stageFlags;
  dsBindings[16].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_RFGridData
  dsBindings[17].binding            = 17;
  dsBindings[17].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[17].descriptorCount    = 1;
  dsBindings[17].stageFlags         = stageFlags;
  dsBindings[17].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfGridOffsets
  dsBindings[18].binding            = 18;
  dsBindings[18].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[18].descriptorCount    = 1;
  dsBindings[18].stageFlags         = stageFlags;
  dsBindings[18].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfGridSizes
  dsBindings[19].binding            = 19;
  dsBindings[19].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[19].descriptorCount    = 1;
  dsBindings[19].stageFlags         = stageFlags;
  dsBindings[19].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_allNodePairs
  dsBindings[20].binding            = 20;
  dsBindings[20].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[20].descriptorCount    = 1;
  dsBindings[20].stageFlags         = stageFlags;
  dsBindings[20].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSVSRoots
  dsBindings[21].binding            = 21;
  dsBindings[21].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[21].descriptorCount    = 1;
  dsBindings[21].stageFlags         = stageFlags;
  dsBindings[21].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfOctreeRoots
  dsBindings[22].binding            = 22;
  dsBindings[22].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[22].descriptorCount    = 1;
  dsBindings[22].stageFlags         = stageFlags;
  dsBindings[22].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfNeuralProperties
  dsBindings[23].binding            = 23;
  dsBindings[23].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[23].descriptorCount    = 1;
  dsBindings[23].stageFlags         = stageFlags;
  dsBindings[23].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_nodesTLAS
  dsBindings[24].binding            = 24;
  dsBindings[24].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[24].descriptorCount    = 1;
  dsBindings[24].stageFlags         = stageFlags;
  dsBindings[24].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSVSNodes
  dsBindings[25].binding            = 25;
  dsBindings[25].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[25].descriptorCount    = 1;
  dsBindings[25].stageFlags         = stageFlags;
  dsBindings[25].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_ConjIndices
  dsBindings[26].binding            = 26;
  dsBindings[26].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[26].descriptorCount    = 1;
  dsBindings[26].stageFlags         = stageFlags;
  dsBindings[26].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_geomTypeByGeomId
  dsBindings[27].binding            = 27;
  dsBindings[27].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[27].descriptorCount    = 1;
  dsBindings[27].stageFlags         = stageFlags;
  dsBindings[27].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_origNodes
  dsBindings[28].binding            = 28;
  dsBindings[28].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[28].descriptorCount    = 1;
  dsBindings[28].stageFlags         = stageFlags;
  dsBindings[28].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSBSHeaders
  dsBindings[29].binding            = 29;
  dsBindings[29].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[29].descriptorCount    = 1;
  dsBindings[29].stageFlags         = stageFlags;
  dsBindings[29].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfObjects
  dsBindings[30].binding            = 30;
  dsBindings[30].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[30].descriptorCount    = 1;
  dsBindings[30].stageFlags         = stageFlags;
  dsBindings[30].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_indices
  dsBindings[31].binding            = 31;
  dsBindings[31].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[31].descriptorCount    = 1;
  dsBindings[31].stageFlags         = stageFlags;
  dsBindings[31].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSBSRemap
  dsBindings[32].binding            = 32;
  dsBindings[32].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[32].descriptorCount    = 1;
  dsBindings[32].stageFlags         = stageFlags;
  dsBindings[32].pImmutableSamplers = nullptr;

  // binding for POD members stored in m_classDataBuffer
  dsBindings[33].binding            = 33;
  dsBindings[33].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[33].descriptorCount    = 1;
  dsBindings[33].stageFlags         = stageFlags;
  dsBindings[33].pImmutableSamplers = nullptr;

  VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCreateInfo = {};
  descriptorSetLayoutCreateInfo.sType        = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
  descriptorSetLayoutCreateInfo.bindingCount = uint32_t(dsBindings.size());
  descriptorSetLayoutCreateInfo.pBindings    = dsBindings.data();

  VkDescriptorSetLayout layout = nullptr;
  VK_CHECK_RESULT(vkCreateDescriptorSetLayout(device, &descriptorSetLayoutCreateInfo, NULL, &layout));
  return layout;
}
VkDescriptorSetLayout MultiRenderer_GPU::CreateCastRaySingleMegaDSLayout()
{
  std::array<VkDescriptorSetLayoutBinding, 34+1> dsBindings;

  const auto stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  // binding for out_color
  dsBindings[0].binding            = 0;
  dsBindings[0].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[0].descriptorCount    = 1;
  dsBindings[0].stageFlags         = stageFlags;
  dsBindings[0].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSBSNodes
  dsBindings[1].binding            = 1;
  dsBindings[1].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[1].descriptorCount    = 1;
  dsBindings[1].stageFlags         = stageFlags;
  dsBindings[1].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfGridData
  dsBindings[2].binding            = 2;
  dsBindings[2].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[2].descriptorCount    = 1;
  dsBindings[2].stageFlags         = stageFlags;
  dsBindings[2].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSVSNodes
  dsBindings[3].binding            = 3;
  dsBindings[3].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[3].descriptorCount    = 1;
  dsBindings[3].stageFlags         = stageFlags;
  dsBindings[3].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_ConjIndices
  dsBindings[4].binding            = 4;
  dsBindings[4].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[4].descriptorCount    = 1;
  dsBindings[4].stageFlags         = stageFlags;
  dsBindings[4].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSBSData
  dsBindings[5].binding            = 5;
  dsBindings[5].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[5].descriptorCount    = 1;
  dsBindings[5].stageFlags         = stageFlags;
  dsBindings[5].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_RFGridData
  dsBindings[6].binding            = 6;
  dsBindings[6].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[6].descriptorCount    = 1;
  dsBindings[6].stageFlags         = stageFlags;
  dsBindings[6].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_geomOffsets
  dsBindings[7].binding            = 7;
  dsBindings[7].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[7].descriptorCount    = 1;
  dsBindings[7].stageFlags         = stageFlags;
  dsBindings[7].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfOctreeNodes
  dsBindings[8].binding            = 8;
  dsBindings[8].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[8].descriptorCount    = 1;
  dsBindings[8].stageFlags         = stageFlags;
  dsBindings[8].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_geomIdByInstId
  dsBindings[9].binding            = 9;
  dsBindings[9].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[9].descriptorCount    = 1;
  dsBindings[9].stageFlags         = stageFlags;
  dsBindings[9].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfFrameOctreeRoots
  dsBindings[10].binding            = 10;
  dsBindings[10].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[10].descriptorCount    = 1;
  dsBindings[10].stageFlags         = stageFlags;
  dsBindings[10].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfConjunctions
  dsBindings[11].binding            = 11;
  dsBindings[11].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[11].descriptorCount    = 1;
  dsBindings[11].stageFlags         = stageFlags;
  dsBindings[11].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfGridOffsets
  dsBindings[12].binding            = 12;
  dsBindings[12].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[12].descriptorCount    = 1;
  dsBindings[12].stageFlags         = stageFlags;
  dsBindings[12].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfGridSizes
  dsBindings[13].binding            = 13;
  dsBindings[13].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[13].descriptorCount    = 1;
  dsBindings[13].stageFlags         = stageFlags;
  dsBindings[13].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_allNodePairs
  dsBindings[14].binding            = 14;
  dsBindings[14].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[14].descriptorCount    = 1;
  dsBindings[14].stageFlags         = stageFlags;
  dsBindings[14].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_instMatricesInv
  dsBindings[15].binding            = 15;
  dsBindings[15].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[15].descriptorCount    = 1;
  dsBindings[15].stageFlags         = stageFlags;
  dsBindings[15].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfParameters
  dsBindings[16].binding            = 16;
  dsBindings[16].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[16].descriptorCount    = 1;
  dsBindings[16].stageFlags         = stageFlags;
  dsBindings[16].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_nodesTLAS
  dsBindings[17].binding            = 17;
  dsBindings[17].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[17].descriptorCount    = 1;
  dsBindings[17].stageFlags         = stageFlags;
  dsBindings[17].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_primIndices
  dsBindings[18].binding            = 18;
  dsBindings[18].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[18].descriptorCount    = 1;
  dsBindings[18].stageFlags         = stageFlags;
  dsBindings[18].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfNeuralProperties
  dsBindings[19].binding            = 19;
  dsBindings[19].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[19].descriptorCount    = 1;
  dsBindings[19].stageFlags         = stageFlags;
  dsBindings[19].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSVSRoots
  dsBindings[20].binding            = 20;
  dsBindings[20].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[20].descriptorCount    = 1;
  dsBindings[20].stageFlags         = stageFlags;
  dsBindings[20].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfOctreeRoots
  dsBindings[21].binding            = 21;
  dsBindings[21].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[21].descriptorCount    = 1;
  dsBindings[21].stageFlags         = stageFlags;
  dsBindings[21].pImmutableSamplers = nullptr;

  // binding for m_packedXY
  dsBindings[22].binding            = 22;
  dsBindings[22].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[22].descriptorCount    = 1;
  dsBindings[22].stageFlags         = stageFlags;
  dsBindings[22].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_bvhOffsets
  dsBindings[23].binding            = 23;
  dsBindings[23].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[23].descriptorCount    = 1;
  dsBindings[23].stageFlags         = stageFlags;
  dsBindings[23].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_RFGridScales
  dsBindings[24].binding            = 24;
  dsBindings[24].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[24].descriptorCount    = 1;
  dsBindings[24].stageFlags         = stageFlags;
  dsBindings[24].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_RFGridSizes
  dsBindings[25].binding            = 25;
  dsBindings[25].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[25].descriptorCount    = 1;
  dsBindings[25].stageFlags         = stageFlags;
  dsBindings[25].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_vertPos
  dsBindings[26].binding            = 26;
  dsBindings[26].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[26].descriptorCount    = 1;
  dsBindings[26].stageFlags         = stageFlags;
  dsBindings[26].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfFrameOctreeNodes
  dsBindings[27].binding            = 27;
  dsBindings[27].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[27].descriptorCount    = 1;
  dsBindings[27].stageFlags         = stageFlags;
  dsBindings[27].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_geomTypeByGeomId
  dsBindings[28].binding            = 28;
  dsBindings[28].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[28].descriptorCount    = 1;
  dsBindings[28].stageFlags         = stageFlags;
  dsBindings[28].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_origNodes
  dsBindings[29].binding            = 29;
  dsBindings[29].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[29].descriptorCount    = 1;
  dsBindings[29].stageFlags         = stageFlags;
  dsBindings[29].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSBSHeaders
  dsBindings[30].binding            = 30;
  dsBindings[30].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[30].descriptorCount    = 1;
  dsBindings[30].stageFlags         = stageFlags;
  dsBindings[30].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfObjects
  dsBindings[31].binding            = 31;
  dsBindings[31].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[31].descriptorCount    = 1;
  dsBindings[31].stageFlags         = stageFlags;
  dsBindings[31].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_indices
  dsBindings[32].binding            = 32;
  dsBindings[32].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[32].descriptorCount    = 1;
  dsBindings[32].stageFlags         = stageFlags;
  dsBindings[32].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSBSRemap
  dsBindings[33].binding            = 33;
  dsBindings[33].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[33].descriptorCount    = 1;
  dsBindings[33].stageFlags         = stageFlags;
  dsBindings[33].pImmutableSamplers = nullptr;

  // binding for POD members stored in m_classDataBuffer
  dsBindings[34].binding            = 34;
  dsBindings[34].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[34].descriptorCount    = 1;
  dsBindings[34].stageFlags         = stageFlags;
  dsBindings[34].pImmutableSamplers = nullptr;

  VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCreateInfo = {};
  descriptorSetLayoutCreateInfo.sType        = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
  descriptorSetLayoutCreateInfo.bindingCount = uint32_t(dsBindings.size());
  descriptorSetLayoutCreateInfo.pBindings    = dsBindings.data();

  VkDescriptorSetLayout layout = nullptr;
  VK_CHECK_RESULT(vkCreateDescriptorSetLayout(device, &descriptorSetLayoutCreateInfo, NULL, &layout));
  return layout;
}

VkDescriptorSetLayout MultiRenderer_GPU::CreatecopyKernelFloatDSLayout()
{
  std::array<VkDescriptorSetLayoutBinding, 2> dsBindings;

  dsBindings[0].binding            = 0;
  dsBindings[0].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[0].descriptorCount    = 1;
  dsBindings[0].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[0].pImmutableSamplers = nullptr;

  dsBindings[1].binding            = 1;
  dsBindings[1].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[1].descriptorCount    = 1;
  dsBindings[1].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[1].pImmutableSamplers = nullptr;

  VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCreateInfo = {};
  descriptorSetLayoutCreateInfo.sType        = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
  descriptorSetLayoutCreateInfo.bindingCount = dsBindings.size();
  descriptorSetLayoutCreateInfo.pBindings    = dsBindings.data();

  VkDescriptorSetLayout layout = nullptr;
  VK_CHECK_RESULT(vkCreateDescriptorSetLayout(device, &descriptorSetLayoutCreateInfo, NULL, &layout));
  return layout;
}

VkDescriptorSetLayout MultiRenderer_GPU::CreatematMulTransposeDSLayout()
{
  std::array<VkDescriptorSetLayoutBinding, 3> dsBindings;

  dsBindings[0].binding            = 0;
  dsBindings[0].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[0].descriptorCount    = 1;
  dsBindings[0].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[0].pImmutableSamplers = nullptr;

  dsBindings[1].binding            = 1;
  dsBindings[1].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[1].descriptorCount    = 1;
  dsBindings[1].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[1].pImmutableSamplers = nullptr;

  dsBindings[2].binding            = 2;
  dsBindings[2].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[2].descriptorCount    = 1;
  dsBindings[2].stageFlags         = VK_SHADER_STAGE_COMPUTE_BIT;
  dsBindings[2].pImmutableSamplers = nullptr;

  VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCreateInfo = {};
  descriptorSetLayoutCreateInfo.sType        = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
  descriptorSetLayoutCreateInfo.bindingCount = dsBindings.size();
  descriptorSetLayoutCreateInfo.pBindings    = dsBindings.data();

  VkDescriptorSetLayout layout = nullptr;
  VK_CHECK_RESULT(vkCreateDescriptorSetLayout(device, &descriptorSetLayoutCreateInfo, NULL, &layout));
  return layout;
}

void MultiRenderer_GPU::InitAllGeneratedDescriptorSets_PackXY()
{
  // now create actual bindings
  //
  // descriptor set #0: PackXYMegaCmd (["m_packedXY","m_pAccelStruct_m_SdfSBSNodes","m_pAccelStruct_m_SdfGridData","m_pAccelStruct_m_SdfOctreeNodes","m_pAccelStruct_m_geomOffsets","m_pAccelStruct_m_geomIdByInstId","m_pAccelStruct_m_SdfFrameOctreeRoots","m_pAccelStruct_m_bvhOffsets","m_pAccelStruct_m_RFGridScales","m_pAccelStruct_m_SdfFrameOctreeNodes","m_pAccelStruct_m_vertPos","m_pAccelStruct_m_RFGridSizes","m_pAccelStruct_m_SdfSBSData","m_pAccelStruct_m_SdfConjunctions","m_pAccelStruct_m_instMatricesInv","m_pAccelStruct_m_SdfParameters","m_pAccelStruct_m_primIndices","m_pAccelStruct_m_RFGridData","m_pAccelStruct_m_SdfGridOffsets","m_pAccelStruct_m_SdfGridSizes","m_pAccelStruct_m_allNodePairs","m_pAccelStruct_m_SdfSVSRoots","m_pAccelStruct_m_SdfOctreeRoots","m_pAccelStruct_m_SdfNeuralProperties","m_pAccelStruct_m_nodesTLAS","m_pAccelStruct_m_SdfSVSNodes","m_pAccelStruct_m_ConjIndices","m_pAccelStruct_m_geomTypeByGeomId","m_pAccelStruct_m_origNodes","m_pAccelStruct_m_SdfSBSHeaders","m_pAccelStruct_m_SdfObjects","m_pAccelStruct_m_indices","m_pAccelStruct_m_SdfSBSRemap"])
  {
    constexpr uint additionalSize = 1;

    std::array<VkDescriptorBufferInfo, 33 + additionalSize> descriptorBufferInfo;
    std::array<VkDescriptorImageInfo,  33 + additionalSize> descriptorImageInfo;
    std::array<VkWriteDescriptorSet,   33 + additionalSize> writeDescriptorSet;

    descriptorBufferInfo[0]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[0].buffer = m_vdata.m_packedXYBuffer;
    descriptorBufferInfo[0].offset = m_vdata.m_packedXYOffset;
    descriptorBufferInfo[0].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[0]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[0].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[0].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[0].dstBinding       = 0;
    writeDescriptorSet[0].descriptorCount  = 1;
    writeDescriptorSet[0].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[0].pBufferInfo      = &descriptorBufferInfo[0];
    writeDescriptorSet[0].pImageInfo       = nullptr;
    writeDescriptorSet[0].pTexelBufferView = nullptr;

    descriptorBufferInfo[1]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[1].buffer = m_vdata.m_pAccelStruct_m_SdfSBSNodesBuffer;
    descriptorBufferInfo[1].offset = m_vdata.m_pAccelStruct_m_SdfSBSNodesOffset;
    descriptorBufferInfo[1].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[1]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[1].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[1].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[1].dstBinding       = 1;
    writeDescriptorSet[1].descriptorCount  = 1;
    writeDescriptorSet[1].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[1].pBufferInfo      = &descriptorBufferInfo[1];
    writeDescriptorSet[1].pImageInfo       = nullptr;
    writeDescriptorSet[1].pTexelBufferView = nullptr;

    descriptorBufferInfo[2]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[2].buffer = m_vdata.m_pAccelStruct_m_SdfGridDataBuffer;
    descriptorBufferInfo[2].offset = m_vdata.m_pAccelStruct_m_SdfGridDataOffset;
    descriptorBufferInfo[2].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[2]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[2].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[2].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[2].dstBinding       = 2;
    writeDescriptorSet[2].descriptorCount  = 1;
    writeDescriptorSet[2].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[2].pBufferInfo      = &descriptorBufferInfo[2];
    writeDescriptorSet[2].pImageInfo       = nullptr;
    writeDescriptorSet[2].pTexelBufferView = nullptr;

    descriptorBufferInfo[3]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[3].buffer = m_vdata.m_pAccelStruct_m_SdfOctreeNodesBuffer;
    descriptorBufferInfo[3].offset = m_vdata.m_pAccelStruct_m_SdfOctreeNodesOffset;
    descriptorBufferInfo[3].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[3]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[3].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[3].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[3].dstBinding       = 3;
    writeDescriptorSet[3].descriptorCount  = 1;
    writeDescriptorSet[3].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[3].pBufferInfo      = &descriptorBufferInfo[3];
    writeDescriptorSet[3].pImageInfo       = nullptr;
    writeDescriptorSet[3].pTexelBufferView = nullptr;

    descriptorBufferInfo[4]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[4].buffer = m_vdata.m_pAccelStruct_m_geomOffsetsBuffer;
    descriptorBufferInfo[4].offset = m_vdata.m_pAccelStruct_m_geomOffsetsOffset;
    descriptorBufferInfo[4].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[4]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[4].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[4].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[4].dstBinding       = 4;
    writeDescriptorSet[4].descriptorCount  = 1;
    writeDescriptorSet[4].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[4].pBufferInfo      = &descriptorBufferInfo[4];
    writeDescriptorSet[4].pImageInfo       = nullptr;
    writeDescriptorSet[4].pTexelBufferView = nullptr;

    descriptorBufferInfo[5]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[5].buffer = m_vdata.m_pAccelStruct_m_geomIdByInstIdBuffer;
    descriptorBufferInfo[5].offset = m_vdata.m_pAccelStruct_m_geomIdByInstIdOffset;
    descriptorBufferInfo[5].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[5]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[5].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[5].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[5].dstBinding       = 5;
    writeDescriptorSet[5].descriptorCount  = 1;
    writeDescriptorSet[5].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[5].pBufferInfo      = &descriptorBufferInfo[5];
    writeDescriptorSet[5].pImageInfo       = nullptr;
    writeDescriptorSet[5].pTexelBufferView = nullptr;

    descriptorBufferInfo[6]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[6].buffer = m_vdata.m_pAccelStruct_m_SdfFrameOctreeRootsBuffer;
    descriptorBufferInfo[6].offset = m_vdata.m_pAccelStruct_m_SdfFrameOctreeRootsOffset;
    descriptorBufferInfo[6].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[6]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[6].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[6].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[6].dstBinding       = 6;
    writeDescriptorSet[6].descriptorCount  = 1;
    writeDescriptorSet[6].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[6].pBufferInfo      = &descriptorBufferInfo[6];
    writeDescriptorSet[6].pImageInfo       = nullptr;
    writeDescriptorSet[6].pTexelBufferView = nullptr;

    descriptorBufferInfo[7]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[7].buffer = m_vdata.m_pAccelStruct_m_bvhOffsetsBuffer;
    descriptorBufferInfo[7].offset = m_vdata.m_pAccelStruct_m_bvhOffsetsOffset;
    descriptorBufferInfo[7].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[7]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[7].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[7].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[7].dstBinding       = 7;
    writeDescriptorSet[7].descriptorCount  = 1;
    writeDescriptorSet[7].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[7].pBufferInfo      = &descriptorBufferInfo[7];
    writeDescriptorSet[7].pImageInfo       = nullptr;
    writeDescriptorSet[7].pTexelBufferView = nullptr;

    descriptorBufferInfo[8]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[8].buffer = m_vdata.m_pAccelStruct_m_RFGridScalesBuffer;
    descriptorBufferInfo[8].offset = m_vdata.m_pAccelStruct_m_RFGridScalesOffset;
    descriptorBufferInfo[8].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[8]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[8].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[8].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[8].dstBinding       = 8;
    writeDescriptorSet[8].descriptorCount  = 1;
    writeDescriptorSet[8].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[8].pBufferInfo      = &descriptorBufferInfo[8];
    writeDescriptorSet[8].pImageInfo       = nullptr;
    writeDescriptorSet[8].pTexelBufferView = nullptr;

    descriptorBufferInfo[9]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[9].buffer = m_vdata.m_pAccelStruct_m_SdfFrameOctreeNodesBuffer;
    descriptorBufferInfo[9].offset = m_vdata.m_pAccelStruct_m_SdfFrameOctreeNodesOffset;
    descriptorBufferInfo[9].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[9]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[9].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[9].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[9].dstBinding       = 9;
    writeDescriptorSet[9].descriptorCount  = 1;
    writeDescriptorSet[9].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[9].pBufferInfo      = &descriptorBufferInfo[9];
    writeDescriptorSet[9].pImageInfo       = nullptr;
    writeDescriptorSet[9].pTexelBufferView = nullptr;

    descriptorBufferInfo[10]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[10].buffer = m_vdata.m_pAccelStruct_m_vertPosBuffer;
    descriptorBufferInfo[10].offset = m_vdata.m_pAccelStruct_m_vertPosOffset;
    descriptorBufferInfo[10].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[10]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[10].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[10].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[10].dstBinding       = 10;
    writeDescriptorSet[10].descriptorCount  = 1;
    writeDescriptorSet[10].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[10].pBufferInfo      = &descriptorBufferInfo[10];
    writeDescriptorSet[10].pImageInfo       = nullptr;
    writeDescriptorSet[10].pTexelBufferView = nullptr;

    descriptorBufferInfo[11]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[11].buffer = m_vdata.m_pAccelStruct_m_RFGridSizesBuffer;
    descriptorBufferInfo[11].offset = m_vdata.m_pAccelStruct_m_RFGridSizesOffset;
    descriptorBufferInfo[11].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[11]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[11].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[11].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[11].dstBinding       = 11;
    writeDescriptorSet[11].descriptorCount  = 1;
    writeDescriptorSet[11].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[11].pBufferInfo      = &descriptorBufferInfo[11];
    writeDescriptorSet[11].pImageInfo       = nullptr;
    writeDescriptorSet[11].pTexelBufferView = nullptr;

    descriptorBufferInfo[12]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[12].buffer = m_vdata.m_pAccelStruct_m_SdfSBSDataBuffer;
    descriptorBufferInfo[12].offset = m_vdata.m_pAccelStruct_m_SdfSBSDataOffset;
    descriptorBufferInfo[12].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[12]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[12].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[12].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[12].dstBinding       = 12;
    writeDescriptorSet[12].descriptorCount  = 1;
    writeDescriptorSet[12].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[12].pBufferInfo      = &descriptorBufferInfo[12];
    writeDescriptorSet[12].pImageInfo       = nullptr;
    writeDescriptorSet[12].pTexelBufferView = nullptr;

    descriptorBufferInfo[13]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[13].buffer = m_vdata.m_pAccelStruct_m_SdfConjunctionsBuffer;
    descriptorBufferInfo[13].offset = m_vdata.m_pAccelStruct_m_SdfConjunctionsOffset;
    descriptorBufferInfo[13].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[13]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[13].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[13].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[13].dstBinding       = 13;
    writeDescriptorSet[13].descriptorCount  = 1;
    writeDescriptorSet[13].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[13].pBufferInfo      = &descriptorBufferInfo[13];
    writeDescriptorSet[13].pImageInfo       = nullptr;
    writeDescriptorSet[13].pTexelBufferView = nullptr;

    descriptorBufferInfo[14]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[14].buffer = m_vdata.m_pAccelStruct_m_instMatricesInvBuffer;
    descriptorBufferInfo[14].offset = m_vdata.m_pAccelStruct_m_instMatricesInvOffset;
    descriptorBufferInfo[14].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[14]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[14].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[14].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[14].dstBinding       = 14;
    writeDescriptorSet[14].descriptorCount  = 1;
    writeDescriptorSet[14].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[14].pBufferInfo      = &descriptorBufferInfo[14];
    writeDescriptorSet[14].pImageInfo       = nullptr;
    writeDescriptorSet[14].pTexelBufferView = nullptr;

    descriptorBufferInfo[15]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[15].buffer = m_vdata.m_pAccelStruct_m_SdfParametersBuffer;
    descriptorBufferInfo[15].offset = m_vdata.m_pAccelStruct_m_SdfParametersOffset;
    descriptorBufferInfo[15].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[15]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[15].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[15].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[15].dstBinding       = 15;
    writeDescriptorSet[15].descriptorCount  = 1;
    writeDescriptorSet[15].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[15].pBufferInfo      = &descriptorBufferInfo[15];
    writeDescriptorSet[15].pImageInfo       = nullptr;
    writeDescriptorSet[15].pTexelBufferView = nullptr;

    descriptorBufferInfo[16]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[16].buffer = m_vdata.m_pAccelStruct_m_primIndicesBuffer;
    descriptorBufferInfo[16].offset = m_vdata.m_pAccelStruct_m_primIndicesOffset;
    descriptorBufferInfo[16].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[16]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[16].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[16].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[16].dstBinding       = 16;
    writeDescriptorSet[16].descriptorCount  = 1;
    writeDescriptorSet[16].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[16].pBufferInfo      = &descriptorBufferInfo[16];
    writeDescriptorSet[16].pImageInfo       = nullptr;
    writeDescriptorSet[16].pTexelBufferView = nullptr;

    descriptorBufferInfo[17]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[17].buffer = m_vdata.m_pAccelStruct_m_RFGridDataBuffer;
    descriptorBufferInfo[17].offset = m_vdata.m_pAccelStruct_m_RFGridDataOffset;
    descriptorBufferInfo[17].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[17]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[17].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[17].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[17].dstBinding       = 17;
    writeDescriptorSet[17].descriptorCount  = 1;
    writeDescriptorSet[17].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[17].pBufferInfo      = &descriptorBufferInfo[17];
    writeDescriptorSet[17].pImageInfo       = nullptr;
    writeDescriptorSet[17].pTexelBufferView = nullptr;

    descriptorBufferInfo[18]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[18].buffer = m_vdata.m_pAccelStruct_m_SdfGridOffsetsBuffer;
    descriptorBufferInfo[18].offset = m_vdata.m_pAccelStruct_m_SdfGridOffsetsOffset;
    descriptorBufferInfo[18].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[18]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[18].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[18].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[18].dstBinding       = 18;
    writeDescriptorSet[18].descriptorCount  = 1;
    writeDescriptorSet[18].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[18].pBufferInfo      = &descriptorBufferInfo[18];
    writeDescriptorSet[18].pImageInfo       = nullptr;
    writeDescriptorSet[18].pTexelBufferView = nullptr;

    descriptorBufferInfo[19]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[19].buffer = m_vdata.m_pAccelStruct_m_SdfGridSizesBuffer;
    descriptorBufferInfo[19].offset = m_vdata.m_pAccelStruct_m_SdfGridSizesOffset;
    descriptorBufferInfo[19].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[19]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[19].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[19].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[19].dstBinding       = 19;
    writeDescriptorSet[19].descriptorCount  = 1;
    writeDescriptorSet[19].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[19].pBufferInfo      = &descriptorBufferInfo[19];
    writeDescriptorSet[19].pImageInfo       = nullptr;
    writeDescriptorSet[19].pTexelBufferView = nullptr;

    descriptorBufferInfo[20]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[20].buffer = m_vdata.m_pAccelStruct_m_allNodePairsBuffer;
    descriptorBufferInfo[20].offset = m_vdata.m_pAccelStruct_m_allNodePairsOffset;
    descriptorBufferInfo[20].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[20]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[20].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[20].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[20].dstBinding       = 20;
    writeDescriptorSet[20].descriptorCount  = 1;
    writeDescriptorSet[20].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[20].pBufferInfo      = &descriptorBufferInfo[20];
    writeDescriptorSet[20].pImageInfo       = nullptr;
    writeDescriptorSet[20].pTexelBufferView = nullptr;

    descriptorBufferInfo[21]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[21].buffer = m_vdata.m_pAccelStruct_m_SdfSVSRootsBuffer;
    descriptorBufferInfo[21].offset = m_vdata.m_pAccelStruct_m_SdfSVSRootsOffset;
    descriptorBufferInfo[21].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[21]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[21].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[21].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[21].dstBinding       = 21;
    writeDescriptorSet[21].descriptorCount  = 1;
    writeDescriptorSet[21].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[21].pBufferInfo      = &descriptorBufferInfo[21];
    writeDescriptorSet[21].pImageInfo       = nullptr;
    writeDescriptorSet[21].pTexelBufferView = nullptr;

    descriptorBufferInfo[22]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[22].buffer = m_vdata.m_pAccelStruct_m_SdfOctreeRootsBuffer;
    descriptorBufferInfo[22].offset = m_vdata.m_pAccelStruct_m_SdfOctreeRootsOffset;
    descriptorBufferInfo[22].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[22]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[22].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[22].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[22].dstBinding       = 22;
    writeDescriptorSet[22].descriptorCount  = 1;
    writeDescriptorSet[22].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[22].pBufferInfo      = &descriptorBufferInfo[22];
    writeDescriptorSet[22].pImageInfo       = nullptr;
    writeDescriptorSet[22].pTexelBufferView = nullptr;

    descriptorBufferInfo[23]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[23].buffer = m_vdata.m_pAccelStruct_m_SdfNeuralPropertiesBuffer;
    descriptorBufferInfo[23].offset = m_vdata.m_pAccelStruct_m_SdfNeuralPropertiesOffset;
    descriptorBufferInfo[23].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[23]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[23].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[23].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[23].dstBinding       = 23;
    writeDescriptorSet[23].descriptorCount  = 1;
    writeDescriptorSet[23].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[23].pBufferInfo      = &descriptorBufferInfo[23];
    writeDescriptorSet[23].pImageInfo       = nullptr;
    writeDescriptorSet[23].pTexelBufferView = nullptr;

    descriptorBufferInfo[24]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[24].buffer = m_vdata.m_pAccelStruct_m_nodesTLASBuffer;
    descriptorBufferInfo[24].offset = m_vdata.m_pAccelStruct_m_nodesTLASOffset;
    descriptorBufferInfo[24].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[24]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[24].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[24].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[24].dstBinding       = 24;
    writeDescriptorSet[24].descriptorCount  = 1;
    writeDescriptorSet[24].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[24].pBufferInfo      = &descriptorBufferInfo[24];
    writeDescriptorSet[24].pImageInfo       = nullptr;
    writeDescriptorSet[24].pTexelBufferView = nullptr;

    descriptorBufferInfo[25]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[25].buffer = m_vdata.m_pAccelStruct_m_SdfSVSNodesBuffer;
    descriptorBufferInfo[25].offset = m_vdata.m_pAccelStruct_m_SdfSVSNodesOffset;
    descriptorBufferInfo[25].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[25]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[25].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[25].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[25].dstBinding       = 25;
    writeDescriptorSet[25].descriptorCount  = 1;
    writeDescriptorSet[25].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[25].pBufferInfo      = &descriptorBufferInfo[25];
    writeDescriptorSet[25].pImageInfo       = nullptr;
    writeDescriptorSet[25].pTexelBufferView = nullptr;

    descriptorBufferInfo[26]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[26].buffer = m_vdata.m_pAccelStruct_m_ConjIndicesBuffer;
    descriptorBufferInfo[26].offset = m_vdata.m_pAccelStruct_m_ConjIndicesOffset;
    descriptorBufferInfo[26].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[26]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[26].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[26].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[26].dstBinding       = 26;
    writeDescriptorSet[26].descriptorCount  = 1;
    writeDescriptorSet[26].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[26].pBufferInfo      = &descriptorBufferInfo[26];
    writeDescriptorSet[26].pImageInfo       = nullptr;
    writeDescriptorSet[26].pTexelBufferView = nullptr;

    descriptorBufferInfo[27]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[27].buffer = m_vdata.m_pAccelStruct_m_geomTypeByGeomIdBuffer;
    descriptorBufferInfo[27].offset = m_vdata.m_pAccelStruct_m_geomTypeByGeomIdOffset;
    descriptorBufferInfo[27].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[27]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[27].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[27].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[27].dstBinding       = 27;
    writeDescriptorSet[27].descriptorCount  = 1;
    writeDescriptorSet[27].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[27].pBufferInfo      = &descriptorBufferInfo[27];
    writeDescriptorSet[27].pImageInfo       = nullptr;
    writeDescriptorSet[27].pTexelBufferView = nullptr;

    descriptorBufferInfo[28]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[28].buffer = m_vdata.m_pAccelStruct_m_origNodesBuffer;
    descriptorBufferInfo[28].offset = m_vdata.m_pAccelStruct_m_origNodesOffset;
    descriptorBufferInfo[28].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[28]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[28].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[28].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[28].dstBinding       = 28;
    writeDescriptorSet[28].descriptorCount  = 1;
    writeDescriptorSet[28].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[28].pBufferInfo      = &descriptorBufferInfo[28];
    writeDescriptorSet[28].pImageInfo       = nullptr;
    writeDescriptorSet[28].pTexelBufferView = nullptr;

    descriptorBufferInfo[29]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[29].buffer = m_vdata.m_pAccelStruct_m_SdfSBSHeadersBuffer;
    descriptorBufferInfo[29].offset = m_vdata.m_pAccelStruct_m_SdfSBSHeadersOffset;
    descriptorBufferInfo[29].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[29]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[29].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[29].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[29].dstBinding       = 29;
    writeDescriptorSet[29].descriptorCount  = 1;
    writeDescriptorSet[29].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[29].pBufferInfo      = &descriptorBufferInfo[29];
    writeDescriptorSet[29].pImageInfo       = nullptr;
    writeDescriptorSet[29].pTexelBufferView = nullptr;

    descriptorBufferInfo[30]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[30].buffer = m_vdata.m_pAccelStruct_m_SdfObjectsBuffer;
    descriptorBufferInfo[30].offset = m_vdata.m_pAccelStruct_m_SdfObjectsOffset;
    descriptorBufferInfo[30].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[30]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[30].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[30].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[30].dstBinding       = 30;
    writeDescriptorSet[30].descriptorCount  = 1;
    writeDescriptorSet[30].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[30].pBufferInfo      = &descriptorBufferInfo[30];
    writeDescriptorSet[30].pImageInfo       = nullptr;
    writeDescriptorSet[30].pTexelBufferView = nullptr;

    descriptorBufferInfo[31]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[31].buffer = m_vdata.m_pAccelStruct_m_indicesBuffer;
    descriptorBufferInfo[31].offset = m_vdata.m_pAccelStruct_m_indicesOffset;
    descriptorBufferInfo[31].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[31]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[31].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[31].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[31].dstBinding       = 31;
    writeDescriptorSet[31].descriptorCount  = 1;
    writeDescriptorSet[31].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[31].pBufferInfo      = &descriptorBufferInfo[31];
    writeDescriptorSet[31].pImageInfo       = nullptr;
    writeDescriptorSet[31].pTexelBufferView = nullptr;

    descriptorBufferInfo[32]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[32].buffer = m_vdata.m_pAccelStruct_m_SdfSBSRemapBuffer;
    descriptorBufferInfo[32].offset = m_vdata.m_pAccelStruct_m_SdfSBSRemapOffset;
    descriptorBufferInfo[32].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[32]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[32].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[32].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[32].dstBinding       = 32;
    writeDescriptorSet[32].descriptorCount  = 1;
    writeDescriptorSet[32].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[32].pBufferInfo      = &descriptorBufferInfo[32];
    writeDescriptorSet[32].pImageInfo       = nullptr;
    writeDescriptorSet[32].pTexelBufferView = nullptr;

    descriptorBufferInfo[33]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[33].buffer = m_classDataBuffer;
    descriptorBufferInfo[33].offset = 0;
    descriptorBufferInfo[33].range  = VK_WHOLE_SIZE;

    writeDescriptorSet[33]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[33].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[33].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[33].dstBinding       = 33;
    writeDescriptorSet[33].descriptorCount  = 1;
    writeDescriptorSet[33].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[33].pBufferInfo      = &descriptorBufferInfo[33];
    writeDescriptorSet[33].pImageInfo       = nullptr;
    writeDescriptorSet[33].pTexelBufferView = nullptr;

    vkUpdateDescriptorSets(device, uint32_t(writeDescriptorSet.size()), writeDescriptorSet.data(), 0, NULL);
  }
}

void MultiRenderer_GPU::InitAllGeneratedDescriptorSets_CastRaySingle()
{
  // now create actual bindings
  //
  // descriptor set #1: CastRaySingleMegaCmd (["out_color","m_pAccelStruct_m_SdfSBSNodes","m_pAccelStruct_m_SdfGridData","m_pAccelStruct_m_SdfSVSNodes","m_pAccelStruct_m_ConjIndices","m_pAccelStruct_m_SdfSBSData","m_pAccelStruct_m_RFGridData","m_pAccelStruct_m_geomOffsets","m_pAccelStruct_m_SdfOctreeNodes","m_pAccelStruct_m_geomIdByInstId","m_pAccelStruct_m_SdfFrameOctreeRoots","m_pAccelStruct_m_SdfConjunctions","m_pAccelStruct_m_SdfGridOffsets","m_pAccelStruct_m_SdfGridSizes","m_pAccelStruct_m_allNodePairs","m_pAccelStruct_m_instMatricesInv","m_pAccelStruct_m_SdfParameters","m_pAccelStruct_m_nodesTLAS","m_pAccelStruct_m_primIndices","m_pAccelStruct_m_SdfNeuralProperties","m_pAccelStruct_m_SdfSVSRoots","m_pAccelStruct_m_SdfOctreeRoots","m_packedXY","m_pAccelStruct_m_bvhOffsets","m_pAccelStruct_m_RFGridScales","m_pAccelStruct_m_RFGridSizes","m_pAccelStruct_m_vertPos","m_pAccelStruct_m_SdfFrameOctreeNodes","m_pAccelStruct_m_geomTypeByGeomId","m_pAccelStruct_m_origNodes","m_pAccelStruct_m_SdfSBSHeaders","m_pAccelStruct_m_SdfObjects","m_pAccelStruct_m_indices","m_pAccelStruct_m_SdfSBSRemap"])
  {
    constexpr uint additionalSize = 1;

    std::array<VkDescriptorBufferInfo, 34 + additionalSize> descriptorBufferInfo;
    std::array<VkDescriptorImageInfo,  34 + additionalSize> descriptorImageInfo;
    std::array<VkWriteDescriptorSet,   34 + additionalSize> writeDescriptorSet;

    descriptorBufferInfo[0]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[0].buffer = CastRaySingle_local.out_colorBuffer;
    descriptorBufferInfo[0].offset = CastRaySingle_local.out_colorOffset;
    descriptorBufferInfo[0].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[0]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[0].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[0].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[0].dstBinding       = 0;
    writeDescriptorSet[0].descriptorCount  = 1;
    writeDescriptorSet[0].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[0].pBufferInfo      = &descriptorBufferInfo[0];
    writeDescriptorSet[0].pImageInfo       = nullptr;
    writeDescriptorSet[0].pTexelBufferView = nullptr;

    descriptorBufferInfo[1]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[1].buffer = m_vdata.m_pAccelStruct_m_SdfSBSNodesBuffer;
    descriptorBufferInfo[1].offset = m_vdata.m_pAccelStruct_m_SdfSBSNodesOffset;
    descriptorBufferInfo[1].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[1]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[1].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[1].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[1].dstBinding       = 1;
    writeDescriptorSet[1].descriptorCount  = 1;
    writeDescriptorSet[1].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[1].pBufferInfo      = &descriptorBufferInfo[1];
    writeDescriptorSet[1].pImageInfo       = nullptr;
    writeDescriptorSet[1].pTexelBufferView = nullptr;

    descriptorBufferInfo[2]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[2].buffer = m_vdata.m_pAccelStruct_m_SdfGridDataBuffer;
    descriptorBufferInfo[2].offset = m_vdata.m_pAccelStruct_m_SdfGridDataOffset;
    descriptorBufferInfo[2].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[2]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[2].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[2].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[2].dstBinding       = 2;
    writeDescriptorSet[2].descriptorCount  = 1;
    writeDescriptorSet[2].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[2].pBufferInfo      = &descriptorBufferInfo[2];
    writeDescriptorSet[2].pImageInfo       = nullptr;
    writeDescriptorSet[2].pTexelBufferView = nullptr;

    descriptorBufferInfo[3]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[3].buffer = m_vdata.m_pAccelStruct_m_SdfSVSNodesBuffer;
    descriptorBufferInfo[3].offset = m_vdata.m_pAccelStruct_m_SdfSVSNodesOffset;
    descriptorBufferInfo[3].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[3]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[3].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[3].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[3].dstBinding       = 3;
    writeDescriptorSet[3].descriptorCount  = 1;
    writeDescriptorSet[3].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[3].pBufferInfo      = &descriptorBufferInfo[3];
    writeDescriptorSet[3].pImageInfo       = nullptr;
    writeDescriptorSet[3].pTexelBufferView = nullptr;

    descriptorBufferInfo[4]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[4].buffer = m_vdata.m_pAccelStruct_m_ConjIndicesBuffer;
    descriptorBufferInfo[4].offset = m_vdata.m_pAccelStruct_m_ConjIndicesOffset;
    descriptorBufferInfo[4].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[4]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[4].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[4].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[4].dstBinding       = 4;
    writeDescriptorSet[4].descriptorCount  = 1;
    writeDescriptorSet[4].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[4].pBufferInfo      = &descriptorBufferInfo[4];
    writeDescriptorSet[4].pImageInfo       = nullptr;
    writeDescriptorSet[4].pTexelBufferView = nullptr;

    descriptorBufferInfo[5]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[5].buffer = m_vdata.m_pAccelStruct_m_SdfSBSDataBuffer;
    descriptorBufferInfo[5].offset = m_vdata.m_pAccelStruct_m_SdfSBSDataOffset;
    descriptorBufferInfo[5].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[5]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[5].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[5].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[5].dstBinding       = 5;
    writeDescriptorSet[5].descriptorCount  = 1;
    writeDescriptorSet[5].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[5].pBufferInfo      = &descriptorBufferInfo[5];
    writeDescriptorSet[5].pImageInfo       = nullptr;
    writeDescriptorSet[5].pTexelBufferView = nullptr;

    descriptorBufferInfo[6]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[6].buffer = m_vdata.m_pAccelStruct_m_RFGridDataBuffer;
    descriptorBufferInfo[6].offset = m_vdata.m_pAccelStruct_m_RFGridDataOffset;
    descriptorBufferInfo[6].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[6]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[6].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[6].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[6].dstBinding       = 6;
    writeDescriptorSet[6].descriptorCount  = 1;
    writeDescriptorSet[6].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[6].pBufferInfo      = &descriptorBufferInfo[6];
    writeDescriptorSet[6].pImageInfo       = nullptr;
    writeDescriptorSet[6].pTexelBufferView = nullptr;

    descriptorBufferInfo[7]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[7].buffer = m_vdata.m_pAccelStruct_m_geomOffsetsBuffer;
    descriptorBufferInfo[7].offset = m_vdata.m_pAccelStruct_m_geomOffsetsOffset;
    descriptorBufferInfo[7].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[7]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[7].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[7].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[7].dstBinding       = 7;
    writeDescriptorSet[7].descriptorCount  = 1;
    writeDescriptorSet[7].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[7].pBufferInfo      = &descriptorBufferInfo[7];
    writeDescriptorSet[7].pImageInfo       = nullptr;
    writeDescriptorSet[7].pTexelBufferView = nullptr;

    descriptorBufferInfo[8]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[8].buffer = m_vdata.m_pAccelStruct_m_SdfOctreeNodesBuffer;
    descriptorBufferInfo[8].offset = m_vdata.m_pAccelStruct_m_SdfOctreeNodesOffset;
    descriptorBufferInfo[8].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[8]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[8].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[8].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[8].dstBinding       = 8;
    writeDescriptorSet[8].descriptorCount  = 1;
    writeDescriptorSet[8].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[8].pBufferInfo      = &descriptorBufferInfo[8];
    writeDescriptorSet[8].pImageInfo       = nullptr;
    writeDescriptorSet[8].pTexelBufferView = nullptr;

    descriptorBufferInfo[9]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[9].buffer = m_vdata.m_pAccelStruct_m_geomIdByInstIdBuffer;
    descriptorBufferInfo[9].offset = m_vdata.m_pAccelStruct_m_geomIdByInstIdOffset;
    descriptorBufferInfo[9].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[9]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[9].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[9].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[9].dstBinding       = 9;
    writeDescriptorSet[9].descriptorCount  = 1;
    writeDescriptorSet[9].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[9].pBufferInfo      = &descriptorBufferInfo[9];
    writeDescriptorSet[9].pImageInfo       = nullptr;
    writeDescriptorSet[9].pTexelBufferView = nullptr;

    descriptorBufferInfo[10]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[10].buffer = m_vdata.m_pAccelStruct_m_SdfFrameOctreeRootsBuffer;
    descriptorBufferInfo[10].offset = m_vdata.m_pAccelStruct_m_SdfFrameOctreeRootsOffset;
    descriptorBufferInfo[10].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[10]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[10].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[10].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[10].dstBinding       = 10;
    writeDescriptorSet[10].descriptorCount  = 1;
    writeDescriptorSet[10].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[10].pBufferInfo      = &descriptorBufferInfo[10];
    writeDescriptorSet[10].pImageInfo       = nullptr;
    writeDescriptorSet[10].pTexelBufferView = nullptr;

    descriptorBufferInfo[11]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[11].buffer = m_vdata.m_pAccelStruct_m_SdfConjunctionsBuffer;
    descriptorBufferInfo[11].offset = m_vdata.m_pAccelStruct_m_SdfConjunctionsOffset;
    descriptorBufferInfo[11].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[11]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[11].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[11].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[11].dstBinding       = 11;
    writeDescriptorSet[11].descriptorCount  = 1;
    writeDescriptorSet[11].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[11].pBufferInfo      = &descriptorBufferInfo[11];
    writeDescriptorSet[11].pImageInfo       = nullptr;
    writeDescriptorSet[11].pTexelBufferView = nullptr;

    descriptorBufferInfo[12]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[12].buffer = m_vdata.m_pAccelStruct_m_SdfGridOffsetsBuffer;
    descriptorBufferInfo[12].offset = m_vdata.m_pAccelStruct_m_SdfGridOffsetsOffset;
    descriptorBufferInfo[12].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[12]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[12].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[12].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[12].dstBinding       = 12;
    writeDescriptorSet[12].descriptorCount  = 1;
    writeDescriptorSet[12].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[12].pBufferInfo      = &descriptorBufferInfo[12];
    writeDescriptorSet[12].pImageInfo       = nullptr;
    writeDescriptorSet[12].pTexelBufferView = nullptr;

    descriptorBufferInfo[13]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[13].buffer = m_vdata.m_pAccelStruct_m_SdfGridSizesBuffer;
    descriptorBufferInfo[13].offset = m_vdata.m_pAccelStruct_m_SdfGridSizesOffset;
    descriptorBufferInfo[13].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[13]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[13].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[13].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[13].dstBinding       = 13;
    writeDescriptorSet[13].descriptorCount  = 1;
    writeDescriptorSet[13].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[13].pBufferInfo      = &descriptorBufferInfo[13];
    writeDescriptorSet[13].pImageInfo       = nullptr;
    writeDescriptorSet[13].pTexelBufferView = nullptr;

    descriptorBufferInfo[14]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[14].buffer = m_vdata.m_pAccelStruct_m_allNodePairsBuffer;
    descriptorBufferInfo[14].offset = m_vdata.m_pAccelStruct_m_allNodePairsOffset;
    descriptorBufferInfo[14].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[14]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[14].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[14].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[14].dstBinding       = 14;
    writeDescriptorSet[14].descriptorCount  = 1;
    writeDescriptorSet[14].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[14].pBufferInfo      = &descriptorBufferInfo[14];
    writeDescriptorSet[14].pImageInfo       = nullptr;
    writeDescriptorSet[14].pTexelBufferView = nullptr;

    descriptorBufferInfo[15]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[15].buffer = m_vdata.m_pAccelStruct_m_instMatricesInvBuffer;
    descriptorBufferInfo[15].offset = m_vdata.m_pAccelStruct_m_instMatricesInvOffset;
    descriptorBufferInfo[15].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[15]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[15].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[15].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[15].dstBinding       = 15;
    writeDescriptorSet[15].descriptorCount  = 1;
    writeDescriptorSet[15].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[15].pBufferInfo      = &descriptorBufferInfo[15];
    writeDescriptorSet[15].pImageInfo       = nullptr;
    writeDescriptorSet[15].pTexelBufferView = nullptr;

    descriptorBufferInfo[16]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[16].buffer = m_vdata.m_pAccelStruct_m_SdfParametersBuffer;
    descriptorBufferInfo[16].offset = m_vdata.m_pAccelStruct_m_SdfParametersOffset;
    descriptorBufferInfo[16].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[16]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[16].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[16].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[16].dstBinding       = 16;
    writeDescriptorSet[16].descriptorCount  = 1;
    writeDescriptorSet[16].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[16].pBufferInfo      = &descriptorBufferInfo[16];
    writeDescriptorSet[16].pImageInfo       = nullptr;
    writeDescriptorSet[16].pTexelBufferView = nullptr;

    descriptorBufferInfo[17]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[17].buffer = m_vdata.m_pAccelStruct_m_nodesTLASBuffer;
    descriptorBufferInfo[17].offset = m_vdata.m_pAccelStruct_m_nodesTLASOffset;
    descriptorBufferInfo[17].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[17]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[17].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[17].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[17].dstBinding       = 17;
    writeDescriptorSet[17].descriptorCount  = 1;
    writeDescriptorSet[17].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[17].pBufferInfo      = &descriptorBufferInfo[17];
    writeDescriptorSet[17].pImageInfo       = nullptr;
    writeDescriptorSet[17].pTexelBufferView = nullptr;

    descriptorBufferInfo[18]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[18].buffer = m_vdata.m_pAccelStruct_m_primIndicesBuffer;
    descriptorBufferInfo[18].offset = m_vdata.m_pAccelStruct_m_primIndicesOffset;
    descriptorBufferInfo[18].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[18]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[18].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[18].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[18].dstBinding       = 18;
    writeDescriptorSet[18].descriptorCount  = 1;
    writeDescriptorSet[18].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[18].pBufferInfo      = &descriptorBufferInfo[18];
    writeDescriptorSet[18].pImageInfo       = nullptr;
    writeDescriptorSet[18].pTexelBufferView = nullptr;

    descriptorBufferInfo[19]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[19].buffer = m_vdata.m_pAccelStruct_m_SdfNeuralPropertiesBuffer;
    descriptorBufferInfo[19].offset = m_vdata.m_pAccelStruct_m_SdfNeuralPropertiesOffset;
    descriptorBufferInfo[19].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[19]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[19].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[19].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[19].dstBinding       = 19;
    writeDescriptorSet[19].descriptorCount  = 1;
    writeDescriptorSet[19].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[19].pBufferInfo      = &descriptorBufferInfo[19];
    writeDescriptorSet[19].pImageInfo       = nullptr;
    writeDescriptorSet[19].pTexelBufferView = nullptr;

    descriptorBufferInfo[20]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[20].buffer = m_vdata.m_pAccelStruct_m_SdfSVSRootsBuffer;
    descriptorBufferInfo[20].offset = m_vdata.m_pAccelStruct_m_SdfSVSRootsOffset;
    descriptorBufferInfo[20].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[20]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[20].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[20].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[20].dstBinding       = 20;
    writeDescriptorSet[20].descriptorCount  = 1;
    writeDescriptorSet[20].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[20].pBufferInfo      = &descriptorBufferInfo[20];
    writeDescriptorSet[20].pImageInfo       = nullptr;
    writeDescriptorSet[20].pTexelBufferView = nullptr;

    descriptorBufferInfo[21]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[21].buffer = m_vdata.m_pAccelStruct_m_SdfOctreeRootsBuffer;
    descriptorBufferInfo[21].offset = m_vdata.m_pAccelStruct_m_SdfOctreeRootsOffset;
    descriptorBufferInfo[21].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[21]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[21].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[21].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[21].dstBinding       = 21;
    writeDescriptorSet[21].descriptorCount  = 1;
    writeDescriptorSet[21].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[21].pBufferInfo      = &descriptorBufferInfo[21];
    writeDescriptorSet[21].pImageInfo       = nullptr;
    writeDescriptorSet[21].pTexelBufferView = nullptr;

    descriptorBufferInfo[22]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[22].buffer = m_vdata.m_packedXYBuffer;
    descriptorBufferInfo[22].offset = m_vdata.m_packedXYOffset;
    descriptorBufferInfo[22].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[22]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[22].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[22].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[22].dstBinding       = 22;
    writeDescriptorSet[22].descriptorCount  = 1;
    writeDescriptorSet[22].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[22].pBufferInfo      = &descriptorBufferInfo[22];
    writeDescriptorSet[22].pImageInfo       = nullptr;
    writeDescriptorSet[22].pTexelBufferView = nullptr;

    descriptorBufferInfo[23]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[23].buffer = m_vdata.m_pAccelStruct_m_bvhOffsetsBuffer;
    descriptorBufferInfo[23].offset = m_vdata.m_pAccelStruct_m_bvhOffsetsOffset;
    descriptorBufferInfo[23].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[23]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[23].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[23].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[23].dstBinding       = 23;
    writeDescriptorSet[23].descriptorCount  = 1;
    writeDescriptorSet[23].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[23].pBufferInfo      = &descriptorBufferInfo[23];
    writeDescriptorSet[23].pImageInfo       = nullptr;
    writeDescriptorSet[23].pTexelBufferView = nullptr;

    descriptorBufferInfo[24]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[24].buffer = m_vdata.m_pAccelStruct_m_RFGridScalesBuffer;
    descriptorBufferInfo[24].offset = m_vdata.m_pAccelStruct_m_RFGridScalesOffset;
    descriptorBufferInfo[24].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[24]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[24].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[24].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[24].dstBinding       = 24;
    writeDescriptorSet[24].descriptorCount  = 1;
    writeDescriptorSet[24].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[24].pBufferInfo      = &descriptorBufferInfo[24];
    writeDescriptorSet[24].pImageInfo       = nullptr;
    writeDescriptorSet[24].pTexelBufferView = nullptr;

    descriptorBufferInfo[25]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[25].buffer = m_vdata.m_pAccelStruct_m_RFGridSizesBuffer;
    descriptorBufferInfo[25].offset = m_vdata.m_pAccelStruct_m_RFGridSizesOffset;
    descriptorBufferInfo[25].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[25]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[25].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[25].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[25].dstBinding       = 25;
    writeDescriptorSet[25].descriptorCount  = 1;
    writeDescriptorSet[25].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[25].pBufferInfo      = &descriptorBufferInfo[25];
    writeDescriptorSet[25].pImageInfo       = nullptr;
    writeDescriptorSet[25].pTexelBufferView = nullptr;

    descriptorBufferInfo[26]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[26].buffer = m_vdata.m_pAccelStruct_m_vertPosBuffer;
    descriptorBufferInfo[26].offset = m_vdata.m_pAccelStruct_m_vertPosOffset;
    descriptorBufferInfo[26].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[26]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[26].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[26].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[26].dstBinding       = 26;
    writeDescriptorSet[26].descriptorCount  = 1;
    writeDescriptorSet[26].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[26].pBufferInfo      = &descriptorBufferInfo[26];
    writeDescriptorSet[26].pImageInfo       = nullptr;
    writeDescriptorSet[26].pTexelBufferView = nullptr;

    descriptorBufferInfo[27]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[27].buffer = m_vdata.m_pAccelStruct_m_SdfFrameOctreeNodesBuffer;
    descriptorBufferInfo[27].offset = m_vdata.m_pAccelStruct_m_SdfFrameOctreeNodesOffset;
    descriptorBufferInfo[27].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[27]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[27].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[27].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[27].dstBinding       = 27;
    writeDescriptorSet[27].descriptorCount  = 1;
    writeDescriptorSet[27].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[27].pBufferInfo      = &descriptorBufferInfo[27];
    writeDescriptorSet[27].pImageInfo       = nullptr;
    writeDescriptorSet[27].pTexelBufferView = nullptr;

    descriptorBufferInfo[28]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[28].buffer = m_vdata.m_pAccelStruct_m_geomTypeByGeomIdBuffer;
    descriptorBufferInfo[28].offset = m_vdata.m_pAccelStruct_m_geomTypeByGeomIdOffset;
    descriptorBufferInfo[28].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[28]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[28].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[28].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[28].dstBinding       = 28;
    writeDescriptorSet[28].descriptorCount  = 1;
    writeDescriptorSet[28].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[28].pBufferInfo      = &descriptorBufferInfo[28];
    writeDescriptorSet[28].pImageInfo       = nullptr;
    writeDescriptorSet[28].pTexelBufferView = nullptr;

    descriptorBufferInfo[29]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[29].buffer = m_vdata.m_pAccelStruct_m_origNodesBuffer;
    descriptorBufferInfo[29].offset = m_vdata.m_pAccelStruct_m_origNodesOffset;
    descriptorBufferInfo[29].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[29]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[29].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[29].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[29].dstBinding       = 29;
    writeDescriptorSet[29].descriptorCount  = 1;
    writeDescriptorSet[29].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[29].pBufferInfo      = &descriptorBufferInfo[29];
    writeDescriptorSet[29].pImageInfo       = nullptr;
    writeDescriptorSet[29].pTexelBufferView = nullptr;

    descriptorBufferInfo[30]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[30].buffer = m_vdata.m_pAccelStruct_m_SdfSBSHeadersBuffer;
    descriptorBufferInfo[30].offset = m_vdata.m_pAccelStruct_m_SdfSBSHeadersOffset;
    descriptorBufferInfo[30].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[30]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[30].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[30].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[30].dstBinding       = 30;
    writeDescriptorSet[30].descriptorCount  = 1;
    writeDescriptorSet[30].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[30].pBufferInfo      = &descriptorBufferInfo[30];
    writeDescriptorSet[30].pImageInfo       = nullptr;
    writeDescriptorSet[30].pTexelBufferView = nullptr;

    descriptorBufferInfo[31]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[31].buffer = m_vdata.m_pAccelStruct_m_SdfObjectsBuffer;
    descriptorBufferInfo[31].offset = m_vdata.m_pAccelStruct_m_SdfObjectsOffset;
    descriptorBufferInfo[31].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[31]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[31].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[31].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[31].dstBinding       = 31;
    writeDescriptorSet[31].descriptorCount  = 1;
    writeDescriptorSet[31].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[31].pBufferInfo      = &descriptorBufferInfo[31];
    writeDescriptorSet[31].pImageInfo       = nullptr;
    writeDescriptorSet[31].pTexelBufferView = nullptr;

    descriptorBufferInfo[32]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[32].buffer = m_vdata.m_pAccelStruct_m_indicesBuffer;
    descriptorBufferInfo[32].offset = m_vdata.m_pAccelStruct_m_indicesOffset;
    descriptorBufferInfo[32].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[32]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[32].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[32].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[32].dstBinding       = 32;
    writeDescriptorSet[32].descriptorCount  = 1;
    writeDescriptorSet[32].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[32].pBufferInfo      = &descriptorBufferInfo[32];
    writeDescriptorSet[32].pImageInfo       = nullptr;
    writeDescriptorSet[32].pTexelBufferView = nullptr;

    descriptorBufferInfo[33]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[33].buffer = m_vdata.m_pAccelStruct_m_SdfSBSRemapBuffer;
    descriptorBufferInfo[33].offset = m_vdata.m_pAccelStruct_m_SdfSBSRemapOffset;
    descriptorBufferInfo[33].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[33]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[33].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[33].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[33].dstBinding       = 33;
    writeDescriptorSet[33].descriptorCount  = 1;
    writeDescriptorSet[33].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[33].pBufferInfo      = &descriptorBufferInfo[33];
    writeDescriptorSet[33].pImageInfo       = nullptr;
    writeDescriptorSet[33].pTexelBufferView = nullptr;

    descriptorBufferInfo[34]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[34].buffer = m_classDataBuffer;
    descriptorBufferInfo[34].offset = 0;
    descriptorBufferInfo[34].range  = VK_WHOLE_SIZE;

    writeDescriptorSet[34]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[34].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[34].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[34].dstBinding       = 34;
    writeDescriptorSet[34].descriptorCount  = 1;
    writeDescriptorSet[34].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[34].pBufferInfo      = &descriptorBufferInfo[34];
    writeDescriptorSet[34].pImageInfo       = nullptr;
    writeDescriptorSet[34].pTexelBufferView = nullptr;

    vkUpdateDescriptorSets(device, uint32_t(writeDescriptorSet.size()), writeDescriptorSet.data(), 0, NULL);
  }
}



