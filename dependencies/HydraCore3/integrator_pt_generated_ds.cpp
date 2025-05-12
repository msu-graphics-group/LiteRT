#include <vector>
#include <array>
#include <memory>
#include <limits>

#include <cassert>
#include "vk_copy.h"
#include "vk_context.h"

#include "integrator_pt_generated.h"


void Integrator_Generated::AllocateAllDescriptorSets()
{
  // allocate pool
  //
  VkDescriptorPoolSize buffersSize, combinedImageSamSize, imageStorageSize;
  buffersSize.type                     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  buffersSize.descriptorCount          = 263 + 64; // + 64 for reserve

  std::vector<VkDescriptorPoolSize> poolSizes = {buffersSize};

  combinedImageSamSize.type            = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
  combinedImageSamSize.descriptorCount = 5*GetDefaultMaxTextures() + 0;

  imageStorageSize.type                = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
  imageStorageSize.descriptorCount     = 0;

  if(combinedImageSamSize.descriptorCount > 0)
    poolSizes.push_back(combinedImageSamSize);
  if(imageStorageSize.descriptorCount > 0)
    poolSizes.push_back(imageStorageSize);

  VkDescriptorPoolCreateInfo descriptorPoolCreateInfo = {};
  descriptorPoolCreateInfo.sType         = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
  descriptorPoolCreateInfo.maxSets       = 6 + 2; // add 1 to prevent zero case and one more for internal needs
  descriptorPoolCreateInfo.poolSizeCount = poolSizes.size();
  descriptorPoolCreateInfo.pPoolSizes    = poolSizes.data();

  VK_CHECK_RESULT(vkCreateDescriptorPool(device, &descriptorPoolCreateInfo, NULL, &m_dsPool));

  // allocate all descriptor sets
  //
  VkDescriptorSetLayout layouts[6] = {};
  layouts[0] = RayTraceMegaDSLayout;
  layouts[1] = CastSingleRayMegaDSLayout;
  layouts[2] = PackXYMegaDSLayout;
  layouts[3] = PathTraceFromInputRaysMegaDSLayout;
  layouts[4] = PathTraceMegaDSLayout;
  layouts[5] = NaivePathTraceMegaDSLayout;

  VkDescriptorSetAllocateInfo descriptorSetAllocateInfo = {};
  descriptorSetAllocateInfo.sType              = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
  descriptorSetAllocateInfo.descriptorPool     = m_dsPool;
  descriptorSetAllocateInfo.descriptorSetCount = 6;
  descriptorSetAllocateInfo.pSetLayouts        = layouts;

  auto tmpRes = vkAllocateDescriptorSets(device, &descriptorSetAllocateInfo, m_allGeneratedDS);
  VK_CHECK_RESULT(tmpRes);
}

VkDescriptorSetLayout Integrator_Generated::CreateRayTraceMegaDSLayout()
{
  std::array<VkDescriptorSetLayoutBinding, 46+1> dsBindings;
  
  const auto stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  // binding for out_color
  dsBindings[0].binding            = 0;
  dsBindings[0].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[0].descriptorCount    = 1;
  dsBindings[0].stageFlags         = stageFlags;
  dsBindings[0].pImmutableSamplers = nullptr;

  // binding for m_packedXY
  dsBindings[1].binding            = 1;
  dsBindings[1].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[1].descriptorCount    = 1;
  dsBindings[1].stageFlags         = stageFlags;
  dsBindings[1].pImmutableSamplers = nullptr;

  // binding for m_triIndices
  dsBindings[2].binding            = 2;
  dsBindings[2].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[2].descriptorCount    = 1;
  dsBindings[2].stageFlags         = stageFlags;
  dsBindings[2].pImmutableSamplers = nullptr;

  // binding for m_normMatrices2
  dsBindings[3].binding            = 3;
  dsBindings[3].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[3].descriptorCount    = 1;
  dsBindings[3].stageFlags         = stageFlags;
  dsBindings[3].pImmutableSamplers = nullptr;

  // binding for m_vTang4f
  dsBindings[4].binding            = 4;
  dsBindings[4].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[4].descriptorCount    = 1;
  dsBindings[4].stageFlags         = stageFlags;
  dsBindings[4].pImmutableSamplers = nullptr;

  // binding for m_matIdOffsets
  dsBindings[5].binding            = 5;
  dsBindings[5].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[5].descriptorCount    = 1;
  dsBindings[5].stageFlags         = stageFlags;
  dsBindings[5].pImmutableSamplers = nullptr;

  // binding for m_vNorm4f
  dsBindings[6].binding            = 6;
  dsBindings[6].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[6].descriptorCount    = 1;
  dsBindings[6].stageFlags         = stageFlags;
  dsBindings[6].pImmutableSamplers = nullptr;

  // binding for m_remapInst
  dsBindings[7].binding            = 7;
  dsBindings[7].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[7].descriptorCount    = 1;
  dsBindings[7].stageFlags         = stageFlags;
  dsBindings[7].pImmutableSamplers = nullptr;

  // binding for m_matIdByPrimId
  dsBindings[8].binding            = 8;
  dsBindings[8].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[8].descriptorCount    = 1;
  dsBindings[8].stageFlags         = stageFlags;
  dsBindings[8].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_abstractObjectPtrs
  dsBindings[9].binding            = 9;
  dsBindings[9].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[9].descriptorCount    = 1;
  dsBindings[9].stageFlags         = stageFlags;
  dsBindings[9].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_nodesTLAS
  dsBindings[10].binding            = 10;
  dsBindings[10].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[10].descriptorCount    = 1;
  dsBindings[10].stageFlags         = stageFlags;
  dsBindings[10].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_NURBSHeaders
  dsBindings[11].binding            = 11;
  dsBindings[11].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[11].descriptorCount    = 1;
  dsBindings[11].stageFlags         = stageFlags;
  dsBindings[11].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_primIndices
  dsBindings[12].binding            = 12;
  dsBindings[12].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[12].descriptorCount    = 1;
  dsBindings[12].stageFlags         = stageFlags;
  dsBindings[12].pImmutableSamplers = nullptr;

  // binding for m_vertOffset
  dsBindings[13].binding            = 13;
  dsBindings[13].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[13].descriptorCount    = 1;
  dsBindings[13].stageFlags         = stageFlags;
  dsBindings[13].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSBSData
  dsBindings[14].binding            = 14;
  dsBindings[14].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[14].descriptorCount    = 1;
  dsBindings[14].stageFlags         = stageFlags;
  dsBindings[14].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_NURBS_approxes
  dsBindings[15].binding            = 15;
  dsBindings[15].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[15].descriptorCount    = 1;
  dsBindings[15].stageFlags         = stageFlags;
  dsBindings[15].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_geomData
  dsBindings[16].binding            = 16;
  dsBindings[16].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[16].descriptorCount    = 1;
  dsBindings[16].stageFlags         = stageFlags;
  dsBindings[16].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfCompactOctreeRotModifiers
  dsBindings[17].binding            = 17;
  dsBindings[17].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[17].descriptorCount    = 1;
  dsBindings[17].stageFlags         = stageFlags;
  dsBindings[17].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSVSNodes
  dsBindings[18].binding            = 18;
  dsBindings[18].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[18].descriptorCount    = 1;
  dsBindings[18].stageFlags         = stageFlags;
  dsBindings[18].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfFrameOctreeRoots
  dsBindings[19].binding            = 19;
  dsBindings[19].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[19].descriptorCount    = 1;
  dsBindings[19].stageFlags         = stageFlags;
  dsBindings[19].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_CatmulClarkHeaders
  dsBindings[20].binding            = 20;
  dsBindings[20].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[20].descriptorCount    = 1;
  dsBindings[20].stageFlags         = stageFlags;
  dsBindings[20].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfCompactOctreeV2Data
  dsBindings[21].binding            = 21;
  dsBindings[21].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[21].descriptorCount    = 1;
  dsBindings[21].stageFlags         = stageFlags;
  dsBindings[21].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSVSRoots
  dsBindings[22].binding            = 22;
  dsBindings[22].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[22].descriptorCount    = 1;
  dsBindings[22].stageFlags         = stageFlags;
  dsBindings[22].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_primIdCount
  dsBindings[23].binding            = 23;
  dsBindings[23].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[23].descriptorCount    = 1;
  dsBindings[23].stageFlags         = stageFlags;
  dsBindings[23].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_allNodePairs
  dsBindings[24].binding            = 24;
  dsBindings[24].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[24].descriptorCount    = 1;
  dsBindings[24].stageFlags         = stageFlags;
  dsBindings[24].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_startEnd
  dsBindings[25].binding            = 25;
  dsBindings[25].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[25].descriptorCount    = 1;
  dsBindings[25].stageFlags         = stageFlags;
  dsBindings[25].pImmutableSamplers = nullptr;

  // binding for all_references
  dsBindings[26].binding            = 26;
  dsBindings[26].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[26].descriptorCount    = 1;
  dsBindings[26].stageFlags         = stageFlags;
  dsBindings[26].pImmutableSamplers = nullptr;

  // binding for m_allRemapLists
  dsBindings[27].binding            = 27;
  dsBindings[27].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[27].descriptorCount    = 1;
  dsBindings[27].stageFlags         = stageFlags;
  dsBindings[27].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_NURBSData
  dsBindings[28].binding            = 28;
  dsBindings[28].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[28].descriptorCount    = 1;
  dsBindings[28].stageFlags         = stageFlags;
  dsBindings[28].pImmutableSamplers = nullptr;

  // binding for m_allRemapListsOffsets
  dsBindings[29].binding            = 29;
  dsBindings[29].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[29].descriptorCount    = 1;
  dsBindings[29].stageFlags         = stageFlags;
  dsBindings[29].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_RibbonHeaders
  dsBindings[30].binding            = 30;
  dsBindings[30].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[30].descriptorCount    = 1;
  dsBindings[30].stageFlags         = stageFlags;
  dsBindings[30].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfCompactOctreeV3Data
  dsBindings[31].binding            = 31;
  dsBindings[31].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[31].descriptorCount    = 1;
  dsBindings[31].stageFlags         = stageFlags;
  dsBindings[31].pImmutableSamplers = nullptr;

  // binding for m_lights
  dsBindings[32].binding            = 32;
  dsBindings[32].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[32].descriptorCount    = 1;
  dsBindings[32].stageFlags         = stageFlags;
  dsBindings[32].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_origNodes
  dsBindings[33].binding            = 33;
  dsBindings[33].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[33].descriptorCount    = 1;
  dsBindings[33].stageFlags         = stageFlags;
  dsBindings[33].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSBSDataF
  dsBindings[34].binding            = 34;
  dsBindings[34].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[34].descriptorCount    = 1;
  dsBindings[34].stageFlags         = stageFlags;
  dsBindings[34].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSBSHeaders
  dsBindings[35].binding            = 35;
  dsBindings[35].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[35].descriptorCount    = 1;
  dsBindings[35].stageFlags         = stageFlags;
  dsBindings[35].pImmutableSamplers = nullptr;

  // binding for m_normMatrices
  dsBindings[36].binding            = 36;
  dsBindings[36].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[36].descriptorCount    = 1;
  dsBindings[36].stageFlags         = stageFlags;
  dsBindings[36].pImmutableSamplers = nullptr;

  // binding for m_textures
  dsBindings[37].binding            = 37;
  dsBindings[37].descriptorType     = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
  m_vdata.m_texturesArrayMaxSize = m_textures.size();
  if(m_vdata.m_texturesArrayMaxSize == 0)
    m_vdata.m_texturesArrayMaxSize = GetDefaultMaxTextures();
  dsBindings[37].descriptorCount    = m_vdata.m_texturesArrayMaxSize;
  dsBindings[37].stageFlags         = stageFlags;
  dsBindings[37].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSBSRoots
  dsBindings[38].binding            = 38;
  dsBindings[38].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[38].descriptorCount    = 1;
  dsBindings[38].stageFlags         = stageFlags;
  dsBindings[38].pImmutableSamplers = nullptr;

  // binding for m_materials
  dsBindings[39].binding            = 39;
  dsBindings[39].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[39].descriptorCount    = 1;
  dsBindings[39].stageFlags         = stageFlags;
  dsBindings[39].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_indices
  dsBindings[40].binding            = 40;
  dsBindings[40].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[40].descriptorCount    = 1;
  dsBindings[40].stageFlags         = stageFlags;
  dsBindings[40].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSBSNodes
  dsBindings[41].binding            = 41;
  dsBindings[41].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[41].descriptorCount    = 1;
  dsBindings[41].stageFlags         = stageFlags;
  dsBindings[41].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_instanceData
  dsBindings[42].binding            = 42;
  dsBindings[42].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[42].descriptorCount    = 1;
  dsBindings[42].stageFlags         = stageFlags;
  dsBindings[42].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_vertNorm
  dsBindings[43].binding            = 43;
  dsBindings[43].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[43].descriptorCount    = 1;
  dsBindings[43].stageFlags         = stageFlags;
  dsBindings[43].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfFrameOctreeNodes
  dsBindings[44].binding            = 44;
  dsBindings[44].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[44].descriptorCount    = 1;
  dsBindings[44].stageFlags         = stageFlags;
  dsBindings[44].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_vertPos
  dsBindings[45].binding            = 45;
  dsBindings[45].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[45].descriptorCount    = 1;
  dsBindings[45].stageFlags         = stageFlags;
  dsBindings[45].pImmutableSamplers = nullptr;


  dsBindings[46].binding            = 46;
  dsBindings[46].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[46].descriptorCount    = 1;
  dsBindings[46].stageFlags         = stageFlags;
  dsBindings[46].pImmutableSamplers = nullptr;


  VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCreateInfo = {};
  descriptorSetLayoutCreateInfo.sType        = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
  descriptorSetLayoutCreateInfo.bindingCount = uint32_t(dsBindings.size());
  descriptorSetLayoutCreateInfo.pBindings    = dsBindings.data();

  VkDescriptorSetLayout layout = nullptr;
  VK_CHECK_RESULT(vkCreateDescriptorSetLayout(device, &descriptorSetLayoutCreateInfo, NULL, &layout));
  return layout;
}
VkDescriptorSetLayout Integrator_Generated::CreateCastSingleRayMegaDSLayout()
{
  std::array<VkDescriptorSetLayoutBinding, 40+1> dsBindings;
  
  const auto stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  // binding for out_color
  dsBindings[0].binding            = 0;
  dsBindings[0].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[0].descriptorCount    = 1;
  dsBindings[0].stageFlags         = stageFlags;
  dsBindings[0].pImmutableSamplers = nullptr;

  // binding for m_packedXY
  dsBindings[1].binding            = 1;
  dsBindings[1].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[1].descriptorCount    = 1;
  dsBindings[1].stageFlags         = stageFlags;
  dsBindings[1].pImmutableSamplers = nullptr;

  // binding for m_matIdByPrimId
  dsBindings[2].binding            = 2;
  dsBindings[2].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[2].descriptorCount    = 1;
  dsBindings[2].stageFlags         = stageFlags;
  dsBindings[2].pImmutableSamplers = nullptr;

  // binding for m_materials
  dsBindings[3].binding            = 3;
  dsBindings[3].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[3].descriptorCount    = 1;
  dsBindings[3].stageFlags         = stageFlags;
  dsBindings[3].pImmutableSamplers = nullptr;

  // binding for m_triIndices
  dsBindings[4].binding            = 4;
  dsBindings[4].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[4].descriptorCount    = 1;
  dsBindings[4].stageFlags         = stageFlags;
  dsBindings[4].pImmutableSamplers = nullptr;

  // binding for m_matIdOffsets
  dsBindings[5].binding            = 5;
  dsBindings[5].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[5].descriptorCount    = 1;
  dsBindings[5].stageFlags         = stageFlags;
  dsBindings[5].pImmutableSamplers = nullptr;

  // binding for m_vTang4f
  dsBindings[6].binding            = 6;
  dsBindings[6].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[6].descriptorCount    = 1;
  dsBindings[6].stageFlags         = stageFlags;
  dsBindings[6].pImmutableSamplers = nullptr;

  // binding for m_textures
  dsBindings[7].binding            = 7;
  dsBindings[7].descriptorType     = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
  m_vdata.m_texturesArrayMaxSize = m_textures.size();
  if(m_vdata.m_texturesArrayMaxSize == 0)
    m_vdata.m_texturesArrayMaxSize = GetDefaultMaxTextures();
  dsBindings[7].descriptorCount    = m_vdata.m_texturesArrayMaxSize;
  dsBindings[7].stageFlags         = stageFlags;
  dsBindings[7].pImmutableSamplers = nullptr;

  // binding for m_vNorm4f
  dsBindings[8].binding            = 8;
  dsBindings[8].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[8].descriptorCount    = 1;
  dsBindings[8].stageFlags         = stageFlags;
  dsBindings[8].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSBSRoots
  dsBindings[9].binding            = 9;
  dsBindings[9].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[9].descriptorCount    = 1;
  dsBindings[9].stageFlags         = stageFlags;
  dsBindings[9].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSBSHeaders
  dsBindings[10].binding            = 10;
  dsBindings[10].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[10].descriptorCount    = 1;
  dsBindings[10].stageFlags         = stageFlags;
  dsBindings[10].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_origNodes
  dsBindings[11].binding            = 11;
  dsBindings[11].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[11].descriptorCount    = 1;
  dsBindings[11].stageFlags         = stageFlags;
  dsBindings[11].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSBSDataF
  dsBindings[12].binding            = 12;
  dsBindings[12].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[12].descriptorCount    = 1;
  dsBindings[12].stageFlags         = stageFlags;
  dsBindings[12].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_NURBSData
  dsBindings[13].binding            = 13;
  dsBindings[13].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[13].descriptorCount    = 1;
  dsBindings[13].stageFlags         = stageFlags;
  dsBindings[13].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfCompactOctreeRotModifiers
  dsBindings[14].binding            = 14;
  dsBindings[14].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[14].descriptorCount    = 1;
  dsBindings[14].stageFlags         = stageFlags;
  dsBindings[14].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfCompactOctreeV2Data
  dsBindings[15].binding            = 15;
  dsBindings[15].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[15].descriptorCount    = 1;
  dsBindings[15].stageFlags         = stageFlags;
  dsBindings[15].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSVSRoots
  dsBindings[16].binding            = 16;
  dsBindings[16].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[16].descriptorCount    = 1;
  dsBindings[16].stageFlags         = stageFlags;
  dsBindings[16].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_CatmulClarkHeaders
  dsBindings[17].binding            = 17;
  dsBindings[17].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[17].descriptorCount    = 1;
  dsBindings[17].stageFlags         = stageFlags;
  dsBindings[17].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfFrameOctreeRoots
  dsBindings[18].binding            = 18;
  dsBindings[18].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[18].descriptorCount    = 1;
  dsBindings[18].stageFlags         = stageFlags;
  dsBindings[18].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSVSNodes
  dsBindings[19].binding            = 19;
  dsBindings[19].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[19].descriptorCount    = 1;
  dsBindings[19].stageFlags         = stageFlags;
  dsBindings[19].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_RibbonHeaders
  dsBindings[20].binding            = 20;
  dsBindings[20].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[20].descriptorCount    = 1;
  dsBindings[20].stageFlags         = stageFlags;
  dsBindings[20].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfCompactOctreeV3Data
  dsBindings[21].binding            = 21;
  dsBindings[21].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[21].descriptorCount    = 1;
  dsBindings[21].stageFlags         = stageFlags;
  dsBindings[21].pImmutableSamplers = nullptr;

  // binding for all_references
  dsBindings[22].binding            = 22;
  dsBindings[22].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[22].descriptorCount    = 1;
  dsBindings[22].stageFlags         = stageFlags;
  dsBindings[22].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSBSNodes
  dsBindings[23].binding            = 23;
  dsBindings[23].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[23].descriptorCount    = 1;
  dsBindings[23].stageFlags         = stageFlags;
  dsBindings[23].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_geomData
  dsBindings[24].binding            = 24;
  dsBindings[24].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[24].descriptorCount    = 1;
  dsBindings[24].stageFlags         = stageFlags;
  dsBindings[24].pImmutableSamplers = nullptr;

  // binding for m_vertOffset
  dsBindings[25].binding            = 25;
  dsBindings[25].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[25].descriptorCount    = 1;
  dsBindings[25].stageFlags         = stageFlags;
  dsBindings[25].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSBSData
  dsBindings[26].binding            = 26;
  dsBindings[26].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[26].descriptorCount    = 1;
  dsBindings[26].stageFlags         = stageFlags;
  dsBindings[26].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_NURBS_approxes
  dsBindings[27].binding            = 27;
  dsBindings[27].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[27].descriptorCount    = 1;
  dsBindings[27].stageFlags         = stageFlags;
  dsBindings[27].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_abstractObjectPtrs
  dsBindings[28].binding            = 28;
  dsBindings[28].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[28].descriptorCount    = 1;
  dsBindings[28].stageFlags         = stageFlags;
  dsBindings[28].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_primIdCount
  dsBindings[29].binding            = 29;
  dsBindings[29].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[29].descriptorCount    = 1;
  dsBindings[29].stageFlags         = stageFlags;
  dsBindings[29].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_allNodePairs
  dsBindings[30].binding            = 30;
  dsBindings[30].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[30].descriptorCount    = 1;
  dsBindings[30].stageFlags         = stageFlags;
  dsBindings[30].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_startEnd
  dsBindings[31].binding            = 31;
  dsBindings[31].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[31].descriptorCount    = 1;
  dsBindings[31].stageFlags         = stageFlags;
  dsBindings[31].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_instanceData
  dsBindings[32].binding            = 32;
  dsBindings[32].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[32].descriptorCount    = 1;
  dsBindings[32].stageFlags         = stageFlags;
  dsBindings[32].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_primIndices
  dsBindings[33].binding            = 33;
  dsBindings[33].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[33].descriptorCount    = 1;
  dsBindings[33].stageFlags         = stageFlags;
  dsBindings[33].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_NURBSHeaders
  dsBindings[34].binding            = 34;
  dsBindings[34].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[34].descriptorCount    = 1;
  dsBindings[34].stageFlags         = stageFlags;
  dsBindings[34].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_nodesTLAS
  dsBindings[35].binding            = 35;
  dsBindings[35].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[35].descriptorCount    = 1;
  dsBindings[35].stageFlags         = stageFlags;
  dsBindings[35].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_vertNorm
  dsBindings[36].binding            = 36;
  dsBindings[36].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[36].descriptorCount    = 1;
  dsBindings[36].stageFlags         = stageFlags;
  dsBindings[36].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfFrameOctreeNodes
  dsBindings[37].binding            = 37;
  dsBindings[37].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[37].descriptorCount    = 1;
  dsBindings[37].stageFlags         = stageFlags;
  dsBindings[37].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_vertPos
  dsBindings[38].binding            = 38;
  dsBindings[38].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[38].descriptorCount    = 1;
  dsBindings[38].stageFlags         = stageFlags;
  dsBindings[38].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_indices
  dsBindings[39].binding            = 39;
  dsBindings[39].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[39].descriptorCount    = 1;
  dsBindings[39].stageFlags         = stageFlags;
  dsBindings[39].pImmutableSamplers = nullptr;


  dsBindings[40].binding            = 40;
  dsBindings[40].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[40].descriptorCount    = 1;
  dsBindings[40].stageFlags         = stageFlags;
  dsBindings[40].pImmutableSamplers = nullptr;


  VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCreateInfo = {};
  descriptorSetLayoutCreateInfo.sType        = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
  descriptorSetLayoutCreateInfo.bindingCount = uint32_t(dsBindings.size());
  descriptorSetLayoutCreateInfo.pBindings    = dsBindings.data();

  VkDescriptorSetLayout layout = nullptr;
  VK_CHECK_RESULT(vkCreateDescriptorSetLayout(device, &descriptorSetLayoutCreateInfo, NULL, &layout));
  return layout;
}
VkDescriptorSetLayout Integrator_Generated::CreatePackXYMegaDSLayout()
{
  std::array<VkDescriptorSetLayoutBinding, 1+1> dsBindings;
  
  const auto stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  // binding for m_packedXY
  dsBindings[0].binding            = 0;
  dsBindings[0].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[0].descriptorCount    = 1;
  dsBindings[0].stageFlags         = stageFlags;
  dsBindings[0].pImmutableSamplers = nullptr;


  dsBindings[1].binding            = 1;
  dsBindings[1].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[1].descriptorCount    = 1;
  dsBindings[1].stageFlags         = stageFlags;
  dsBindings[1].pImmutableSamplers = nullptr;


  VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCreateInfo = {};
  descriptorSetLayoutCreateInfo.sType        = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
  descriptorSetLayoutCreateInfo.bindingCount = uint32_t(dsBindings.size());
  descriptorSetLayoutCreateInfo.pBindings    = dsBindings.data();

  VkDescriptorSetLayout layout = nullptr;
  VK_CHECK_RESULT(vkCreateDescriptorSetLayout(device, &descriptorSetLayoutCreateInfo, NULL, &layout));
  return layout;
}
VkDescriptorSetLayout Integrator_Generated::CreatePathTraceFromInputRaysMegaDSLayout()
{
  std::array<VkDescriptorSetLayoutBinding, 59+1> dsBindings;
  
  const auto stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  // binding for in_rayPosAndNear
  dsBindings[0].binding            = 0;
  dsBindings[0].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[0].descriptorCount    = 1;
  dsBindings[0].stageFlags         = stageFlags;
  dsBindings[0].pImmutableSamplers = nullptr;

  // binding for in_rayDirAndFar
  dsBindings[1].binding            = 1;
  dsBindings[1].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[1].descriptorCount    = 1;
  dsBindings[1].stageFlags         = stageFlags;
  dsBindings[1].pImmutableSamplers = nullptr;

  // binding for out_color
  dsBindings[2].binding            = 2;
  dsBindings[2].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[2].descriptorCount    = 1;
  dsBindings[2].stageFlags         = stageFlags;
  dsBindings[2].pImmutableSamplers = nullptr;

  // binding for m_packedXY
  dsBindings[3].binding            = 3;
  dsBindings[3].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[3].descriptorCount    = 1;
  dsBindings[3].stageFlags         = stageFlags;
  dsBindings[3].pImmutableSamplers = nullptr;

  // binding for m_randomGens
  dsBindings[4].binding            = 4;
  dsBindings[4].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[4].descriptorCount    = 1;
  dsBindings[4].stageFlags         = stageFlags;
  dsBindings[4].pImmutableSamplers = nullptr;

  // binding for m_pdfLightData
  dsBindings[5].binding            = 5;
  dsBindings[5].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[5].descriptorCount    = 1;
  dsBindings[5].stageFlags         = stageFlags;
  dsBindings[5].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_primIndices
  dsBindings[6].binding            = 6;
  dsBindings[6].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[6].descriptorCount    = 1;
  dsBindings[6].stageFlags         = stageFlags;
  dsBindings[6].pImmutableSamplers = nullptr;

  // binding for m_vTang4f
  dsBindings[7].binding            = 7;
  dsBindings[7].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[7].descriptorCount    = 1;
  dsBindings[7].stageFlags         = stageFlags;
  dsBindings[7].pImmutableSamplers = nullptr;

  // binding for m_matIdOffsets
  dsBindings[8].binding            = 8;
  dsBindings[8].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[8].descriptorCount    = 1;
  dsBindings[8].stageFlags         = stageFlags;
  dsBindings[8].pImmutableSamplers = nullptr;

  // binding for m_vNorm4f
  dsBindings[9].binding            = 9;
  dsBindings[9].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[9].descriptorCount    = 1;
  dsBindings[9].stageFlags         = stageFlags;
  dsBindings[9].pImmutableSamplers = nullptr;

  // binding for m_remapInst
  dsBindings[10].binding            = 10;
  dsBindings[10].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[10].descriptorCount    = 1;
  dsBindings[10].stageFlags         = stageFlags;
  dsBindings[10].pImmutableSamplers = nullptr;

  // binding for m_matIdByPrimId
  dsBindings[11].binding            = 11;
  dsBindings[11].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[11].descriptorCount    = 1;
  dsBindings[11].stageFlags         = stageFlags;
  dsBindings[11].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_abstractObjectPtrs
  dsBindings[12].binding            = 12;
  dsBindings[12].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[12].descriptorCount    = 1;
  dsBindings[12].stageFlags         = stageFlags;
  dsBindings[12].pImmutableSamplers = nullptr;

  // binding for m_triIndices
  dsBindings[13].binding            = 13;
  dsBindings[13].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[13].descriptorCount    = 1;
  dsBindings[13].stageFlags         = stageFlags;
  dsBindings[13].pImmutableSamplers = nullptr;

  // binding for m_normMatrices2
  dsBindings[14].binding            = 14;
  dsBindings[14].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[14].descriptorCount    = 1;
  dsBindings[14].stageFlags         = stageFlags;
  dsBindings[14].pImmutableSamplers = nullptr;

  // binding for m_films_eta_k_vec
  dsBindings[15].binding            = 15;
  dsBindings[15].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[15].descriptorCount    = 1;
  dsBindings[15].stageFlags         = stageFlags;
  dsBindings[15].pImmutableSamplers = nullptr;

  // binding for m_spec_offset_sz
  dsBindings[16].binding            = 16;
  dsBindings[16].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[16].descriptorCount    = 1;
  dsBindings[16].stageFlags         = stageFlags;
  dsBindings[16].pImmutableSamplers = nullptr;

  // binding for all_references
  dsBindings[17].binding            = 17;
  dsBindings[17].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[17].descriptorCount    = 1;
  dsBindings[17].stageFlags         = stageFlags;
  dsBindings[17].pImmutableSamplers = nullptr;

  // binding for m_spec_tex_offset_sz
  dsBindings[18].binding            = 18;
  dsBindings[18].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[18].descriptorCount    = 1;
  dsBindings[18].stageFlags         = stageFlags;
  dsBindings[18].pImmutableSamplers = nullptr;

  // binding for m_spec_values
  dsBindings[19].binding            = 19;
  dsBindings[19].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[19].descriptorCount    = 1;
  dsBindings[19].stageFlags         = stageFlags;
  dsBindings[19].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_origNodes
  dsBindings[20].binding            = 20;
  dsBindings[20].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[20].descriptorCount    = 1;
  dsBindings[20].stageFlags         = stageFlags;
  dsBindings[20].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_vertPos
  dsBindings[21].binding            = 21;
  dsBindings[21].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[21].descriptorCount    = 1;
  dsBindings[21].stageFlags         = stageFlags;
  dsBindings[21].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfFrameOctreeNodes
  dsBindings[22].binding            = 22;
  dsBindings[22].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[22].descriptorCount    = 1;
  dsBindings[22].stageFlags         = stageFlags;
  dsBindings[22].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_vertNorm
  dsBindings[23].binding            = 23;
  dsBindings[23].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[23].descriptorCount    = 1;
  dsBindings[23].stageFlags         = stageFlags;
  dsBindings[23].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfCompactOctreeV2Data
  dsBindings[24].binding            = 24;
  dsBindings[24].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[24].descriptorCount    = 1;
  dsBindings[24].stageFlags         = stageFlags;
  dsBindings[24].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfCompactOctreeRotModifiers
  dsBindings[25].binding            = 25;
  dsBindings[25].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[25].descriptorCount    = 1;
  dsBindings[25].stageFlags         = stageFlags;
  dsBindings[25].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSBSHeaders
  dsBindings[26].binding            = 26;
  dsBindings[26].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[26].descriptorCount    = 1;
  dsBindings[26].stageFlags         = stageFlags;
  dsBindings[26].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSBSDataF
  dsBindings[27].binding            = 27;
  dsBindings[27].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[27].descriptorCount    = 1;
  dsBindings[27].stageFlags         = stageFlags;
  dsBindings[27].pImmutableSamplers = nullptr;

  // binding for m_lights
  dsBindings[28].binding            = 28;
  dsBindings[28].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[28].descriptorCount    = 1;
  dsBindings[28].stageFlags         = stageFlags;
  dsBindings[28].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_CatmulClarkHeaders
  dsBindings[29].binding            = 29;
  dsBindings[29].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[29].descriptorCount    = 1;
  dsBindings[29].stageFlags         = stageFlags;
  dsBindings[29].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfFrameOctreeRoots
  dsBindings[30].binding            = 30;
  dsBindings[30].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[30].descriptorCount    = 1;
  dsBindings[30].stageFlags         = stageFlags;
  dsBindings[30].pImmutableSamplers = nullptr;

  // binding for m_spec_tex_ids_wavelengths
  dsBindings[31].binding            = 31;
  dsBindings[31].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[31].descriptorCount    = 1;
  dsBindings[31].stageFlags         = stageFlags;
  dsBindings[31].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_primIdCount
  dsBindings[32].binding            = 32;
  dsBindings[32].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[32].descriptorCount    = 1;
  dsBindings[32].stageFlags         = stageFlags;
  dsBindings[32].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSVSNodes
  dsBindings[33].binding            = 33;
  dsBindings[33].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[33].descriptorCount    = 1;
  dsBindings[33].stageFlags         = stageFlags;
  dsBindings[33].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_allNodePairs
  dsBindings[34].binding            = 34;
  dsBindings[34].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[34].descriptorCount    = 1;
  dsBindings[34].stageFlags         = stageFlags;
  dsBindings[34].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSVSRoots
  dsBindings[35].binding            = 35;
  dsBindings[35].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[35].descriptorCount    = 1;
  dsBindings[35].stageFlags         = stageFlags;
  dsBindings[35].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_geomData
  dsBindings[36].binding            = 36;
  dsBindings[36].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[36].descriptorCount    = 1;
  dsBindings[36].stageFlags         = stageFlags;
  dsBindings[36].pImmutableSamplers = nullptr;

  // binding for m_vertOffset
  dsBindings[37].binding            = 37;
  dsBindings[37].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[37].descriptorCount    = 1;
  dsBindings[37].stageFlags         = stageFlags;
  dsBindings[37].pImmutableSamplers = nullptr;

  // binding for m_films_spec_id_vec
  dsBindings[38].binding            = 38;
  dsBindings[38].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[38].descriptorCount    = 1;
  dsBindings[38].stageFlags         = stageFlags;
  dsBindings[38].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSBSData
  dsBindings[39].binding            = 39;
  dsBindings[39].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[39].descriptorCount    = 1;
  dsBindings[39].stageFlags         = stageFlags;
  dsBindings[39].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_NURBS_approxes
  dsBindings[40].binding            = 40;
  dsBindings[40].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[40].descriptorCount    = 1;
  dsBindings[40].stageFlags         = stageFlags;
  dsBindings[40].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_startEnd
  dsBindings[41].binding            = 41;
  dsBindings[41].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[41].descriptorCount    = 1;
  dsBindings[41].stageFlags         = stageFlags;
  dsBindings[41].pImmutableSamplers = nullptr;

  // binding for m_allRemapLists
  dsBindings[42].binding            = 42;
  dsBindings[42].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[42].descriptorCount    = 1;
  dsBindings[42].stageFlags         = stageFlags;
  dsBindings[42].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_NURBSData
  dsBindings[43].binding            = 43;
  dsBindings[43].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[43].descriptorCount    = 1;
  dsBindings[43].stageFlags         = stageFlags;
  dsBindings[43].pImmutableSamplers = nullptr;

  // binding for m_allRemapListsOffsets
  dsBindings[44].binding            = 44;
  dsBindings[44].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[44].descriptorCount    = 1;
  dsBindings[44].stageFlags         = stageFlags;
  dsBindings[44].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_RibbonHeaders
  dsBindings[45].binding            = 45;
  dsBindings[45].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[45].descriptorCount    = 1;
  dsBindings[45].stageFlags         = stageFlags;
  dsBindings[45].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfCompactOctreeV3Data
  dsBindings[46].binding            = 46;
  dsBindings[46].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[46].descriptorCount    = 1;
  dsBindings[46].stageFlags         = stageFlags;
  dsBindings[46].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSBSNodes
  dsBindings[47].binding            = 47;
  dsBindings[47].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[47].descriptorCount    = 1;
  dsBindings[47].stageFlags         = stageFlags;
  dsBindings[47].pImmutableSamplers = nullptr;

  // binding for m_instIdToLightInstId
  dsBindings[48].binding            = 48;
  dsBindings[48].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[48].descriptorCount    = 1;
  dsBindings[48].stageFlags         = stageFlags;
  dsBindings[48].pImmutableSamplers = nullptr;

  // binding for m_normMatrices
  dsBindings[49].binding            = 49;
  dsBindings[49].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[49].descriptorCount    = 1;
  dsBindings[49].stageFlags         = stageFlags;
  dsBindings[49].pImmutableSamplers = nullptr;

  // binding for m_textures
  dsBindings[50].binding            = 50;
  dsBindings[50].descriptorType     = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
  m_vdata.m_texturesArrayMaxSize = m_textures.size();
  if(m_vdata.m_texturesArrayMaxSize == 0)
    m_vdata.m_texturesArrayMaxSize = GetDefaultMaxTextures();
  dsBindings[50].descriptorCount    = m_vdata.m_texturesArrayMaxSize;
  dsBindings[50].stageFlags         = stageFlags;
  dsBindings[50].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSBSRoots
  dsBindings[51].binding            = 51;
  dsBindings[51].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[51].descriptorCount    = 1;
  dsBindings[51].stageFlags         = stageFlags;
  dsBindings[51].pImmutableSamplers = nullptr;

  // binding for m_materials
  dsBindings[52].binding            = 52;
  dsBindings[52].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[52].descriptorCount    = 1;
  dsBindings[52].stageFlags         = stageFlags;
  dsBindings[52].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_indices
  dsBindings[53].binding            = 53;
  dsBindings[53].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[53].descriptorCount    = 1;
  dsBindings[53].stageFlags         = stageFlags;
  dsBindings[53].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_instanceData
  dsBindings[54].binding            = 54;
  dsBindings[54].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[54].descriptorCount    = 1;
  dsBindings[54].stageFlags         = stageFlags;
  dsBindings[54].pImmutableSamplers = nullptr;

  // binding for m_precomp_thin_films
  dsBindings[55].binding            = 55;
  dsBindings[55].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[55].descriptorCount    = 1;
  dsBindings[55].stageFlags         = stageFlags;
  dsBindings[55].pImmutableSamplers = nullptr;

  // binding for m_precomp_coat_transmittance
  dsBindings[56].binding            = 56;
  dsBindings[56].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[56].descriptorCount    = 1;
  dsBindings[56].stageFlags         = stageFlags;
  dsBindings[56].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_nodesTLAS
  dsBindings[57].binding            = 57;
  dsBindings[57].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[57].descriptorCount    = 1;
  dsBindings[57].stageFlags         = stageFlags;
  dsBindings[57].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_NURBSHeaders
  dsBindings[58].binding            = 58;
  dsBindings[58].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[58].descriptorCount    = 1;
  dsBindings[58].stageFlags         = stageFlags;
  dsBindings[58].pImmutableSamplers = nullptr;


  dsBindings[59].binding            = 59;
  dsBindings[59].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[59].descriptorCount    = 1;
  dsBindings[59].stageFlags         = stageFlags;
  dsBindings[59].pImmutableSamplers = nullptr;


  VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCreateInfo = {};
  descriptorSetLayoutCreateInfo.sType        = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
  descriptorSetLayoutCreateInfo.bindingCount = uint32_t(dsBindings.size());
  descriptorSetLayoutCreateInfo.pBindings    = dsBindings.data();

  VkDescriptorSetLayout layout = nullptr;
  VK_CHECK_RESULT(vkCreateDescriptorSetLayout(device, &descriptorSetLayoutCreateInfo, NULL, &layout));
  return layout;
}
VkDescriptorSetLayout Integrator_Generated::CreatePathTraceMegaDSLayout()
{
  std::array<VkDescriptorSetLayoutBinding, 61+1> dsBindings;
  
  const auto stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  // binding for out_color
  dsBindings[0].binding            = 0;
  dsBindings[0].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[0].descriptorCount    = 1;
  dsBindings[0].stageFlags         = stageFlags;
  dsBindings[0].pImmutableSamplers = nullptr;

  // binding for m_lines
  dsBindings[1].binding            = 1;
  dsBindings[1].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[1].descriptorCount    = 1;
  dsBindings[1].stageFlags         = stageFlags;
  dsBindings[1].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_NURBSHeaders
  dsBindings[2].binding            = 2;
  dsBindings[2].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[2].descriptorCount    = 1;
  dsBindings[2].stageFlags         = stageFlags;
  dsBindings[2].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_primIndices
  dsBindings[3].binding            = 3;
  dsBindings[3].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[3].descriptorCount    = 1;
  dsBindings[3].stageFlags         = stageFlags;
  dsBindings[3].pImmutableSamplers = nullptr;

  // binding for m_precomp_thin_films
  dsBindings[4].binding            = 4;
  dsBindings[4].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[4].descriptorCount    = 1;
  dsBindings[4].stageFlags         = stageFlags;
  dsBindings[4].pImmutableSamplers = nullptr;

  // binding for m_films_spec_id_vec
  dsBindings[5].binding            = 5;
  dsBindings[5].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[5].descriptorCount    = 1;
  dsBindings[5].stageFlags         = stageFlags;
  dsBindings[5].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSBSData
  dsBindings[6].binding            = 6;
  dsBindings[6].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[6].descriptorCount    = 1;
  dsBindings[6].stageFlags         = stageFlags;
  dsBindings[6].pImmutableSamplers = nullptr;

  // binding for m_vertOffset
  dsBindings[7].binding            = 7;
  dsBindings[7].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[7].descriptorCount    = 1;
  dsBindings[7].stageFlags         = stageFlags;
  dsBindings[7].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfCompactOctreeRotModifiers
  dsBindings[8].binding            = 8;
  dsBindings[8].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[8].descriptorCount    = 1;
  dsBindings[8].stageFlags         = stageFlags;
  dsBindings[8].pImmutableSamplers = nullptr;

  // binding for m_cie_y
  dsBindings[9].binding            = 9;
  dsBindings[9].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[9].descriptorCount    = 1;
  dsBindings[9].stageFlags         = stageFlags;
  dsBindings[9].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfCompactOctreeV3Data
  dsBindings[10].binding            = 10;
  dsBindings[10].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[10].descriptorCount    = 1;
  dsBindings[10].stageFlags         = stageFlags;
  dsBindings[10].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_RibbonHeaders
  dsBindings[11].binding            = 11;
  dsBindings[11].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[11].descriptorCount    = 1;
  dsBindings[11].stageFlags         = stageFlags;
  dsBindings[11].pImmutableSamplers = nullptr;

  // binding for m_allRemapListsOffsets
  dsBindings[12].binding            = 12;
  dsBindings[12].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[12].descriptorCount    = 1;
  dsBindings[12].stageFlags         = stageFlags;
  dsBindings[12].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_indices
  dsBindings[13].binding            = 13;
  dsBindings[13].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[13].descriptorCount    = 1;
  dsBindings[13].stageFlags         = stageFlags;
  dsBindings[13].pImmutableSamplers = nullptr;

  // binding for m_materials
  dsBindings[14].binding            = 14;
  dsBindings[14].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[14].descriptorCount    = 1;
  dsBindings[14].stageFlags         = stageFlags;
  dsBindings[14].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSVSRoots
  dsBindings[15].binding            = 15;
  dsBindings[15].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[15].descriptorCount    = 1;
  dsBindings[15].stageFlags         = stageFlags;
  dsBindings[15].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSVSNodes
  dsBindings[16].binding            = 16;
  dsBindings[16].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[16].descriptorCount    = 1;
  dsBindings[16].stageFlags         = stageFlags;
  dsBindings[16].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfCompactOctreeV2Data
  dsBindings[17].binding            = 17;
  dsBindings[17].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[17].descriptorCount    = 1;
  dsBindings[17].stageFlags         = stageFlags;
  dsBindings[17].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfFrameOctreeRoots
  dsBindings[18].binding            = 18;
  dsBindings[18].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[18].descriptorCount    = 1;
  dsBindings[18].stageFlags         = stageFlags;
  dsBindings[18].pImmutableSamplers = nullptr;

  // binding for m_pdfLightData
  dsBindings[19].binding            = 19;
  dsBindings[19].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[19].descriptorCount    = 1;
  dsBindings[19].stageFlags         = stageFlags;
  dsBindings[19].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_primIdCount
  dsBindings[20].binding            = 20;
  dsBindings[20].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[20].descriptorCount    = 1;
  dsBindings[20].stageFlags         = stageFlags;
  dsBindings[20].pImmutableSamplers = nullptr;

  // binding for m_spec_tex_ids_wavelengths
  dsBindings[21].binding            = 21;
  dsBindings[21].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[21].descriptorCount    = 1;
  dsBindings[21].stageFlags         = stageFlags;
  dsBindings[21].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_geomData
  dsBindings[22].binding            = 22;
  dsBindings[22].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[22].descriptorCount    = 1;
  dsBindings[22].stageFlags         = stageFlags;
  dsBindings[22].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSBSDataF
  dsBindings[23].binding            = 23;
  dsBindings[23].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[23].descriptorCount    = 1;
  dsBindings[23].stageFlags         = stageFlags;
  dsBindings[23].pImmutableSamplers = nullptr;

  // binding for m_lights
  dsBindings[24].binding            = 24;
  dsBindings[24].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[24].descriptorCount    = 1;
  dsBindings[24].stageFlags         = stageFlags;
  dsBindings[24].pImmutableSamplers = nullptr;

  // binding for m_cie_x
  dsBindings[25].binding            = 25;
  dsBindings[25].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[25].descriptorCount    = 1;
  dsBindings[25].stageFlags         = stageFlags;
  dsBindings[25].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSBSHeaders
  dsBindings[26].binding            = 26;
  dsBindings[26].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[26].descriptorCount    = 1;
  dsBindings[26].stageFlags         = stageFlags;
  dsBindings[26].pImmutableSamplers = nullptr;

  // binding for m_textures
  dsBindings[27].binding            = 27;
  dsBindings[27].descriptorType     = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
  m_vdata.m_texturesArrayMaxSize = m_textures.size();
  if(m_vdata.m_texturesArrayMaxSize == 0)
    m_vdata.m_texturesArrayMaxSize = GetDefaultMaxTextures();
  dsBindings[27].descriptorCount    = m_vdata.m_texturesArrayMaxSize;
  dsBindings[27].stageFlags         = stageFlags;
  dsBindings[27].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_nodesTLAS
  dsBindings[28].binding            = 28;
  dsBindings[28].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[28].descriptorCount    = 1;
  dsBindings[28].stageFlags         = stageFlags;
  dsBindings[28].pImmutableSamplers = nullptr;

  // binding for m_normMatrices
  dsBindings[29].binding            = 29;
  dsBindings[29].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[29].descriptorCount    = 1;
  dsBindings[29].stageFlags         = stageFlags;
  dsBindings[29].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSBSRoots
  dsBindings[30].binding            = 30;
  dsBindings[30].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[30].descriptorCount    = 1;
  dsBindings[30].stageFlags         = stageFlags;
  dsBindings[30].pImmutableSamplers = nullptr;

  // binding for all_references
  dsBindings[31].binding            = 31;
  dsBindings[31].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[31].descriptorCount    = 1;
  dsBindings[31].stageFlags         = stageFlags;
  dsBindings[31].pImmutableSamplers = nullptr;

  // binding for m_spec_tex_offset_sz
  dsBindings[32].binding            = 32;
  dsBindings[32].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[32].descriptorCount    = 1;
  dsBindings[32].stageFlags         = stageFlags;
  dsBindings[32].pImmutableSamplers = nullptr;

  // binding for m_precomp_coat_transmittance
  dsBindings[33].binding            = 33;
  dsBindings[33].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[33].descriptorCount    = 1;
  dsBindings[33].stageFlags         = stageFlags;
  dsBindings[33].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_vertNorm
  dsBindings[34].binding            = 34;
  dsBindings[34].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[34].descriptorCount    = 1;
  dsBindings[34].stageFlags         = stageFlags;
  dsBindings[34].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfFrameOctreeNodes
  dsBindings[35].binding            = 35;
  dsBindings[35].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[35].descriptorCount    = 1;
  dsBindings[35].stageFlags         = stageFlags;
  dsBindings[35].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSBSNodes
  dsBindings[36].binding            = 36;
  dsBindings[36].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[36].descriptorCount    = 1;
  dsBindings[36].stageFlags         = stageFlags;
  dsBindings[36].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_vertPos
  dsBindings[37].binding            = 37;
  dsBindings[37].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[37].descriptorCount    = 1;
  dsBindings[37].stageFlags         = stageFlags;
  dsBindings[37].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_origNodes
  dsBindings[38].binding            = 38;
  dsBindings[38].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[38].descriptorCount    = 1;
  dsBindings[38].stageFlags         = stageFlags;
  dsBindings[38].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_NURBS_approxes
  dsBindings[39].binding            = 39;
  dsBindings[39].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[39].descriptorCount    = 1;
  dsBindings[39].stageFlags         = stageFlags;
  dsBindings[39].pImmutableSamplers = nullptr;

  // binding for m_spec_values
  dsBindings[40].binding            = 40;
  dsBindings[40].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[40].descriptorCount    = 1;
  dsBindings[40].stageFlags         = stageFlags;
  dsBindings[40].pImmutableSamplers = nullptr;

  // binding for m_spec_offset_sz
  dsBindings[41].binding            = 41;
  dsBindings[41].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[41].descriptorCount    = 1;
  dsBindings[41].stageFlags         = stageFlags;
  dsBindings[41].pImmutableSamplers = nullptr;

  // binding for m_films_eta_k_vec
  dsBindings[42].binding            = 42;
  dsBindings[42].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[42].descriptorCount    = 1;
  dsBindings[42].stageFlags         = stageFlags;
  dsBindings[42].pImmutableSamplers = nullptr;

  // binding for m_normMatrices2
  dsBindings[43].binding            = 43;
  dsBindings[43].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[43].descriptorCount    = 1;
  dsBindings[43].stageFlags         = stageFlags;
  dsBindings[43].pImmutableSamplers = nullptr;

  // binding for m_triIndices
  dsBindings[44].binding            = 44;
  dsBindings[44].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[44].descriptorCount    = 1;
  dsBindings[44].stageFlags         = stageFlags;
  dsBindings[44].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_NURBSData
  dsBindings[45].binding            = 45;
  dsBindings[45].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[45].descriptorCount    = 1;
  dsBindings[45].stageFlags         = stageFlags;
  dsBindings[45].pImmutableSamplers = nullptr;

  // binding for m_allRemapLists
  dsBindings[46].binding            = 46;
  dsBindings[46].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[46].descriptorCount    = 1;
  dsBindings[46].stageFlags         = stageFlags;
  dsBindings[46].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_abstractObjectPtrs
  dsBindings[47].binding            = 47;
  dsBindings[47].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[47].descriptorCount    = 1;
  dsBindings[47].stageFlags         = stageFlags;
  dsBindings[47].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_startEnd
  dsBindings[48].binding            = 48;
  dsBindings[48].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[48].descriptorCount    = 1;
  dsBindings[48].stageFlags         = stageFlags;
  dsBindings[48].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_allNodePairs
  dsBindings[49].binding            = 49;
  dsBindings[49].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[49].descriptorCount    = 1;
  dsBindings[49].stageFlags         = stageFlags;
  dsBindings[49].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_CatmulClarkHeaders
  dsBindings[50].binding            = 50;
  dsBindings[50].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[50].descriptorCount    = 1;
  dsBindings[50].stageFlags         = stageFlags;
  dsBindings[50].pImmutableSamplers = nullptr;

  // binding for m_instIdToLightInstId
  dsBindings[51].binding            = 51;
  dsBindings[51].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[51].descriptorCount    = 1;
  dsBindings[51].stageFlags         = stageFlags;
  dsBindings[51].pImmutableSamplers = nullptr;

  // binding for m_matIdByPrimId
  dsBindings[52].binding            = 52;
  dsBindings[52].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[52].descriptorCount    = 1;
  dsBindings[52].stageFlags         = stageFlags;
  dsBindings[52].pImmutableSamplers = nullptr;

  // binding for m_remapInst
  dsBindings[53].binding            = 53;
  dsBindings[53].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[53].descriptorCount    = 1;
  dsBindings[53].stageFlags         = stageFlags;
  dsBindings[53].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_instanceData
  dsBindings[54].binding            = 54;
  dsBindings[54].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[54].descriptorCount    = 1;
  dsBindings[54].stageFlags         = stageFlags;
  dsBindings[54].pImmutableSamplers = nullptr;

  // binding for m_cie_z
  dsBindings[55].binding            = 55;
  dsBindings[55].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[55].descriptorCount    = 1;
  dsBindings[55].stageFlags         = stageFlags;
  dsBindings[55].pImmutableSamplers = nullptr;

  // binding for m_vNorm4f
  dsBindings[56].binding            = 56;
  dsBindings[56].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[56].descriptorCount    = 1;
  dsBindings[56].stageFlags         = stageFlags;
  dsBindings[56].pImmutableSamplers = nullptr;

  // binding for m_matIdOffsets
  dsBindings[57].binding            = 57;
  dsBindings[57].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[57].descriptorCount    = 1;
  dsBindings[57].stageFlags         = stageFlags;
  dsBindings[57].pImmutableSamplers = nullptr;

  // binding for m_vTang4f
  dsBindings[58].binding            = 58;
  dsBindings[58].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[58].descriptorCount    = 1;
  dsBindings[58].stageFlags         = stageFlags;
  dsBindings[58].pImmutableSamplers = nullptr;

  // binding for m_randomGens
  dsBindings[59].binding            = 59;
  dsBindings[59].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[59].descriptorCount    = 1;
  dsBindings[59].stageFlags         = stageFlags;
  dsBindings[59].pImmutableSamplers = nullptr;

  // binding for m_packedXY
  dsBindings[60].binding            = 60;
  dsBindings[60].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[60].descriptorCount    = 1;
  dsBindings[60].stageFlags         = stageFlags;
  dsBindings[60].pImmutableSamplers = nullptr;


  dsBindings[61].binding            = 61;
  dsBindings[61].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[61].descriptorCount    = 1;
  dsBindings[61].stageFlags         = stageFlags;
  dsBindings[61].pImmutableSamplers = nullptr;


  VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCreateInfo = {};
  descriptorSetLayoutCreateInfo.sType        = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
  descriptorSetLayoutCreateInfo.bindingCount = uint32_t(dsBindings.size());
  descriptorSetLayoutCreateInfo.pBindings    = dsBindings.data();

  VkDescriptorSetLayout layout = nullptr;
  VK_CHECK_RESULT(vkCreateDescriptorSetLayout(device, &descriptorSetLayoutCreateInfo, NULL, &layout));
  return layout;
}
VkDescriptorSetLayout Integrator_Generated::CreateNaivePathTraceMegaDSLayout()
{
  std::array<VkDescriptorSetLayoutBinding, 61+1> dsBindings;
  
  const auto stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  // binding for out_color
  dsBindings[0].binding            = 0;
  dsBindings[0].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[0].descriptorCount    = 1;
  dsBindings[0].stageFlags         = stageFlags;
  dsBindings[0].pImmutableSamplers = nullptr;

  // binding for m_lines
  dsBindings[1].binding            = 1;
  dsBindings[1].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[1].descriptorCount    = 1;
  dsBindings[1].stageFlags         = stageFlags;
  dsBindings[1].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_NURBSHeaders
  dsBindings[2].binding            = 2;
  dsBindings[2].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[2].descriptorCount    = 1;
  dsBindings[2].stageFlags         = stageFlags;
  dsBindings[2].pImmutableSamplers = nullptr;

  // binding for m_precomp_coat_transmittance
  dsBindings[3].binding            = 3;
  dsBindings[3].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[3].descriptorCount    = 1;
  dsBindings[3].stageFlags         = stageFlags;
  dsBindings[3].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_indices
  dsBindings[4].binding            = 4;
  dsBindings[4].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[4].descriptorCount    = 1;
  dsBindings[4].stageFlags         = stageFlags;
  dsBindings[4].pImmutableSamplers = nullptr;

  // binding for m_materials
  dsBindings[5].binding            = 5;
  dsBindings[5].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[5].descriptorCount    = 1;
  dsBindings[5].stageFlags         = stageFlags;
  dsBindings[5].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSBSRoots
  dsBindings[6].binding            = 6;
  dsBindings[6].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[6].descriptorCount    = 1;
  dsBindings[6].stageFlags         = stageFlags;
  dsBindings[6].pImmutableSamplers = nullptr;

  // binding for m_textures
  dsBindings[7].binding            = 7;
  dsBindings[7].descriptorType     = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
  m_vdata.m_texturesArrayMaxSize = m_textures.size();
  if(m_vdata.m_texturesArrayMaxSize == 0)
    m_vdata.m_texturesArrayMaxSize = GetDefaultMaxTextures();
  dsBindings[7].descriptorCount    = m_vdata.m_texturesArrayMaxSize;
  dsBindings[7].stageFlags         = stageFlags;
  dsBindings[7].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_nodesTLAS
  dsBindings[8].binding            = 8;
  dsBindings[8].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[8].descriptorCount    = 1;
  dsBindings[8].stageFlags         = stageFlags;
  dsBindings[8].pImmutableSamplers = nullptr;

  // binding for m_normMatrices
  dsBindings[9].binding            = 9;
  dsBindings[9].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[9].descriptorCount    = 1;
  dsBindings[9].stageFlags         = stageFlags;
  dsBindings[9].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfCompactOctreeV3Data
  dsBindings[10].binding            = 10;
  dsBindings[10].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[10].descriptorCount    = 1;
  dsBindings[10].stageFlags         = stageFlags;
  dsBindings[10].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_RibbonHeaders
  dsBindings[11].binding            = 11;
  dsBindings[11].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[11].descriptorCount    = 1;
  dsBindings[11].stageFlags         = stageFlags;
  dsBindings[11].pImmutableSamplers = nullptr;

  // binding for m_allRemapListsOffsets
  dsBindings[12].binding            = 12;
  dsBindings[12].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[12].descriptorCount    = 1;
  dsBindings[12].stageFlags         = stageFlags;
  dsBindings[12].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_NURBSData
  dsBindings[13].binding            = 13;
  dsBindings[13].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[13].descriptorCount    = 1;
  dsBindings[13].stageFlags         = stageFlags;
  dsBindings[13].pImmutableSamplers = nullptr;

  // binding for m_allRemapLists
  dsBindings[14].binding            = 14;
  dsBindings[14].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[14].descriptorCount    = 1;
  dsBindings[14].stageFlags         = stageFlags;
  dsBindings[14].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSBSData
  dsBindings[15].binding            = 15;
  dsBindings[15].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[15].descriptorCount    = 1;
  dsBindings[15].stageFlags         = stageFlags;
  dsBindings[15].pImmutableSamplers = nullptr;

  // binding for m_films_spec_id_vec
  dsBindings[16].binding            = 16;
  dsBindings[16].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[16].descriptorCount    = 1;
  dsBindings[16].stageFlags         = stageFlags;
  dsBindings[16].pImmutableSamplers = nullptr;

  // binding for m_vertOffset
  dsBindings[17].binding            = 17;
  dsBindings[17].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[17].descriptorCount    = 1;
  dsBindings[17].stageFlags         = stageFlags;
  dsBindings[17].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_geomData
  dsBindings[18].binding            = 18;
  dsBindings[18].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[18].descriptorCount    = 1;
  dsBindings[18].stageFlags         = stageFlags;
  dsBindings[18].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSVSRoots
  dsBindings[19].binding            = 19;
  dsBindings[19].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[19].descriptorCount    = 1;
  dsBindings[19].stageFlags         = stageFlags;
  dsBindings[19].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_startEnd
  dsBindings[20].binding            = 20;
  dsBindings[20].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[20].descriptorCount    = 1;
  dsBindings[20].stageFlags         = stageFlags;
  dsBindings[20].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_allNodePairs
  dsBindings[21].binding            = 21;
  dsBindings[21].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[21].descriptorCount    = 1;
  dsBindings[21].stageFlags         = stageFlags;
  dsBindings[21].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_primIdCount
  dsBindings[22].binding            = 22;
  dsBindings[22].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[22].descriptorCount    = 1;
  dsBindings[22].stageFlags         = stageFlags;
  dsBindings[22].pImmutableSamplers = nullptr;

  // binding for m_spec_tex_ids_wavelengths
  dsBindings[23].binding            = 23;
  dsBindings[23].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[23].descriptorCount    = 1;
  dsBindings[23].stageFlags         = stageFlags;
  dsBindings[23].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfFrameOctreeRoots
  dsBindings[24].binding            = 24;
  dsBindings[24].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[24].descriptorCount    = 1;
  dsBindings[24].stageFlags         = stageFlags;
  dsBindings[24].pImmutableSamplers = nullptr;

  // binding for m_instIdToLightInstId
  dsBindings[25].binding            = 25;
  dsBindings[25].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[25].descriptorCount    = 1;
  dsBindings[25].stageFlags         = stageFlags;
  dsBindings[25].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_CatmulClarkHeaders
  dsBindings[26].binding            = 26;
  dsBindings[26].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[26].descriptorCount    = 1;
  dsBindings[26].stageFlags         = stageFlags;
  dsBindings[26].pImmutableSamplers = nullptr;

  // binding for m_lights
  dsBindings[27].binding            = 27;
  dsBindings[27].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[27].descriptorCount    = 1;
  dsBindings[27].stageFlags         = stageFlags;
  dsBindings[27].pImmutableSamplers = nullptr;

  // binding for m_cie_x
  dsBindings[28].binding            = 28;
  dsBindings[28].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[28].descriptorCount    = 1;
  dsBindings[28].stageFlags         = stageFlags;
  dsBindings[28].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSBSDataF
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

  // binding for m_pAccelStruct_m_SdfCompactOctreeRotModifiers
  dsBindings[31].binding            = 31;
  dsBindings[31].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[31].descriptorCount    = 1;
  dsBindings[31].stageFlags         = stageFlags;
  dsBindings[31].pImmutableSamplers = nullptr;

  // binding for m_cie_y
  dsBindings[32].binding            = 32;
  dsBindings[32].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[32].descriptorCount    = 1;
  dsBindings[32].stageFlags         = stageFlags;
  dsBindings[32].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSVSNodes
  dsBindings[33].binding            = 33;
  dsBindings[33].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[33].descriptorCount    = 1;
  dsBindings[33].stageFlags         = stageFlags;
  dsBindings[33].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfCompactOctreeV2Data
  dsBindings[34].binding            = 34;
  dsBindings[34].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[34].descriptorCount    = 1;
  dsBindings[34].stageFlags         = stageFlags;
  dsBindings[34].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_vertNorm
  dsBindings[35].binding            = 35;
  dsBindings[35].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[35].descriptorCount    = 1;
  dsBindings[35].stageFlags         = stageFlags;
  dsBindings[35].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfFrameOctreeNodes
  dsBindings[36].binding            = 36;
  dsBindings[36].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[36].descriptorCount    = 1;
  dsBindings[36].stageFlags         = stageFlags;
  dsBindings[36].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_SdfSBSNodes
  dsBindings[37].binding            = 37;
  dsBindings[37].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[37].descriptorCount    = 1;
  dsBindings[37].stageFlags         = stageFlags;
  dsBindings[37].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_vertPos
  dsBindings[38].binding            = 38;
  dsBindings[38].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[38].descriptorCount    = 1;
  dsBindings[38].stageFlags         = stageFlags;
  dsBindings[38].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_origNodes
  dsBindings[39].binding            = 39;
  dsBindings[39].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[39].descriptorCount    = 1;
  dsBindings[39].stageFlags         = stageFlags;
  dsBindings[39].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_NURBS_approxes
  dsBindings[40].binding            = 40;
  dsBindings[40].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[40].descriptorCount    = 1;
  dsBindings[40].stageFlags         = stageFlags;
  dsBindings[40].pImmutableSamplers = nullptr;

  // binding for m_spec_values
  dsBindings[41].binding            = 41;
  dsBindings[41].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[41].descriptorCount    = 1;
  dsBindings[41].stageFlags         = stageFlags;
  dsBindings[41].pImmutableSamplers = nullptr;

  // binding for m_spec_tex_offset_sz
  dsBindings[42].binding            = 42;
  dsBindings[42].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[42].descriptorCount    = 1;
  dsBindings[42].stageFlags         = stageFlags;
  dsBindings[42].pImmutableSamplers = nullptr;

  // binding for all_references
  dsBindings[43].binding            = 43;
  dsBindings[43].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[43].descriptorCount    = 1;
  dsBindings[43].stageFlags         = stageFlags;
  dsBindings[43].pImmutableSamplers = nullptr;

  // binding for m_spec_offset_sz
  dsBindings[44].binding            = 44;
  dsBindings[44].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[44].descriptorCount    = 1;
  dsBindings[44].stageFlags         = stageFlags;
  dsBindings[44].pImmutableSamplers = nullptr;

  // binding for m_films_eta_k_vec
  dsBindings[45].binding            = 45;
  dsBindings[45].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[45].descriptorCount    = 1;
  dsBindings[45].stageFlags         = stageFlags;
  dsBindings[45].pImmutableSamplers = nullptr;

  // binding for m_normMatrices2
  dsBindings[46].binding            = 46;
  dsBindings[46].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[46].descriptorCount    = 1;
  dsBindings[46].stageFlags         = stageFlags;
  dsBindings[46].pImmutableSamplers = nullptr;

  // binding for m_triIndices
  dsBindings[47].binding            = 47;
  dsBindings[47].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[47].descriptorCount    = 1;
  dsBindings[47].stageFlags         = stageFlags;
  dsBindings[47].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_abstractObjectPtrs
  dsBindings[48].binding            = 48;
  dsBindings[48].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[48].descriptorCount    = 1;
  dsBindings[48].stageFlags         = stageFlags;
  dsBindings[48].pImmutableSamplers = nullptr;

  // binding for m_matIdByPrimId
  dsBindings[49].binding            = 49;
  dsBindings[49].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[49].descriptorCount    = 1;
  dsBindings[49].stageFlags         = stageFlags;
  dsBindings[49].pImmutableSamplers = nullptr;

  // binding for m_remapInst
  dsBindings[50].binding            = 50;
  dsBindings[50].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[50].descriptorCount    = 1;
  dsBindings[50].stageFlags         = stageFlags;
  dsBindings[50].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_instanceData
  dsBindings[51].binding            = 51;
  dsBindings[51].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[51].descriptorCount    = 1;
  dsBindings[51].stageFlags         = stageFlags;
  dsBindings[51].pImmutableSamplers = nullptr;

  // binding for m_cie_z
  dsBindings[52].binding            = 52;
  dsBindings[52].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[52].descriptorCount    = 1;
  dsBindings[52].stageFlags         = stageFlags;
  dsBindings[52].pImmutableSamplers = nullptr;

  // binding for m_vNorm4f
  dsBindings[53].binding            = 53;
  dsBindings[53].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[53].descriptorCount    = 1;
  dsBindings[53].stageFlags         = stageFlags;
  dsBindings[53].pImmutableSamplers = nullptr;

  // binding for m_matIdOffsets
  dsBindings[54].binding            = 54;
  dsBindings[54].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[54].descriptorCount    = 1;
  dsBindings[54].stageFlags         = stageFlags;
  dsBindings[54].pImmutableSamplers = nullptr;

  // binding for m_vTang4f
  dsBindings[55].binding            = 55;
  dsBindings[55].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[55].descriptorCount    = 1;
  dsBindings[55].stageFlags         = stageFlags;
  dsBindings[55].pImmutableSamplers = nullptr;

  // binding for m_precomp_thin_films
  dsBindings[56].binding            = 56;
  dsBindings[56].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[56].descriptorCount    = 1;
  dsBindings[56].stageFlags         = stageFlags;
  dsBindings[56].pImmutableSamplers = nullptr;

  // binding for m_pAccelStruct_m_primIndices
  dsBindings[57].binding            = 57;
  dsBindings[57].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[57].descriptorCount    = 1;
  dsBindings[57].stageFlags         = stageFlags;
  dsBindings[57].pImmutableSamplers = nullptr;

  // binding for m_randomGens
  dsBindings[58].binding            = 58;
  dsBindings[58].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[58].descriptorCount    = 1;
  dsBindings[58].stageFlags         = stageFlags;
  dsBindings[58].pImmutableSamplers = nullptr;

  // binding for m_pdfLightData
  dsBindings[59].binding            = 59;
  dsBindings[59].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[59].descriptorCount    = 1;
  dsBindings[59].stageFlags         = stageFlags;
  dsBindings[59].pImmutableSamplers = nullptr;

  // binding for m_packedXY
  dsBindings[60].binding            = 60;
  dsBindings[60].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[60].descriptorCount    = 1;
  dsBindings[60].stageFlags         = stageFlags;
  dsBindings[60].pImmutableSamplers = nullptr;


  dsBindings[61].binding            = 61;
  dsBindings[61].descriptorType     = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  dsBindings[61].descriptorCount    = 1;
  dsBindings[61].stageFlags         = stageFlags;
  dsBindings[61].pImmutableSamplers = nullptr;


  VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCreateInfo = {};
  descriptorSetLayoutCreateInfo.sType        = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
  descriptorSetLayoutCreateInfo.bindingCount = uint32_t(dsBindings.size());
  descriptorSetLayoutCreateInfo.pBindings    = dsBindings.data();

  VkDescriptorSetLayout layout = nullptr;
  VK_CHECK_RESULT(vkCreateDescriptorSetLayout(device, &descriptorSetLayoutCreateInfo, NULL, &layout));
  return layout;
}

VkDescriptorSetLayout Integrator_Generated::CreatecopyKernelFloatDSLayout()
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

VkDescriptorSetLayout Integrator_Generated::CreatematMulTransposeDSLayout()
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

void Integrator_Generated::InitAllGeneratedDescriptorSets_RayTrace()
{
  // now create actual bindings
  //
  // descriptor set #0: RayTraceMegaCmd (["out_color","m_packedXY","m_triIndices","m_normMatrices2","m_vTang4f","m_matIdOffsets","m_vNorm4f","m_remapInst","m_matIdByPrimId","m_pAccelStruct_m_abstractObjectPtrs","m_pAccelStruct_m_nodesTLAS","m_pAccelStruct_m_NURBSHeaders","m_pAccelStruct_m_primIndices","m_vertOffset","m_pAccelStruct_m_SdfSBSData","m_pAccelStruct_m_NURBS_approxes","m_pAccelStruct_m_geomData","m_pAccelStruct_m_SdfCompactOctreeRotModifiers","m_pAccelStruct_m_SdfSVSNodes","m_pAccelStruct_m_SdfFrameOctreeRoots","m_pAccelStruct_m_CatmulClarkHeaders","m_pAccelStruct_m_SdfCompactOctreeV2Data","m_pAccelStruct_m_SdfSVSRoots","m_pAccelStruct_m_primIdCount","m_pAccelStruct_m_allNodePairs","m_pAccelStruct_startEnd","all_references","m_allRemapLists","m_pAccelStruct_m_NURBSData","m_allRemapListsOffsets","m_pAccelStruct_m_RibbonHeaders","m_pAccelStruct_m_SdfCompactOctreeV3Data","m_lights","m_pAccelStruct_m_origNodes","m_pAccelStruct_m_SdfSBSDataF","m_pAccelStruct_m_SdfSBSHeaders","m_normMatrices","m_textures","m_pAccelStruct_m_SdfSBSRoots","m_materials","m_pAccelStruct_m_indices","m_pAccelStruct_m_SdfSBSNodes","m_pAccelStruct_m_instanceData","m_pAccelStruct_m_vertNorm","m_pAccelStruct_m_SdfFrameOctreeNodes","m_pAccelStruct_m_vertPos"])
  {
    constexpr uint additionalSize = 1;

    std::array<VkDescriptorBufferInfo, 46 + additionalSize> descriptorBufferInfo;
    std::array<VkDescriptorImageInfo,  46 + additionalSize> descriptorImageInfo;
    std::array<VkWriteDescriptorSet,   46 + additionalSize> writeDescriptorSet;

    descriptorBufferInfo[0]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[0].buffer = RayTrace_local.out_colorBuffer;
    descriptorBufferInfo[0].offset = RayTrace_local.out_colorOffset;
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
    descriptorBufferInfo[1].buffer = m_vdata.m_packedXYBuffer;
    descriptorBufferInfo[1].offset = m_vdata.m_packedXYOffset;
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
    descriptorBufferInfo[2].buffer = m_vdata.m_triIndicesBuffer;
    descriptorBufferInfo[2].offset = m_vdata.m_triIndicesOffset;
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
    descriptorBufferInfo[3].buffer = m_vdata.m_normMatrices2Buffer;
    descriptorBufferInfo[3].offset = m_vdata.m_normMatrices2Offset;
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
    descriptorBufferInfo[4].buffer = m_vdata.m_vTang4fBuffer;
    descriptorBufferInfo[4].offset = m_vdata.m_vTang4fOffset;
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
    descriptorBufferInfo[5].buffer = m_vdata.m_matIdOffsetsBuffer;
    descriptorBufferInfo[5].offset = m_vdata.m_matIdOffsetsOffset;
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
    descriptorBufferInfo[6].buffer = m_vdata.m_vNorm4fBuffer;
    descriptorBufferInfo[6].offset = m_vdata.m_vNorm4fOffset;
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
    descriptorBufferInfo[7].buffer = m_vdata.m_remapInstBuffer;
    descriptorBufferInfo[7].offset = m_vdata.m_remapInstOffset;
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
    descriptorBufferInfo[8].buffer = m_vdata.m_matIdByPrimIdBuffer;
    descriptorBufferInfo[8].offset = m_vdata.m_matIdByPrimIdOffset;
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
    descriptorBufferInfo[9].buffer = m_vdata.m_pAccelStruct_m_abstractObjectPtrsBuffer;
    descriptorBufferInfo[9].offset = m_vdata.m_pAccelStruct_m_abstractObjectPtrsOffset;
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
    descriptorBufferInfo[10].buffer = m_vdata.m_pAccelStruct_m_nodesTLASBuffer;
    descriptorBufferInfo[10].offset = m_vdata.m_pAccelStruct_m_nodesTLASOffset;
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
    descriptorBufferInfo[11].buffer = m_vdata.m_pAccelStruct_m_NURBSHeadersBuffer;
    descriptorBufferInfo[11].offset = m_vdata.m_pAccelStruct_m_NURBSHeadersOffset;
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
    descriptorBufferInfo[12].buffer = m_vdata.m_pAccelStruct_m_primIndicesBuffer;
    descriptorBufferInfo[12].offset = m_vdata.m_pAccelStruct_m_primIndicesOffset;
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
    descriptorBufferInfo[13].buffer = m_vdata.m_vertOffsetBuffer;
    descriptorBufferInfo[13].offset = m_vdata.m_vertOffsetOffset;
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
    descriptorBufferInfo[14].buffer = m_vdata.m_pAccelStruct_m_SdfSBSDataBuffer;
    descriptorBufferInfo[14].offset = m_vdata.m_pAccelStruct_m_SdfSBSDataOffset;
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
    descriptorBufferInfo[15].buffer = m_vdata.m_pAccelStruct_m_NURBS_approxesBuffer;
    descriptorBufferInfo[15].offset = m_vdata.m_pAccelStruct_m_NURBS_approxesOffset;
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
    descriptorBufferInfo[16].buffer = m_vdata.m_pAccelStruct_m_geomDataBuffer;
    descriptorBufferInfo[16].offset = m_vdata.m_pAccelStruct_m_geomDataOffset;
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
    descriptorBufferInfo[17].buffer = m_vdata.m_pAccelStruct_m_SdfCompactOctreeRotModifiersBuffer;
    descriptorBufferInfo[17].offset = m_vdata.m_pAccelStruct_m_SdfCompactOctreeRotModifiersOffset;
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
    descriptorBufferInfo[18].buffer = m_vdata.m_pAccelStruct_m_SdfSVSNodesBuffer;
    descriptorBufferInfo[18].offset = m_vdata.m_pAccelStruct_m_SdfSVSNodesOffset;
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
    descriptorBufferInfo[19].buffer = m_vdata.m_pAccelStruct_m_SdfFrameOctreeRootsBuffer;
    descriptorBufferInfo[19].offset = m_vdata.m_pAccelStruct_m_SdfFrameOctreeRootsOffset;
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
    descriptorBufferInfo[20].buffer = m_vdata.m_pAccelStruct_m_CatmulClarkHeadersBuffer;
    descriptorBufferInfo[20].offset = m_vdata.m_pAccelStruct_m_CatmulClarkHeadersOffset;
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
    descriptorBufferInfo[21].buffer = m_vdata.m_pAccelStruct_m_SdfCompactOctreeV2DataBuffer;
    descriptorBufferInfo[21].offset = m_vdata.m_pAccelStruct_m_SdfCompactOctreeV2DataOffset;
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
    descriptorBufferInfo[22].buffer = m_vdata.m_pAccelStruct_m_SdfSVSRootsBuffer;
    descriptorBufferInfo[22].offset = m_vdata.m_pAccelStruct_m_SdfSVSRootsOffset;
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
    descriptorBufferInfo[23].buffer = m_vdata.m_pAccelStruct_m_primIdCountBuffer;
    descriptorBufferInfo[23].offset = m_vdata.m_pAccelStruct_m_primIdCountOffset;
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
    descriptorBufferInfo[24].buffer = m_vdata.m_pAccelStruct_m_allNodePairsBuffer;
    descriptorBufferInfo[24].offset = m_vdata.m_pAccelStruct_m_allNodePairsOffset;
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
    descriptorBufferInfo[25].buffer = m_vdata.m_pAccelStruct_startEndBuffer;
    descriptorBufferInfo[25].offset = m_vdata.m_pAccelStruct_startEndOffset;
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
    descriptorBufferInfo[26].buffer = m_vdata.all_referencesBuffer;
    descriptorBufferInfo[26].offset = m_vdata.all_referencesOffset;
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
    descriptorBufferInfo[27].buffer = m_vdata.m_allRemapListsBuffer;
    descriptorBufferInfo[27].offset = m_vdata.m_allRemapListsOffset;
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
    descriptorBufferInfo[28].buffer = m_vdata.m_pAccelStruct_m_NURBSDataBuffer;
    descriptorBufferInfo[28].offset = m_vdata.m_pAccelStruct_m_NURBSDataOffset;
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
    descriptorBufferInfo[29].buffer = m_vdata.m_allRemapListsOffsetsBuffer;
    descriptorBufferInfo[29].offset = m_vdata.m_allRemapListsOffsetsOffset;
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
    descriptorBufferInfo[30].buffer = m_vdata.m_pAccelStruct_m_RibbonHeadersBuffer;
    descriptorBufferInfo[30].offset = m_vdata.m_pAccelStruct_m_RibbonHeadersOffset;
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
    descriptorBufferInfo[31].buffer = m_vdata.m_pAccelStruct_m_SdfCompactOctreeV3DataBuffer;
    descriptorBufferInfo[31].offset = m_vdata.m_pAccelStruct_m_SdfCompactOctreeV3DataOffset;
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
    descriptorBufferInfo[32].buffer = m_vdata.m_lightsBuffer;
    descriptorBufferInfo[32].offset = m_vdata.m_lightsOffset;
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
    descriptorBufferInfo[33].buffer = m_vdata.m_pAccelStruct_m_origNodesBuffer;
    descriptorBufferInfo[33].offset = m_vdata.m_pAccelStruct_m_origNodesOffset;
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

    descriptorBufferInfo[34]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[34].buffer = m_vdata.m_pAccelStruct_m_SdfSBSDataFBuffer;
    descriptorBufferInfo[34].offset = m_vdata.m_pAccelStruct_m_SdfSBSDataFOffset;
    descriptorBufferInfo[34].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[34]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[34].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[34].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[34].dstBinding       = 34;
    writeDescriptorSet[34].descriptorCount  = 1;
    writeDescriptorSet[34].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[34].pBufferInfo      = &descriptorBufferInfo[34];
    writeDescriptorSet[34].pImageInfo       = nullptr;
    writeDescriptorSet[34].pTexelBufferView = nullptr;

    descriptorBufferInfo[35]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[35].buffer = m_vdata.m_pAccelStruct_m_SdfSBSHeadersBuffer;
    descriptorBufferInfo[35].offset = m_vdata.m_pAccelStruct_m_SdfSBSHeadersOffset;
    descriptorBufferInfo[35].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[35]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[35].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[35].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[35].dstBinding       = 35;
    writeDescriptorSet[35].descriptorCount  = 1;
    writeDescriptorSet[35].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[35].pBufferInfo      = &descriptorBufferInfo[35];
    writeDescriptorSet[35].pImageInfo       = nullptr;
    writeDescriptorSet[35].pTexelBufferView = nullptr;

    descriptorBufferInfo[36]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[36].buffer = m_vdata.m_normMatricesBuffer;
    descriptorBufferInfo[36].offset = m_vdata.m_normMatricesOffset;
    descriptorBufferInfo[36].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[36]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[36].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[36].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[36].dstBinding       = 36;
    writeDescriptorSet[36].descriptorCount  = 1;
    writeDescriptorSet[36].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[36].pBufferInfo      = &descriptorBufferInfo[36];
    writeDescriptorSet[36].pImageInfo       = nullptr;
    writeDescriptorSet[36].pTexelBufferView = nullptr;

    std::vector<VkDescriptorImageInfo> m_texturesInfo(m_vdata.m_texturesArrayMaxSize);
    for(size_t i=0; i<m_vdata.m_texturesArrayMaxSize; i++)
    {
      if(i < m_textures.size())
      {
        m_texturesInfo[i].sampler     = m_vdata.m_texturesArraySampler[i];
        m_texturesInfo[i].imageView   = m_vdata.m_texturesArrayView   [i];
        m_texturesInfo[i].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
      }
      else
      {
        m_texturesInfo[i].sampler     = m_vdata.m_texturesArraySampler[0];
        m_texturesInfo[i].imageView   = m_vdata.m_texturesArrayView   [0];
        m_texturesInfo[i].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
      }
    }
    writeDescriptorSet[37]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[37].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[37].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[37].dstBinding       = 37;
    writeDescriptorSet[37].descriptorCount  = 1;
    writeDescriptorSet[37].descriptorCount  = m_texturesInfo.size();
    writeDescriptorSet[37].descriptorType   = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    writeDescriptorSet[37].pBufferInfo      = nullptr;
    writeDescriptorSet[37].pImageInfo       = m_texturesInfo.data();
    writeDescriptorSet[37].pTexelBufferView = nullptr;

    descriptorBufferInfo[38]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[38].buffer = m_vdata.m_pAccelStruct_m_SdfSBSRootsBuffer;
    descriptorBufferInfo[38].offset = m_vdata.m_pAccelStruct_m_SdfSBSRootsOffset;
    descriptorBufferInfo[38].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[38]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[38].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[38].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[38].dstBinding       = 38;
    writeDescriptorSet[38].descriptorCount  = 1;
    writeDescriptorSet[38].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[38].pBufferInfo      = &descriptorBufferInfo[38];
    writeDescriptorSet[38].pImageInfo       = nullptr;
    writeDescriptorSet[38].pTexelBufferView = nullptr;

    descriptorBufferInfo[39]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[39].buffer = m_vdata.m_materialsBuffer;
    descriptorBufferInfo[39].offset = m_vdata.m_materialsOffset;
    descriptorBufferInfo[39].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[39]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[39].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[39].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[39].dstBinding       = 39;
    writeDescriptorSet[39].descriptorCount  = 1;
    writeDescriptorSet[39].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[39].pBufferInfo      = &descriptorBufferInfo[39];
    writeDescriptorSet[39].pImageInfo       = nullptr;
    writeDescriptorSet[39].pTexelBufferView = nullptr;

    descriptorBufferInfo[40]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[40].buffer = m_vdata.m_pAccelStruct_m_indicesBuffer;
    descriptorBufferInfo[40].offset = m_vdata.m_pAccelStruct_m_indicesOffset;
    descriptorBufferInfo[40].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[40]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[40].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[40].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[40].dstBinding       = 40;
    writeDescriptorSet[40].descriptorCount  = 1;
    writeDescriptorSet[40].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[40].pBufferInfo      = &descriptorBufferInfo[40];
    writeDescriptorSet[40].pImageInfo       = nullptr;
    writeDescriptorSet[40].pTexelBufferView = nullptr;

    descriptorBufferInfo[41]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[41].buffer = m_vdata.m_pAccelStruct_m_SdfSBSNodesBuffer;
    descriptorBufferInfo[41].offset = m_vdata.m_pAccelStruct_m_SdfSBSNodesOffset;
    descriptorBufferInfo[41].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[41]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[41].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[41].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[41].dstBinding       = 41;
    writeDescriptorSet[41].descriptorCount  = 1;
    writeDescriptorSet[41].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[41].pBufferInfo      = &descriptorBufferInfo[41];
    writeDescriptorSet[41].pImageInfo       = nullptr;
    writeDescriptorSet[41].pTexelBufferView = nullptr;

    descriptorBufferInfo[42]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[42].buffer = m_vdata.m_pAccelStruct_m_instanceDataBuffer;
    descriptorBufferInfo[42].offset = m_vdata.m_pAccelStruct_m_instanceDataOffset;
    descriptorBufferInfo[42].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[42]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[42].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[42].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[42].dstBinding       = 42;
    writeDescriptorSet[42].descriptorCount  = 1;
    writeDescriptorSet[42].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[42].pBufferInfo      = &descriptorBufferInfo[42];
    writeDescriptorSet[42].pImageInfo       = nullptr;
    writeDescriptorSet[42].pTexelBufferView = nullptr;

    descriptorBufferInfo[43]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[43].buffer = m_vdata.m_pAccelStruct_m_vertNormBuffer;
    descriptorBufferInfo[43].offset = m_vdata.m_pAccelStruct_m_vertNormOffset;
    descriptorBufferInfo[43].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[43]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[43].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[43].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[43].dstBinding       = 43;
    writeDescriptorSet[43].descriptorCount  = 1;
    writeDescriptorSet[43].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[43].pBufferInfo      = &descriptorBufferInfo[43];
    writeDescriptorSet[43].pImageInfo       = nullptr;
    writeDescriptorSet[43].pTexelBufferView = nullptr;

    descriptorBufferInfo[44]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[44].buffer = m_vdata.m_pAccelStruct_m_SdfFrameOctreeNodesBuffer;
    descriptorBufferInfo[44].offset = m_vdata.m_pAccelStruct_m_SdfFrameOctreeNodesOffset;
    descriptorBufferInfo[44].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[44]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[44].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[44].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[44].dstBinding       = 44;
    writeDescriptorSet[44].descriptorCount  = 1;
    writeDescriptorSet[44].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[44].pBufferInfo      = &descriptorBufferInfo[44];
    writeDescriptorSet[44].pImageInfo       = nullptr;
    writeDescriptorSet[44].pTexelBufferView = nullptr;

    descriptorBufferInfo[45]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[45].buffer = m_vdata.m_pAccelStruct_m_vertPosBuffer;
    descriptorBufferInfo[45].offset = m_vdata.m_pAccelStruct_m_vertPosOffset;
    descriptorBufferInfo[45].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[45]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[45].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[45].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[45].dstBinding       = 45;
    writeDescriptorSet[45].descriptorCount  = 1;
    writeDescriptorSet[45].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[45].pBufferInfo      = &descriptorBufferInfo[45];
    writeDescriptorSet[45].pImageInfo       = nullptr;
    writeDescriptorSet[45].pTexelBufferView = nullptr;

    descriptorBufferInfo[46]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[46].buffer = m_classDataBuffer;
    descriptorBufferInfo[46].offset = 0;
    descriptorBufferInfo[46].range  = VK_WHOLE_SIZE;

    writeDescriptorSet[46]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[46].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[46].dstSet           = m_allGeneratedDS[0];
    writeDescriptorSet[46].dstBinding       = 46;
    writeDescriptorSet[46].descriptorCount  = 1;
    writeDescriptorSet[46].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[46].pBufferInfo      = &descriptorBufferInfo[46];
    writeDescriptorSet[46].pImageInfo       = nullptr;
    writeDescriptorSet[46].pTexelBufferView = nullptr;

    vkUpdateDescriptorSets(device, uint32_t(writeDescriptorSet.size()), writeDescriptorSet.data(), 0, NULL);
  }
}

void Integrator_Generated::InitAllGeneratedDescriptorSets_CastSingleRay()
{
  // now create actual bindings
  //
  // descriptor set #1: CastSingleRayMegaCmd (["out_color","m_packedXY","m_matIdByPrimId","m_materials","m_triIndices","m_matIdOffsets","m_vTang4f","m_textures","m_vNorm4f","m_pAccelStruct_m_SdfSBSRoots","m_pAccelStruct_m_SdfSBSHeaders","m_pAccelStruct_m_origNodes","m_pAccelStruct_m_SdfSBSDataF","m_pAccelStruct_m_NURBSData","m_pAccelStruct_m_SdfCompactOctreeRotModifiers","m_pAccelStruct_m_SdfCompactOctreeV2Data","m_pAccelStruct_m_SdfSVSRoots","m_pAccelStruct_m_CatmulClarkHeaders","m_pAccelStruct_m_SdfFrameOctreeRoots","m_pAccelStruct_m_SdfSVSNodes","m_pAccelStruct_m_RibbonHeaders","m_pAccelStruct_m_SdfCompactOctreeV3Data","all_references","m_pAccelStruct_m_SdfSBSNodes","m_pAccelStruct_m_geomData","m_vertOffset","m_pAccelStruct_m_SdfSBSData","m_pAccelStruct_m_NURBS_approxes","m_pAccelStruct_m_abstractObjectPtrs","m_pAccelStruct_m_primIdCount","m_pAccelStruct_m_allNodePairs","m_pAccelStruct_startEnd","m_pAccelStruct_m_instanceData","m_pAccelStruct_m_primIndices","m_pAccelStruct_m_NURBSHeaders","m_pAccelStruct_m_nodesTLAS","m_pAccelStruct_m_vertNorm","m_pAccelStruct_m_SdfFrameOctreeNodes","m_pAccelStruct_m_vertPos","m_pAccelStruct_m_indices"])
  {
    constexpr uint additionalSize = 1;

    std::array<VkDescriptorBufferInfo, 40 + additionalSize> descriptorBufferInfo;
    std::array<VkDescriptorImageInfo,  40 + additionalSize> descriptorImageInfo;
    std::array<VkWriteDescriptorSet,   40 + additionalSize> writeDescriptorSet;

    descriptorBufferInfo[0]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[0].buffer = CastSingleRay_local.out_colorBuffer;
    descriptorBufferInfo[0].offset = CastSingleRay_local.out_colorOffset;
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
    descriptorBufferInfo[1].buffer = m_vdata.m_packedXYBuffer;
    descriptorBufferInfo[1].offset = m_vdata.m_packedXYOffset;
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
    descriptorBufferInfo[2].buffer = m_vdata.m_matIdByPrimIdBuffer;
    descriptorBufferInfo[2].offset = m_vdata.m_matIdByPrimIdOffset;
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
    descriptorBufferInfo[3].buffer = m_vdata.m_materialsBuffer;
    descriptorBufferInfo[3].offset = m_vdata.m_materialsOffset;
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
    descriptorBufferInfo[4].buffer = m_vdata.m_triIndicesBuffer;
    descriptorBufferInfo[4].offset = m_vdata.m_triIndicesOffset;
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
    descriptorBufferInfo[5].buffer = m_vdata.m_matIdOffsetsBuffer;
    descriptorBufferInfo[5].offset = m_vdata.m_matIdOffsetsOffset;
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
    descriptorBufferInfo[6].buffer = m_vdata.m_vTang4fBuffer;
    descriptorBufferInfo[6].offset = m_vdata.m_vTang4fOffset;
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

    std::vector<VkDescriptorImageInfo> m_texturesInfo(m_vdata.m_texturesArrayMaxSize);
    for(size_t i=0; i<m_vdata.m_texturesArrayMaxSize; i++)
    {
      if(i < m_textures.size())
      {
        m_texturesInfo[i].sampler     = m_vdata.m_texturesArraySampler[i];
        m_texturesInfo[i].imageView   = m_vdata.m_texturesArrayView   [i];
        m_texturesInfo[i].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
      }
      else
      {
        m_texturesInfo[i].sampler     = m_vdata.m_texturesArraySampler[0];
        m_texturesInfo[i].imageView   = m_vdata.m_texturesArrayView   [0];
        m_texturesInfo[i].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
      }
    }
    writeDescriptorSet[7]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[7].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[7].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[7].dstBinding       = 7;
    writeDescriptorSet[7].descriptorCount  = 1;
    writeDescriptorSet[7].descriptorCount  = m_texturesInfo.size();
    writeDescriptorSet[7].descriptorType   = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    writeDescriptorSet[7].pBufferInfo      = nullptr;
    writeDescriptorSet[7].pImageInfo       = m_texturesInfo.data();
    writeDescriptorSet[7].pTexelBufferView = nullptr;

    descriptorBufferInfo[8]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[8].buffer = m_vdata.m_vNorm4fBuffer;
    descriptorBufferInfo[8].offset = m_vdata.m_vNorm4fOffset;
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
    descriptorBufferInfo[9].buffer = m_vdata.m_pAccelStruct_m_SdfSBSRootsBuffer;
    descriptorBufferInfo[9].offset = m_vdata.m_pAccelStruct_m_SdfSBSRootsOffset;
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
    descriptorBufferInfo[10].buffer = m_vdata.m_pAccelStruct_m_SdfSBSHeadersBuffer;
    descriptorBufferInfo[10].offset = m_vdata.m_pAccelStruct_m_SdfSBSHeadersOffset;
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
    descriptorBufferInfo[11].buffer = m_vdata.m_pAccelStruct_m_origNodesBuffer;
    descriptorBufferInfo[11].offset = m_vdata.m_pAccelStruct_m_origNodesOffset;
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
    descriptorBufferInfo[12].buffer = m_vdata.m_pAccelStruct_m_SdfSBSDataFBuffer;
    descriptorBufferInfo[12].offset = m_vdata.m_pAccelStruct_m_SdfSBSDataFOffset;
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
    descriptorBufferInfo[13].buffer = m_vdata.m_pAccelStruct_m_NURBSDataBuffer;
    descriptorBufferInfo[13].offset = m_vdata.m_pAccelStruct_m_NURBSDataOffset;
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
    descriptorBufferInfo[14].buffer = m_vdata.m_pAccelStruct_m_SdfCompactOctreeRotModifiersBuffer;
    descriptorBufferInfo[14].offset = m_vdata.m_pAccelStruct_m_SdfCompactOctreeRotModifiersOffset;
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
    descriptorBufferInfo[15].buffer = m_vdata.m_pAccelStruct_m_SdfCompactOctreeV2DataBuffer;
    descriptorBufferInfo[15].offset = m_vdata.m_pAccelStruct_m_SdfCompactOctreeV2DataOffset;
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
    descriptorBufferInfo[16].buffer = m_vdata.m_pAccelStruct_m_SdfSVSRootsBuffer;
    descriptorBufferInfo[16].offset = m_vdata.m_pAccelStruct_m_SdfSVSRootsOffset;
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
    descriptorBufferInfo[17].buffer = m_vdata.m_pAccelStruct_m_CatmulClarkHeadersBuffer;
    descriptorBufferInfo[17].offset = m_vdata.m_pAccelStruct_m_CatmulClarkHeadersOffset;
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
    descriptorBufferInfo[18].buffer = m_vdata.m_pAccelStruct_m_SdfFrameOctreeRootsBuffer;
    descriptorBufferInfo[18].offset = m_vdata.m_pAccelStruct_m_SdfFrameOctreeRootsOffset;
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
    descriptorBufferInfo[19].buffer = m_vdata.m_pAccelStruct_m_SdfSVSNodesBuffer;
    descriptorBufferInfo[19].offset = m_vdata.m_pAccelStruct_m_SdfSVSNodesOffset;
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
    descriptorBufferInfo[20].buffer = m_vdata.m_pAccelStruct_m_RibbonHeadersBuffer;
    descriptorBufferInfo[20].offset = m_vdata.m_pAccelStruct_m_RibbonHeadersOffset;
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
    descriptorBufferInfo[21].buffer = m_vdata.m_pAccelStruct_m_SdfCompactOctreeV3DataBuffer;
    descriptorBufferInfo[21].offset = m_vdata.m_pAccelStruct_m_SdfCompactOctreeV3DataOffset;
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
    descriptorBufferInfo[22].buffer = m_vdata.all_referencesBuffer;
    descriptorBufferInfo[22].offset = m_vdata.all_referencesOffset;
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
    descriptorBufferInfo[23].buffer = m_vdata.m_pAccelStruct_m_SdfSBSNodesBuffer;
    descriptorBufferInfo[23].offset = m_vdata.m_pAccelStruct_m_SdfSBSNodesOffset;
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
    descriptorBufferInfo[24].buffer = m_vdata.m_pAccelStruct_m_geomDataBuffer;
    descriptorBufferInfo[24].offset = m_vdata.m_pAccelStruct_m_geomDataOffset;
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
    descriptorBufferInfo[25].buffer = m_vdata.m_vertOffsetBuffer;
    descriptorBufferInfo[25].offset = m_vdata.m_vertOffsetOffset;
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
    descriptorBufferInfo[26].buffer = m_vdata.m_pAccelStruct_m_SdfSBSDataBuffer;
    descriptorBufferInfo[26].offset = m_vdata.m_pAccelStruct_m_SdfSBSDataOffset;
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
    descriptorBufferInfo[27].buffer = m_vdata.m_pAccelStruct_m_NURBS_approxesBuffer;
    descriptorBufferInfo[27].offset = m_vdata.m_pAccelStruct_m_NURBS_approxesOffset;
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
    descriptorBufferInfo[28].buffer = m_vdata.m_pAccelStruct_m_abstractObjectPtrsBuffer;
    descriptorBufferInfo[28].offset = m_vdata.m_pAccelStruct_m_abstractObjectPtrsOffset;
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
    descriptorBufferInfo[29].buffer = m_vdata.m_pAccelStruct_m_primIdCountBuffer;
    descriptorBufferInfo[29].offset = m_vdata.m_pAccelStruct_m_primIdCountOffset;
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
    descriptorBufferInfo[30].buffer = m_vdata.m_pAccelStruct_m_allNodePairsBuffer;
    descriptorBufferInfo[30].offset = m_vdata.m_pAccelStruct_m_allNodePairsOffset;
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
    descriptorBufferInfo[31].buffer = m_vdata.m_pAccelStruct_startEndBuffer;
    descriptorBufferInfo[31].offset = m_vdata.m_pAccelStruct_startEndOffset;
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
    descriptorBufferInfo[32].buffer = m_vdata.m_pAccelStruct_m_instanceDataBuffer;
    descriptorBufferInfo[32].offset = m_vdata.m_pAccelStruct_m_instanceDataOffset;
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
    descriptorBufferInfo[33].buffer = m_vdata.m_pAccelStruct_m_primIndicesBuffer;
    descriptorBufferInfo[33].offset = m_vdata.m_pAccelStruct_m_primIndicesOffset;
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
    descriptorBufferInfo[34].buffer = m_vdata.m_pAccelStruct_m_NURBSHeadersBuffer;
    descriptorBufferInfo[34].offset = m_vdata.m_pAccelStruct_m_NURBSHeadersOffset;
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

    descriptorBufferInfo[35]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[35].buffer = m_vdata.m_pAccelStruct_m_nodesTLASBuffer;
    descriptorBufferInfo[35].offset = m_vdata.m_pAccelStruct_m_nodesTLASOffset;
    descriptorBufferInfo[35].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[35]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[35].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[35].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[35].dstBinding       = 35;
    writeDescriptorSet[35].descriptorCount  = 1;
    writeDescriptorSet[35].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[35].pBufferInfo      = &descriptorBufferInfo[35];
    writeDescriptorSet[35].pImageInfo       = nullptr;
    writeDescriptorSet[35].pTexelBufferView = nullptr;

    descriptorBufferInfo[36]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[36].buffer = m_vdata.m_pAccelStruct_m_vertNormBuffer;
    descriptorBufferInfo[36].offset = m_vdata.m_pAccelStruct_m_vertNormOffset;
    descriptorBufferInfo[36].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[36]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[36].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[36].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[36].dstBinding       = 36;
    writeDescriptorSet[36].descriptorCount  = 1;
    writeDescriptorSet[36].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[36].pBufferInfo      = &descriptorBufferInfo[36];
    writeDescriptorSet[36].pImageInfo       = nullptr;
    writeDescriptorSet[36].pTexelBufferView = nullptr;

    descriptorBufferInfo[37]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[37].buffer = m_vdata.m_pAccelStruct_m_SdfFrameOctreeNodesBuffer;
    descriptorBufferInfo[37].offset = m_vdata.m_pAccelStruct_m_SdfFrameOctreeNodesOffset;
    descriptorBufferInfo[37].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[37]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[37].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[37].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[37].dstBinding       = 37;
    writeDescriptorSet[37].descriptorCount  = 1;
    writeDescriptorSet[37].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[37].pBufferInfo      = &descriptorBufferInfo[37];
    writeDescriptorSet[37].pImageInfo       = nullptr;
    writeDescriptorSet[37].pTexelBufferView = nullptr;

    descriptorBufferInfo[38]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[38].buffer = m_vdata.m_pAccelStruct_m_vertPosBuffer;
    descriptorBufferInfo[38].offset = m_vdata.m_pAccelStruct_m_vertPosOffset;
    descriptorBufferInfo[38].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[38]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[38].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[38].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[38].dstBinding       = 38;
    writeDescriptorSet[38].descriptorCount  = 1;
    writeDescriptorSet[38].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[38].pBufferInfo      = &descriptorBufferInfo[38];
    writeDescriptorSet[38].pImageInfo       = nullptr;
    writeDescriptorSet[38].pTexelBufferView = nullptr;

    descriptorBufferInfo[39]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[39].buffer = m_vdata.m_pAccelStruct_m_indicesBuffer;
    descriptorBufferInfo[39].offset = m_vdata.m_pAccelStruct_m_indicesOffset;
    descriptorBufferInfo[39].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[39]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[39].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[39].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[39].dstBinding       = 39;
    writeDescriptorSet[39].descriptorCount  = 1;
    writeDescriptorSet[39].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[39].pBufferInfo      = &descriptorBufferInfo[39];
    writeDescriptorSet[39].pImageInfo       = nullptr;
    writeDescriptorSet[39].pTexelBufferView = nullptr;

    descriptorBufferInfo[40]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[40].buffer = m_classDataBuffer;
    descriptorBufferInfo[40].offset = 0;
    descriptorBufferInfo[40].range  = VK_WHOLE_SIZE;

    writeDescriptorSet[40]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[40].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[40].dstSet           = m_allGeneratedDS[1];
    writeDescriptorSet[40].dstBinding       = 40;
    writeDescriptorSet[40].descriptorCount  = 1;
    writeDescriptorSet[40].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[40].pBufferInfo      = &descriptorBufferInfo[40];
    writeDescriptorSet[40].pImageInfo       = nullptr;
    writeDescriptorSet[40].pTexelBufferView = nullptr;

    vkUpdateDescriptorSets(device, uint32_t(writeDescriptorSet.size()), writeDescriptorSet.data(), 0, NULL);
  }
}

void Integrator_Generated::InitAllGeneratedDescriptorSets_PackXY()
{
  // now create actual bindings
  //
  // descriptor set #2: PackXYMegaCmd (["m_packedXY"])
  {
    constexpr uint additionalSize = 1;

    std::array<VkDescriptorBufferInfo, 1 + additionalSize> descriptorBufferInfo;
    std::array<VkDescriptorImageInfo,  1 + additionalSize> descriptorImageInfo;
    std::array<VkWriteDescriptorSet,   1 + additionalSize> writeDescriptorSet;

    descriptorBufferInfo[0]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[0].buffer = m_vdata.m_packedXYBuffer;
    descriptorBufferInfo[0].offset = m_vdata.m_packedXYOffset;
    descriptorBufferInfo[0].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[0]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[0].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[0].dstSet           = m_allGeneratedDS[2];
    writeDescriptorSet[0].dstBinding       = 0;
    writeDescriptorSet[0].descriptorCount  = 1;
    writeDescriptorSet[0].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[0].pBufferInfo      = &descriptorBufferInfo[0];
    writeDescriptorSet[0].pImageInfo       = nullptr;
    writeDescriptorSet[0].pTexelBufferView = nullptr;

    descriptorBufferInfo[1]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[1].buffer = m_classDataBuffer;
    descriptorBufferInfo[1].offset = 0;
    descriptorBufferInfo[1].range  = VK_WHOLE_SIZE;

    writeDescriptorSet[1]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[1].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[1].dstSet           = m_allGeneratedDS[2];
    writeDescriptorSet[1].dstBinding       = 1;
    writeDescriptorSet[1].descriptorCount  = 1;
    writeDescriptorSet[1].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[1].pBufferInfo      = &descriptorBufferInfo[1];
    writeDescriptorSet[1].pImageInfo       = nullptr;
    writeDescriptorSet[1].pTexelBufferView = nullptr;

    vkUpdateDescriptorSets(device, uint32_t(writeDescriptorSet.size()), writeDescriptorSet.data(), 0, NULL);
  }
}

void Integrator_Generated::InitAllGeneratedDescriptorSets_PathTraceFromInputRays()
{
  // now create actual bindings
  //
  // descriptor set #3: PathTraceFromInputRaysMegaCmd (["in_rayPosAndNear","in_rayDirAndFar","out_color","m_packedXY","m_randomGens","m_pdfLightData","m_pAccelStruct_m_primIndices","m_vTang4f","m_matIdOffsets","m_vNorm4f","m_remapInst","m_matIdByPrimId","m_pAccelStruct_m_abstractObjectPtrs","m_triIndices","m_normMatrices2","m_films_eta_k_vec","m_spec_offset_sz","all_references","m_spec_tex_offset_sz","m_spec_values","m_pAccelStruct_m_origNodes","m_pAccelStruct_m_vertPos","m_pAccelStruct_m_SdfFrameOctreeNodes","m_pAccelStruct_m_vertNorm","m_pAccelStruct_m_SdfCompactOctreeV2Data","m_pAccelStruct_m_SdfCompactOctreeRotModifiers","m_pAccelStruct_m_SdfSBSHeaders","m_pAccelStruct_m_SdfSBSDataF","m_lights","m_pAccelStruct_m_CatmulClarkHeaders","m_pAccelStruct_m_SdfFrameOctreeRoots","m_spec_tex_ids_wavelengths","m_pAccelStruct_m_primIdCount","m_pAccelStruct_m_SdfSVSNodes","m_pAccelStruct_m_allNodePairs","m_pAccelStruct_m_SdfSVSRoots","m_pAccelStruct_m_geomData","m_vertOffset","m_films_spec_id_vec","m_pAccelStruct_m_SdfSBSData","m_pAccelStruct_m_NURBS_approxes","m_pAccelStruct_startEnd","m_allRemapLists","m_pAccelStruct_m_NURBSData","m_allRemapListsOffsets","m_pAccelStruct_m_RibbonHeaders","m_pAccelStruct_m_SdfCompactOctreeV3Data","m_pAccelStruct_m_SdfSBSNodes","m_instIdToLightInstId","m_normMatrices","m_textures","m_pAccelStruct_m_SdfSBSRoots","m_materials","m_pAccelStruct_m_indices","m_pAccelStruct_m_instanceData","m_precomp_thin_films","m_precomp_coat_transmittance","m_pAccelStruct_m_nodesTLAS","m_pAccelStruct_m_NURBSHeaders"])
  {
    constexpr uint additionalSize = 1;

    std::array<VkDescriptorBufferInfo, 59 + additionalSize> descriptorBufferInfo;
    std::array<VkDescriptorImageInfo,  59 + additionalSize> descriptorImageInfo;
    std::array<VkWriteDescriptorSet,   59 + additionalSize> writeDescriptorSet;

    descriptorBufferInfo[0]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[0].buffer = PathTraceFromInputRays_local.in_rayPosAndNearBuffer;
    descriptorBufferInfo[0].offset = PathTraceFromInputRays_local.in_rayPosAndNearOffset;
    descriptorBufferInfo[0].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[0]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[0].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[0].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[0].dstBinding       = 0;
    writeDescriptorSet[0].descriptorCount  = 1;
    writeDescriptorSet[0].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[0].pBufferInfo      = &descriptorBufferInfo[0];
    writeDescriptorSet[0].pImageInfo       = nullptr;
    writeDescriptorSet[0].pTexelBufferView = nullptr;

    descriptorBufferInfo[1]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[1].buffer = PathTraceFromInputRays_local.in_rayDirAndFarBuffer;
    descriptorBufferInfo[1].offset = PathTraceFromInputRays_local.in_rayDirAndFarOffset;
    descriptorBufferInfo[1].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[1]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[1].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[1].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[1].dstBinding       = 1;
    writeDescriptorSet[1].descriptorCount  = 1;
    writeDescriptorSet[1].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[1].pBufferInfo      = &descriptorBufferInfo[1];
    writeDescriptorSet[1].pImageInfo       = nullptr;
    writeDescriptorSet[1].pTexelBufferView = nullptr;

    descriptorBufferInfo[2]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[2].buffer = PathTraceFromInputRays_local.out_colorBuffer;
    descriptorBufferInfo[2].offset = PathTraceFromInputRays_local.out_colorOffset;
    descriptorBufferInfo[2].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[2]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[2].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[2].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[2].dstBinding       = 2;
    writeDescriptorSet[2].descriptorCount  = 1;
    writeDescriptorSet[2].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[2].pBufferInfo      = &descriptorBufferInfo[2];
    writeDescriptorSet[2].pImageInfo       = nullptr;
    writeDescriptorSet[2].pTexelBufferView = nullptr;

    descriptorBufferInfo[3]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[3].buffer = m_vdata.m_packedXYBuffer;
    descriptorBufferInfo[3].offset = m_vdata.m_packedXYOffset;
    descriptorBufferInfo[3].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[3]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[3].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[3].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[3].dstBinding       = 3;
    writeDescriptorSet[3].descriptorCount  = 1;
    writeDescriptorSet[3].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[3].pBufferInfo      = &descriptorBufferInfo[3];
    writeDescriptorSet[3].pImageInfo       = nullptr;
    writeDescriptorSet[3].pTexelBufferView = nullptr;

    descriptorBufferInfo[4]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[4].buffer = m_vdata.m_randomGensBuffer;
    descriptorBufferInfo[4].offset = m_vdata.m_randomGensOffset;
    descriptorBufferInfo[4].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[4]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[4].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[4].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[4].dstBinding       = 4;
    writeDescriptorSet[4].descriptorCount  = 1;
    writeDescriptorSet[4].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[4].pBufferInfo      = &descriptorBufferInfo[4];
    writeDescriptorSet[4].pImageInfo       = nullptr;
    writeDescriptorSet[4].pTexelBufferView = nullptr;

    descriptorBufferInfo[5]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[5].buffer = m_vdata.m_pdfLightDataBuffer;
    descriptorBufferInfo[5].offset = m_vdata.m_pdfLightDataOffset;
    descriptorBufferInfo[5].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[5]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[5].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[5].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[5].dstBinding       = 5;
    writeDescriptorSet[5].descriptorCount  = 1;
    writeDescriptorSet[5].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[5].pBufferInfo      = &descriptorBufferInfo[5];
    writeDescriptorSet[5].pImageInfo       = nullptr;
    writeDescriptorSet[5].pTexelBufferView = nullptr;

    descriptorBufferInfo[6]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[6].buffer = m_vdata.m_pAccelStruct_m_primIndicesBuffer;
    descriptorBufferInfo[6].offset = m_vdata.m_pAccelStruct_m_primIndicesOffset;
    descriptorBufferInfo[6].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[6]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[6].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[6].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[6].dstBinding       = 6;
    writeDescriptorSet[6].descriptorCount  = 1;
    writeDescriptorSet[6].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[6].pBufferInfo      = &descriptorBufferInfo[6];
    writeDescriptorSet[6].pImageInfo       = nullptr;
    writeDescriptorSet[6].pTexelBufferView = nullptr;

    descriptorBufferInfo[7]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[7].buffer = m_vdata.m_vTang4fBuffer;
    descriptorBufferInfo[7].offset = m_vdata.m_vTang4fOffset;
    descriptorBufferInfo[7].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[7]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[7].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[7].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[7].dstBinding       = 7;
    writeDescriptorSet[7].descriptorCount  = 1;
    writeDescriptorSet[7].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[7].pBufferInfo      = &descriptorBufferInfo[7];
    writeDescriptorSet[7].pImageInfo       = nullptr;
    writeDescriptorSet[7].pTexelBufferView = nullptr;

    descriptorBufferInfo[8]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[8].buffer = m_vdata.m_matIdOffsetsBuffer;
    descriptorBufferInfo[8].offset = m_vdata.m_matIdOffsetsOffset;
    descriptorBufferInfo[8].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[8]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[8].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[8].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[8].dstBinding       = 8;
    writeDescriptorSet[8].descriptorCount  = 1;
    writeDescriptorSet[8].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[8].pBufferInfo      = &descriptorBufferInfo[8];
    writeDescriptorSet[8].pImageInfo       = nullptr;
    writeDescriptorSet[8].pTexelBufferView = nullptr;

    descriptorBufferInfo[9]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[9].buffer = m_vdata.m_vNorm4fBuffer;
    descriptorBufferInfo[9].offset = m_vdata.m_vNorm4fOffset;
    descriptorBufferInfo[9].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[9]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[9].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[9].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[9].dstBinding       = 9;
    writeDescriptorSet[9].descriptorCount  = 1;
    writeDescriptorSet[9].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[9].pBufferInfo      = &descriptorBufferInfo[9];
    writeDescriptorSet[9].pImageInfo       = nullptr;
    writeDescriptorSet[9].pTexelBufferView = nullptr;

    descriptorBufferInfo[10]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[10].buffer = m_vdata.m_remapInstBuffer;
    descriptorBufferInfo[10].offset = m_vdata.m_remapInstOffset;
    descriptorBufferInfo[10].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[10]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[10].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[10].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[10].dstBinding       = 10;
    writeDescriptorSet[10].descriptorCount  = 1;
    writeDescriptorSet[10].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[10].pBufferInfo      = &descriptorBufferInfo[10];
    writeDescriptorSet[10].pImageInfo       = nullptr;
    writeDescriptorSet[10].pTexelBufferView = nullptr;

    descriptorBufferInfo[11]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[11].buffer = m_vdata.m_matIdByPrimIdBuffer;
    descriptorBufferInfo[11].offset = m_vdata.m_matIdByPrimIdOffset;
    descriptorBufferInfo[11].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[11]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[11].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[11].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[11].dstBinding       = 11;
    writeDescriptorSet[11].descriptorCount  = 1;
    writeDescriptorSet[11].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[11].pBufferInfo      = &descriptorBufferInfo[11];
    writeDescriptorSet[11].pImageInfo       = nullptr;
    writeDescriptorSet[11].pTexelBufferView = nullptr;

    descriptorBufferInfo[12]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[12].buffer = m_vdata.m_pAccelStruct_m_abstractObjectPtrsBuffer;
    descriptorBufferInfo[12].offset = m_vdata.m_pAccelStruct_m_abstractObjectPtrsOffset;
    descriptorBufferInfo[12].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[12]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[12].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[12].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[12].dstBinding       = 12;
    writeDescriptorSet[12].descriptorCount  = 1;
    writeDescriptorSet[12].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[12].pBufferInfo      = &descriptorBufferInfo[12];
    writeDescriptorSet[12].pImageInfo       = nullptr;
    writeDescriptorSet[12].pTexelBufferView = nullptr;

    descriptorBufferInfo[13]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[13].buffer = m_vdata.m_triIndicesBuffer;
    descriptorBufferInfo[13].offset = m_vdata.m_triIndicesOffset;
    descriptorBufferInfo[13].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[13]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[13].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[13].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[13].dstBinding       = 13;
    writeDescriptorSet[13].descriptorCount  = 1;
    writeDescriptorSet[13].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[13].pBufferInfo      = &descriptorBufferInfo[13];
    writeDescriptorSet[13].pImageInfo       = nullptr;
    writeDescriptorSet[13].pTexelBufferView = nullptr;

    descriptorBufferInfo[14]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[14].buffer = m_vdata.m_normMatrices2Buffer;
    descriptorBufferInfo[14].offset = m_vdata.m_normMatrices2Offset;
    descriptorBufferInfo[14].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[14]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[14].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[14].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[14].dstBinding       = 14;
    writeDescriptorSet[14].descriptorCount  = 1;
    writeDescriptorSet[14].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[14].pBufferInfo      = &descriptorBufferInfo[14];
    writeDescriptorSet[14].pImageInfo       = nullptr;
    writeDescriptorSet[14].pTexelBufferView = nullptr;

    descriptorBufferInfo[15]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[15].buffer = m_vdata.m_films_eta_k_vecBuffer;
    descriptorBufferInfo[15].offset = m_vdata.m_films_eta_k_vecOffset;
    descriptorBufferInfo[15].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[15]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[15].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[15].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[15].dstBinding       = 15;
    writeDescriptorSet[15].descriptorCount  = 1;
    writeDescriptorSet[15].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[15].pBufferInfo      = &descriptorBufferInfo[15];
    writeDescriptorSet[15].pImageInfo       = nullptr;
    writeDescriptorSet[15].pTexelBufferView = nullptr;

    descriptorBufferInfo[16]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[16].buffer = m_vdata.m_spec_offset_szBuffer;
    descriptorBufferInfo[16].offset = m_vdata.m_spec_offset_szOffset;
    descriptorBufferInfo[16].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[16]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[16].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[16].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[16].dstBinding       = 16;
    writeDescriptorSet[16].descriptorCount  = 1;
    writeDescriptorSet[16].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[16].pBufferInfo      = &descriptorBufferInfo[16];
    writeDescriptorSet[16].pImageInfo       = nullptr;
    writeDescriptorSet[16].pTexelBufferView = nullptr;

    descriptorBufferInfo[17]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[17].buffer = m_vdata.all_referencesBuffer;
    descriptorBufferInfo[17].offset = m_vdata.all_referencesOffset;
    descriptorBufferInfo[17].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[17]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[17].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[17].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[17].dstBinding       = 17;
    writeDescriptorSet[17].descriptorCount  = 1;
    writeDescriptorSet[17].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[17].pBufferInfo      = &descriptorBufferInfo[17];
    writeDescriptorSet[17].pImageInfo       = nullptr;
    writeDescriptorSet[17].pTexelBufferView = nullptr;

    descriptorBufferInfo[18]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[18].buffer = m_vdata.m_spec_tex_offset_szBuffer;
    descriptorBufferInfo[18].offset = m_vdata.m_spec_tex_offset_szOffset;
    descriptorBufferInfo[18].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[18]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[18].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[18].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[18].dstBinding       = 18;
    writeDescriptorSet[18].descriptorCount  = 1;
    writeDescriptorSet[18].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[18].pBufferInfo      = &descriptorBufferInfo[18];
    writeDescriptorSet[18].pImageInfo       = nullptr;
    writeDescriptorSet[18].pTexelBufferView = nullptr;

    descriptorBufferInfo[19]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[19].buffer = m_vdata.m_spec_valuesBuffer;
    descriptorBufferInfo[19].offset = m_vdata.m_spec_valuesOffset;
    descriptorBufferInfo[19].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[19]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[19].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[19].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[19].dstBinding       = 19;
    writeDescriptorSet[19].descriptorCount  = 1;
    writeDescriptorSet[19].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[19].pBufferInfo      = &descriptorBufferInfo[19];
    writeDescriptorSet[19].pImageInfo       = nullptr;
    writeDescriptorSet[19].pTexelBufferView = nullptr;

    descriptorBufferInfo[20]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[20].buffer = m_vdata.m_pAccelStruct_m_origNodesBuffer;
    descriptorBufferInfo[20].offset = m_vdata.m_pAccelStruct_m_origNodesOffset;
    descriptorBufferInfo[20].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[20]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[20].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[20].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[20].dstBinding       = 20;
    writeDescriptorSet[20].descriptorCount  = 1;
    writeDescriptorSet[20].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[20].pBufferInfo      = &descriptorBufferInfo[20];
    writeDescriptorSet[20].pImageInfo       = nullptr;
    writeDescriptorSet[20].pTexelBufferView = nullptr;

    descriptorBufferInfo[21]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[21].buffer = m_vdata.m_pAccelStruct_m_vertPosBuffer;
    descriptorBufferInfo[21].offset = m_vdata.m_pAccelStruct_m_vertPosOffset;
    descriptorBufferInfo[21].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[21]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[21].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[21].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[21].dstBinding       = 21;
    writeDescriptorSet[21].descriptorCount  = 1;
    writeDescriptorSet[21].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[21].pBufferInfo      = &descriptorBufferInfo[21];
    writeDescriptorSet[21].pImageInfo       = nullptr;
    writeDescriptorSet[21].pTexelBufferView = nullptr;

    descriptorBufferInfo[22]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[22].buffer = m_vdata.m_pAccelStruct_m_SdfFrameOctreeNodesBuffer;
    descriptorBufferInfo[22].offset = m_vdata.m_pAccelStruct_m_SdfFrameOctreeNodesOffset;
    descriptorBufferInfo[22].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[22]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[22].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[22].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[22].dstBinding       = 22;
    writeDescriptorSet[22].descriptorCount  = 1;
    writeDescriptorSet[22].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[22].pBufferInfo      = &descriptorBufferInfo[22];
    writeDescriptorSet[22].pImageInfo       = nullptr;
    writeDescriptorSet[22].pTexelBufferView = nullptr;

    descriptorBufferInfo[23]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[23].buffer = m_vdata.m_pAccelStruct_m_vertNormBuffer;
    descriptorBufferInfo[23].offset = m_vdata.m_pAccelStruct_m_vertNormOffset;
    descriptorBufferInfo[23].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[23]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[23].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[23].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[23].dstBinding       = 23;
    writeDescriptorSet[23].descriptorCount  = 1;
    writeDescriptorSet[23].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[23].pBufferInfo      = &descriptorBufferInfo[23];
    writeDescriptorSet[23].pImageInfo       = nullptr;
    writeDescriptorSet[23].pTexelBufferView = nullptr;

    descriptorBufferInfo[24]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[24].buffer = m_vdata.m_pAccelStruct_m_SdfCompactOctreeV2DataBuffer;
    descriptorBufferInfo[24].offset = m_vdata.m_pAccelStruct_m_SdfCompactOctreeV2DataOffset;
    descriptorBufferInfo[24].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[24]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[24].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[24].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[24].dstBinding       = 24;
    writeDescriptorSet[24].descriptorCount  = 1;
    writeDescriptorSet[24].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[24].pBufferInfo      = &descriptorBufferInfo[24];
    writeDescriptorSet[24].pImageInfo       = nullptr;
    writeDescriptorSet[24].pTexelBufferView = nullptr;

    descriptorBufferInfo[25]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[25].buffer = m_vdata.m_pAccelStruct_m_SdfCompactOctreeRotModifiersBuffer;
    descriptorBufferInfo[25].offset = m_vdata.m_pAccelStruct_m_SdfCompactOctreeRotModifiersOffset;
    descriptorBufferInfo[25].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[25]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[25].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[25].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[25].dstBinding       = 25;
    writeDescriptorSet[25].descriptorCount  = 1;
    writeDescriptorSet[25].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[25].pBufferInfo      = &descriptorBufferInfo[25];
    writeDescriptorSet[25].pImageInfo       = nullptr;
    writeDescriptorSet[25].pTexelBufferView = nullptr;

    descriptorBufferInfo[26]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[26].buffer = m_vdata.m_pAccelStruct_m_SdfSBSHeadersBuffer;
    descriptorBufferInfo[26].offset = m_vdata.m_pAccelStruct_m_SdfSBSHeadersOffset;
    descriptorBufferInfo[26].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[26]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[26].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[26].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[26].dstBinding       = 26;
    writeDescriptorSet[26].descriptorCount  = 1;
    writeDescriptorSet[26].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[26].pBufferInfo      = &descriptorBufferInfo[26];
    writeDescriptorSet[26].pImageInfo       = nullptr;
    writeDescriptorSet[26].pTexelBufferView = nullptr;

    descriptorBufferInfo[27]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[27].buffer = m_vdata.m_pAccelStruct_m_SdfSBSDataFBuffer;
    descriptorBufferInfo[27].offset = m_vdata.m_pAccelStruct_m_SdfSBSDataFOffset;
    descriptorBufferInfo[27].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[27]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[27].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[27].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[27].dstBinding       = 27;
    writeDescriptorSet[27].descriptorCount  = 1;
    writeDescriptorSet[27].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[27].pBufferInfo      = &descriptorBufferInfo[27];
    writeDescriptorSet[27].pImageInfo       = nullptr;
    writeDescriptorSet[27].pTexelBufferView = nullptr;

    descriptorBufferInfo[28]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[28].buffer = m_vdata.m_lightsBuffer;
    descriptorBufferInfo[28].offset = m_vdata.m_lightsOffset;
    descriptorBufferInfo[28].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[28]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[28].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[28].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[28].dstBinding       = 28;
    writeDescriptorSet[28].descriptorCount  = 1;
    writeDescriptorSet[28].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[28].pBufferInfo      = &descriptorBufferInfo[28];
    writeDescriptorSet[28].pImageInfo       = nullptr;
    writeDescriptorSet[28].pTexelBufferView = nullptr;

    descriptorBufferInfo[29]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[29].buffer = m_vdata.m_pAccelStruct_m_CatmulClarkHeadersBuffer;
    descriptorBufferInfo[29].offset = m_vdata.m_pAccelStruct_m_CatmulClarkHeadersOffset;
    descriptorBufferInfo[29].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[29]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[29].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[29].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[29].dstBinding       = 29;
    writeDescriptorSet[29].descriptorCount  = 1;
    writeDescriptorSet[29].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[29].pBufferInfo      = &descriptorBufferInfo[29];
    writeDescriptorSet[29].pImageInfo       = nullptr;
    writeDescriptorSet[29].pTexelBufferView = nullptr;

    descriptorBufferInfo[30]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[30].buffer = m_vdata.m_pAccelStruct_m_SdfFrameOctreeRootsBuffer;
    descriptorBufferInfo[30].offset = m_vdata.m_pAccelStruct_m_SdfFrameOctreeRootsOffset;
    descriptorBufferInfo[30].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[30]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[30].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[30].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[30].dstBinding       = 30;
    writeDescriptorSet[30].descriptorCount  = 1;
    writeDescriptorSet[30].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[30].pBufferInfo      = &descriptorBufferInfo[30];
    writeDescriptorSet[30].pImageInfo       = nullptr;
    writeDescriptorSet[30].pTexelBufferView = nullptr;

    descriptorBufferInfo[31]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[31].buffer = m_vdata.m_spec_tex_ids_wavelengthsBuffer;
    descriptorBufferInfo[31].offset = m_vdata.m_spec_tex_ids_wavelengthsOffset;
    descriptorBufferInfo[31].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[31]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[31].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[31].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[31].dstBinding       = 31;
    writeDescriptorSet[31].descriptorCount  = 1;
    writeDescriptorSet[31].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[31].pBufferInfo      = &descriptorBufferInfo[31];
    writeDescriptorSet[31].pImageInfo       = nullptr;
    writeDescriptorSet[31].pTexelBufferView = nullptr;

    descriptorBufferInfo[32]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[32].buffer = m_vdata.m_pAccelStruct_m_primIdCountBuffer;
    descriptorBufferInfo[32].offset = m_vdata.m_pAccelStruct_m_primIdCountOffset;
    descriptorBufferInfo[32].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[32]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[32].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[32].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[32].dstBinding       = 32;
    writeDescriptorSet[32].descriptorCount  = 1;
    writeDescriptorSet[32].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[32].pBufferInfo      = &descriptorBufferInfo[32];
    writeDescriptorSet[32].pImageInfo       = nullptr;
    writeDescriptorSet[32].pTexelBufferView = nullptr;

    descriptorBufferInfo[33]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[33].buffer = m_vdata.m_pAccelStruct_m_SdfSVSNodesBuffer;
    descriptorBufferInfo[33].offset = m_vdata.m_pAccelStruct_m_SdfSVSNodesOffset;
    descriptorBufferInfo[33].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[33]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[33].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[33].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[33].dstBinding       = 33;
    writeDescriptorSet[33].descriptorCount  = 1;
    writeDescriptorSet[33].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[33].pBufferInfo      = &descriptorBufferInfo[33];
    writeDescriptorSet[33].pImageInfo       = nullptr;
    writeDescriptorSet[33].pTexelBufferView = nullptr;

    descriptorBufferInfo[34]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[34].buffer = m_vdata.m_pAccelStruct_m_allNodePairsBuffer;
    descriptorBufferInfo[34].offset = m_vdata.m_pAccelStruct_m_allNodePairsOffset;
    descriptorBufferInfo[34].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[34]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[34].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[34].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[34].dstBinding       = 34;
    writeDescriptorSet[34].descriptorCount  = 1;
    writeDescriptorSet[34].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[34].pBufferInfo      = &descriptorBufferInfo[34];
    writeDescriptorSet[34].pImageInfo       = nullptr;
    writeDescriptorSet[34].pTexelBufferView = nullptr;

    descriptorBufferInfo[35]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[35].buffer = m_vdata.m_pAccelStruct_m_SdfSVSRootsBuffer;
    descriptorBufferInfo[35].offset = m_vdata.m_pAccelStruct_m_SdfSVSRootsOffset;
    descriptorBufferInfo[35].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[35]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[35].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[35].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[35].dstBinding       = 35;
    writeDescriptorSet[35].descriptorCount  = 1;
    writeDescriptorSet[35].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[35].pBufferInfo      = &descriptorBufferInfo[35];
    writeDescriptorSet[35].pImageInfo       = nullptr;
    writeDescriptorSet[35].pTexelBufferView = nullptr;

    descriptorBufferInfo[36]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[36].buffer = m_vdata.m_pAccelStruct_m_geomDataBuffer;
    descriptorBufferInfo[36].offset = m_vdata.m_pAccelStruct_m_geomDataOffset;
    descriptorBufferInfo[36].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[36]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[36].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[36].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[36].dstBinding       = 36;
    writeDescriptorSet[36].descriptorCount  = 1;
    writeDescriptorSet[36].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[36].pBufferInfo      = &descriptorBufferInfo[36];
    writeDescriptorSet[36].pImageInfo       = nullptr;
    writeDescriptorSet[36].pTexelBufferView = nullptr;

    descriptorBufferInfo[37]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[37].buffer = m_vdata.m_vertOffsetBuffer;
    descriptorBufferInfo[37].offset = m_vdata.m_vertOffsetOffset;
    descriptorBufferInfo[37].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[37]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[37].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[37].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[37].dstBinding       = 37;
    writeDescriptorSet[37].descriptorCount  = 1;
    writeDescriptorSet[37].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[37].pBufferInfo      = &descriptorBufferInfo[37];
    writeDescriptorSet[37].pImageInfo       = nullptr;
    writeDescriptorSet[37].pTexelBufferView = nullptr;

    descriptorBufferInfo[38]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[38].buffer = m_vdata.m_films_spec_id_vecBuffer;
    descriptorBufferInfo[38].offset = m_vdata.m_films_spec_id_vecOffset;
    descriptorBufferInfo[38].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[38]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[38].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[38].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[38].dstBinding       = 38;
    writeDescriptorSet[38].descriptorCount  = 1;
    writeDescriptorSet[38].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[38].pBufferInfo      = &descriptorBufferInfo[38];
    writeDescriptorSet[38].pImageInfo       = nullptr;
    writeDescriptorSet[38].pTexelBufferView = nullptr;

    descriptorBufferInfo[39]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[39].buffer = m_vdata.m_pAccelStruct_m_SdfSBSDataBuffer;
    descriptorBufferInfo[39].offset = m_vdata.m_pAccelStruct_m_SdfSBSDataOffset;
    descriptorBufferInfo[39].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[39]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[39].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[39].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[39].dstBinding       = 39;
    writeDescriptorSet[39].descriptorCount  = 1;
    writeDescriptorSet[39].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[39].pBufferInfo      = &descriptorBufferInfo[39];
    writeDescriptorSet[39].pImageInfo       = nullptr;
    writeDescriptorSet[39].pTexelBufferView = nullptr;

    descriptorBufferInfo[40]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[40].buffer = m_vdata.m_pAccelStruct_m_NURBS_approxesBuffer;
    descriptorBufferInfo[40].offset = m_vdata.m_pAccelStruct_m_NURBS_approxesOffset;
    descriptorBufferInfo[40].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[40]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[40].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[40].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[40].dstBinding       = 40;
    writeDescriptorSet[40].descriptorCount  = 1;
    writeDescriptorSet[40].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[40].pBufferInfo      = &descriptorBufferInfo[40];
    writeDescriptorSet[40].pImageInfo       = nullptr;
    writeDescriptorSet[40].pTexelBufferView = nullptr;

    descriptorBufferInfo[41]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[41].buffer = m_vdata.m_pAccelStruct_startEndBuffer;
    descriptorBufferInfo[41].offset = m_vdata.m_pAccelStruct_startEndOffset;
    descriptorBufferInfo[41].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[41]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[41].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[41].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[41].dstBinding       = 41;
    writeDescriptorSet[41].descriptorCount  = 1;
    writeDescriptorSet[41].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[41].pBufferInfo      = &descriptorBufferInfo[41];
    writeDescriptorSet[41].pImageInfo       = nullptr;
    writeDescriptorSet[41].pTexelBufferView = nullptr;

    descriptorBufferInfo[42]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[42].buffer = m_vdata.m_allRemapListsBuffer;
    descriptorBufferInfo[42].offset = m_vdata.m_allRemapListsOffset;
    descriptorBufferInfo[42].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[42]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[42].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[42].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[42].dstBinding       = 42;
    writeDescriptorSet[42].descriptorCount  = 1;
    writeDescriptorSet[42].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[42].pBufferInfo      = &descriptorBufferInfo[42];
    writeDescriptorSet[42].pImageInfo       = nullptr;
    writeDescriptorSet[42].pTexelBufferView = nullptr;

    descriptorBufferInfo[43]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[43].buffer = m_vdata.m_pAccelStruct_m_NURBSDataBuffer;
    descriptorBufferInfo[43].offset = m_vdata.m_pAccelStruct_m_NURBSDataOffset;
    descriptorBufferInfo[43].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[43]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[43].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[43].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[43].dstBinding       = 43;
    writeDescriptorSet[43].descriptorCount  = 1;
    writeDescriptorSet[43].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[43].pBufferInfo      = &descriptorBufferInfo[43];
    writeDescriptorSet[43].pImageInfo       = nullptr;
    writeDescriptorSet[43].pTexelBufferView = nullptr;

    descriptorBufferInfo[44]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[44].buffer = m_vdata.m_allRemapListsOffsetsBuffer;
    descriptorBufferInfo[44].offset = m_vdata.m_allRemapListsOffsetsOffset;
    descriptorBufferInfo[44].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[44]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[44].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[44].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[44].dstBinding       = 44;
    writeDescriptorSet[44].descriptorCount  = 1;
    writeDescriptorSet[44].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[44].pBufferInfo      = &descriptorBufferInfo[44];
    writeDescriptorSet[44].pImageInfo       = nullptr;
    writeDescriptorSet[44].pTexelBufferView = nullptr;

    descriptorBufferInfo[45]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[45].buffer = m_vdata.m_pAccelStruct_m_RibbonHeadersBuffer;
    descriptorBufferInfo[45].offset = m_vdata.m_pAccelStruct_m_RibbonHeadersOffset;
    descriptorBufferInfo[45].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[45]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[45].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[45].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[45].dstBinding       = 45;
    writeDescriptorSet[45].descriptorCount  = 1;
    writeDescriptorSet[45].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[45].pBufferInfo      = &descriptorBufferInfo[45];
    writeDescriptorSet[45].pImageInfo       = nullptr;
    writeDescriptorSet[45].pTexelBufferView = nullptr;

    descriptorBufferInfo[46]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[46].buffer = m_vdata.m_pAccelStruct_m_SdfCompactOctreeV3DataBuffer;
    descriptorBufferInfo[46].offset = m_vdata.m_pAccelStruct_m_SdfCompactOctreeV3DataOffset;
    descriptorBufferInfo[46].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[46]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[46].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[46].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[46].dstBinding       = 46;
    writeDescriptorSet[46].descriptorCount  = 1;
    writeDescriptorSet[46].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[46].pBufferInfo      = &descriptorBufferInfo[46];
    writeDescriptorSet[46].pImageInfo       = nullptr;
    writeDescriptorSet[46].pTexelBufferView = nullptr;

    descriptorBufferInfo[47]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[47].buffer = m_vdata.m_pAccelStruct_m_SdfSBSNodesBuffer;
    descriptorBufferInfo[47].offset = m_vdata.m_pAccelStruct_m_SdfSBSNodesOffset;
    descriptorBufferInfo[47].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[47]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[47].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[47].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[47].dstBinding       = 47;
    writeDescriptorSet[47].descriptorCount  = 1;
    writeDescriptorSet[47].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[47].pBufferInfo      = &descriptorBufferInfo[47];
    writeDescriptorSet[47].pImageInfo       = nullptr;
    writeDescriptorSet[47].pTexelBufferView = nullptr;

    descriptorBufferInfo[48]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[48].buffer = m_vdata.m_instIdToLightInstIdBuffer;
    descriptorBufferInfo[48].offset = m_vdata.m_instIdToLightInstIdOffset;
    descriptorBufferInfo[48].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[48]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[48].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[48].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[48].dstBinding       = 48;
    writeDescriptorSet[48].descriptorCount  = 1;
    writeDescriptorSet[48].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[48].pBufferInfo      = &descriptorBufferInfo[48];
    writeDescriptorSet[48].pImageInfo       = nullptr;
    writeDescriptorSet[48].pTexelBufferView = nullptr;

    descriptorBufferInfo[49]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[49].buffer = m_vdata.m_normMatricesBuffer;
    descriptorBufferInfo[49].offset = m_vdata.m_normMatricesOffset;
    descriptorBufferInfo[49].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[49]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[49].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[49].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[49].dstBinding       = 49;
    writeDescriptorSet[49].descriptorCount  = 1;
    writeDescriptorSet[49].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[49].pBufferInfo      = &descriptorBufferInfo[49];
    writeDescriptorSet[49].pImageInfo       = nullptr;
    writeDescriptorSet[49].pTexelBufferView = nullptr;

    std::vector<VkDescriptorImageInfo> m_texturesInfo(m_vdata.m_texturesArrayMaxSize);
    for(size_t i=0; i<m_vdata.m_texturesArrayMaxSize; i++)
    {
      if(i < m_textures.size())
      {
        m_texturesInfo[i].sampler     = m_vdata.m_texturesArraySampler[i];
        m_texturesInfo[i].imageView   = m_vdata.m_texturesArrayView   [i];
        m_texturesInfo[i].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
      }
      else
      {
        m_texturesInfo[i].sampler     = m_vdata.m_texturesArraySampler[0];
        m_texturesInfo[i].imageView   = m_vdata.m_texturesArrayView   [0];
        m_texturesInfo[i].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
      }
    }
    writeDescriptorSet[50]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[50].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[50].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[50].dstBinding       = 50;
    writeDescriptorSet[50].descriptorCount  = 1;
    writeDescriptorSet[50].descriptorCount  = m_texturesInfo.size();
    writeDescriptorSet[50].descriptorType   = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    writeDescriptorSet[50].pBufferInfo      = nullptr;
    writeDescriptorSet[50].pImageInfo       = m_texturesInfo.data();
    writeDescriptorSet[50].pTexelBufferView = nullptr;

    descriptorBufferInfo[51]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[51].buffer = m_vdata.m_pAccelStruct_m_SdfSBSRootsBuffer;
    descriptorBufferInfo[51].offset = m_vdata.m_pAccelStruct_m_SdfSBSRootsOffset;
    descriptorBufferInfo[51].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[51]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[51].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[51].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[51].dstBinding       = 51;
    writeDescriptorSet[51].descriptorCount  = 1;
    writeDescriptorSet[51].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[51].pBufferInfo      = &descriptorBufferInfo[51];
    writeDescriptorSet[51].pImageInfo       = nullptr;
    writeDescriptorSet[51].pTexelBufferView = nullptr;

    descriptorBufferInfo[52]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[52].buffer = m_vdata.m_materialsBuffer;
    descriptorBufferInfo[52].offset = m_vdata.m_materialsOffset;
    descriptorBufferInfo[52].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[52]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[52].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[52].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[52].dstBinding       = 52;
    writeDescriptorSet[52].descriptorCount  = 1;
    writeDescriptorSet[52].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[52].pBufferInfo      = &descriptorBufferInfo[52];
    writeDescriptorSet[52].pImageInfo       = nullptr;
    writeDescriptorSet[52].pTexelBufferView = nullptr;

    descriptorBufferInfo[53]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[53].buffer = m_vdata.m_pAccelStruct_m_indicesBuffer;
    descriptorBufferInfo[53].offset = m_vdata.m_pAccelStruct_m_indicesOffset;
    descriptorBufferInfo[53].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[53]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[53].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[53].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[53].dstBinding       = 53;
    writeDescriptorSet[53].descriptorCount  = 1;
    writeDescriptorSet[53].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[53].pBufferInfo      = &descriptorBufferInfo[53];
    writeDescriptorSet[53].pImageInfo       = nullptr;
    writeDescriptorSet[53].pTexelBufferView = nullptr;

    descriptorBufferInfo[54]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[54].buffer = m_vdata.m_pAccelStruct_m_instanceDataBuffer;
    descriptorBufferInfo[54].offset = m_vdata.m_pAccelStruct_m_instanceDataOffset;
    descriptorBufferInfo[54].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[54]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[54].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[54].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[54].dstBinding       = 54;
    writeDescriptorSet[54].descriptorCount  = 1;
    writeDescriptorSet[54].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[54].pBufferInfo      = &descriptorBufferInfo[54];
    writeDescriptorSet[54].pImageInfo       = nullptr;
    writeDescriptorSet[54].pTexelBufferView = nullptr;

    descriptorBufferInfo[55]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[55].buffer = m_vdata.m_precomp_thin_filmsBuffer;
    descriptorBufferInfo[55].offset = m_vdata.m_precomp_thin_filmsOffset;
    descriptorBufferInfo[55].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[55]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[55].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[55].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[55].dstBinding       = 55;
    writeDescriptorSet[55].descriptorCount  = 1;
    writeDescriptorSet[55].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[55].pBufferInfo      = &descriptorBufferInfo[55];
    writeDescriptorSet[55].pImageInfo       = nullptr;
    writeDescriptorSet[55].pTexelBufferView = nullptr;

    descriptorBufferInfo[56]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[56].buffer = m_vdata.m_precomp_coat_transmittanceBuffer;
    descriptorBufferInfo[56].offset = m_vdata.m_precomp_coat_transmittanceOffset;
    descriptorBufferInfo[56].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[56]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[56].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[56].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[56].dstBinding       = 56;
    writeDescriptorSet[56].descriptorCount  = 1;
    writeDescriptorSet[56].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[56].pBufferInfo      = &descriptorBufferInfo[56];
    writeDescriptorSet[56].pImageInfo       = nullptr;
    writeDescriptorSet[56].pTexelBufferView = nullptr;

    descriptorBufferInfo[57]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[57].buffer = m_vdata.m_pAccelStruct_m_nodesTLASBuffer;
    descriptorBufferInfo[57].offset = m_vdata.m_pAccelStruct_m_nodesTLASOffset;
    descriptorBufferInfo[57].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[57]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[57].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[57].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[57].dstBinding       = 57;
    writeDescriptorSet[57].descriptorCount  = 1;
    writeDescriptorSet[57].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[57].pBufferInfo      = &descriptorBufferInfo[57];
    writeDescriptorSet[57].pImageInfo       = nullptr;
    writeDescriptorSet[57].pTexelBufferView = nullptr;

    descriptorBufferInfo[58]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[58].buffer = m_vdata.m_pAccelStruct_m_NURBSHeadersBuffer;
    descriptorBufferInfo[58].offset = m_vdata.m_pAccelStruct_m_NURBSHeadersOffset;
    descriptorBufferInfo[58].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[58]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[58].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[58].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[58].dstBinding       = 58;
    writeDescriptorSet[58].descriptorCount  = 1;
    writeDescriptorSet[58].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[58].pBufferInfo      = &descriptorBufferInfo[58];
    writeDescriptorSet[58].pImageInfo       = nullptr;
    writeDescriptorSet[58].pTexelBufferView = nullptr;

    descriptorBufferInfo[59]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[59].buffer = m_classDataBuffer;
    descriptorBufferInfo[59].offset = 0;
    descriptorBufferInfo[59].range  = VK_WHOLE_SIZE;

    writeDescriptorSet[59]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[59].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[59].dstSet           = m_allGeneratedDS[3];
    writeDescriptorSet[59].dstBinding       = 59;
    writeDescriptorSet[59].descriptorCount  = 1;
    writeDescriptorSet[59].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[59].pBufferInfo      = &descriptorBufferInfo[59];
    writeDescriptorSet[59].pImageInfo       = nullptr;
    writeDescriptorSet[59].pTexelBufferView = nullptr;

    vkUpdateDescriptorSets(device, uint32_t(writeDescriptorSet.size()), writeDescriptorSet.data(), 0, NULL);
  }
}

void Integrator_Generated::InitAllGeneratedDescriptorSets_PathTrace()
{
  // now create actual bindings
  //
  // descriptor set #4: PathTraceMegaCmd (["out_color","m_lines","m_pAccelStruct_m_NURBSHeaders","m_pAccelStruct_m_primIndices","m_precomp_thin_films","m_films_spec_id_vec","m_pAccelStruct_m_SdfSBSData","m_vertOffset","m_pAccelStruct_m_SdfCompactOctreeRotModifiers","m_cie_y","m_pAccelStruct_m_SdfCompactOctreeV3Data","m_pAccelStruct_m_RibbonHeaders","m_allRemapListsOffsets","m_pAccelStruct_m_indices","m_materials","m_pAccelStruct_m_SdfSVSRoots","m_pAccelStruct_m_SdfSVSNodes","m_pAccelStruct_m_SdfCompactOctreeV2Data","m_pAccelStruct_m_SdfFrameOctreeRoots","m_pdfLightData","m_pAccelStruct_m_primIdCount","m_spec_tex_ids_wavelengths","m_pAccelStruct_m_geomData","m_pAccelStruct_m_SdfSBSDataF","m_lights","m_cie_x","m_pAccelStruct_m_SdfSBSHeaders","m_textures","m_pAccelStruct_m_nodesTLAS","m_normMatrices","m_pAccelStruct_m_SdfSBSRoots","all_references","m_spec_tex_offset_sz","m_precomp_coat_transmittance","m_pAccelStruct_m_vertNorm","m_pAccelStruct_m_SdfFrameOctreeNodes","m_pAccelStruct_m_SdfSBSNodes","m_pAccelStruct_m_vertPos","m_pAccelStruct_m_origNodes","m_pAccelStruct_m_NURBS_approxes","m_spec_values","m_spec_offset_sz","m_films_eta_k_vec","m_normMatrices2","m_triIndices","m_pAccelStruct_m_NURBSData","m_allRemapLists","m_pAccelStruct_m_abstractObjectPtrs","m_pAccelStruct_startEnd","m_pAccelStruct_m_allNodePairs","m_pAccelStruct_m_CatmulClarkHeaders","m_instIdToLightInstId","m_matIdByPrimId","m_remapInst","m_pAccelStruct_m_instanceData","m_cie_z","m_vNorm4f","m_matIdOffsets","m_vTang4f","m_randomGens","m_packedXY"])
  {
    constexpr uint additionalSize = 1;

    std::array<VkDescriptorBufferInfo, 61 + additionalSize> descriptorBufferInfo;
    std::array<VkDescriptorImageInfo,  61 + additionalSize> descriptorImageInfo;
    std::array<VkWriteDescriptorSet,   61 + additionalSize> writeDescriptorSet;

    descriptorBufferInfo[0]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[0].buffer = PathTrace_local.out_colorBuffer;
    descriptorBufferInfo[0].offset = PathTrace_local.out_colorOffset;
    descriptorBufferInfo[0].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[0]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[0].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[0].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[0].dstBinding       = 0;
    writeDescriptorSet[0].descriptorCount  = 1;
    writeDescriptorSet[0].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[0].pBufferInfo      = &descriptorBufferInfo[0];
    writeDescriptorSet[0].pImageInfo       = nullptr;
    writeDescriptorSet[0].pTexelBufferView = nullptr;

    descriptorBufferInfo[1]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[1].buffer = m_vdata.m_linesBuffer;
    descriptorBufferInfo[1].offset = m_vdata.m_linesOffset;
    descriptorBufferInfo[1].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[1]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[1].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[1].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[1].dstBinding       = 1;
    writeDescriptorSet[1].descriptorCount  = 1;
    writeDescriptorSet[1].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[1].pBufferInfo      = &descriptorBufferInfo[1];
    writeDescriptorSet[1].pImageInfo       = nullptr;
    writeDescriptorSet[1].pTexelBufferView = nullptr;

    descriptorBufferInfo[2]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[2].buffer = m_vdata.m_pAccelStruct_m_NURBSHeadersBuffer;
    descriptorBufferInfo[2].offset = m_vdata.m_pAccelStruct_m_NURBSHeadersOffset;
    descriptorBufferInfo[2].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[2]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[2].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[2].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[2].dstBinding       = 2;
    writeDescriptorSet[2].descriptorCount  = 1;
    writeDescriptorSet[2].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[2].pBufferInfo      = &descriptorBufferInfo[2];
    writeDescriptorSet[2].pImageInfo       = nullptr;
    writeDescriptorSet[2].pTexelBufferView = nullptr;

    descriptorBufferInfo[3]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[3].buffer = m_vdata.m_pAccelStruct_m_primIndicesBuffer;
    descriptorBufferInfo[3].offset = m_vdata.m_pAccelStruct_m_primIndicesOffset;
    descriptorBufferInfo[3].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[3]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[3].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[3].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[3].dstBinding       = 3;
    writeDescriptorSet[3].descriptorCount  = 1;
    writeDescriptorSet[3].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[3].pBufferInfo      = &descriptorBufferInfo[3];
    writeDescriptorSet[3].pImageInfo       = nullptr;
    writeDescriptorSet[3].pTexelBufferView = nullptr;

    descriptorBufferInfo[4]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[4].buffer = m_vdata.m_precomp_thin_filmsBuffer;
    descriptorBufferInfo[4].offset = m_vdata.m_precomp_thin_filmsOffset;
    descriptorBufferInfo[4].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[4]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[4].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[4].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[4].dstBinding       = 4;
    writeDescriptorSet[4].descriptorCount  = 1;
    writeDescriptorSet[4].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[4].pBufferInfo      = &descriptorBufferInfo[4];
    writeDescriptorSet[4].pImageInfo       = nullptr;
    writeDescriptorSet[4].pTexelBufferView = nullptr;

    descriptorBufferInfo[5]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[5].buffer = m_vdata.m_films_spec_id_vecBuffer;
    descriptorBufferInfo[5].offset = m_vdata.m_films_spec_id_vecOffset;
    descriptorBufferInfo[5].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[5]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[5].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[5].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[5].dstBinding       = 5;
    writeDescriptorSet[5].descriptorCount  = 1;
    writeDescriptorSet[5].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[5].pBufferInfo      = &descriptorBufferInfo[5];
    writeDescriptorSet[5].pImageInfo       = nullptr;
    writeDescriptorSet[5].pTexelBufferView = nullptr;

    descriptorBufferInfo[6]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[6].buffer = m_vdata.m_pAccelStruct_m_SdfSBSDataBuffer;
    descriptorBufferInfo[6].offset = m_vdata.m_pAccelStruct_m_SdfSBSDataOffset;
    descriptorBufferInfo[6].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[6]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[6].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[6].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[6].dstBinding       = 6;
    writeDescriptorSet[6].descriptorCount  = 1;
    writeDescriptorSet[6].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[6].pBufferInfo      = &descriptorBufferInfo[6];
    writeDescriptorSet[6].pImageInfo       = nullptr;
    writeDescriptorSet[6].pTexelBufferView = nullptr;

    descriptorBufferInfo[7]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[7].buffer = m_vdata.m_vertOffsetBuffer;
    descriptorBufferInfo[7].offset = m_vdata.m_vertOffsetOffset;
    descriptorBufferInfo[7].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[7]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[7].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[7].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[7].dstBinding       = 7;
    writeDescriptorSet[7].descriptorCount  = 1;
    writeDescriptorSet[7].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[7].pBufferInfo      = &descriptorBufferInfo[7];
    writeDescriptorSet[7].pImageInfo       = nullptr;
    writeDescriptorSet[7].pTexelBufferView = nullptr;

    descriptorBufferInfo[8]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[8].buffer = m_vdata.m_pAccelStruct_m_SdfCompactOctreeRotModifiersBuffer;
    descriptorBufferInfo[8].offset = m_vdata.m_pAccelStruct_m_SdfCompactOctreeRotModifiersOffset;
    descriptorBufferInfo[8].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[8]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[8].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[8].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[8].dstBinding       = 8;
    writeDescriptorSet[8].descriptorCount  = 1;
    writeDescriptorSet[8].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[8].pBufferInfo      = &descriptorBufferInfo[8];
    writeDescriptorSet[8].pImageInfo       = nullptr;
    writeDescriptorSet[8].pTexelBufferView = nullptr;

    descriptorBufferInfo[9]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[9].buffer = m_vdata.m_cie_yBuffer;
    descriptorBufferInfo[9].offset = m_vdata.m_cie_yOffset;
    descriptorBufferInfo[9].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[9]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[9].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[9].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[9].dstBinding       = 9;
    writeDescriptorSet[9].descriptorCount  = 1;
    writeDescriptorSet[9].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[9].pBufferInfo      = &descriptorBufferInfo[9];
    writeDescriptorSet[9].pImageInfo       = nullptr;
    writeDescriptorSet[9].pTexelBufferView = nullptr;

    descriptorBufferInfo[10]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[10].buffer = m_vdata.m_pAccelStruct_m_SdfCompactOctreeV3DataBuffer;
    descriptorBufferInfo[10].offset = m_vdata.m_pAccelStruct_m_SdfCompactOctreeV3DataOffset;
    descriptorBufferInfo[10].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[10]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[10].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[10].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[10].dstBinding       = 10;
    writeDescriptorSet[10].descriptorCount  = 1;
    writeDescriptorSet[10].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[10].pBufferInfo      = &descriptorBufferInfo[10];
    writeDescriptorSet[10].pImageInfo       = nullptr;
    writeDescriptorSet[10].pTexelBufferView = nullptr;

    descriptorBufferInfo[11]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[11].buffer = m_vdata.m_pAccelStruct_m_RibbonHeadersBuffer;
    descriptorBufferInfo[11].offset = m_vdata.m_pAccelStruct_m_RibbonHeadersOffset;
    descriptorBufferInfo[11].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[11]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[11].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[11].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[11].dstBinding       = 11;
    writeDescriptorSet[11].descriptorCount  = 1;
    writeDescriptorSet[11].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[11].pBufferInfo      = &descriptorBufferInfo[11];
    writeDescriptorSet[11].pImageInfo       = nullptr;
    writeDescriptorSet[11].pTexelBufferView = nullptr;

    descriptorBufferInfo[12]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[12].buffer = m_vdata.m_allRemapListsOffsetsBuffer;
    descriptorBufferInfo[12].offset = m_vdata.m_allRemapListsOffsetsOffset;
    descriptorBufferInfo[12].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[12]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[12].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[12].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[12].dstBinding       = 12;
    writeDescriptorSet[12].descriptorCount  = 1;
    writeDescriptorSet[12].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[12].pBufferInfo      = &descriptorBufferInfo[12];
    writeDescriptorSet[12].pImageInfo       = nullptr;
    writeDescriptorSet[12].pTexelBufferView = nullptr;

    descriptorBufferInfo[13]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[13].buffer = m_vdata.m_pAccelStruct_m_indicesBuffer;
    descriptorBufferInfo[13].offset = m_vdata.m_pAccelStruct_m_indicesOffset;
    descriptorBufferInfo[13].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[13]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[13].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[13].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[13].dstBinding       = 13;
    writeDescriptorSet[13].descriptorCount  = 1;
    writeDescriptorSet[13].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[13].pBufferInfo      = &descriptorBufferInfo[13];
    writeDescriptorSet[13].pImageInfo       = nullptr;
    writeDescriptorSet[13].pTexelBufferView = nullptr;

    descriptorBufferInfo[14]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[14].buffer = m_vdata.m_materialsBuffer;
    descriptorBufferInfo[14].offset = m_vdata.m_materialsOffset;
    descriptorBufferInfo[14].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[14]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[14].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[14].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[14].dstBinding       = 14;
    writeDescriptorSet[14].descriptorCount  = 1;
    writeDescriptorSet[14].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[14].pBufferInfo      = &descriptorBufferInfo[14];
    writeDescriptorSet[14].pImageInfo       = nullptr;
    writeDescriptorSet[14].pTexelBufferView = nullptr;

    descriptorBufferInfo[15]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[15].buffer = m_vdata.m_pAccelStruct_m_SdfSVSRootsBuffer;
    descriptorBufferInfo[15].offset = m_vdata.m_pAccelStruct_m_SdfSVSRootsOffset;
    descriptorBufferInfo[15].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[15]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[15].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[15].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[15].dstBinding       = 15;
    writeDescriptorSet[15].descriptorCount  = 1;
    writeDescriptorSet[15].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[15].pBufferInfo      = &descriptorBufferInfo[15];
    writeDescriptorSet[15].pImageInfo       = nullptr;
    writeDescriptorSet[15].pTexelBufferView = nullptr;

    descriptorBufferInfo[16]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[16].buffer = m_vdata.m_pAccelStruct_m_SdfSVSNodesBuffer;
    descriptorBufferInfo[16].offset = m_vdata.m_pAccelStruct_m_SdfSVSNodesOffset;
    descriptorBufferInfo[16].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[16]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[16].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[16].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[16].dstBinding       = 16;
    writeDescriptorSet[16].descriptorCount  = 1;
    writeDescriptorSet[16].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[16].pBufferInfo      = &descriptorBufferInfo[16];
    writeDescriptorSet[16].pImageInfo       = nullptr;
    writeDescriptorSet[16].pTexelBufferView = nullptr;

    descriptorBufferInfo[17]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[17].buffer = m_vdata.m_pAccelStruct_m_SdfCompactOctreeV2DataBuffer;
    descriptorBufferInfo[17].offset = m_vdata.m_pAccelStruct_m_SdfCompactOctreeV2DataOffset;
    descriptorBufferInfo[17].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[17]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[17].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[17].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[17].dstBinding       = 17;
    writeDescriptorSet[17].descriptorCount  = 1;
    writeDescriptorSet[17].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[17].pBufferInfo      = &descriptorBufferInfo[17];
    writeDescriptorSet[17].pImageInfo       = nullptr;
    writeDescriptorSet[17].pTexelBufferView = nullptr;

    descriptorBufferInfo[18]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[18].buffer = m_vdata.m_pAccelStruct_m_SdfFrameOctreeRootsBuffer;
    descriptorBufferInfo[18].offset = m_vdata.m_pAccelStruct_m_SdfFrameOctreeRootsOffset;
    descriptorBufferInfo[18].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[18]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[18].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[18].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[18].dstBinding       = 18;
    writeDescriptorSet[18].descriptorCount  = 1;
    writeDescriptorSet[18].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[18].pBufferInfo      = &descriptorBufferInfo[18];
    writeDescriptorSet[18].pImageInfo       = nullptr;
    writeDescriptorSet[18].pTexelBufferView = nullptr;

    descriptorBufferInfo[19]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[19].buffer = m_vdata.m_pdfLightDataBuffer;
    descriptorBufferInfo[19].offset = m_vdata.m_pdfLightDataOffset;
    descriptorBufferInfo[19].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[19]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[19].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[19].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[19].dstBinding       = 19;
    writeDescriptorSet[19].descriptorCount  = 1;
    writeDescriptorSet[19].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[19].pBufferInfo      = &descriptorBufferInfo[19];
    writeDescriptorSet[19].pImageInfo       = nullptr;
    writeDescriptorSet[19].pTexelBufferView = nullptr;

    descriptorBufferInfo[20]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[20].buffer = m_vdata.m_pAccelStruct_m_primIdCountBuffer;
    descriptorBufferInfo[20].offset = m_vdata.m_pAccelStruct_m_primIdCountOffset;
    descriptorBufferInfo[20].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[20]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[20].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[20].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[20].dstBinding       = 20;
    writeDescriptorSet[20].descriptorCount  = 1;
    writeDescriptorSet[20].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[20].pBufferInfo      = &descriptorBufferInfo[20];
    writeDescriptorSet[20].pImageInfo       = nullptr;
    writeDescriptorSet[20].pTexelBufferView = nullptr;

    descriptorBufferInfo[21]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[21].buffer = m_vdata.m_spec_tex_ids_wavelengthsBuffer;
    descriptorBufferInfo[21].offset = m_vdata.m_spec_tex_ids_wavelengthsOffset;
    descriptorBufferInfo[21].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[21]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[21].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[21].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[21].dstBinding       = 21;
    writeDescriptorSet[21].descriptorCount  = 1;
    writeDescriptorSet[21].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[21].pBufferInfo      = &descriptorBufferInfo[21];
    writeDescriptorSet[21].pImageInfo       = nullptr;
    writeDescriptorSet[21].pTexelBufferView = nullptr;

    descriptorBufferInfo[22]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[22].buffer = m_vdata.m_pAccelStruct_m_geomDataBuffer;
    descriptorBufferInfo[22].offset = m_vdata.m_pAccelStruct_m_geomDataOffset;
    descriptorBufferInfo[22].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[22]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[22].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[22].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[22].dstBinding       = 22;
    writeDescriptorSet[22].descriptorCount  = 1;
    writeDescriptorSet[22].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[22].pBufferInfo      = &descriptorBufferInfo[22];
    writeDescriptorSet[22].pImageInfo       = nullptr;
    writeDescriptorSet[22].pTexelBufferView = nullptr;

    descriptorBufferInfo[23]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[23].buffer = m_vdata.m_pAccelStruct_m_SdfSBSDataFBuffer;
    descriptorBufferInfo[23].offset = m_vdata.m_pAccelStruct_m_SdfSBSDataFOffset;
    descriptorBufferInfo[23].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[23]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[23].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[23].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[23].dstBinding       = 23;
    writeDescriptorSet[23].descriptorCount  = 1;
    writeDescriptorSet[23].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[23].pBufferInfo      = &descriptorBufferInfo[23];
    writeDescriptorSet[23].pImageInfo       = nullptr;
    writeDescriptorSet[23].pTexelBufferView = nullptr;

    descriptorBufferInfo[24]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[24].buffer = m_vdata.m_lightsBuffer;
    descriptorBufferInfo[24].offset = m_vdata.m_lightsOffset;
    descriptorBufferInfo[24].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[24]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[24].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[24].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[24].dstBinding       = 24;
    writeDescriptorSet[24].descriptorCount  = 1;
    writeDescriptorSet[24].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[24].pBufferInfo      = &descriptorBufferInfo[24];
    writeDescriptorSet[24].pImageInfo       = nullptr;
    writeDescriptorSet[24].pTexelBufferView = nullptr;

    descriptorBufferInfo[25]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[25].buffer = m_vdata.m_cie_xBuffer;
    descriptorBufferInfo[25].offset = m_vdata.m_cie_xOffset;
    descriptorBufferInfo[25].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[25]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[25].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[25].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[25].dstBinding       = 25;
    writeDescriptorSet[25].descriptorCount  = 1;
    writeDescriptorSet[25].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[25].pBufferInfo      = &descriptorBufferInfo[25];
    writeDescriptorSet[25].pImageInfo       = nullptr;
    writeDescriptorSet[25].pTexelBufferView = nullptr;

    descriptorBufferInfo[26]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[26].buffer = m_vdata.m_pAccelStruct_m_SdfSBSHeadersBuffer;
    descriptorBufferInfo[26].offset = m_vdata.m_pAccelStruct_m_SdfSBSHeadersOffset;
    descriptorBufferInfo[26].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[26]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[26].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[26].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[26].dstBinding       = 26;
    writeDescriptorSet[26].descriptorCount  = 1;
    writeDescriptorSet[26].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[26].pBufferInfo      = &descriptorBufferInfo[26];
    writeDescriptorSet[26].pImageInfo       = nullptr;
    writeDescriptorSet[26].pTexelBufferView = nullptr;

    std::vector<VkDescriptorImageInfo> m_texturesInfo(m_vdata.m_texturesArrayMaxSize);
    for(size_t i=0; i<m_vdata.m_texturesArrayMaxSize; i++)
    {
      if(i < m_textures.size())
      {
        m_texturesInfo[i].sampler     = m_vdata.m_texturesArraySampler[i];
        m_texturesInfo[i].imageView   = m_vdata.m_texturesArrayView   [i];
        m_texturesInfo[i].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
      }
      else
      {
        m_texturesInfo[i].sampler     = m_vdata.m_texturesArraySampler[0];
        m_texturesInfo[i].imageView   = m_vdata.m_texturesArrayView   [0];
        m_texturesInfo[i].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
      }
    }
    writeDescriptorSet[27]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[27].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[27].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[27].dstBinding       = 27;
    writeDescriptorSet[27].descriptorCount  = 1;
    writeDescriptorSet[27].descriptorCount  = m_texturesInfo.size();
    writeDescriptorSet[27].descriptorType   = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    writeDescriptorSet[27].pBufferInfo      = nullptr;
    writeDescriptorSet[27].pImageInfo       = m_texturesInfo.data();
    writeDescriptorSet[27].pTexelBufferView = nullptr;

    descriptorBufferInfo[28]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[28].buffer = m_vdata.m_pAccelStruct_m_nodesTLASBuffer;
    descriptorBufferInfo[28].offset = m_vdata.m_pAccelStruct_m_nodesTLASOffset;
    descriptorBufferInfo[28].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[28]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[28].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[28].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[28].dstBinding       = 28;
    writeDescriptorSet[28].descriptorCount  = 1;
    writeDescriptorSet[28].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[28].pBufferInfo      = &descriptorBufferInfo[28];
    writeDescriptorSet[28].pImageInfo       = nullptr;
    writeDescriptorSet[28].pTexelBufferView = nullptr;

    descriptorBufferInfo[29]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[29].buffer = m_vdata.m_normMatricesBuffer;
    descriptorBufferInfo[29].offset = m_vdata.m_normMatricesOffset;
    descriptorBufferInfo[29].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[29]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[29].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[29].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[29].dstBinding       = 29;
    writeDescriptorSet[29].descriptorCount  = 1;
    writeDescriptorSet[29].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[29].pBufferInfo      = &descriptorBufferInfo[29];
    writeDescriptorSet[29].pImageInfo       = nullptr;
    writeDescriptorSet[29].pTexelBufferView = nullptr;

    descriptorBufferInfo[30]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[30].buffer = m_vdata.m_pAccelStruct_m_SdfSBSRootsBuffer;
    descriptorBufferInfo[30].offset = m_vdata.m_pAccelStruct_m_SdfSBSRootsOffset;
    descriptorBufferInfo[30].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[30]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[30].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[30].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[30].dstBinding       = 30;
    writeDescriptorSet[30].descriptorCount  = 1;
    writeDescriptorSet[30].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[30].pBufferInfo      = &descriptorBufferInfo[30];
    writeDescriptorSet[30].pImageInfo       = nullptr;
    writeDescriptorSet[30].pTexelBufferView = nullptr;

    descriptorBufferInfo[31]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[31].buffer = m_vdata.all_referencesBuffer;
    descriptorBufferInfo[31].offset = m_vdata.all_referencesOffset;
    descriptorBufferInfo[31].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[31]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[31].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[31].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[31].dstBinding       = 31;
    writeDescriptorSet[31].descriptorCount  = 1;
    writeDescriptorSet[31].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[31].pBufferInfo      = &descriptorBufferInfo[31];
    writeDescriptorSet[31].pImageInfo       = nullptr;
    writeDescriptorSet[31].pTexelBufferView = nullptr;

    descriptorBufferInfo[32]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[32].buffer = m_vdata.m_spec_tex_offset_szBuffer;
    descriptorBufferInfo[32].offset = m_vdata.m_spec_tex_offset_szOffset;
    descriptorBufferInfo[32].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[32]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[32].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[32].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[32].dstBinding       = 32;
    writeDescriptorSet[32].descriptorCount  = 1;
    writeDescriptorSet[32].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[32].pBufferInfo      = &descriptorBufferInfo[32];
    writeDescriptorSet[32].pImageInfo       = nullptr;
    writeDescriptorSet[32].pTexelBufferView = nullptr;

    descriptorBufferInfo[33]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[33].buffer = m_vdata.m_precomp_coat_transmittanceBuffer;
    descriptorBufferInfo[33].offset = m_vdata.m_precomp_coat_transmittanceOffset;
    descriptorBufferInfo[33].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[33]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[33].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[33].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[33].dstBinding       = 33;
    writeDescriptorSet[33].descriptorCount  = 1;
    writeDescriptorSet[33].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[33].pBufferInfo      = &descriptorBufferInfo[33];
    writeDescriptorSet[33].pImageInfo       = nullptr;
    writeDescriptorSet[33].pTexelBufferView = nullptr;

    descriptorBufferInfo[34]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[34].buffer = m_vdata.m_pAccelStruct_m_vertNormBuffer;
    descriptorBufferInfo[34].offset = m_vdata.m_pAccelStruct_m_vertNormOffset;
    descriptorBufferInfo[34].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[34]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[34].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[34].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[34].dstBinding       = 34;
    writeDescriptorSet[34].descriptorCount  = 1;
    writeDescriptorSet[34].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[34].pBufferInfo      = &descriptorBufferInfo[34];
    writeDescriptorSet[34].pImageInfo       = nullptr;
    writeDescriptorSet[34].pTexelBufferView = nullptr;

    descriptorBufferInfo[35]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[35].buffer = m_vdata.m_pAccelStruct_m_SdfFrameOctreeNodesBuffer;
    descriptorBufferInfo[35].offset = m_vdata.m_pAccelStruct_m_SdfFrameOctreeNodesOffset;
    descriptorBufferInfo[35].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[35]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[35].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[35].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[35].dstBinding       = 35;
    writeDescriptorSet[35].descriptorCount  = 1;
    writeDescriptorSet[35].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[35].pBufferInfo      = &descriptorBufferInfo[35];
    writeDescriptorSet[35].pImageInfo       = nullptr;
    writeDescriptorSet[35].pTexelBufferView = nullptr;

    descriptorBufferInfo[36]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[36].buffer = m_vdata.m_pAccelStruct_m_SdfSBSNodesBuffer;
    descriptorBufferInfo[36].offset = m_vdata.m_pAccelStruct_m_SdfSBSNodesOffset;
    descriptorBufferInfo[36].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[36]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[36].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[36].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[36].dstBinding       = 36;
    writeDescriptorSet[36].descriptorCount  = 1;
    writeDescriptorSet[36].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[36].pBufferInfo      = &descriptorBufferInfo[36];
    writeDescriptorSet[36].pImageInfo       = nullptr;
    writeDescriptorSet[36].pTexelBufferView = nullptr;

    descriptorBufferInfo[37]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[37].buffer = m_vdata.m_pAccelStruct_m_vertPosBuffer;
    descriptorBufferInfo[37].offset = m_vdata.m_pAccelStruct_m_vertPosOffset;
    descriptorBufferInfo[37].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[37]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[37].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[37].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[37].dstBinding       = 37;
    writeDescriptorSet[37].descriptorCount  = 1;
    writeDescriptorSet[37].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[37].pBufferInfo      = &descriptorBufferInfo[37];
    writeDescriptorSet[37].pImageInfo       = nullptr;
    writeDescriptorSet[37].pTexelBufferView = nullptr;

    descriptorBufferInfo[38]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[38].buffer = m_vdata.m_pAccelStruct_m_origNodesBuffer;
    descriptorBufferInfo[38].offset = m_vdata.m_pAccelStruct_m_origNodesOffset;
    descriptorBufferInfo[38].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[38]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[38].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[38].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[38].dstBinding       = 38;
    writeDescriptorSet[38].descriptorCount  = 1;
    writeDescriptorSet[38].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[38].pBufferInfo      = &descriptorBufferInfo[38];
    writeDescriptorSet[38].pImageInfo       = nullptr;
    writeDescriptorSet[38].pTexelBufferView = nullptr;

    descriptorBufferInfo[39]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[39].buffer = m_vdata.m_pAccelStruct_m_NURBS_approxesBuffer;
    descriptorBufferInfo[39].offset = m_vdata.m_pAccelStruct_m_NURBS_approxesOffset;
    descriptorBufferInfo[39].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[39]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[39].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[39].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[39].dstBinding       = 39;
    writeDescriptorSet[39].descriptorCount  = 1;
    writeDescriptorSet[39].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[39].pBufferInfo      = &descriptorBufferInfo[39];
    writeDescriptorSet[39].pImageInfo       = nullptr;
    writeDescriptorSet[39].pTexelBufferView = nullptr;

    descriptorBufferInfo[40]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[40].buffer = m_vdata.m_spec_valuesBuffer;
    descriptorBufferInfo[40].offset = m_vdata.m_spec_valuesOffset;
    descriptorBufferInfo[40].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[40]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[40].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[40].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[40].dstBinding       = 40;
    writeDescriptorSet[40].descriptorCount  = 1;
    writeDescriptorSet[40].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[40].pBufferInfo      = &descriptorBufferInfo[40];
    writeDescriptorSet[40].pImageInfo       = nullptr;
    writeDescriptorSet[40].pTexelBufferView = nullptr;

    descriptorBufferInfo[41]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[41].buffer = m_vdata.m_spec_offset_szBuffer;
    descriptorBufferInfo[41].offset = m_vdata.m_spec_offset_szOffset;
    descriptorBufferInfo[41].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[41]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[41].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[41].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[41].dstBinding       = 41;
    writeDescriptorSet[41].descriptorCount  = 1;
    writeDescriptorSet[41].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[41].pBufferInfo      = &descriptorBufferInfo[41];
    writeDescriptorSet[41].pImageInfo       = nullptr;
    writeDescriptorSet[41].pTexelBufferView = nullptr;

    descriptorBufferInfo[42]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[42].buffer = m_vdata.m_films_eta_k_vecBuffer;
    descriptorBufferInfo[42].offset = m_vdata.m_films_eta_k_vecOffset;
    descriptorBufferInfo[42].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[42]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[42].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[42].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[42].dstBinding       = 42;
    writeDescriptorSet[42].descriptorCount  = 1;
    writeDescriptorSet[42].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[42].pBufferInfo      = &descriptorBufferInfo[42];
    writeDescriptorSet[42].pImageInfo       = nullptr;
    writeDescriptorSet[42].pTexelBufferView = nullptr;

    descriptorBufferInfo[43]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[43].buffer = m_vdata.m_normMatrices2Buffer;
    descriptorBufferInfo[43].offset = m_vdata.m_normMatrices2Offset;
    descriptorBufferInfo[43].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[43]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[43].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[43].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[43].dstBinding       = 43;
    writeDescriptorSet[43].descriptorCount  = 1;
    writeDescriptorSet[43].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[43].pBufferInfo      = &descriptorBufferInfo[43];
    writeDescriptorSet[43].pImageInfo       = nullptr;
    writeDescriptorSet[43].pTexelBufferView = nullptr;

    descriptorBufferInfo[44]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[44].buffer = m_vdata.m_triIndicesBuffer;
    descriptorBufferInfo[44].offset = m_vdata.m_triIndicesOffset;
    descriptorBufferInfo[44].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[44]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[44].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[44].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[44].dstBinding       = 44;
    writeDescriptorSet[44].descriptorCount  = 1;
    writeDescriptorSet[44].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[44].pBufferInfo      = &descriptorBufferInfo[44];
    writeDescriptorSet[44].pImageInfo       = nullptr;
    writeDescriptorSet[44].pTexelBufferView = nullptr;

    descriptorBufferInfo[45]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[45].buffer = m_vdata.m_pAccelStruct_m_NURBSDataBuffer;
    descriptorBufferInfo[45].offset = m_vdata.m_pAccelStruct_m_NURBSDataOffset;
    descriptorBufferInfo[45].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[45]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[45].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[45].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[45].dstBinding       = 45;
    writeDescriptorSet[45].descriptorCount  = 1;
    writeDescriptorSet[45].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[45].pBufferInfo      = &descriptorBufferInfo[45];
    writeDescriptorSet[45].pImageInfo       = nullptr;
    writeDescriptorSet[45].pTexelBufferView = nullptr;

    descriptorBufferInfo[46]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[46].buffer = m_vdata.m_allRemapListsBuffer;
    descriptorBufferInfo[46].offset = m_vdata.m_allRemapListsOffset;
    descriptorBufferInfo[46].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[46]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[46].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[46].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[46].dstBinding       = 46;
    writeDescriptorSet[46].descriptorCount  = 1;
    writeDescriptorSet[46].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[46].pBufferInfo      = &descriptorBufferInfo[46];
    writeDescriptorSet[46].pImageInfo       = nullptr;
    writeDescriptorSet[46].pTexelBufferView = nullptr;

    descriptorBufferInfo[47]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[47].buffer = m_vdata.m_pAccelStruct_m_abstractObjectPtrsBuffer;
    descriptorBufferInfo[47].offset = m_vdata.m_pAccelStruct_m_abstractObjectPtrsOffset;
    descriptorBufferInfo[47].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[47]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[47].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[47].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[47].dstBinding       = 47;
    writeDescriptorSet[47].descriptorCount  = 1;
    writeDescriptorSet[47].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[47].pBufferInfo      = &descriptorBufferInfo[47];
    writeDescriptorSet[47].pImageInfo       = nullptr;
    writeDescriptorSet[47].pTexelBufferView = nullptr;

    descriptorBufferInfo[48]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[48].buffer = m_vdata.m_pAccelStruct_startEndBuffer;
    descriptorBufferInfo[48].offset = m_vdata.m_pAccelStruct_startEndOffset;
    descriptorBufferInfo[48].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[48]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[48].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[48].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[48].dstBinding       = 48;
    writeDescriptorSet[48].descriptorCount  = 1;
    writeDescriptorSet[48].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[48].pBufferInfo      = &descriptorBufferInfo[48];
    writeDescriptorSet[48].pImageInfo       = nullptr;
    writeDescriptorSet[48].pTexelBufferView = nullptr;

    descriptorBufferInfo[49]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[49].buffer = m_vdata.m_pAccelStruct_m_allNodePairsBuffer;
    descriptorBufferInfo[49].offset = m_vdata.m_pAccelStruct_m_allNodePairsOffset;
    descriptorBufferInfo[49].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[49]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[49].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[49].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[49].dstBinding       = 49;
    writeDescriptorSet[49].descriptorCount  = 1;
    writeDescriptorSet[49].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[49].pBufferInfo      = &descriptorBufferInfo[49];
    writeDescriptorSet[49].pImageInfo       = nullptr;
    writeDescriptorSet[49].pTexelBufferView = nullptr;

    descriptorBufferInfo[50]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[50].buffer = m_vdata.m_pAccelStruct_m_CatmulClarkHeadersBuffer;
    descriptorBufferInfo[50].offset = m_vdata.m_pAccelStruct_m_CatmulClarkHeadersOffset;
    descriptorBufferInfo[50].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[50]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[50].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[50].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[50].dstBinding       = 50;
    writeDescriptorSet[50].descriptorCount  = 1;
    writeDescriptorSet[50].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[50].pBufferInfo      = &descriptorBufferInfo[50];
    writeDescriptorSet[50].pImageInfo       = nullptr;
    writeDescriptorSet[50].pTexelBufferView = nullptr;

    descriptorBufferInfo[51]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[51].buffer = m_vdata.m_instIdToLightInstIdBuffer;
    descriptorBufferInfo[51].offset = m_vdata.m_instIdToLightInstIdOffset;
    descriptorBufferInfo[51].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[51]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[51].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[51].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[51].dstBinding       = 51;
    writeDescriptorSet[51].descriptorCount  = 1;
    writeDescriptorSet[51].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[51].pBufferInfo      = &descriptorBufferInfo[51];
    writeDescriptorSet[51].pImageInfo       = nullptr;
    writeDescriptorSet[51].pTexelBufferView = nullptr;

    descriptorBufferInfo[52]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[52].buffer = m_vdata.m_matIdByPrimIdBuffer;
    descriptorBufferInfo[52].offset = m_vdata.m_matIdByPrimIdOffset;
    descriptorBufferInfo[52].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[52]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[52].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[52].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[52].dstBinding       = 52;
    writeDescriptorSet[52].descriptorCount  = 1;
    writeDescriptorSet[52].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[52].pBufferInfo      = &descriptorBufferInfo[52];
    writeDescriptorSet[52].pImageInfo       = nullptr;
    writeDescriptorSet[52].pTexelBufferView = nullptr;

    descriptorBufferInfo[53]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[53].buffer = m_vdata.m_remapInstBuffer;
    descriptorBufferInfo[53].offset = m_vdata.m_remapInstOffset;
    descriptorBufferInfo[53].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[53]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[53].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[53].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[53].dstBinding       = 53;
    writeDescriptorSet[53].descriptorCount  = 1;
    writeDescriptorSet[53].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[53].pBufferInfo      = &descriptorBufferInfo[53];
    writeDescriptorSet[53].pImageInfo       = nullptr;
    writeDescriptorSet[53].pTexelBufferView = nullptr;

    descriptorBufferInfo[54]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[54].buffer = m_vdata.m_pAccelStruct_m_instanceDataBuffer;
    descriptorBufferInfo[54].offset = m_vdata.m_pAccelStruct_m_instanceDataOffset;
    descriptorBufferInfo[54].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[54]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[54].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[54].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[54].dstBinding       = 54;
    writeDescriptorSet[54].descriptorCount  = 1;
    writeDescriptorSet[54].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[54].pBufferInfo      = &descriptorBufferInfo[54];
    writeDescriptorSet[54].pImageInfo       = nullptr;
    writeDescriptorSet[54].pTexelBufferView = nullptr;

    descriptorBufferInfo[55]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[55].buffer = m_vdata.m_cie_zBuffer;
    descriptorBufferInfo[55].offset = m_vdata.m_cie_zOffset;
    descriptorBufferInfo[55].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[55]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[55].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[55].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[55].dstBinding       = 55;
    writeDescriptorSet[55].descriptorCount  = 1;
    writeDescriptorSet[55].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[55].pBufferInfo      = &descriptorBufferInfo[55];
    writeDescriptorSet[55].pImageInfo       = nullptr;
    writeDescriptorSet[55].pTexelBufferView = nullptr;

    descriptorBufferInfo[56]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[56].buffer = m_vdata.m_vNorm4fBuffer;
    descriptorBufferInfo[56].offset = m_vdata.m_vNorm4fOffset;
    descriptorBufferInfo[56].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[56]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[56].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[56].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[56].dstBinding       = 56;
    writeDescriptorSet[56].descriptorCount  = 1;
    writeDescriptorSet[56].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[56].pBufferInfo      = &descriptorBufferInfo[56];
    writeDescriptorSet[56].pImageInfo       = nullptr;
    writeDescriptorSet[56].pTexelBufferView = nullptr;

    descriptorBufferInfo[57]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[57].buffer = m_vdata.m_matIdOffsetsBuffer;
    descriptorBufferInfo[57].offset = m_vdata.m_matIdOffsetsOffset;
    descriptorBufferInfo[57].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[57]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[57].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[57].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[57].dstBinding       = 57;
    writeDescriptorSet[57].descriptorCount  = 1;
    writeDescriptorSet[57].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[57].pBufferInfo      = &descriptorBufferInfo[57];
    writeDescriptorSet[57].pImageInfo       = nullptr;
    writeDescriptorSet[57].pTexelBufferView = nullptr;

    descriptorBufferInfo[58]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[58].buffer = m_vdata.m_vTang4fBuffer;
    descriptorBufferInfo[58].offset = m_vdata.m_vTang4fOffset;
    descriptorBufferInfo[58].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[58]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[58].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[58].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[58].dstBinding       = 58;
    writeDescriptorSet[58].descriptorCount  = 1;
    writeDescriptorSet[58].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[58].pBufferInfo      = &descriptorBufferInfo[58];
    writeDescriptorSet[58].pImageInfo       = nullptr;
    writeDescriptorSet[58].pTexelBufferView = nullptr;

    descriptorBufferInfo[59]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[59].buffer = m_vdata.m_randomGensBuffer;
    descriptorBufferInfo[59].offset = m_vdata.m_randomGensOffset;
    descriptorBufferInfo[59].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[59]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[59].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[59].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[59].dstBinding       = 59;
    writeDescriptorSet[59].descriptorCount  = 1;
    writeDescriptorSet[59].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[59].pBufferInfo      = &descriptorBufferInfo[59];
    writeDescriptorSet[59].pImageInfo       = nullptr;
    writeDescriptorSet[59].pTexelBufferView = nullptr;

    descriptorBufferInfo[60]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[60].buffer = m_vdata.m_packedXYBuffer;
    descriptorBufferInfo[60].offset = m_vdata.m_packedXYOffset;
    descriptorBufferInfo[60].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[60]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[60].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[60].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[60].dstBinding       = 60;
    writeDescriptorSet[60].descriptorCount  = 1;
    writeDescriptorSet[60].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[60].pBufferInfo      = &descriptorBufferInfo[60];
    writeDescriptorSet[60].pImageInfo       = nullptr;
    writeDescriptorSet[60].pTexelBufferView = nullptr;

    descriptorBufferInfo[61]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[61].buffer = m_classDataBuffer;
    descriptorBufferInfo[61].offset = 0;
    descriptorBufferInfo[61].range  = VK_WHOLE_SIZE;

    writeDescriptorSet[61]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[61].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[61].dstSet           = m_allGeneratedDS[4];
    writeDescriptorSet[61].dstBinding       = 61;
    writeDescriptorSet[61].descriptorCount  = 1;
    writeDescriptorSet[61].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[61].pBufferInfo      = &descriptorBufferInfo[61];
    writeDescriptorSet[61].pImageInfo       = nullptr;
    writeDescriptorSet[61].pTexelBufferView = nullptr;

    vkUpdateDescriptorSets(device, uint32_t(writeDescriptorSet.size()), writeDescriptorSet.data(), 0, NULL);
  }
}

void Integrator_Generated::InitAllGeneratedDescriptorSets_NaivePathTrace()
{
  // now create actual bindings
  //
  // descriptor set #5: NaivePathTraceMegaCmd (["out_color","m_lines","m_pAccelStruct_m_NURBSHeaders","m_precomp_coat_transmittance","m_pAccelStruct_m_indices","m_materials","m_pAccelStruct_m_SdfSBSRoots","m_textures","m_pAccelStruct_m_nodesTLAS","m_normMatrices","m_pAccelStruct_m_SdfCompactOctreeV3Data","m_pAccelStruct_m_RibbonHeaders","m_allRemapListsOffsets","m_pAccelStruct_m_NURBSData","m_allRemapLists","m_pAccelStruct_m_SdfSBSData","m_films_spec_id_vec","m_vertOffset","m_pAccelStruct_m_geomData","m_pAccelStruct_m_SdfSVSRoots","m_pAccelStruct_startEnd","m_pAccelStruct_m_allNodePairs","m_pAccelStruct_m_primIdCount","m_spec_tex_ids_wavelengths","m_pAccelStruct_m_SdfFrameOctreeRoots","m_instIdToLightInstId","m_pAccelStruct_m_CatmulClarkHeaders","m_lights","m_cie_x","m_pAccelStruct_m_SdfSBSDataF","m_pAccelStruct_m_SdfSBSHeaders","m_pAccelStruct_m_SdfCompactOctreeRotModifiers","m_cie_y","m_pAccelStruct_m_SdfSVSNodes","m_pAccelStruct_m_SdfCompactOctreeV2Data","m_pAccelStruct_m_vertNorm","m_pAccelStruct_m_SdfFrameOctreeNodes","m_pAccelStruct_m_SdfSBSNodes","m_pAccelStruct_m_vertPos","m_pAccelStruct_m_origNodes","m_pAccelStruct_m_NURBS_approxes","m_spec_values","m_spec_tex_offset_sz","all_references","m_spec_offset_sz","m_films_eta_k_vec","m_normMatrices2","m_triIndices","m_pAccelStruct_m_abstractObjectPtrs","m_matIdByPrimId","m_remapInst","m_pAccelStruct_m_instanceData","m_cie_z","m_vNorm4f","m_matIdOffsets","m_vTang4f","m_precomp_thin_films","m_pAccelStruct_m_primIndices","m_randomGens","m_pdfLightData","m_packedXY"])
  {
    constexpr uint additionalSize = 1;

    std::array<VkDescriptorBufferInfo, 61 + additionalSize> descriptorBufferInfo;
    std::array<VkDescriptorImageInfo,  61 + additionalSize> descriptorImageInfo;
    std::array<VkWriteDescriptorSet,   61 + additionalSize> writeDescriptorSet;

    descriptorBufferInfo[0]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[0].buffer = NaivePathTrace_local.out_colorBuffer;
    descriptorBufferInfo[0].offset = NaivePathTrace_local.out_colorOffset;
    descriptorBufferInfo[0].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[0]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[0].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[0].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[0].dstBinding       = 0;
    writeDescriptorSet[0].descriptorCount  = 1;
    writeDescriptorSet[0].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[0].pBufferInfo      = &descriptorBufferInfo[0];
    writeDescriptorSet[0].pImageInfo       = nullptr;
    writeDescriptorSet[0].pTexelBufferView = nullptr;

    descriptorBufferInfo[1]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[1].buffer = m_vdata.m_linesBuffer;
    descriptorBufferInfo[1].offset = m_vdata.m_linesOffset;
    descriptorBufferInfo[1].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[1]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[1].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[1].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[1].dstBinding       = 1;
    writeDescriptorSet[1].descriptorCount  = 1;
    writeDescriptorSet[1].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[1].pBufferInfo      = &descriptorBufferInfo[1];
    writeDescriptorSet[1].pImageInfo       = nullptr;
    writeDescriptorSet[1].pTexelBufferView = nullptr;

    descriptorBufferInfo[2]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[2].buffer = m_vdata.m_pAccelStruct_m_NURBSHeadersBuffer;
    descriptorBufferInfo[2].offset = m_vdata.m_pAccelStruct_m_NURBSHeadersOffset;
    descriptorBufferInfo[2].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[2]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[2].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[2].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[2].dstBinding       = 2;
    writeDescriptorSet[2].descriptorCount  = 1;
    writeDescriptorSet[2].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[2].pBufferInfo      = &descriptorBufferInfo[2];
    writeDescriptorSet[2].pImageInfo       = nullptr;
    writeDescriptorSet[2].pTexelBufferView = nullptr;

    descriptorBufferInfo[3]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[3].buffer = m_vdata.m_precomp_coat_transmittanceBuffer;
    descriptorBufferInfo[3].offset = m_vdata.m_precomp_coat_transmittanceOffset;
    descriptorBufferInfo[3].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[3]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[3].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[3].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[3].dstBinding       = 3;
    writeDescriptorSet[3].descriptorCount  = 1;
    writeDescriptorSet[3].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[3].pBufferInfo      = &descriptorBufferInfo[3];
    writeDescriptorSet[3].pImageInfo       = nullptr;
    writeDescriptorSet[3].pTexelBufferView = nullptr;

    descriptorBufferInfo[4]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[4].buffer = m_vdata.m_pAccelStruct_m_indicesBuffer;
    descriptorBufferInfo[4].offset = m_vdata.m_pAccelStruct_m_indicesOffset;
    descriptorBufferInfo[4].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[4]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[4].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[4].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[4].dstBinding       = 4;
    writeDescriptorSet[4].descriptorCount  = 1;
    writeDescriptorSet[4].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[4].pBufferInfo      = &descriptorBufferInfo[4];
    writeDescriptorSet[4].pImageInfo       = nullptr;
    writeDescriptorSet[4].pTexelBufferView = nullptr;

    descriptorBufferInfo[5]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[5].buffer = m_vdata.m_materialsBuffer;
    descriptorBufferInfo[5].offset = m_vdata.m_materialsOffset;
    descriptorBufferInfo[5].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[5]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[5].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[5].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[5].dstBinding       = 5;
    writeDescriptorSet[5].descriptorCount  = 1;
    writeDescriptorSet[5].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[5].pBufferInfo      = &descriptorBufferInfo[5];
    writeDescriptorSet[5].pImageInfo       = nullptr;
    writeDescriptorSet[5].pTexelBufferView = nullptr;

    descriptorBufferInfo[6]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[6].buffer = m_vdata.m_pAccelStruct_m_SdfSBSRootsBuffer;
    descriptorBufferInfo[6].offset = m_vdata.m_pAccelStruct_m_SdfSBSRootsOffset;
    descriptorBufferInfo[6].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[6]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[6].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[6].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[6].dstBinding       = 6;
    writeDescriptorSet[6].descriptorCount  = 1;
    writeDescriptorSet[6].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[6].pBufferInfo      = &descriptorBufferInfo[6];
    writeDescriptorSet[6].pImageInfo       = nullptr;
    writeDescriptorSet[6].pTexelBufferView = nullptr;

    std::vector<VkDescriptorImageInfo> m_texturesInfo(m_vdata.m_texturesArrayMaxSize);
    for(size_t i=0; i<m_vdata.m_texturesArrayMaxSize; i++)
    {
      if(i < m_textures.size())
      {
        m_texturesInfo[i].sampler     = m_vdata.m_texturesArraySampler[i];
        m_texturesInfo[i].imageView   = m_vdata.m_texturesArrayView   [i];
        m_texturesInfo[i].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
      }
      else
      {
        m_texturesInfo[i].sampler     = m_vdata.m_texturesArraySampler[0];
        m_texturesInfo[i].imageView   = m_vdata.m_texturesArrayView   [0];
        m_texturesInfo[i].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
      }
    }
    writeDescriptorSet[7]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[7].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[7].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[7].dstBinding       = 7;
    writeDescriptorSet[7].descriptorCount  = 1;
    writeDescriptorSet[7].descriptorCount  = m_texturesInfo.size();
    writeDescriptorSet[7].descriptorType   = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    writeDescriptorSet[7].pBufferInfo      = nullptr;
    writeDescriptorSet[7].pImageInfo       = m_texturesInfo.data();
    writeDescriptorSet[7].pTexelBufferView = nullptr;

    descriptorBufferInfo[8]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[8].buffer = m_vdata.m_pAccelStruct_m_nodesTLASBuffer;
    descriptorBufferInfo[8].offset = m_vdata.m_pAccelStruct_m_nodesTLASOffset;
    descriptorBufferInfo[8].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[8]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[8].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[8].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[8].dstBinding       = 8;
    writeDescriptorSet[8].descriptorCount  = 1;
    writeDescriptorSet[8].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[8].pBufferInfo      = &descriptorBufferInfo[8];
    writeDescriptorSet[8].pImageInfo       = nullptr;
    writeDescriptorSet[8].pTexelBufferView = nullptr;

    descriptorBufferInfo[9]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[9].buffer = m_vdata.m_normMatricesBuffer;
    descriptorBufferInfo[9].offset = m_vdata.m_normMatricesOffset;
    descriptorBufferInfo[9].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[9]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[9].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[9].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[9].dstBinding       = 9;
    writeDescriptorSet[9].descriptorCount  = 1;
    writeDescriptorSet[9].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[9].pBufferInfo      = &descriptorBufferInfo[9];
    writeDescriptorSet[9].pImageInfo       = nullptr;
    writeDescriptorSet[9].pTexelBufferView = nullptr;

    descriptorBufferInfo[10]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[10].buffer = m_vdata.m_pAccelStruct_m_SdfCompactOctreeV3DataBuffer;
    descriptorBufferInfo[10].offset = m_vdata.m_pAccelStruct_m_SdfCompactOctreeV3DataOffset;
    descriptorBufferInfo[10].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[10]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[10].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[10].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[10].dstBinding       = 10;
    writeDescriptorSet[10].descriptorCount  = 1;
    writeDescriptorSet[10].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[10].pBufferInfo      = &descriptorBufferInfo[10];
    writeDescriptorSet[10].pImageInfo       = nullptr;
    writeDescriptorSet[10].pTexelBufferView = nullptr;

    descriptorBufferInfo[11]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[11].buffer = m_vdata.m_pAccelStruct_m_RibbonHeadersBuffer;
    descriptorBufferInfo[11].offset = m_vdata.m_pAccelStruct_m_RibbonHeadersOffset;
    descriptorBufferInfo[11].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[11]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[11].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[11].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[11].dstBinding       = 11;
    writeDescriptorSet[11].descriptorCount  = 1;
    writeDescriptorSet[11].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[11].pBufferInfo      = &descriptorBufferInfo[11];
    writeDescriptorSet[11].pImageInfo       = nullptr;
    writeDescriptorSet[11].pTexelBufferView = nullptr;

    descriptorBufferInfo[12]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[12].buffer = m_vdata.m_allRemapListsOffsetsBuffer;
    descriptorBufferInfo[12].offset = m_vdata.m_allRemapListsOffsetsOffset;
    descriptorBufferInfo[12].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[12]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[12].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[12].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[12].dstBinding       = 12;
    writeDescriptorSet[12].descriptorCount  = 1;
    writeDescriptorSet[12].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[12].pBufferInfo      = &descriptorBufferInfo[12];
    writeDescriptorSet[12].pImageInfo       = nullptr;
    writeDescriptorSet[12].pTexelBufferView = nullptr;

    descriptorBufferInfo[13]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[13].buffer = m_vdata.m_pAccelStruct_m_NURBSDataBuffer;
    descriptorBufferInfo[13].offset = m_vdata.m_pAccelStruct_m_NURBSDataOffset;
    descriptorBufferInfo[13].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[13]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[13].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[13].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[13].dstBinding       = 13;
    writeDescriptorSet[13].descriptorCount  = 1;
    writeDescriptorSet[13].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[13].pBufferInfo      = &descriptorBufferInfo[13];
    writeDescriptorSet[13].pImageInfo       = nullptr;
    writeDescriptorSet[13].pTexelBufferView = nullptr;

    descriptorBufferInfo[14]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[14].buffer = m_vdata.m_allRemapListsBuffer;
    descriptorBufferInfo[14].offset = m_vdata.m_allRemapListsOffset;
    descriptorBufferInfo[14].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[14]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[14].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[14].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[14].dstBinding       = 14;
    writeDescriptorSet[14].descriptorCount  = 1;
    writeDescriptorSet[14].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[14].pBufferInfo      = &descriptorBufferInfo[14];
    writeDescriptorSet[14].pImageInfo       = nullptr;
    writeDescriptorSet[14].pTexelBufferView = nullptr;

    descriptorBufferInfo[15]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[15].buffer = m_vdata.m_pAccelStruct_m_SdfSBSDataBuffer;
    descriptorBufferInfo[15].offset = m_vdata.m_pAccelStruct_m_SdfSBSDataOffset;
    descriptorBufferInfo[15].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[15]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[15].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[15].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[15].dstBinding       = 15;
    writeDescriptorSet[15].descriptorCount  = 1;
    writeDescriptorSet[15].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[15].pBufferInfo      = &descriptorBufferInfo[15];
    writeDescriptorSet[15].pImageInfo       = nullptr;
    writeDescriptorSet[15].pTexelBufferView = nullptr;

    descriptorBufferInfo[16]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[16].buffer = m_vdata.m_films_spec_id_vecBuffer;
    descriptorBufferInfo[16].offset = m_vdata.m_films_spec_id_vecOffset;
    descriptorBufferInfo[16].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[16]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[16].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[16].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[16].dstBinding       = 16;
    writeDescriptorSet[16].descriptorCount  = 1;
    writeDescriptorSet[16].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[16].pBufferInfo      = &descriptorBufferInfo[16];
    writeDescriptorSet[16].pImageInfo       = nullptr;
    writeDescriptorSet[16].pTexelBufferView = nullptr;

    descriptorBufferInfo[17]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[17].buffer = m_vdata.m_vertOffsetBuffer;
    descriptorBufferInfo[17].offset = m_vdata.m_vertOffsetOffset;
    descriptorBufferInfo[17].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[17]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[17].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[17].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[17].dstBinding       = 17;
    writeDescriptorSet[17].descriptorCount  = 1;
    writeDescriptorSet[17].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[17].pBufferInfo      = &descriptorBufferInfo[17];
    writeDescriptorSet[17].pImageInfo       = nullptr;
    writeDescriptorSet[17].pTexelBufferView = nullptr;

    descriptorBufferInfo[18]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[18].buffer = m_vdata.m_pAccelStruct_m_geomDataBuffer;
    descriptorBufferInfo[18].offset = m_vdata.m_pAccelStruct_m_geomDataOffset;
    descriptorBufferInfo[18].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[18]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[18].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[18].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[18].dstBinding       = 18;
    writeDescriptorSet[18].descriptorCount  = 1;
    writeDescriptorSet[18].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[18].pBufferInfo      = &descriptorBufferInfo[18];
    writeDescriptorSet[18].pImageInfo       = nullptr;
    writeDescriptorSet[18].pTexelBufferView = nullptr;

    descriptorBufferInfo[19]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[19].buffer = m_vdata.m_pAccelStruct_m_SdfSVSRootsBuffer;
    descriptorBufferInfo[19].offset = m_vdata.m_pAccelStruct_m_SdfSVSRootsOffset;
    descriptorBufferInfo[19].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[19]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[19].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[19].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[19].dstBinding       = 19;
    writeDescriptorSet[19].descriptorCount  = 1;
    writeDescriptorSet[19].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[19].pBufferInfo      = &descriptorBufferInfo[19];
    writeDescriptorSet[19].pImageInfo       = nullptr;
    writeDescriptorSet[19].pTexelBufferView = nullptr;

    descriptorBufferInfo[20]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[20].buffer = m_vdata.m_pAccelStruct_startEndBuffer;
    descriptorBufferInfo[20].offset = m_vdata.m_pAccelStruct_startEndOffset;
    descriptorBufferInfo[20].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[20]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[20].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[20].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[20].dstBinding       = 20;
    writeDescriptorSet[20].descriptorCount  = 1;
    writeDescriptorSet[20].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[20].pBufferInfo      = &descriptorBufferInfo[20];
    writeDescriptorSet[20].pImageInfo       = nullptr;
    writeDescriptorSet[20].pTexelBufferView = nullptr;

    descriptorBufferInfo[21]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[21].buffer = m_vdata.m_pAccelStruct_m_allNodePairsBuffer;
    descriptorBufferInfo[21].offset = m_vdata.m_pAccelStruct_m_allNodePairsOffset;
    descriptorBufferInfo[21].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[21]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[21].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[21].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[21].dstBinding       = 21;
    writeDescriptorSet[21].descriptorCount  = 1;
    writeDescriptorSet[21].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[21].pBufferInfo      = &descriptorBufferInfo[21];
    writeDescriptorSet[21].pImageInfo       = nullptr;
    writeDescriptorSet[21].pTexelBufferView = nullptr;

    descriptorBufferInfo[22]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[22].buffer = m_vdata.m_pAccelStruct_m_primIdCountBuffer;
    descriptorBufferInfo[22].offset = m_vdata.m_pAccelStruct_m_primIdCountOffset;
    descriptorBufferInfo[22].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[22]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[22].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[22].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[22].dstBinding       = 22;
    writeDescriptorSet[22].descriptorCount  = 1;
    writeDescriptorSet[22].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[22].pBufferInfo      = &descriptorBufferInfo[22];
    writeDescriptorSet[22].pImageInfo       = nullptr;
    writeDescriptorSet[22].pTexelBufferView = nullptr;

    descriptorBufferInfo[23]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[23].buffer = m_vdata.m_spec_tex_ids_wavelengthsBuffer;
    descriptorBufferInfo[23].offset = m_vdata.m_spec_tex_ids_wavelengthsOffset;
    descriptorBufferInfo[23].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[23]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[23].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[23].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[23].dstBinding       = 23;
    writeDescriptorSet[23].descriptorCount  = 1;
    writeDescriptorSet[23].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[23].pBufferInfo      = &descriptorBufferInfo[23];
    writeDescriptorSet[23].pImageInfo       = nullptr;
    writeDescriptorSet[23].pTexelBufferView = nullptr;

    descriptorBufferInfo[24]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[24].buffer = m_vdata.m_pAccelStruct_m_SdfFrameOctreeRootsBuffer;
    descriptorBufferInfo[24].offset = m_vdata.m_pAccelStruct_m_SdfFrameOctreeRootsOffset;
    descriptorBufferInfo[24].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[24]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[24].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[24].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[24].dstBinding       = 24;
    writeDescriptorSet[24].descriptorCount  = 1;
    writeDescriptorSet[24].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[24].pBufferInfo      = &descriptorBufferInfo[24];
    writeDescriptorSet[24].pImageInfo       = nullptr;
    writeDescriptorSet[24].pTexelBufferView = nullptr;

    descriptorBufferInfo[25]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[25].buffer = m_vdata.m_instIdToLightInstIdBuffer;
    descriptorBufferInfo[25].offset = m_vdata.m_instIdToLightInstIdOffset;
    descriptorBufferInfo[25].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[25]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[25].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[25].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[25].dstBinding       = 25;
    writeDescriptorSet[25].descriptorCount  = 1;
    writeDescriptorSet[25].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[25].pBufferInfo      = &descriptorBufferInfo[25];
    writeDescriptorSet[25].pImageInfo       = nullptr;
    writeDescriptorSet[25].pTexelBufferView = nullptr;

    descriptorBufferInfo[26]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[26].buffer = m_vdata.m_pAccelStruct_m_CatmulClarkHeadersBuffer;
    descriptorBufferInfo[26].offset = m_vdata.m_pAccelStruct_m_CatmulClarkHeadersOffset;
    descriptorBufferInfo[26].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[26]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[26].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[26].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[26].dstBinding       = 26;
    writeDescriptorSet[26].descriptorCount  = 1;
    writeDescriptorSet[26].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[26].pBufferInfo      = &descriptorBufferInfo[26];
    writeDescriptorSet[26].pImageInfo       = nullptr;
    writeDescriptorSet[26].pTexelBufferView = nullptr;

    descriptorBufferInfo[27]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[27].buffer = m_vdata.m_lightsBuffer;
    descriptorBufferInfo[27].offset = m_vdata.m_lightsOffset;
    descriptorBufferInfo[27].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[27]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[27].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[27].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[27].dstBinding       = 27;
    writeDescriptorSet[27].descriptorCount  = 1;
    writeDescriptorSet[27].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[27].pBufferInfo      = &descriptorBufferInfo[27];
    writeDescriptorSet[27].pImageInfo       = nullptr;
    writeDescriptorSet[27].pTexelBufferView = nullptr;

    descriptorBufferInfo[28]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[28].buffer = m_vdata.m_cie_xBuffer;
    descriptorBufferInfo[28].offset = m_vdata.m_cie_xOffset;
    descriptorBufferInfo[28].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[28]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[28].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[28].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[28].dstBinding       = 28;
    writeDescriptorSet[28].descriptorCount  = 1;
    writeDescriptorSet[28].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[28].pBufferInfo      = &descriptorBufferInfo[28];
    writeDescriptorSet[28].pImageInfo       = nullptr;
    writeDescriptorSet[28].pTexelBufferView = nullptr;

    descriptorBufferInfo[29]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[29].buffer = m_vdata.m_pAccelStruct_m_SdfSBSDataFBuffer;
    descriptorBufferInfo[29].offset = m_vdata.m_pAccelStruct_m_SdfSBSDataFOffset;
    descriptorBufferInfo[29].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[29]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[29].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[29].dstSet           = m_allGeneratedDS[5];
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
    writeDescriptorSet[30].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[30].dstBinding       = 30;
    writeDescriptorSet[30].descriptorCount  = 1;
    writeDescriptorSet[30].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[30].pBufferInfo      = &descriptorBufferInfo[30];
    writeDescriptorSet[30].pImageInfo       = nullptr;
    writeDescriptorSet[30].pTexelBufferView = nullptr;

    descriptorBufferInfo[31]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[31].buffer = m_vdata.m_pAccelStruct_m_SdfCompactOctreeRotModifiersBuffer;
    descriptorBufferInfo[31].offset = m_vdata.m_pAccelStruct_m_SdfCompactOctreeRotModifiersOffset;
    descriptorBufferInfo[31].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[31]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[31].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[31].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[31].dstBinding       = 31;
    writeDescriptorSet[31].descriptorCount  = 1;
    writeDescriptorSet[31].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[31].pBufferInfo      = &descriptorBufferInfo[31];
    writeDescriptorSet[31].pImageInfo       = nullptr;
    writeDescriptorSet[31].pTexelBufferView = nullptr;

    descriptorBufferInfo[32]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[32].buffer = m_vdata.m_cie_yBuffer;
    descriptorBufferInfo[32].offset = m_vdata.m_cie_yOffset;
    descriptorBufferInfo[32].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[32]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[32].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[32].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[32].dstBinding       = 32;
    writeDescriptorSet[32].descriptorCount  = 1;
    writeDescriptorSet[32].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[32].pBufferInfo      = &descriptorBufferInfo[32];
    writeDescriptorSet[32].pImageInfo       = nullptr;
    writeDescriptorSet[32].pTexelBufferView = nullptr;

    descriptorBufferInfo[33]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[33].buffer = m_vdata.m_pAccelStruct_m_SdfSVSNodesBuffer;
    descriptorBufferInfo[33].offset = m_vdata.m_pAccelStruct_m_SdfSVSNodesOffset;
    descriptorBufferInfo[33].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[33]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[33].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[33].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[33].dstBinding       = 33;
    writeDescriptorSet[33].descriptorCount  = 1;
    writeDescriptorSet[33].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[33].pBufferInfo      = &descriptorBufferInfo[33];
    writeDescriptorSet[33].pImageInfo       = nullptr;
    writeDescriptorSet[33].pTexelBufferView = nullptr;

    descriptorBufferInfo[34]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[34].buffer = m_vdata.m_pAccelStruct_m_SdfCompactOctreeV2DataBuffer;
    descriptorBufferInfo[34].offset = m_vdata.m_pAccelStruct_m_SdfCompactOctreeV2DataOffset;
    descriptorBufferInfo[34].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[34]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[34].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[34].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[34].dstBinding       = 34;
    writeDescriptorSet[34].descriptorCount  = 1;
    writeDescriptorSet[34].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[34].pBufferInfo      = &descriptorBufferInfo[34];
    writeDescriptorSet[34].pImageInfo       = nullptr;
    writeDescriptorSet[34].pTexelBufferView = nullptr;

    descriptorBufferInfo[35]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[35].buffer = m_vdata.m_pAccelStruct_m_vertNormBuffer;
    descriptorBufferInfo[35].offset = m_vdata.m_pAccelStruct_m_vertNormOffset;
    descriptorBufferInfo[35].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[35]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[35].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[35].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[35].dstBinding       = 35;
    writeDescriptorSet[35].descriptorCount  = 1;
    writeDescriptorSet[35].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[35].pBufferInfo      = &descriptorBufferInfo[35];
    writeDescriptorSet[35].pImageInfo       = nullptr;
    writeDescriptorSet[35].pTexelBufferView = nullptr;

    descriptorBufferInfo[36]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[36].buffer = m_vdata.m_pAccelStruct_m_SdfFrameOctreeNodesBuffer;
    descriptorBufferInfo[36].offset = m_vdata.m_pAccelStruct_m_SdfFrameOctreeNodesOffset;
    descriptorBufferInfo[36].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[36]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[36].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[36].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[36].dstBinding       = 36;
    writeDescriptorSet[36].descriptorCount  = 1;
    writeDescriptorSet[36].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[36].pBufferInfo      = &descriptorBufferInfo[36];
    writeDescriptorSet[36].pImageInfo       = nullptr;
    writeDescriptorSet[36].pTexelBufferView = nullptr;

    descriptorBufferInfo[37]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[37].buffer = m_vdata.m_pAccelStruct_m_SdfSBSNodesBuffer;
    descriptorBufferInfo[37].offset = m_vdata.m_pAccelStruct_m_SdfSBSNodesOffset;
    descriptorBufferInfo[37].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[37]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[37].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[37].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[37].dstBinding       = 37;
    writeDescriptorSet[37].descriptorCount  = 1;
    writeDescriptorSet[37].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[37].pBufferInfo      = &descriptorBufferInfo[37];
    writeDescriptorSet[37].pImageInfo       = nullptr;
    writeDescriptorSet[37].pTexelBufferView = nullptr;

    descriptorBufferInfo[38]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[38].buffer = m_vdata.m_pAccelStruct_m_vertPosBuffer;
    descriptorBufferInfo[38].offset = m_vdata.m_pAccelStruct_m_vertPosOffset;
    descriptorBufferInfo[38].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[38]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[38].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[38].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[38].dstBinding       = 38;
    writeDescriptorSet[38].descriptorCount  = 1;
    writeDescriptorSet[38].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[38].pBufferInfo      = &descriptorBufferInfo[38];
    writeDescriptorSet[38].pImageInfo       = nullptr;
    writeDescriptorSet[38].pTexelBufferView = nullptr;

    descriptorBufferInfo[39]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[39].buffer = m_vdata.m_pAccelStruct_m_origNodesBuffer;
    descriptorBufferInfo[39].offset = m_vdata.m_pAccelStruct_m_origNodesOffset;
    descriptorBufferInfo[39].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[39]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[39].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[39].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[39].dstBinding       = 39;
    writeDescriptorSet[39].descriptorCount  = 1;
    writeDescriptorSet[39].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[39].pBufferInfo      = &descriptorBufferInfo[39];
    writeDescriptorSet[39].pImageInfo       = nullptr;
    writeDescriptorSet[39].pTexelBufferView = nullptr;

    descriptorBufferInfo[40]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[40].buffer = m_vdata.m_pAccelStruct_m_NURBS_approxesBuffer;
    descriptorBufferInfo[40].offset = m_vdata.m_pAccelStruct_m_NURBS_approxesOffset;
    descriptorBufferInfo[40].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[40]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[40].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[40].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[40].dstBinding       = 40;
    writeDescriptorSet[40].descriptorCount  = 1;
    writeDescriptorSet[40].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[40].pBufferInfo      = &descriptorBufferInfo[40];
    writeDescriptorSet[40].pImageInfo       = nullptr;
    writeDescriptorSet[40].pTexelBufferView = nullptr;

    descriptorBufferInfo[41]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[41].buffer = m_vdata.m_spec_valuesBuffer;
    descriptorBufferInfo[41].offset = m_vdata.m_spec_valuesOffset;
    descriptorBufferInfo[41].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[41]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[41].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[41].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[41].dstBinding       = 41;
    writeDescriptorSet[41].descriptorCount  = 1;
    writeDescriptorSet[41].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[41].pBufferInfo      = &descriptorBufferInfo[41];
    writeDescriptorSet[41].pImageInfo       = nullptr;
    writeDescriptorSet[41].pTexelBufferView = nullptr;

    descriptorBufferInfo[42]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[42].buffer = m_vdata.m_spec_tex_offset_szBuffer;
    descriptorBufferInfo[42].offset = m_vdata.m_spec_tex_offset_szOffset;
    descriptorBufferInfo[42].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[42]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[42].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[42].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[42].dstBinding       = 42;
    writeDescriptorSet[42].descriptorCount  = 1;
    writeDescriptorSet[42].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[42].pBufferInfo      = &descriptorBufferInfo[42];
    writeDescriptorSet[42].pImageInfo       = nullptr;
    writeDescriptorSet[42].pTexelBufferView = nullptr;

    descriptorBufferInfo[43]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[43].buffer = m_vdata.all_referencesBuffer;
    descriptorBufferInfo[43].offset = m_vdata.all_referencesOffset;
    descriptorBufferInfo[43].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[43]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[43].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[43].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[43].dstBinding       = 43;
    writeDescriptorSet[43].descriptorCount  = 1;
    writeDescriptorSet[43].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[43].pBufferInfo      = &descriptorBufferInfo[43];
    writeDescriptorSet[43].pImageInfo       = nullptr;
    writeDescriptorSet[43].pTexelBufferView = nullptr;

    descriptorBufferInfo[44]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[44].buffer = m_vdata.m_spec_offset_szBuffer;
    descriptorBufferInfo[44].offset = m_vdata.m_spec_offset_szOffset;
    descriptorBufferInfo[44].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[44]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[44].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[44].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[44].dstBinding       = 44;
    writeDescriptorSet[44].descriptorCount  = 1;
    writeDescriptorSet[44].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[44].pBufferInfo      = &descriptorBufferInfo[44];
    writeDescriptorSet[44].pImageInfo       = nullptr;
    writeDescriptorSet[44].pTexelBufferView = nullptr;

    descriptorBufferInfo[45]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[45].buffer = m_vdata.m_films_eta_k_vecBuffer;
    descriptorBufferInfo[45].offset = m_vdata.m_films_eta_k_vecOffset;
    descriptorBufferInfo[45].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[45]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[45].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[45].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[45].dstBinding       = 45;
    writeDescriptorSet[45].descriptorCount  = 1;
    writeDescriptorSet[45].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[45].pBufferInfo      = &descriptorBufferInfo[45];
    writeDescriptorSet[45].pImageInfo       = nullptr;
    writeDescriptorSet[45].pTexelBufferView = nullptr;

    descriptorBufferInfo[46]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[46].buffer = m_vdata.m_normMatrices2Buffer;
    descriptorBufferInfo[46].offset = m_vdata.m_normMatrices2Offset;
    descriptorBufferInfo[46].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[46]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[46].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[46].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[46].dstBinding       = 46;
    writeDescriptorSet[46].descriptorCount  = 1;
    writeDescriptorSet[46].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[46].pBufferInfo      = &descriptorBufferInfo[46];
    writeDescriptorSet[46].pImageInfo       = nullptr;
    writeDescriptorSet[46].pTexelBufferView = nullptr;

    descriptorBufferInfo[47]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[47].buffer = m_vdata.m_triIndicesBuffer;
    descriptorBufferInfo[47].offset = m_vdata.m_triIndicesOffset;
    descriptorBufferInfo[47].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[47]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[47].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[47].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[47].dstBinding       = 47;
    writeDescriptorSet[47].descriptorCount  = 1;
    writeDescriptorSet[47].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[47].pBufferInfo      = &descriptorBufferInfo[47];
    writeDescriptorSet[47].pImageInfo       = nullptr;
    writeDescriptorSet[47].pTexelBufferView = nullptr;

    descriptorBufferInfo[48]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[48].buffer = m_vdata.m_pAccelStruct_m_abstractObjectPtrsBuffer;
    descriptorBufferInfo[48].offset = m_vdata.m_pAccelStruct_m_abstractObjectPtrsOffset;
    descriptorBufferInfo[48].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[48]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[48].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[48].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[48].dstBinding       = 48;
    writeDescriptorSet[48].descriptorCount  = 1;
    writeDescriptorSet[48].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[48].pBufferInfo      = &descriptorBufferInfo[48];
    writeDescriptorSet[48].pImageInfo       = nullptr;
    writeDescriptorSet[48].pTexelBufferView = nullptr;

    descriptorBufferInfo[49]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[49].buffer = m_vdata.m_matIdByPrimIdBuffer;
    descriptorBufferInfo[49].offset = m_vdata.m_matIdByPrimIdOffset;
    descriptorBufferInfo[49].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[49]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[49].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[49].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[49].dstBinding       = 49;
    writeDescriptorSet[49].descriptorCount  = 1;
    writeDescriptorSet[49].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[49].pBufferInfo      = &descriptorBufferInfo[49];
    writeDescriptorSet[49].pImageInfo       = nullptr;
    writeDescriptorSet[49].pTexelBufferView = nullptr;

    descriptorBufferInfo[50]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[50].buffer = m_vdata.m_remapInstBuffer;
    descriptorBufferInfo[50].offset = m_vdata.m_remapInstOffset;
    descriptorBufferInfo[50].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[50]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[50].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[50].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[50].dstBinding       = 50;
    writeDescriptorSet[50].descriptorCount  = 1;
    writeDescriptorSet[50].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[50].pBufferInfo      = &descriptorBufferInfo[50];
    writeDescriptorSet[50].pImageInfo       = nullptr;
    writeDescriptorSet[50].pTexelBufferView = nullptr;

    descriptorBufferInfo[51]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[51].buffer = m_vdata.m_pAccelStruct_m_instanceDataBuffer;
    descriptorBufferInfo[51].offset = m_vdata.m_pAccelStruct_m_instanceDataOffset;
    descriptorBufferInfo[51].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[51]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[51].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[51].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[51].dstBinding       = 51;
    writeDescriptorSet[51].descriptorCount  = 1;
    writeDescriptorSet[51].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[51].pBufferInfo      = &descriptorBufferInfo[51];
    writeDescriptorSet[51].pImageInfo       = nullptr;
    writeDescriptorSet[51].pTexelBufferView = nullptr;

    descriptorBufferInfo[52]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[52].buffer = m_vdata.m_cie_zBuffer;
    descriptorBufferInfo[52].offset = m_vdata.m_cie_zOffset;
    descriptorBufferInfo[52].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[52]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[52].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[52].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[52].dstBinding       = 52;
    writeDescriptorSet[52].descriptorCount  = 1;
    writeDescriptorSet[52].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[52].pBufferInfo      = &descriptorBufferInfo[52];
    writeDescriptorSet[52].pImageInfo       = nullptr;
    writeDescriptorSet[52].pTexelBufferView = nullptr;

    descriptorBufferInfo[53]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[53].buffer = m_vdata.m_vNorm4fBuffer;
    descriptorBufferInfo[53].offset = m_vdata.m_vNorm4fOffset;
    descriptorBufferInfo[53].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[53]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[53].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[53].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[53].dstBinding       = 53;
    writeDescriptorSet[53].descriptorCount  = 1;
    writeDescriptorSet[53].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[53].pBufferInfo      = &descriptorBufferInfo[53];
    writeDescriptorSet[53].pImageInfo       = nullptr;
    writeDescriptorSet[53].pTexelBufferView = nullptr;

    descriptorBufferInfo[54]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[54].buffer = m_vdata.m_matIdOffsetsBuffer;
    descriptorBufferInfo[54].offset = m_vdata.m_matIdOffsetsOffset;
    descriptorBufferInfo[54].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[54]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[54].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[54].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[54].dstBinding       = 54;
    writeDescriptorSet[54].descriptorCount  = 1;
    writeDescriptorSet[54].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[54].pBufferInfo      = &descriptorBufferInfo[54];
    writeDescriptorSet[54].pImageInfo       = nullptr;
    writeDescriptorSet[54].pTexelBufferView = nullptr;

    descriptorBufferInfo[55]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[55].buffer = m_vdata.m_vTang4fBuffer;
    descriptorBufferInfo[55].offset = m_vdata.m_vTang4fOffset;
    descriptorBufferInfo[55].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[55]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[55].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[55].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[55].dstBinding       = 55;
    writeDescriptorSet[55].descriptorCount  = 1;
    writeDescriptorSet[55].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[55].pBufferInfo      = &descriptorBufferInfo[55];
    writeDescriptorSet[55].pImageInfo       = nullptr;
    writeDescriptorSet[55].pTexelBufferView = nullptr;

    descriptorBufferInfo[56]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[56].buffer = m_vdata.m_precomp_thin_filmsBuffer;
    descriptorBufferInfo[56].offset = m_vdata.m_precomp_thin_filmsOffset;
    descriptorBufferInfo[56].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[56]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[56].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[56].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[56].dstBinding       = 56;
    writeDescriptorSet[56].descriptorCount  = 1;
    writeDescriptorSet[56].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[56].pBufferInfo      = &descriptorBufferInfo[56];
    writeDescriptorSet[56].pImageInfo       = nullptr;
    writeDescriptorSet[56].pTexelBufferView = nullptr;

    descriptorBufferInfo[57]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[57].buffer = m_vdata.m_pAccelStruct_m_primIndicesBuffer;
    descriptorBufferInfo[57].offset = m_vdata.m_pAccelStruct_m_primIndicesOffset;
    descriptorBufferInfo[57].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[57]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[57].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[57].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[57].dstBinding       = 57;
    writeDescriptorSet[57].descriptorCount  = 1;
    writeDescriptorSet[57].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[57].pBufferInfo      = &descriptorBufferInfo[57];
    writeDescriptorSet[57].pImageInfo       = nullptr;
    writeDescriptorSet[57].pTexelBufferView = nullptr;

    descriptorBufferInfo[58]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[58].buffer = m_vdata.m_randomGensBuffer;
    descriptorBufferInfo[58].offset = m_vdata.m_randomGensOffset;
    descriptorBufferInfo[58].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[58]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[58].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[58].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[58].dstBinding       = 58;
    writeDescriptorSet[58].descriptorCount  = 1;
    writeDescriptorSet[58].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[58].pBufferInfo      = &descriptorBufferInfo[58];
    writeDescriptorSet[58].pImageInfo       = nullptr;
    writeDescriptorSet[58].pTexelBufferView = nullptr;

    descriptorBufferInfo[59]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[59].buffer = m_vdata.m_pdfLightDataBuffer;
    descriptorBufferInfo[59].offset = m_vdata.m_pdfLightDataOffset;
    descriptorBufferInfo[59].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[59]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[59].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[59].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[59].dstBinding       = 59;
    writeDescriptorSet[59].descriptorCount  = 1;
    writeDescriptorSet[59].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[59].pBufferInfo      = &descriptorBufferInfo[59];
    writeDescriptorSet[59].pImageInfo       = nullptr;
    writeDescriptorSet[59].pTexelBufferView = nullptr;

    descriptorBufferInfo[60]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[60].buffer = m_vdata.m_packedXYBuffer;
    descriptorBufferInfo[60].offset = m_vdata.m_packedXYOffset;
    descriptorBufferInfo[60].range  = VK_WHOLE_SIZE;
    writeDescriptorSet[60]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[60].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[60].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[60].dstBinding       = 60;
    writeDescriptorSet[60].descriptorCount  = 1;
    writeDescriptorSet[60].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[60].pBufferInfo      = &descriptorBufferInfo[60];
    writeDescriptorSet[60].pImageInfo       = nullptr;
    writeDescriptorSet[60].pTexelBufferView = nullptr;

    descriptorBufferInfo[61]        = VkDescriptorBufferInfo{};
    descriptorBufferInfo[61].buffer = m_classDataBuffer;
    descriptorBufferInfo[61].offset = 0;
    descriptorBufferInfo[61].range  = VK_WHOLE_SIZE;

    writeDescriptorSet[61]                  = VkWriteDescriptorSet{};
    writeDescriptorSet[61].sType            = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writeDescriptorSet[61].dstSet           = m_allGeneratedDS[5];
    writeDescriptorSet[61].dstBinding       = 61;
    writeDescriptorSet[61].descriptorCount  = 1;
    writeDescriptorSet[61].descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writeDescriptorSet[61].pBufferInfo      = &descriptorBufferInfo[61];
    writeDescriptorSet[61].pImageInfo       = nullptr;
    writeDescriptorSet[61].pTexelBufferView = nullptr;

    vkUpdateDescriptorSets(device, uint32_t(writeDescriptorSet.size()), writeDescriptorSet.data(), 0, NULL);
  }
}



