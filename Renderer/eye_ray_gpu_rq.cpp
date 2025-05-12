#include <vector>
#include <memory>
#include <limits>
#include <cassert>
#include <chrono>
#include <array>

#include "vk_copy.h"
#include "vk_context.h"
#include "vk_images.h"

#include "eye_ray_gpu_rq.h"
#include "include/MultiRenderer_gpu_rq_ubo.h"
#include "VulkanRTX.h"

static uint32_t ComputeReductionSteps(uint32_t whole_size, uint32_t wg_size)
{
  uint32_t steps = 0;
  while (whole_size > 1)
  {
    steps++;
    whole_size = (whole_size + wg_size - 1) / wg_size;
  }
  return steps;
}

constexpr uint32_t KGEN_FLAG_RETURN            = 1;
constexpr uint32_t KGEN_FLAG_BREAK             = 2;
constexpr uint32_t KGEN_FLAG_DONT_SET_EXIT     = 4;
constexpr uint32_t KGEN_FLAG_SET_EXIT_NEGATIVE = 8;
constexpr uint32_t KGEN_REDUCTION_LAST_STEP    = 16;


void MultiRenderer_gpu_rq::UpdatePlainMembers(std::shared_ptr<vk_utils::ICopyEngine> a_pCopyEngine)
{
  const size_t maxAllowedSize = std::numeric_limits<uint32_t>::max();
  auto pUnderlyingImpl = dynamic_cast<BVHRT*>(m_pAccelStruct->UnderlyingImpl(0));
  m_uboData.m_projInv = m_projInv;
  m_uboData.m_worldViewInv = m_worldViewInv;
  m_uboData.m_pAccelStruct_coctree_v3_header = pUnderlyingImpl->coctree_v3_header;
  m_uboData.m_pAccelStruct_m_preset = pUnderlyingImpl->m_preset;
  m_uboData.m_preset = m_preset;
  m_uboData.m_height = m_height;
  m_uboData.m_packedXY_height = m_packedXY_height;
  m_uboData.m_packedXY_width = m_packedXY_width;
  m_uboData.m_seed = m_seed;
  m_uboData.m_width = m_width;
  m_uboData.all_references_size     = uint32_t( all_references.size() );     assert( all_references.size() < maxAllowedSize );
  m_uboData.all_references_capacity = uint32_t( all_references.capacity() ); assert( all_references.capacity() < maxAllowedSize );
  m_uboData.m_geomOffsets_size     = uint32_t( m_geomOffsets.size() );     assert( m_geomOffsets.size() < maxAllowedSize );
  m_uboData.m_geomOffsets_capacity = uint32_t( m_geomOffsets.capacity() ); assert( m_geomOffsets.capacity() < maxAllowedSize );
  m_uboData.m_indices_size     = uint32_t( m_indices.size() );     assert( m_indices.size() < maxAllowedSize );
  m_uboData.m_indices_capacity = uint32_t( m_indices.capacity() ); assert( m_indices.capacity() < maxAllowedSize );
  m_uboData.m_instanceTransformInvTransposed_size     = uint32_t( m_instanceTransformInvTransposed.size() );     assert( m_instanceTransformInvTransposed.size() < maxAllowedSize );
  m_uboData.m_instanceTransformInvTransposed_capacity = uint32_t( m_instanceTransformInvTransposed.capacity() ); assert( m_instanceTransformInvTransposed.capacity() < maxAllowedSize );
  m_uboData.m_lights_size     = uint32_t( m_lights.size() );     assert( m_lights.size() < maxAllowedSize );
  m_uboData.m_lights_capacity = uint32_t( m_lights.capacity() ); assert( m_lights.capacity() < maxAllowedSize );
  m_uboData.m_matIdOffsets_size     = uint32_t( m_matIdOffsets.size() );     assert( m_matIdOffsets.size() < maxAllowedSize );
  m_uboData.m_matIdOffsets_capacity = uint32_t( m_matIdOffsets.capacity() ); assert( m_matIdOffsets.capacity() < maxAllowedSize );
  m_uboData.m_matIdbyPrimId_size     = uint32_t( m_matIdbyPrimId.size() );     assert( m_matIdbyPrimId.size() < maxAllowedSize );
  m_uboData.m_matIdbyPrimId_capacity = uint32_t( m_matIdbyPrimId.capacity() ); assert( m_matIdbyPrimId.capacity() < maxAllowedSize );
  m_uboData.m_materials_size     = uint32_t( m_materials.size() );     assert( m_materials.size() < maxAllowedSize );
  m_uboData.m_materials_capacity = uint32_t( m_materials.capacity() ); assert( m_materials.capacity() < maxAllowedSize );
  m_uboData.m_normals_size     = uint32_t( m_normals.size() );     assert( m_normals.size() < maxAllowedSize );
  m_uboData.m_normals_capacity = uint32_t( m_normals.capacity() ); assert( m_normals.capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_CatmulClarkHeaders_size     = uint32_t( m_pAccelStruct_m_CatmulClarkHeaders->size() );     assert( m_pAccelStruct_m_CatmulClarkHeaders->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_CatmulClarkHeaders_capacity = uint32_t( m_pAccelStruct_m_CatmulClarkHeaders->capacity() ); assert( m_pAccelStruct_m_CatmulClarkHeaders->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_GraphicsPrimHeaders_size     = uint32_t( m_pAccelStruct_m_GraphicsPrimHeaders->size() );     assert( m_pAccelStruct_m_GraphicsPrimHeaders->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_GraphicsPrimHeaders_capacity = uint32_t( m_pAccelStruct_m_GraphicsPrimHeaders->capacity() ); assert( m_pAccelStruct_m_GraphicsPrimHeaders->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_GraphicsPrimPoints_size     = uint32_t( m_pAccelStruct_m_GraphicsPrimPoints->size() );     assert( m_pAccelStruct_m_GraphicsPrimPoints->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_GraphicsPrimPoints_capacity = uint32_t( m_pAccelStruct_m_GraphicsPrimPoints->capacity() ); assert( m_pAccelStruct_m_GraphicsPrimPoints->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_NURBSData_size     = uint32_t( m_pAccelStruct_m_NURBSData->size() );     assert( m_pAccelStruct_m_NURBSData->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_NURBSData_capacity = uint32_t( m_pAccelStruct_m_NURBSData->capacity() ); assert( m_pAccelStruct_m_NURBSData->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_NURBSHeaders_size     = uint32_t( m_pAccelStruct_m_NURBSHeaders->size() );     assert( m_pAccelStruct_m_NURBSHeaders->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_NURBSHeaders_capacity = uint32_t( m_pAccelStruct_m_NURBSHeaders->capacity() ); assert( m_pAccelStruct_m_NURBSHeaders->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_NURBS_approxes_size     = uint32_t( m_pAccelStruct_m_NURBS_approxes->size() );     assert( m_pAccelStruct_m_NURBS_approxes->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_NURBS_approxes_capacity = uint32_t( m_pAccelStruct_m_NURBS_approxes->capacity() ); assert( m_pAccelStruct_m_NURBS_approxes->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_RibbonHeaders_size     = uint32_t( m_pAccelStruct_m_RibbonHeaders->size() );     assert( m_pAccelStruct_m_RibbonHeaders->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_RibbonHeaders_capacity = uint32_t( m_pAccelStruct_m_RibbonHeaders->capacity() ); assert( m_pAccelStruct_m_RibbonHeaders->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfCompactOctreeRotModifiers_size     = uint32_t( m_pAccelStruct_m_SdfCompactOctreeRotModifiers->size() );     assert( m_pAccelStruct_m_SdfCompactOctreeRotModifiers->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfCompactOctreeRotModifiers_capacity = uint32_t( m_pAccelStruct_m_SdfCompactOctreeRotModifiers->capacity() ); assert( m_pAccelStruct_m_SdfCompactOctreeRotModifiers->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfCompactOctreeV2Data_size     = uint32_t( m_pAccelStruct_m_SdfCompactOctreeV2Data->size() );     assert( m_pAccelStruct_m_SdfCompactOctreeV2Data->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfCompactOctreeV2Data_capacity = uint32_t( m_pAccelStruct_m_SdfCompactOctreeV2Data->capacity() ); assert( m_pAccelStruct_m_SdfCompactOctreeV2Data->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfCompactOctreeV3Data_size     = uint32_t( m_pAccelStruct_m_SdfCompactOctreeV3Data->size() );     assert( m_pAccelStruct_m_SdfCompactOctreeV3Data->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfCompactOctreeV3Data_capacity = uint32_t( m_pAccelStruct_m_SdfCompactOctreeV3Data->capacity() ); assert( m_pAccelStruct_m_SdfCompactOctreeV3Data->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfFrameOctreeNodes_size     = uint32_t( m_pAccelStruct_m_SdfFrameOctreeNodes->size() );     assert( m_pAccelStruct_m_SdfFrameOctreeNodes->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfFrameOctreeNodes_capacity = uint32_t( m_pAccelStruct_m_SdfFrameOctreeNodes->capacity() ); assert( m_pAccelStruct_m_SdfFrameOctreeNodes->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfFrameOctreeRoots_size     = uint32_t( m_pAccelStruct_m_SdfFrameOctreeRoots->size() );     assert( m_pAccelStruct_m_SdfFrameOctreeRoots->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfFrameOctreeRoots_capacity = uint32_t( m_pAccelStruct_m_SdfFrameOctreeRoots->capacity() ); assert( m_pAccelStruct_m_SdfFrameOctreeRoots->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfFrameOctreeTexNodes_size     = uint32_t( m_pAccelStruct_m_SdfFrameOctreeTexNodes->size() );     assert( m_pAccelStruct_m_SdfFrameOctreeTexNodes->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfFrameOctreeTexNodes_capacity = uint32_t( m_pAccelStruct_m_SdfFrameOctreeTexNodes->capacity() ); assert( m_pAccelStruct_m_SdfFrameOctreeTexNodes->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfFrameOctreeTexRoots_size     = uint32_t( m_pAccelStruct_m_SdfFrameOctreeTexRoots->size() );     assert( m_pAccelStruct_m_SdfFrameOctreeTexRoots->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfFrameOctreeTexRoots_capacity = uint32_t( m_pAccelStruct_m_SdfFrameOctreeTexRoots->capacity() ); assert( m_pAccelStruct_m_SdfFrameOctreeTexRoots->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfGridData_size     = uint32_t( m_pAccelStruct_m_SdfGridData->size() );     assert( m_pAccelStruct_m_SdfGridData->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfGridData_capacity = uint32_t( m_pAccelStruct_m_SdfGridData->capacity() ); assert( m_pAccelStruct_m_SdfGridData->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfGridOffsets_size     = uint32_t( m_pAccelStruct_m_SdfGridOffsets->size() );     assert( m_pAccelStruct_m_SdfGridOffsets->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfGridOffsets_capacity = uint32_t( m_pAccelStruct_m_SdfGridOffsets->capacity() ); assert( m_pAccelStruct_m_SdfGridOffsets->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfGridSizes_size     = uint32_t( m_pAccelStruct_m_SdfGridSizes->size() );     assert( m_pAccelStruct_m_SdfGridSizes->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfGridSizes_capacity = uint32_t( m_pAccelStruct_m_SdfGridSizes->capacity() ); assert( m_pAccelStruct_m_SdfGridSizes->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfSBSAdaptData_size     = uint32_t( m_pAccelStruct_m_SdfSBSAdaptData->size() );     assert( m_pAccelStruct_m_SdfSBSAdaptData->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfSBSAdaptData_capacity = uint32_t( m_pAccelStruct_m_SdfSBSAdaptData->capacity() ); assert( m_pAccelStruct_m_SdfSBSAdaptData->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfSBSAdaptDataF_size     = uint32_t( m_pAccelStruct_m_SdfSBSAdaptDataF->size() );     assert( m_pAccelStruct_m_SdfSBSAdaptDataF->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfSBSAdaptDataF_capacity = uint32_t( m_pAccelStruct_m_SdfSBSAdaptDataF->capacity() ); assert( m_pAccelStruct_m_SdfSBSAdaptDataF->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfSBSAdaptHeaders_size     = uint32_t( m_pAccelStruct_m_SdfSBSAdaptHeaders->size() );     assert( m_pAccelStruct_m_SdfSBSAdaptHeaders->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfSBSAdaptHeaders_capacity = uint32_t( m_pAccelStruct_m_SdfSBSAdaptHeaders->capacity() ); assert( m_pAccelStruct_m_SdfSBSAdaptHeaders->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfSBSAdaptNodes_size     = uint32_t( m_pAccelStruct_m_SdfSBSAdaptNodes->size() );     assert( m_pAccelStruct_m_SdfSBSAdaptNodes->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfSBSAdaptNodes_capacity = uint32_t( m_pAccelStruct_m_SdfSBSAdaptNodes->capacity() ); assert( m_pAccelStruct_m_SdfSBSAdaptNodes->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfSBSAdaptRoots_size     = uint32_t( m_pAccelStruct_m_SdfSBSAdaptRoots->size() );     assert( m_pAccelStruct_m_SdfSBSAdaptRoots->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfSBSAdaptRoots_capacity = uint32_t( m_pAccelStruct_m_SdfSBSAdaptRoots->capacity() ); assert( m_pAccelStruct_m_SdfSBSAdaptRoots->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfSBSData_size     = uint32_t( m_pAccelStruct_m_SdfSBSData->size() );     assert( m_pAccelStruct_m_SdfSBSData->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfSBSData_capacity = uint32_t( m_pAccelStruct_m_SdfSBSData->capacity() ); assert( m_pAccelStruct_m_SdfSBSData->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfSBSDataF_size     = uint32_t( m_pAccelStruct_m_SdfSBSDataF->size() );     assert( m_pAccelStruct_m_SdfSBSDataF->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfSBSDataF_capacity = uint32_t( m_pAccelStruct_m_SdfSBSDataF->capacity() ); assert( m_pAccelStruct_m_SdfSBSDataF->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfSBSHeaders_size     = uint32_t( m_pAccelStruct_m_SdfSBSHeaders->size() );     assert( m_pAccelStruct_m_SdfSBSHeaders->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfSBSHeaders_capacity = uint32_t( m_pAccelStruct_m_SdfSBSHeaders->capacity() ); assert( m_pAccelStruct_m_SdfSBSHeaders->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfSBSNodes_size     = uint32_t( m_pAccelStruct_m_SdfSBSNodes->size() );     assert( m_pAccelStruct_m_SdfSBSNodes->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfSBSNodes_capacity = uint32_t( m_pAccelStruct_m_SdfSBSNodes->capacity() ); assert( m_pAccelStruct_m_SdfSBSNodes->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfSBSRoots_size     = uint32_t( m_pAccelStruct_m_SdfSBSRoots->size() );     assert( m_pAccelStruct_m_SdfSBSRoots->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfSBSRoots_capacity = uint32_t( m_pAccelStruct_m_SdfSBSRoots->capacity() ); assert( m_pAccelStruct_m_SdfSBSRoots->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfSVSNodes_size     = uint32_t( m_pAccelStruct_m_SdfSVSNodes->size() );     assert( m_pAccelStruct_m_SdfSVSNodes->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfSVSNodes_capacity = uint32_t( m_pAccelStruct_m_SdfSVSNodes->capacity() ); assert( m_pAccelStruct_m_SdfSVSNodes->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfSVSRoots_size     = uint32_t( m_pAccelStruct_m_SdfSVSRoots->size() );     assert( m_pAccelStruct_m_SdfSVSRoots->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfSVSRoots_capacity = uint32_t( m_pAccelStruct_m_SdfSVSRoots->capacity() ); assert( m_pAccelStruct_m_SdfSVSRoots->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_abstractObjectPtrs_size     = uint32_t( m_pAccelStruct_m_abstractObjectPtrs->size() );     assert( m_pAccelStruct_m_abstractObjectPtrs->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_abstractObjectPtrs_capacity = uint32_t( m_pAccelStruct_m_abstractObjectPtrs->capacity() ); assert( m_pAccelStruct_m_abstractObjectPtrs->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_allNodePairs_size     = uint32_t( m_pAccelStruct_m_allNodePairs->size() );     assert( m_pAccelStruct_m_allNodePairs->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_allNodePairs_capacity = uint32_t( m_pAccelStruct_m_allNodePairs->capacity() ); assert( m_pAccelStruct_m_allNodePairs->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_geomData_size     = uint32_t( m_pAccelStruct_m_geomData->size() );     assert( m_pAccelStruct_m_geomData->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_geomData_capacity = uint32_t( m_pAccelStruct_m_geomData->capacity() ); assert( m_pAccelStruct_m_geomData->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_indices_size     = uint32_t( m_pAccelStruct_m_indices->size() );     assert( m_pAccelStruct_m_indices->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_indices_capacity = uint32_t( m_pAccelStruct_m_indices->capacity() ); assert( m_pAccelStruct_m_indices->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_instanceData_size     = uint32_t( m_pAccelStruct_m_instanceData->size() );     assert( m_pAccelStruct_m_instanceData->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_instanceData_capacity = uint32_t( m_pAccelStruct_m_instanceData->capacity() ); assert( m_pAccelStruct_m_instanceData->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_nodesTLAS_size     = uint32_t( m_pAccelStruct_m_nodesTLAS->size() );     assert( m_pAccelStruct_m_nodesTLAS->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_nodesTLAS_capacity = uint32_t( m_pAccelStruct_m_nodesTLAS->capacity() ); assert( m_pAccelStruct_m_nodesTLAS->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_origNodes_size     = uint32_t( m_pAccelStruct_m_origNodes->size() );     assert( m_pAccelStruct_m_origNodes->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_origNodes_capacity = uint32_t( m_pAccelStruct_m_origNodes->capacity() ); assert( m_pAccelStruct_m_origNodes->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_primIdCount_size     = uint32_t( m_pAccelStruct_m_primIdCount->size() );     assert( m_pAccelStruct_m_primIdCount->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_primIdCount_capacity = uint32_t( m_pAccelStruct_m_primIdCount->capacity() ); assert( m_pAccelStruct_m_primIdCount->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_primIndices_size     = uint32_t( m_pAccelStruct_m_primIndices->size() );     assert( m_pAccelStruct_m_primIndices->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_primIndices_capacity = uint32_t( m_pAccelStruct_m_primIndices->capacity() ); assert( m_pAccelStruct_m_primIndices->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_vertNorm_size     = uint32_t( m_pAccelStruct_m_vertNorm->size() );     assert( m_pAccelStruct_m_vertNorm->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_vertNorm_capacity = uint32_t( m_pAccelStruct_m_vertNorm->capacity() ); assert( m_pAccelStruct_m_vertNorm->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_vertPos_size     = uint32_t( m_pAccelStruct_m_vertPos->size() );     assert( m_pAccelStruct_m_vertPos->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_vertPos_capacity = uint32_t( m_pAccelStruct_m_vertPos->capacity() ); assert( m_pAccelStruct_m_vertPos->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_startEnd_size     = uint32_t( m_pAccelStruct_startEnd->size() );     assert( m_pAccelStruct_startEnd->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_startEnd_capacity = uint32_t( m_pAccelStruct_startEnd->capacity() ); assert( m_pAccelStruct_startEnd->capacity() < maxAllowedSize );
  m_uboData.m_packedXY_size     = uint32_t( m_packedXY.size() );     assert( m_packedXY.size() < maxAllowedSize );
  m_uboData.m_packedXY_capacity = uint32_t( m_packedXY.capacity() ); assert( m_packedXY.capacity() < maxAllowedSize );
  m_uboData.m_vertices_size     = uint32_t( m_vertices.size() );     assert( m_vertices.size() < maxAllowedSize );
  m_uboData.m_vertices_capacity = uint32_t( m_vertices.capacity() ); assert( m_vertices.capacity() < maxAllowedSize );
  a_pCopyEngine->UpdateBuffer(m_classDataBuffer, 0, &m_uboData, sizeof(m_uboData));
}

void MultiRenderer_gpu_rq::ReadPlainMembers(std::shared_ptr<vk_utils::ICopyEngine> a_pCopyEngine)
{
  a_pCopyEngine->ReadBuffer(m_classDataBuffer, 0, &m_uboData, sizeof(m_uboData));
  auto pUnderlyingImpl = dynamic_cast<BVHRT*>(m_pAccelStruct->UnderlyingImpl(0));
  m_projInv = m_uboData.m_projInv;
  m_worldViewInv = m_uboData.m_worldViewInv;
  pUnderlyingImpl->coctree_v3_header = m_uboData.m_pAccelStruct_coctree_v3_header;
  pUnderlyingImpl->m_preset = m_uboData.m_pAccelStruct_m_preset;
  m_preset = m_uboData.m_preset;
  m_height = m_uboData.m_height;
  m_packedXY_height = m_uboData.m_packedXY_height;
  m_packedXY_width = m_uboData.m_packedXY_width;
  m_seed = m_uboData.m_seed;
  m_width = m_uboData.m_width;
  all_references.resize(m_uboData.all_references_size);
  m_geomOffsets.resize(m_uboData.m_geomOffsets_size);
  m_indices.resize(m_uboData.m_indices_size);
  m_instanceTransformInvTransposed.resize(m_uboData.m_instanceTransformInvTransposed_size);
  m_lights.resize(m_uboData.m_lights_size);
  m_matIdOffsets.resize(m_uboData.m_matIdOffsets_size);
  m_matIdbyPrimId.resize(m_uboData.m_matIdbyPrimId_size);
  m_materials.resize(m_uboData.m_materials_size);
  m_normals.resize(m_uboData.m_normals_size);
  m_pAccelStruct_m_CatmulClarkHeaders->resize(m_uboData.m_pAccelStruct_m_CatmulClarkHeaders_size);
  m_pAccelStruct_m_GraphicsPrimHeaders->resize(m_uboData.m_pAccelStruct_m_GraphicsPrimHeaders_size);
  m_pAccelStruct_m_GraphicsPrimPoints->resize(m_uboData.m_pAccelStruct_m_GraphicsPrimPoints_size);
  m_pAccelStruct_m_NURBSData->resize(m_uboData.m_pAccelStruct_m_NURBSData_size);
  m_pAccelStruct_m_NURBSHeaders->resize(m_uboData.m_pAccelStruct_m_NURBSHeaders_size);
  m_pAccelStruct_m_NURBS_approxes->resize(m_uboData.m_pAccelStruct_m_NURBS_approxes_size);
  m_pAccelStruct_m_RibbonHeaders->resize(m_uboData.m_pAccelStruct_m_RibbonHeaders_size);
  m_pAccelStruct_m_SdfCompactOctreeRotModifiers->resize(m_uboData.m_pAccelStruct_m_SdfCompactOctreeRotModifiers_size);
  m_pAccelStruct_m_SdfCompactOctreeV2Data->resize(m_uboData.m_pAccelStruct_m_SdfCompactOctreeV2Data_size);
  m_pAccelStruct_m_SdfCompactOctreeV3Data->resize(m_uboData.m_pAccelStruct_m_SdfCompactOctreeV3Data_size);
  m_pAccelStruct_m_SdfFrameOctreeNodes->resize(m_uboData.m_pAccelStruct_m_SdfFrameOctreeNodes_size);
  m_pAccelStruct_m_SdfFrameOctreeRoots->resize(m_uboData.m_pAccelStruct_m_SdfFrameOctreeRoots_size);
  m_pAccelStruct_m_SdfFrameOctreeTexNodes->resize(m_uboData.m_pAccelStruct_m_SdfFrameOctreeTexNodes_size);
  m_pAccelStruct_m_SdfFrameOctreeTexRoots->resize(m_uboData.m_pAccelStruct_m_SdfFrameOctreeTexRoots_size);
  m_pAccelStruct_m_SdfGridData->resize(m_uboData.m_pAccelStruct_m_SdfGridData_size);
  m_pAccelStruct_m_SdfGridOffsets->resize(m_uboData.m_pAccelStruct_m_SdfGridOffsets_size);
  m_pAccelStruct_m_SdfGridSizes->resize(m_uboData.m_pAccelStruct_m_SdfGridSizes_size);
  m_pAccelStruct_m_SdfSBSAdaptData->resize(m_uboData.m_pAccelStruct_m_SdfSBSAdaptData_size);
  m_pAccelStruct_m_SdfSBSAdaptDataF->resize(m_uboData.m_pAccelStruct_m_SdfSBSAdaptDataF_size);
  m_pAccelStruct_m_SdfSBSAdaptHeaders->resize(m_uboData.m_pAccelStruct_m_SdfSBSAdaptHeaders_size);
  m_pAccelStruct_m_SdfSBSAdaptNodes->resize(m_uboData.m_pAccelStruct_m_SdfSBSAdaptNodes_size);
  m_pAccelStruct_m_SdfSBSAdaptRoots->resize(m_uboData.m_pAccelStruct_m_SdfSBSAdaptRoots_size);
  m_pAccelStruct_m_SdfSBSData->resize(m_uboData.m_pAccelStruct_m_SdfSBSData_size);
  m_pAccelStruct_m_SdfSBSDataF->resize(m_uboData.m_pAccelStruct_m_SdfSBSDataF_size);
  m_pAccelStruct_m_SdfSBSHeaders->resize(m_uboData.m_pAccelStruct_m_SdfSBSHeaders_size);
  m_pAccelStruct_m_SdfSBSNodes->resize(m_uboData.m_pAccelStruct_m_SdfSBSNodes_size);
  m_pAccelStruct_m_SdfSBSRoots->resize(m_uboData.m_pAccelStruct_m_SdfSBSRoots_size);
  m_pAccelStruct_m_SdfSVSNodes->resize(m_uboData.m_pAccelStruct_m_SdfSVSNodes_size);
  m_pAccelStruct_m_SdfSVSRoots->resize(m_uboData.m_pAccelStruct_m_SdfSVSRoots_size);
  m_pAccelStruct_m_abstractObjectPtrs->resize(m_uboData.m_pAccelStruct_m_abstractObjectPtrs_size);
  m_pAccelStruct_m_allNodePairs->resize(m_uboData.m_pAccelStruct_m_allNodePairs_size);
  m_pAccelStruct_m_geomData->resize(m_uboData.m_pAccelStruct_m_geomData_size);
  m_pAccelStruct_m_indices->resize(m_uboData.m_pAccelStruct_m_indices_size);
  m_pAccelStruct_m_instanceData->resize(m_uboData.m_pAccelStruct_m_instanceData_size);
  m_pAccelStruct_m_nodesTLAS->resize(m_uboData.m_pAccelStruct_m_nodesTLAS_size);
  m_pAccelStruct_m_origNodes->resize(m_uboData.m_pAccelStruct_m_origNodes_size);
  m_pAccelStruct_m_primIdCount->resize(m_uboData.m_pAccelStruct_m_primIdCount_size);
  m_pAccelStruct_m_primIndices->resize(m_uboData.m_pAccelStruct_m_primIndices_size);
  m_pAccelStruct_m_vertNorm->resize(m_uboData.m_pAccelStruct_m_vertNorm_size);
  m_pAccelStruct_m_vertPos->resize(m_uboData.m_pAccelStruct_m_vertPos_size);
  m_pAccelStruct_startEnd->resize(m_uboData.m_pAccelStruct_startEnd_size);
  m_packedXY.resize(m_uboData.m_packedXY_size);
  m_vertices.resize(m_uboData.m_vertices_size);
}

void MultiRenderer_gpu_rq::UpdateVectorMembers(std::shared_ptr<vk_utils::ICopyEngine> a_pCopyEngine)
{
  {
    auto pProxyObj = dynamic_cast<RTX_Proxy*>(m_pAccelStruct.get());
    auto tablePtrs = pProxyObj->GetAABBToPrimTable();
    if(tablePtrs.tableSize != 0 && tablePtrs.geomSize != 0)
    {
      a_pCopyEngine->UpdateBuffer(m_vdata.AbstractObjectRemapTableBuffer, 0, tablePtrs.table, tablePtrs.tableSize*sizeof(LiteMath::uint2));
      a_pCopyEngine->UpdateBuffer(m_vdata.AbstractObjectGeomTagsBuffer, 0, tablePtrs.geomTable, tablePtrs.geomSize*sizeof(LiteMath::uint));
    }
  }
  if(all_references.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.all_referencesBuffer, 0, all_references.data(), all_references.size()*sizeof(AllBufferReferences) );
  if(m_geomOffsets.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_geomOffsetsBuffer, 0, m_geomOffsets.data(), m_geomOffsets.size()*sizeof(struct LiteMath::uint2) );
  if(m_indices.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_indicesBuffer, 0, m_indices.data(), m_indices.size()*sizeof(unsigned int) );
  if(m_instanceTransformInvTransposed.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_instanceTransformInvTransposedBuffer, 0, m_instanceTransformInvTransposed.data(), m_instanceTransformInvTransposed.size()*sizeof(struct LiteMath::float4x4) );
  if(m_lights.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_lightsBuffer, 0, m_lights.data(), m_lights.size()*sizeof(struct Light) );
  if(m_matIdOffsets.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_matIdOffsetsBuffer, 0, m_matIdOffsets.data(), m_matIdOffsets.size()*sizeof(struct LiteMath::uint2) );
  if(m_matIdbyPrimId.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_matIdbyPrimIdBuffer, 0, m_matIdbyPrimId.data(), m_matIdbyPrimId.size()*sizeof(unsigned int) );
  if(m_materials.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_materialsBuffer, 0, m_materials.data(), m_materials.size()*sizeof(struct MultiRendererMaterial) );
  if(m_normals.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_normalsBuffer, 0, m_normals.data(), m_normals.size()*sizeof(struct LiteMath::float4) );
  if(m_pAccelStruct_m_CatmulClarkHeaders->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_CatmulClarkHeadersBuffer, 0, m_pAccelStruct_m_CatmulClarkHeaders->data(), m_pAccelStruct_m_CatmulClarkHeaders->size()*sizeof(struct CatmulClarkHeader) );
  if(m_pAccelStruct_m_GraphicsPrimHeaders->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_GraphicsPrimHeadersBuffer, 0, m_pAccelStruct_m_GraphicsPrimHeaders->data(), m_pAccelStruct_m_GraphicsPrimHeaders->size()*sizeof(struct GraphicsPrimHeader) );
  if(m_pAccelStruct_m_GraphicsPrimPoints->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_GraphicsPrimPointsBuffer, 0, m_pAccelStruct_m_GraphicsPrimPoints->data(), m_pAccelStruct_m_GraphicsPrimPoints->size()*sizeof(struct LiteMath::float4) );
  if(m_pAccelStruct_m_NURBSData->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_NURBSDataBuffer, 0, m_pAccelStruct_m_NURBSData->data(), m_pAccelStruct_m_NURBSData->size()*sizeof(float) );
  if(m_pAccelStruct_m_NURBSHeaders->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_NURBSHeadersBuffer, 0, m_pAccelStruct_m_NURBSHeaders->data(), m_pAccelStruct_m_NURBSHeaders->size()*sizeof(struct NURBSHeader) );
  if(m_pAccelStruct_m_NURBS_approxes->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_NURBS_approxesBuffer, 0, m_pAccelStruct_m_NURBS_approxes->data(), m_pAccelStruct_m_NURBS_approxes->size()*sizeof(float) );
  if(m_pAccelStruct_m_RibbonHeaders->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_RibbonHeadersBuffer, 0, m_pAccelStruct_m_RibbonHeaders->data(), m_pAccelStruct_m_RibbonHeaders->size()*sizeof(struct RibbonHeader) );
  if(m_pAccelStruct_m_SdfCompactOctreeRotModifiers->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_SdfCompactOctreeRotModifiersBuffer, 0, m_pAccelStruct_m_SdfCompactOctreeRotModifiers->data(), m_pAccelStruct_m_SdfCompactOctreeRotModifiers->size()*sizeof(struct LiteMath::int4) );
  if(m_pAccelStruct_m_SdfCompactOctreeV2Data->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_SdfCompactOctreeV2DataBuffer, 0, m_pAccelStruct_m_SdfCompactOctreeV2Data->data(), m_pAccelStruct_m_SdfCompactOctreeV2Data->size()*sizeof(unsigned int) );
  if(m_pAccelStruct_m_SdfCompactOctreeV3Data->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_SdfCompactOctreeV3DataBuffer, 0, m_pAccelStruct_m_SdfCompactOctreeV3Data->data(), m_pAccelStruct_m_SdfCompactOctreeV3Data->size()*sizeof(unsigned int) );
  if(m_pAccelStruct_m_SdfFrameOctreeNodes->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_SdfFrameOctreeNodesBuffer, 0, m_pAccelStruct_m_SdfFrameOctreeNodes->data(), m_pAccelStruct_m_SdfFrameOctreeNodes->size()*sizeof(struct SdfFrameOctreeNode) );
  if(m_pAccelStruct_m_SdfFrameOctreeRoots->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_SdfFrameOctreeRootsBuffer, 0, m_pAccelStruct_m_SdfFrameOctreeRoots->data(), m_pAccelStruct_m_SdfFrameOctreeRoots->size()*sizeof(unsigned int) );
  if(m_pAccelStruct_m_SdfFrameOctreeTexNodes->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_SdfFrameOctreeTexNodesBuffer, 0, m_pAccelStruct_m_SdfFrameOctreeTexNodes->data(), m_pAccelStruct_m_SdfFrameOctreeTexNodes->size()*sizeof(struct SdfFrameOctreeTexNode) );
  if(m_pAccelStruct_m_SdfFrameOctreeTexRoots->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_SdfFrameOctreeTexRootsBuffer, 0, m_pAccelStruct_m_SdfFrameOctreeTexRoots->data(), m_pAccelStruct_m_SdfFrameOctreeTexRoots->size()*sizeof(unsigned int) );
  if(m_pAccelStruct_m_SdfGridData->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_SdfGridDataBuffer, 0, m_pAccelStruct_m_SdfGridData->data(), m_pAccelStruct_m_SdfGridData->size()*sizeof(float) );
  if(m_pAccelStruct_m_SdfGridOffsets->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_SdfGridOffsetsBuffer, 0, m_pAccelStruct_m_SdfGridOffsets->data(), m_pAccelStruct_m_SdfGridOffsets->size()*sizeof(unsigned int) );
  if(m_pAccelStruct_m_SdfGridSizes->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_SdfGridSizesBuffer, 0, m_pAccelStruct_m_SdfGridSizes->data(), m_pAccelStruct_m_SdfGridSizes->size()*sizeof(struct LiteMath::uint3) );
  if(m_pAccelStruct_m_SdfSBSAdaptData->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_SdfSBSAdaptDataBuffer, 0, m_pAccelStruct_m_SdfSBSAdaptData->data(), m_pAccelStruct_m_SdfSBSAdaptData->size()*sizeof(unsigned int) );
  if(m_pAccelStruct_m_SdfSBSAdaptDataF->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_SdfSBSAdaptDataFBuffer, 0, m_pAccelStruct_m_SdfSBSAdaptDataF->data(), m_pAccelStruct_m_SdfSBSAdaptDataF->size()*sizeof(float) );
  if(m_pAccelStruct_m_SdfSBSAdaptHeaders->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_SdfSBSAdaptHeadersBuffer, 0, m_pAccelStruct_m_SdfSBSAdaptHeaders->data(), m_pAccelStruct_m_SdfSBSAdaptHeaders->size()*sizeof(struct SdfSBSAdaptHeader) );
  if(m_pAccelStruct_m_SdfSBSAdaptNodes->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_SdfSBSAdaptNodesBuffer, 0, m_pAccelStruct_m_SdfSBSAdaptNodes->data(), m_pAccelStruct_m_SdfSBSAdaptNodes->size()*sizeof(struct SdfSBSAdaptNode) );
  if(m_pAccelStruct_m_SdfSBSAdaptRoots->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_SdfSBSAdaptRootsBuffer, 0, m_pAccelStruct_m_SdfSBSAdaptRoots->data(), m_pAccelStruct_m_SdfSBSAdaptRoots->size()*sizeof(unsigned int) );
  if(m_pAccelStruct_m_SdfSBSData->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_SdfSBSDataBuffer, 0, m_pAccelStruct_m_SdfSBSData->data(), m_pAccelStruct_m_SdfSBSData->size()*sizeof(unsigned int) );
  if(m_pAccelStruct_m_SdfSBSDataF->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_SdfSBSDataFBuffer, 0, m_pAccelStruct_m_SdfSBSDataF->data(), m_pAccelStruct_m_SdfSBSDataF->size()*sizeof(float) );
  if(m_pAccelStruct_m_SdfSBSHeaders->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_SdfSBSHeadersBuffer, 0, m_pAccelStruct_m_SdfSBSHeaders->data(), m_pAccelStruct_m_SdfSBSHeaders->size()*sizeof(struct SdfSBSHeader) );
  if(m_pAccelStruct_m_SdfSBSNodes->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_SdfSBSNodesBuffer, 0, m_pAccelStruct_m_SdfSBSNodes->data(), m_pAccelStruct_m_SdfSBSNodes->size()*sizeof(struct SdfSBSNode) );
  if(m_pAccelStruct_m_SdfSBSRoots->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_SdfSBSRootsBuffer, 0, m_pAccelStruct_m_SdfSBSRoots->data(), m_pAccelStruct_m_SdfSBSRoots->size()*sizeof(unsigned int) );
  if(m_pAccelStruct_m_SdfSVSNodes->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_SdfSVSNodesBuffer, 0, m_pAccelStruct_m_SdfSVSNodes->data(), m_pAccelStruct_m_SdfSVSNodes->size()*sizeof(struct SdfSVSNode) );
  if(m_pAccelStruct_m_SdfSVSRoots->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_SdfSVSRootsBuffer, 0, m_pAccelStruct_m_SdfSVSRoots->data(), m_pAccelStruct_m_SdfSVSRoots->size()*sizeof(unsigned int) );
  if(m_pAccelStruct_m_abstractObjectPtrs->size() > 0)
  {
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_abstractObjectPtrsBuffer      , 0, m_pAccelStruct_m_abstractObjectPtrs_vtable.data(), m_pAccelStruct_m_abstractObjectPtrs_vtable.size()*sizeof(unsigned)*2 );
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_abstractObjectPtrs_dataVBuffer, 0, m_pAccelStruct_m_abstractObjectPtrs_dataV.data(), m_pAccelStruct_m_abstractObjectPtrs_dataV.size());
    for(auto it = m_pAccelStruct_m_abstractObjectPtrs_obj_storage_offsets.begin(); it != m_pAccelStruct_m_abstractObjectPtrs_obj_storage_offsets.end(); ++it) 
    {
      size_t     offset = it->second;
      const auto& odata = m_pAccelStruct_m_abstractObjectPtrs_sorted[it->first];
      if(odata.size() != 0)
        a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_abstractObjectPtrs_dataSBuffer, offset, odata.data(), odata.size());
    }
  }
  if(m_pAccelStruct_m_allNodePairs->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_allNodePairsBuffer, 0, m_pAccelStruct_m_allNodePairs->data(), m_pAccelStruct_m_allNodePairs->size()*sizeof(struct BVHNodePair) );
  if(m_pAccelStruct_m_geomData->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_geomDataBuffer, 0, m_pAccelStruct_m_geomData->data(), m_pAccelStruct_m_geomData->size()*sizeof(struct GeomData) );
  if(m_pAccelStruct_m_indices->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_indicesBuffer, 0, m_pAccelStruct_m_indices->data(), m_pAccelStruct_m_indices->size()*sizeof(unsigned int) );
  if(m_pAccelStruct_m_instanceData->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_instanceDataBuffer, 0, m_pAccelStruct_m_instanceData->data(), m_pAccelStruct_m_instanceData->size()*sizeof(struct InstanceData) );
  if(m_pAccelStruct_m_nodesTLAS->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_nodesTLASBuffer, 0, m_pAccelStruct_m_nodesTLAS->data(), m_pAccelStruct_m_nodesTLAS->size()*sizeof(struct BVHNode) );
  if(m_pAccelStruct_m_origNodes->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_origNodesBuffer, 0, m_pAccelStruct_m_origNodes->data(), m_pAccelStruct_m_origNodes->size()*sizeof(struct BVHNode) );
  if(m_pAccelStruct_m_primIdCount->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_primIdCountBuffer, 0, m_pAccelStruct_m_primIdCount->data(), m_pAccelStruct_m_primIdCount->size()*sizeof(unsigned int) );
  if(m_pAccelStruct_m_primIndices->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_primIndicesBuffer, 0, m_pAccelStruct_m_primIndices->data(), m_pAccelStruct_m_primIndices->size()*sizeof(unsigned int) );
  if(m_pAccelStruct_m_vertNorm->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_vertNormBuffer, 0, m_pAccelStruct_m_vertNorm->data(), m_pAccelStruct_m_vertNorm->size()*sizeof(struct LiteMath::float4) );
  if(m_pAccelStruct_m_vertPos->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_vertPosBuffer, 0, m_pAccelStruct_m_vertPos->data(), m_pAccelStruct_m_vertPos->size()*sizeof(struct LiteMath::float4) );
  if(m_pAccelStruct_startEnd->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_startEndBuffer, 0, m_pAccelStruct_startEnd->data(), m_pAccelStruct_startEnd->size()*sizeof(struct LiteMath::uint2) );
  if(m_packedXY.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_packedXYBuffer, 0, m_packedXY.data(), m_packedXY.size()*sizeof(unsigned int) );
  if(m_vertices.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_verticesBuffer, 0, m_vertices.data(), m_vertices.size()*sizeof(struct LiteMath::float4) );
}


void MultiRenderer_gpu_rq::UpdateTextureMembers(std::shared_ptr<vk_utils::ICopyEngine> a_pCopyEngine)
{
  for(int i=0;i<m_vdata.m_texturesArrayTexture.size();i++)
    a_pCopyEngine->UpdateImage(m_vdata.m_texturesArrayTexture[i], m_textures[i]->data(), m_textures[i]->width(), m_textures[i]->height(), m_textures[i]->bpp(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

  std::array<VkImageMemoryBarrier, 0> barriers;


  VkCommandBuffer cmdBuff       = a_pCopyEngine->CmdBuffer();
  VkQueue         transferQueue = a_pCopyEngine->TransferQueue();

  vkResetCommandBuffer(cmdBuff, 0);
  VkCommandBufferBeginInfo beginInfo = {};
  beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
  beginInfo.flags = VK_COMMAND_BUFFER_USAGE_SIMULTANEOUS_USE_BIT;
  if (vkBeginCommandBuffer(cmdBuff, &beginInfo) != VK_SUCCESS)
    throw std::runtime_error("MultiRenderer_gpu_rq::UpdateTextureMembers: failed to begin command buffer!");
  vkCmdPipelineBarrier(cmdBuff,VK_PIPELINE_STAGE_TRANSFER_BIT,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,0,0,nullptr,0,nullptr,uint32_t(barriers.size()),barriers.data());
  vkEndCommandBuffer(cmdBuff);

  vk_utils::executeCommandBufferNow(cmdBuff, transferQueue, device);
}

void MultiRenderer_gpu_rq::PackXYMegaCmd(uint tidX, uint tidY)
{
  uint32_t blockSizeX = 256;
  uint32_t blockSizeY = 1;
  uint32_t blockSizeZ = 1;

  struct KernelArgsPC
  {
    uint32_t m_sizeX;
    uint32_t m_sizeY;
    uint32_t m_sizeZ;
    uint32_t m_tFlags;
  } pcData;

  uint32_t sizeX  = uint32_t(tidX);
  uint32_t sizeY  = uint32_t(tidY);
  uint32_t sizeZ  = uint32_t(1);

  pcData.m_sizeX  = tidX;
  pcData.m_sizeY  = tidY;
  pcData.m_sizeZ  = 1;
  pcData.m_tFlags = m_currThreadFlags;
  vkCmdPushConstants(m_currCmdBuffer, PackXYMegaLayout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(KernelArgsPC), &pcData);
  
  vkCmdBindPipeline(m_currCmdBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, PackXYMegaPipeline);
  vkCmdDispatch    (m_currCmdBuffer, (sizeX + blockSizeX - 1) / blockSizeX, (sizeY + blockSizeY - 1) / blockSizeY, (sizeZ + blockSizeZ - 1) / blockSizeZ);
}

void MultiRenderer_gpu_rq::CastRayFloatSingleMegaCmd(uint32_t tidX, float4* out_color)
{
  uint32_t blockSizeX = 256;
  uint32_t blockSizeY = 1;
  uint32_t blockSizeZ = 1;

  struct KernelArgsPC
  {
    uint32_t m_sizeX;
    uint32_t m_sizeY;
    uint32_t m_sizeZ;
    uint32_t m_tFlags;
  } pcData;

  uint32_t sizeX  = uint32_t(tidX);
  uint32_t sizeY  = uint32_t(1);
  uint32_t sizeZ  = uint32_t(1);

  pcData.m_sizeX  = tidX;
  pcData.m_sizeY  = 1;
  pcData.m_sizeZ  = 1;
  pcData.m_tFlags = m_currThreadFlags;
  vkCmdPushConstants(m_currCmdBuffer, CastRayFloatSingleMegaLayout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(KernelArgsPC), &pcData);
  
  vkCmdBindPipeline(m_currCmdBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, CastRayFloatSingleMegaPipeline);
  vkCmdDispatch    (m_currCmdBuffer, (sizeX + blockSizeX - 1) / blockSizeX, (sizeY + blockSizeY - 1) / blockSizeY, (sizeZ + blockSizeZ - 1) / blockSizeZ);
}

void MultiRenderer_gpu_rq::CastRaySingleMegaCmd(uint32_t tidX, uint32_t* out_color)
{
  uint32_t blockSizeX = 256;
  uint32_t blockSizeY = 1;
  uint32_t blockSizeZ = 1;

  struct KernelArgsPC
  {
    uint32_t m_sizeX;
    uint32_t m_sizeY;
    uint32_t m_sizeZ;
    uint32_t m_tFlags;
  } pcData;

  uint32_t sizeX  = uint32_t(tidX);
  uint32_t sizeY  = uint32_t(1);
  uint32_t sizeZ  = uint32_t(1);

  pcData.m_sizeX  = tidX;
  pcData.m_sizeY  = 1;
  pcData.m_sizeZ  = 1;
  pcData.m_tFlags = m_currThreadFlags;
  vkCmdPushConstants(m_currCmdBuffer, CastRaySingleMegaLayout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(KernelArgsPC), &pcData);
  
  vkCmdBindPipeline(m_currCmdBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, CastRaySingleMegaPipeline);
  vkCmdDispatch    (m_currCmdBuffer, (sizeX + blockSizeX - 1) / blockSizeX, (sizeY + blockSizeY - 1) / blockSizeY, (sizeZ + blockSizeZ - 1) / blockSizeZ);
}


void MultiRenderer_gpu_rq::copyKernelFloatCmd(uint32_t length)
{
  uint32_t blockSizeX = MEMCPY_BLOCK_SIZE;

  vkCmdBindPipeline(m_currCmdBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, copyKernelFloatPipeline);
  vkCmdPushConstants(m_currCmdBuffer, copyKernelFloatLayout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(uint32_t), &length);
  vkCmdDispatch(m_currCmdBuffer, (length + blockSizeX - 1) / blockSizeX, 1, 1);
}

void MultiRenderer_gpu_rq::matMulTransposeCmd(uint32_t A_offset, uint32_t B_offset, uint32_t C_offset, uint32_t A_col_len, uint32_t B_col_len, uint32_t A_row_len)
{
  const uint32_t blockSizeX = 8;
  const uint32_t blockSizeY = 8;

  vkCmdBindPipeline(m_currCmdBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, matMulTransposePipeline);
  struct KernelArgsPC
  {
    uint32_t m_A_row_len;
    uint32_t m_sizeX;
    uint32_t m_sizeY;
    uint32_t m_A_offset;
    uint32_t m_B_offset;
    uint32_t m_C_offset;
  } pcData;
  pcData.m_A_row_len = A_row_len;
  pcData.m_sizeX = B_col_len;
  pcData.m_sizeY = A_col_len;
  pcData.m_A_offset = A_offset;
  pcData.m_B_offset = B_offset;
  pcData.m_C_offset = C_offset;
  vkCmdPushConstants(m_currCmdBuffer, matMulTransposeLayout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(pcData), &pcData);
  vkCmdDispatch(m_currCmdBuffer, (B_col_len + blockSizeX - 1) / blockSizeX, (A_col_len + blockSizeY - 1) / blockSizeY, 1);
}

VkBufferMemoryBarrier MultiRenderer_gpu_rq::BarrierForClearFlags(VkBuffer a_buffer)
{
  VkBufferMemoryBarrier bar = {};
  bar.sType               = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
  bar.pNext               = NULL;
  bar.srcAccessMask       = VK_ACCESS_TRANSFER_WRITE_BIT;
  bar.dstAccessMask       = VK_ACCESS_SHADER_READ_BIT;
  bar.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  bar.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  bar.buffer              = a_buffer;
  bar.offset              = 0;
  bar.size                = VK_WHOLE_SIZE;
  return bar;
}

VkBufferMemoryBarrier MultiRenderer_gpu_rq::BarrierForSingleBuffer(VkBuffer a_buffer)
{
  VkBufferMemoryBarrier bar = {};
  bar.sType               = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
  bar.pNext               = NULL;
  bar.srcAccessMask       = VK_ACCESS_SHADER_WRITE_BIT;
  bar.dstAccessMask       = VK_ACCESS_SHADER_READ_BIT;
  bar.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  bar.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  bar.buffer              = a_buffer;
  bar.offset              = 0;
  bar.size                = VK_WHOLE_SIZE;
  return bar;
}

void MultiRenderer_gpu_rq::BarriersForSeveralBuffers(VkBuffer* a_inBuffers, VkBufferMemoryBarrier* a_outBarriers, uint32_t a_buffersNum)
{
  for(uint32_t i=0; i<a_buffersNum;i++)
  {
    a_outBarriers[i].sType               = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
    a_outBarriers[i].pNext               = NULL;
    a_outBarriers[i].srcAccessMask       = VK_ACCESS_SHADER_WRITE_BIT;
    a_outBarriers[i].dstAccessMask       = VK_ACCESS_SHADER_READ_BIT;
    a_outBarriers[i].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    a_outBarriers[i].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    a_outBarriers[i].buffer              = a_inBuffers[i];
    a_outBarriers[i].offset              = 0;
    a_outBarriers[i].size                = VK_WHOLE_SIZE;
  }
}

void MultiRenderer_gpu_rq::PackXYCmd(VkCommandBuffer a_commandBuffer, uint tidX, uint tidY)
{
  VkPipelineStageFlagBits prevStageBits = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
  m_currCmdBuffer = a_commandBuffer;
  VkMemoryBarrier memoryBarrier = { VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr, VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT };
  vkCmdBindDescriptorSets(a_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, PackXYMegaLayout, 0, 1, &m_allGeneratedDS[0], 0, nullptr);
  vkCmdWriteTimestamp(a_commandBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, m_queryPoolTimestamps, 0);
  PackXYMegaCmd(tidX, tidY);
  vkCmdWriteTimestamp(a_commandBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, m_queryPoolTimestamps, 1);
  m_tsIdToKernelName[0] = "PackXYMega";
  vkCmdPipelineBarrier(m_currCmdBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1, &memoryBarrier, 0, nullptr, 0, nullptr);
}

void MultiRenderer_gpu_rq::CastRayFloatSingleCmd(VkCommandBuffer a_commandBuffer, uint32_t tidX, float4* out_color)
{
  VkPipelineStageFlagBits prevStageBits = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
  m_currCmdBuffer = a_commandBuffer;
  VkMemoryBarrier memoryBarrier = { VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr, VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT };
  vkCmdBindDescriptorSets(a_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, CastRayFloatSingleMegaLayout, 0, 1, &m_allGeneratedDS[1], 0, nullptr);
  vkCmdWriteTimestamp(a_commandBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, m_queryPoolTimestamps, 0);
  CastRayFloatSingleMegaCmd(tidX, out_color);
  vkCmdWriteTimestamp(a_commandBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, m_queryPoolTimestamps, 1);
  m_tsIdToKernelName[0] = "CastRayFloatSingleMega";
  vkCmdPipelineBarrier(m_currCmdBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1, &memoryBarrier, 0, nullptr, 0, nullptr);
}

void MultiRenderer_gpu_rq::CastRaySingleCmd(VkCommandBuffer a_commandBuffer, uint32_t tidX, uint32_t* out_color)
{
  VkPipelineStageFlagBits prevStageBits = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
  m_currCmdBuffer = a_commandBuffer;
  VkMemoryBarrier memoryBarrier = { VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr, VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT };
  vkCmdBindDescriptorSets(a_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, CastRaySingleMegaLayout, 0, 1, &m_allGeneratedDS[2], 0, nullptr);
  vkCmdWriteTimestamp(a_commandBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, m_queryPoolTimestamps, 0);
  CastRaySingleMegaCmd(tidX, out_color);
  vkCmdWriteTimestamp(a_commandBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, m_queryPoolTimestamps, 1);
  m_tsIdToKernelName[0] = "CastRaySingleMega";
  vkCmdPipelineBarrier(m_currCmdBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1, &memoryBarrier, 0, nullptr, 0, nullptr);
}



void MultiRenderer_gpu_rq::PackXYBlock(uint tidX, uint tidY, uint32_t a_numPasses)
{
  // (1) get global Vulkan context objects
  //
  VkInstance       instance       = m_ctx.instance;
  VkPhysicalDevice physicalDevice = m_ctx.physicalDevice;
  VkDevice         device         = m_ctx.device;
  VkCommandPool    commandPool    = m_ctx.commandPool;
  VkQueue          computeQueue   = m_ctx.computeQueue;
  VkQueue          transferQueue  = m_ctx.transferQueue;
  auto             pCopyHelper    = m_ctx.pCopyHelper;
  auto             pAllocatorSpec = m_ctx.pAllocatorSpecial;

  // (2) create GPU objects
  //
  auto outFlags = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
  if(PackXY_local.needToClearOutput)
    outFlags |= VK_IMAGE_USAGE_TRANSFER_DST_BIT;
  std::vector<VkBuffer> buffers;
  std::vector<VkImage>  images2;
  std::vector<vk_utils::VulkanImageMem*> images;
  auto beforeCreateObjects = std::chrono::high_resolution_clock::now();


  VkDeviceMemory buffersMem = VK_NULL_HANDLE; // vk_utils::allocateAndBindWithPadding(device, physicalDevice, buffers);
  VkDeviceMemory imagesMem  = VK_NULL_HANDLE; // vk_utils::allocateAndBindWithPadding(device, physicalDevice, std::vector<VkBuffer>(), images);

  vk_utils::MemAllocInfo tempMemoryAllocInfo;
  tempMemoryAllocInfo.memUsage = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT; // TODO, select depending on device and sample/application (???)
  if(buffers.size() != 0)
    pAllocatorSpec->Allocate(tempMemoryAllocInfo, buffers);
  if(images.size() != 0)
  {
    pAllocatorSpec->Allocate(tempMemoryAllocInfo, images2);
    for(auto imgMem : images)
    {
      VkImageViewCreateInfo imageView{};
      imageView.sType    = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
      imageView.viewType = VK_IMAGE_VIEW_TYPE_2D;
      imageView.image    = imgMem->image;
      imageView.format   = imgMem->format;
      imageView.subresourceRange = {};
      imageView.subresourceRange.aspectMask     = imgMem->aspectMask;
      imageView.subresourceRange.baseMipLevel   = 0;
      imageView.subresourceRange.levelCount     = imgMem->mipLvls;
      imageView.subresourceRange.baseArrayLayer = 0;
      imageView.subresourceRange.layerCount     = 1;
      VK_CHECK_RESULT(vkCreateImageView(device, &imageView, nullptr, &imgMem->view));
    }
  }

  auto afterCreateObjects = std::chrono::high_resolution_clock::now();
  m_exTimePackXY.msAPIOverhead = std::chrono::duration_cast<std::chrono::microseconds>(afterCreateObjects - beforeCreateObjects).count()/1000.f;

  auto afterCopy2 = std::chrono::high_resolution_clock::now(); // just declare it here, replace value later

  auto afterInitBuffers = std::chrono::high_resolution_clock::now();
  m_exTimePackXY.msAPIOverhead += std::chrono::duration_cast<std::chrono::microseconds>(afterInitBuffers - afterCreateObjects).count()/1000.f;

  auto beforeSetInOut = std::chrono::high_resolution_clock::now();
  this->SetVulkanInOutFor_PackXY();

  // (3) copy input data to GPU
  //
  auto beforeCopy = std::chrono::high_resolution_clock::now();
  m_exTimePackXY.msAPIOverhead += std::chrono::duration_cast<std::chrono::microseconds>(beforeCopy - beforeSetInOut).count()/1000.f;
  auto afterCopy = std::chrono::high_resolution_clock::now();
  m_exTimePackXY.msCopyToGPU = std::chrono::duration_cast<std::chrono::microseconds>(afterCopy - beforeCopy).count()/1000.f;
  //
  m_exTimePackXY.msExecuteOnGPU = 0;
  //// (3.1) clear all outputs if we are in RTV pattern
  //
  if(PackXY_local.needToClearOutput)
  {
    VkCommandBuffer commandBuffer = vk_utils::createCommandBuffer(device, commandPool);
    VkCommandBufferBeginInfo beginCommandBufferInfo = {};
    beginCommandBufferInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginCommandBufferInfo.flags = VK_COMMAND_BUFFER_USAGE_SIMULTANEOUS_USE_BIT;
    vkBeginCommandBuffer(commandBuffer, &beginCommandBufferInfo);
    vkEndCommandBuffer(commandBuffer);
    auto start = std::chrono::high_resolution_clock::now();
    vk_utils::executeCommandBufferNow(commandBuffer, computeQueue, device);
    vkFreeCommandBuffers(device, commandPool, 1, &commandBuffer);
    auto stop = std::chrono::high_resolution_clock::now();
    m_exTimePackXY.msExecuteOnGPU  += std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count()/1000.f;
  }

  // (4) now execute algorithm on GPU
  //
  {
    VkCommandBuffer commandBuffer = vk_utils::createCommandBuffer(device, commandPool);
    VkCommandBufferBeginInfo beginCommandBufferInfo = {};
    beginCommandBufferInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginCommandBufferInfo.flags = VK_COMMAND_BUFFER_USAGE_SIMULTANEOUS_USE_BIT;
    vkBeginCommandBuffer(commandBuffer, &beginCommandBufferInfo);
    vkCmdResetQueryPool(commandBuffer, m_queryPoolTimestamps, 0, m_timestampPoolSize);
    PackXYCmd(commandBuffer, tidX, tidY);
    vkEndCommandBuffer(commandBuffer);
    auto start = std::chrono::high_resolution_clock::now();
    for(uint32_t pass = 0; pass < a_numPasses; pass++) {
      vk_utils::executeCommandBufferNow(commandBuffer, computeQueue, device);
      AccumTimeStampMeasurements(0*2, 1*2);
    }
    vkFreeCommandBuffers(device, commandPool, 1, &commandBuffer);
    auto stop = std::chrono::high_resolution_clock::now();
    m_exTimePackXY.msExecuteOnGPU += std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count()/1000.f;
  }

  // (5) copy output data to CPU
  //
  auto beforeCopy2 = std::chrono::high_resolution_clock::now();
  this->ReadPlainMembers(pCopyHelper);
  afterCopy2 = std::chrono::high_resolution_clock::now();
  m_exTimePackXY.msCopyFromGPU = std::chrono::duration_cast<std::chrono::microseconds>(afterCopy2 - beforeCopy2).count()/1000.f;

  // (6) free resources
  //
  if(buffersMem != VK_NULL_HANDLE)
    vkFreeMemory(device, buffersMem, nullptr);
  if(imagesMem != VK_NULL_HANDLE)
    vkFreeMemory(device, imagesMem, nullptr);

  m_exTimePackXY.msAPIOverhead += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - afterCopy2).count()/1000.f;
}

void MultiRenderer_gpu_rq::CastRayFloatSingleBlock(uint32_t tidX, float4* out_color, uint32_t a_numPasses)
{
  // (1) get global Vulkan context objects
  //
  VkInstance       instance       = m_ctx.instance;
  VkPhysicalDevice physicalDevice = m_ctx.physicalDevice;
  VkDevice         device         = m_ctx.device;
  VkCommandPool    commandPool    = m_ctx.commandPool;
  VkQueue          computeQueue   = m_ctx.computeQueue;
  VkQueue          transferQueue  = m_ctx.transferQueue;
  auto             pCopyHelper    = m_ctx.pCopyHelper;
  auto             pAllocatorSpec = m_ctx.pAllocatorSpecial;

  // (2) create GPU objects
  //
  auto outFlags = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
  if(CastRayFloatSingle_local.needToClearOutput)
    outFlags |= VK_IMAGE_USAGE_TRANSFER_DST_BIT;
  std::vector<VkBuffer> buffers;
  std::vector<VkImage>  images2;
  std::vector<vk_utils::VulkanImageMem*> images;
  auto beforeCreateObjects = std::chrono::high_resolution_clock::now();
  VkBuffer out_colorGPU = vk_utils::createBuffer(device, tidX*sizeof(float4 ), outFlags);
  buffers.push_back(out_colorGPU);


  VkDeviceMemory buffersMem = VK_NULL_HANDLE; // vk_utils::allocateAndBindWithPadding(device, physicalDevice, buffers);
  VkDeviceMemory imagesMem  = VK_NULL_HANDLE; // vk_utils::allocateAndBindWithPadding(device, physicalDevice, std::vector<VkBuffer>(), images);

  vk_utils::MemAllocInfo tempMemoryAllocInfo;
  tempMemoryAllocInfo.memUsage = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT; // TODO, select depending on device and sample/application (???)
  if(buffers.size() != 0)
    pAllocatorSpec->Allocate(tempMemoryAllocInfo, buffers);
  if(images.size() != 0)
  {
    pAllocatorSpec->Allocate(tempMemoryAllocInfo, images2);
    for(auto imgMem : images)
    {
      VkImageViewCreateInfo imageView{};
      imageView.sType    = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
      imageView.viewType = VK_IMAGE_VIEW_TYPE_2D;
      imageView.image    = imgMem->image;
      imageView.format   = imgMem->format;
      imageView.subresourceRange = {};
      imageView.subresourceRange.aspectMask     = imgMem->aspectMask;
      imageView.subresourceRange.baseMipLevel   = 0;
      imageView.subresourceRange.levelCount     = imgMem->mipLvls;
      imageView.subresourceRange.baseArrayLayer = 0;
      imageView.subresourceRange.layerCount     = 1;
      VK_CHECK_RESULT(vkCreateImageView(device, &imageView, nullptr, &imgMem->view));
    }
  }

  auto afterCreateObjects = std::chrono::high_resolution_clock::now();
  m_exTimeCastRayFloatSingle.msAPIOverhead = std::chrono::duration_cast<std::chrono::microseconds>(afterCreateObjects - beforeCreateObjects).count()/1000.f;

  auto afterCopy2 = std::chrono::high_resolution_clock::now(); // just declare it here, replace value later

  auto afterInitBuffers = std::chrono::high_resolution_clock::now();
  m_exTimeCastRayFloatSingle.msAPIOverhead += std::chrono::duration_cast<std::chrono::microseconds>(afterInitBuffers - afterCreateObjects).count()/1000.f;

  auto beforeSetInOut = std::chrono::high_resolution_clock::now();
  this->SetVulkanInOutFor_CastRayFloatSingle(out_colorGPU, 0);

  // (3) copy input data to GPU
  //
  auto beforeCopy = std::chrono::high_resolution_clock::now();
  m_exTimeCastRayFloatSingle.msAPIOverhead += std::chrono::duration_cast<std::chrono::microseconds>(beforeCopy - beforeSetInOut).count()/1000.f;
  auto afterCopy = std::chrono::high_resolution_clock::now();
  m_exTimeCastRayFloatSingle.msCopyToGPU = std::chrono::duration_cast<std::chrono::microseconds>(afterCopy - beforeCopy).count()/1000.f;
  //
  m_exTimeCastRayFloatSingle.msExecuteOnGPU = 0;
  //// (3.1) clear all outputs if we are in RTV pattern
  //
  if(CastRayFloatSingle_local.needToClearOutput)
  {
    VkCommandBuffer commandBuffer = vk_utils::createCommandBuffer(device, commandPool);
    VkCommandBufferBeginInfo beginCommandBufferInfo = {};
    beginCommandBufferInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginCommandBufferInfo.flags = VK_COMMAND_BUFFER_USAGE_SIMULTANEOUS_USE_BIT;
    vkBeginCommandBuffer(commandBuffer, &beginCommandBufferInfo);
    vkCmdFillBuffer(commandBuffer, out_colorGPU, 0, VK_WHOLE_SIZE, 0); // zero output buffer out_colorGPU
    vkEndCommandBuffer(commandBuffer);
    auto start = std::chrono::high_resolution_clock::now();
    vk_utils::executeCommandBufferNow(commandBuffer, computeQueue, device);
    vkFreeCommandBuffers(device, commandPool, 1, &commandBuffer);
    auto stop = std::chrono::high_resolution_clock::now();
    m_exTimeCastRayFloatSingle.msExecuteOnGPU  += std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count()/1000.f;
  }

  // (4) now execute algorithm on GPU
  //
  {
    VkCommandBuffer commandBuffer = vk_utils::createCommandBuffer(device, commandPool);
    VkCommandBufferBeginInfo beginCommandBufferInfo = {};
    beginCommandBufferInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginCommandBufferInfo.flags = VK_COMMAND_BUFFER_USAGE_SIMULTANEOUS_USE_BIT;
    vkBeginCommandBuffer(commandBuffer, &beginCommandBufferInfo);
    vkCmdResetQueryPool(commandBuffer, m_queryPoolTimestamps, 0, m_timestampPoolSize);
    CastRayFloatSingleCmd(commandBuffer, tidX, out_color);
    vkEndCommandBuffer(commandBuffer);
    auto start = std::chrono::high_resolution_clock::now();
    for(uint32_t pass = 0; pass < a_numPasses; pass++) {
      vk_utils::executeCommandBufferNow(commandBuffer, computeQueue, device);
      AccumTimeStampMeasurements(0*2, 1*2);
    }
    vkFreeCommandBuffers(device, commandPool, 1, &commandBuffer);
    auto stop = std::chrono::high_resolution_clock::now();
    m_exTimeCastRayFloatSingle.msExecuteOnGPU += std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count()/1000.f;
  }

  // (5) copy output data to CPU
  //
  auto beforeCopy2 = std::chrono::high_resolution_clock::now();
  pCopyHelper->ReadBuffer(out_colorGPU, 0, out_color, tidX*sizeof(float4 ));
  this->ReadPlainMembers(pCopyHelper);
  afterCopy2 = std::chrono::high_resolution_clock::now();
  m_exTimeCastRayFloatSingle.msCopyFromGPU = std::chrono::duration_cast<std::chrono::microseconds>(afterCopy2 - beforeCopy2).count()/1000.f;

  // (6) free resources
  //
  vkDestroyBuffer(device, out_colorGPU, nullptr);
  if(buffersMem != VK_NULL_HANDLE)
    vkFreeMemory(device, buffersMem, nullptr);
  if(imagesMem != VK_NULL_HANDLE)
    vkFreeMemory(device, imagesMem, nullptr);

  m_exTimeCastRayFloatSingle.msAPIOverhead += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - afterCopy2).count()/1000.f;
}

void MultiRenderer_gpu_rq::CastRaySingleBlock(uint32_t tidX, uint32_t* out_color, uint32_t a_numPasses)
{
  // (1) get global Vulkan context objects
  //
  VkInstance       instance       = m_ctx.instance;
  VkPhysicalDevice physicalDevice = m_ctx.physicalDevice;
  VkDevice         device         = m_ctx.device;
  VkCommandPool    commandPool    = m_ctx.commandPool;
  VkQueue          computeQueue   = m_ctx.computeQueue;
  VkQueue          transferQueue  = m_ctx.transferQueue;
  auto             pCopyHelper    = m_ctx.pCopyHelper;
  auto             pAllocatorSpec = m_ctx.pAllocatorSpecial;

  // (2) create GPU objects
  //
  auto outFlags = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
  if(CastRaySingle_local.needToClearOutput)
    outFlags |= VK_IMAGE_USAGE_TRANSFER_DST_BIT;
  std::vector<VkBuffer> buffers;
  std::vector<VkImage>  images2;
  std::vector<vk_utils::VulkanImageMem*> images;
  auto beforeCreateObjects = std::chrono::high_resolution_clock::now();
  VkBuffer out_colorGPU = vk_utils::createBuffer(device, tidX*sizeof(uint32_t ), outFlags);
  buffers.push_back(out_colorGPU);


  VkDeviceMemory buffersMem = VK_NULL_HANDLE; // vk_utils::allocateAndBindWithPadding(device, physicalDevice, buffers);
  VkDeviceMemory imagesMem  = VK_NULL_HANDLE; // vk_utils::allocateAndBindWithPadding(device, physicalDevice, std::vector<VkBuffer>(), images);

  vk_utils::MemAllocInfo tempMemoryAllocInfo;
  tempMemoryAllocInfo.memUsage = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT; // TODO, select depending on device and sample/application (???)
  if(buffers.size() != 0)
    pAllocatorSpec->Allocate(tempMemoryAllocInfo, buffers);
  if(images.size() != 0)
  {
    pAllocatorSpec->Allocate(tempMemoryAllocInfo, images2);
    for(auto imgMem : images)
    {
      VkImageViewCreateInfo imageView{};
      imageView.sType    = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
      imageView.viewType = VK_IMAGE_VIEW_TYPE_2D;
      imageView.image    = imgMem->image;
      imageView.format   = imgMem->format;
      imageView.subresourceRange = {};
      imageView.subresourceRange.aspectMask     = imgMem->aspectMask;
      imageView.subresourceRange.baseMipLevel   = 0;
      imageView.subresourceRange.levelCount     = imgMem->mipLvls;
      imageView.subresourceRange.baseArrayLayer = 0;
      imageView.subresourceRange.layerCount     = 1;
      VK_CHECK_RESULT(vkCreateImageView(device, &imageView, nullptr, &imgMem->view));
    }
  }

  auto afterCreateObjects = std::chrono::high_resolution_clock::now();
  m_exTimeCastRaySingle.msAPIOverhead = std::chrono::duration_cast<std::chrono::microseconds>(afterCreateObjects - beforeCreateObjects).count()/1000.f;

  auto afterCopy2 = std::chrono::high_resolution_clock::now(); // just declare it here, replace value later

  auto afterInitBuffers = std::chrono::high_resolution_clock::now();
  m_exTimeCastRaySingle.msAPIOverhead += std::chrono::duration_cast<std::chrono::microseconds>(afterInitBuffers - afterCreateObjects).count()/1000.f;

  auto beforeSetInOut = std::chrono::high_resolution_clock::now();
  this->SetVulkanInOutFor_CastRaySingle(out_colorGPU, 0);

  // (3) copy input data to GPU
  //
  auto beforeCopy = std::chrono::high_resolution_clock::now();
  m_exTimeCastRaySingle.msAPIOverhead += std::chrono::duration_cast<std::chrono::microseconds>(beforeCopy - beforeSetInOut).count()/1000.f;
  auto afterCopy = std::chrono::high_resolution_clock::now();
  m_exTimeCastRaySingle.msCopyToGPU = std::chrono::duration_cast<std::chrono::microseconds>(afterCopy - beforeCopy).count()/1000.f;
  //
  m_exTimeCastRaySingle.msExecuteOnGPU = 0;
  //// (3.1) clear all outputs if we are in RTV pattern
  //
  if(CastRaySingle_local.needToClearOutput)
  {
    VkCommandBuffer commandBuffer = vk_utils::createCommandBuffer(device, commandPool);
    VkCommandBufferBeginInfo beginCommandBufferInfo = {};
    beginCommandBufferInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginCommandBufferInfo.flags = VK_COMMAND_BUFFER_USAGE_SIMULTANEOUS_USE_BIT;
    vkBeginCommandBuffer(commandBuffer, &beginCommandBufferInfo);
    vkCmdFillBuffer(commandBuffer, out_colorGPU, 0, VK_WHOLE_SIZE, 0); // zero output buffer out_colorGPU
    vkEndCommandBuffer(commandBuffer);
    auto start = std::chrono::high_resolution_clock::now();
    vk_utils::executeCommandBufferNow(commandBuffer, computeQueue, device);
    vkFreeCommandBuffers(device, commandPool, 1, &commandBuffer);
    auto stop = std::chrono::high_resolution_clock::now();
    m_exTimeCastRaySingle.msExecuteOnGPU  += std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count()/1000.f;
  }

  // (4) now execute algorithm on GPU
  //
  {
    VkCommandBuffer commandBuffer = vk_utils::createCommandBuffer(device, commandPool);
    VkCommandBufferBeginInfo beginCommandBufferInfo = {};
    beginCommandBufferInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginCommandBufferInfo.flags = VK_COMMAND_BUFFER_USAGE_SIMULTANEOUS_USE_BIT;
    vkBeginCommandBuffer(commandBuffer, &beginCommandBufferInfo);
    vkCmdResetQueryPool(commandBuffer, m_queryPoolTimestamps, 0, m_timestampPoolSize);
    CastRaySingleCmd(commandBuffer, tidX, out_color);
    vkEndCommandBuffer(commandBuffer);
    auto start = std::chrono::high_resolution_clock::now();
    for(uint32_t pass = 0; pass < a_numPasses; pass++) {
      vk_utils::executeCommandBufferNow(commandBuffer, computeQueue, device);
      AccumTimeStampMeasurements(0*2, 1*2);
    }
    vkFreeCommandBuffers(device, commandPool, 1, &commandBuffer);
    auto stop = std::chrono::high_resolution_clock::now();
    m_exTimeCastRaySingle.msExecuteOnGPU += std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count()/1000.f;
  }

  // (5) copy output data to CPU
  //
  auto beforeCopy2 = std::chrono::high_resolution_clock::now();
  pCopyHelper->ReadBuffer(out_colorGPU, 0, out_color, tidX*sizeof(uint32_t ));
  this->ReadPlainMembers(pCopyHelper);
  afterCopy2 = std::chrono::high_resolution_clock::now();
  m_exTimeCastRaySingle.msCopyFromGPU = std::chrono::duration_cast<std::chrono::microseconds>(afterCopy2 - beforeCopy2).count()/1000.f;

  // (6) free resources
  //
  vkDestroyBuffer(device, out_colorGPU, nullptr);
  if(buffersMem != VK_NULL_HANDLE)
    vkFreeMemory(device, buffersMem, nullptr);
  if(imagesMem != VK_NULL_HANDLE)
    vkFreeMemory(device, imagesMem, nullptr);

  m_exTimeCastRaySingle.msAPIOverhead += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - afterCopy2).count()/1000.f;
}

void MultiRenderer_gpu_rq::ResetTimeStampMeasurements()
{
  m_tsIdToKernelName.resize(m_timestampPoolSize/2);
  m_kernelTimings.clear();
}

void MultiRenderer_gpu_rq::AccumTimeStampMeasurements(uint32_t a_start, uint32_t a_size)
{
  std::vector<uint64_t> time_stamps(m_timestampPoolSize);
  vkGetQueryPoolResults(device, m_queryPoolTimestamps, 
                        a_start, a_size, 
                        a_size * sizeof(uint64_t), time_stamps.data() + a_start, 
                        sizeof(uint64_t), VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WAIT_BIT);
  
  for(size_t id=a_start; id < a_size; id++) 
  {
    float deltaInMs = float(time_stamps[id*2+1] - time_stamps[id*2+0]) * m_timestampPeriod / 1000000.0f;
    if(id >= m_tsIdToKernelName.size())
      break;
    const std::string& kernelName = m_tsIdToKernelName[id];
    auto p = m_kernelTimings.find(kernelName);
    if(p == m_kernelTimings.end())
      p = m_kernelTimings.insert(std::make_pair(kernelName, PerKernelMeasure{deltaInMs, deltaInMs, deltaInMs, 1})).first;
    else
    {
      p->second.avg += deltaInMs;
      p->second.min = std::min(p->second.min, deltaInMs);
      p->second.max = std::max(p->second.max, deltaInMs);
      p->second.count++;
    }

  }
}


void MultiRenderer_gpu_rq::GetExecutionTime(const char* a_funcName, float a_out[4])
{
  vk_utils::ExecTime res = {};
  if(std::string(a_funcName) == "PackXY" || std::string(a_funcName) == "PackXYBlock")
    res = m_exTimePackXY;
  if(std::string(a_funcName) == "CastRayFloatSingle" || std::string(a_funcName) == "CastRayFloatSingleBlock")
    res = m_exTimeCastRayFloatSingle;
  if(std::string(a_funcName) == "CastRaySingle" || std::string(a_funcName) == "CastRaySingleBlock")
    res = m_exTimeCastRaySingle;
  a_out[0] = res.msExecuteOnGPU;
  a_out[1] = res.msCopyToGPU;
  a_out[2] = res.msCopyFromGPU;
  a_out[3] = res.msAPIOverhead;
  auto p = m_kernelTimings.find(a_funcName);
  if(p != m_kernelTimings.end())
  {
    a_out[0] = p->second.avg / float(p->second.count);
    a_out[1] = p->second.min;
    a_out[2] = p->second.max;
    a_out[3] = 0.0f;
  }
}

