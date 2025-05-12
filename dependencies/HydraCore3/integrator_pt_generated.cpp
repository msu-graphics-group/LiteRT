#include <vector>
#include <memory>
#include <limits>
#include <cassert>
#include <chrono>
#include <array>

#include "vk_copy.h"
#include "vk_context.h"
#include "vk_images.h"

#include "integrator_pt_generated.h"
#include "include/Integrator_generated_ubo.h"

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


void Integrator_Generated::UpdatePlainMembers(std::shared_ptr<vk_utils::ICopyEngine> a_pCopyEngine)
{
  const size_t maxAllowedSize = std::numeric_limits<uint32_t>::max();
  auto pUnderlyingImpl = dynamic_cast<BVHRT*>(m_pAccelStruct->UnderlyingImpl(0));
  m_uboData.m_projInv = m_projInv;
  m_uboData.m_worldViewInv = m_worldViewInv;
  m_uboData.m_camRespoceRGB = m_camRespoceRGB;
  m_uboData.m_envColor = m_envColor;
  m_uboData.m_envSamRow0 = m_envSamRow0;
  m_uboData.m_envSamRow1 = m_envSamRow1;
  m_uboData.m_physSize = m_physSize;
  m_uboData.m_pAccelStruct_coctree_v3_header = pUnderlyingImpl->coctree_v3_header;
  m_uboData.m_pAccelStruct_m_preset = pUnderlyingImpl->m_preset;
  memcpy(m_uboData.m_camResponseSpectrumId,m_camResponseSpectrumId,sizeof(m_camResponseSpectrumId));
  m_uboData.m_camLensRadius = m_camLensRadius;
  m_uboData.m_camResponseType = m_camResponseType;
  m_uboData.m_camTargetDist = m_camTargetDist;
  m_uboData.m_disableImageContrib = m_disableImageContrib;
  m_uboData.m_enableOpticSim = m_enableOpticSim;
  m_uboData.m_envCamBackId = m_envCamBackId;
  m_uboData.m_envEnableSam = m_envEnableSam;
  m_uboData.m_envLightId = m_envLightId;
  m_uboData.m_envTexId = m_envTexId;
  m_uboData.m_exposureMult = m_exposureMult;
  m_uboData.m_fbHeight = m_fbHeight;
  m_uboData.m_fbWidth = m_fbWidth;
  m_uboData.m_intergatorType = m_intergatorType;
  m_uboData.m_maxThreadId = m_maxThreadId;
  m_uboData.m_renderLayer = m_renderLayer;
  m_uboData.m_spectral_mode = m_spectral_mode;
  m_uboData.m_tileSize = m_tileSize;
  m_uboData.m_traceDepth = m_traceDepth;
  m_uboData.m_winHeight = m_winHeight;
  m_uboData.m_winStartX = m_winStartX;
  m_uboData.m_winStartY = m_winStartY;
  m_uboData.m_winWidth = m_winWidth;
  m_uboData.all_references_size     = uint32_t( all_references.size() );     assert( all_references.size() < maxAllowedSize );
  m_uboData.all_references_capacity = uint32_t( all_references.capacity() ); assert( all_references.capacity() < maxAllowedSize );
  m_uboData.m_allRemapLists_size     = uint32_t( m_allRemapLists.size() );     assert( m_allRemapLists.size() < maxAllowedSize );
  m_uboData.m_allRemapLists_capacity = uint32_t( m_allRemapLists.capacity() ); assert( m_allRemapLists.capacity() < maxAllowedSize );
  m_uboData.m_allRemapListsOffsets_size     = uint32_t( m_allRemapListsOffsets.size() );     assert( m_allRemapListsOffsets.size() < maxAllowedSize );
  m_uboData.m_allRemapListsOffsets_capacity = uint32_t( m_allRemapListsOffsets.capacity() ); assert( m_allRemapListsOffsets.capacity() < maxAllowedSize );
  m_uboData.m_cie_x_size     = uint32_t( m_cie_x.size() );     assert( m_cie_x.size() < maxAllowedSize );
  m_uboData.m_cie_x_capacity = uint32_t( m_cie_x.capacity() ); assert( m_cie_x.capacity() < maxAllowedSize );
  m_uboData.m_cie_y_size     = uint32_t( m_cie_y.size() );     assert( m_cie_y.size() < maxAllowedSize );
  m_uboData.m_cie_y_capacity = uint32_t( m_cie_y.capacity() ); assert( m_cie_y.capacity() < maxAllowedSize );
  m_uboData.m_cie_z_size     = uint32_t( m_cie_z.size() );     assert( m_cie_z.size() < maxAllowedSize );
  m_uboData.m_cie_z_capacity = uint32_t( m_cie_z.capacity() ); assert( m_cie_z.capacity() < maxAllowedSize );
  m_uboData.m_films_eta_k_vec_size     = uint32_t( m_films_eta_k_vec.size() );     assert( m_films_eta_k_vec.size() < maxAllowedSize );
  m_uboData.m_films_eta_k_vec_capacity = uint32_t( m_films_eta_k_vec.capacity() ); assert( m_films_eta_k_vec.capacity() < maxAllowedSize );
  m_uboData.m_films_spec_id_vec_size     = uint32_t( m_films_spec_id_vec.size() );     assert( m_films_spec_id_vec.size() < maxAllowedSize );
  m_uboData.m_films_spec_id_vec_capacity = uint32_t( m_films_spec_id_vec.capacity() ); assert( m_films_spec_id_vec.capacity() < maxAllowedSize );
  m_uboData.m_instIdToLightInstId_size     = uint32_t( m_instIdToLightInstId.size() );     assert( m_instIdToLightInstId.size() < maxAllowedSize );
  m_uboData.m_instIdToLightInstId_capacity = uint32_t( m_instIdToLightInstId.capacity() ); assert( m_instIdToLightInstId.capacity() < maxAllowedSize );
  m_uboData.m_lights_size     = uint32_t( m_lights.size() );     assert( m_lights.size() < maxAllowedSize );
  m_uboData.m_lights_capacity = uint32_t( m_lights.capacity() ); assert( m_lights.capacity() < maxAllowedSize );
  m_uboData.m_lines_size     = uint32_t( m_lines.size() );     assert( m_lines.size() < maxAllowedSize );
  m_uboData.m_lines_capacity = uint32_t( m_lines.capacity() ); assert( m_lines.capacity() < maxAllowedSize );
  m_uboData.m_matIdByPrimId_size     = uint32_t( m_matIdByPrimId.size() );     assert( m_matIdByPrimId.size() < maxAllowedSize );
  m_uboData.m_matIdByPrimId_capacity = uint32_t( m_matIdByPrimId.capacity() ); assert( m_matIdByPrimId.capacity() < maxAllowedSize );
  m_uboData.m_matIdOffsets_size     = uint32_t( m_matIdOffsets.size() );     assert( m_matIdOffsets.size() < maxAllowedSize );
  m_uboData.m_matIdOffsets_capacity = uint32_t( m_matIdOffsets.capacity() ); assert( m_matIdOffsets.capacity() < maxAllowedSize );
  m_uboData.m_materials_size     = uint32_t( m_materials.size() );     assert( m_materials.size() < maxAllowedSize );
  m_uboData.m_materials_capacity = uint32_t( m_materials.capacity() ); assert( m_materials.capacity() < maxAllowedSize );
  m_uboData.m_normMatrices_size     = uint32_t( m_normMatrices.size() );     assert( m_normMatrices.size() < maxAllowedSize );
  m_uboData.m_normMatrices_capacity = uint32_t( m_normMatrices.capacity() ); assert( m_normMatrices.capacity() < maxAllowedSize );
  m_uboData.m_normMatrices2_size     = uint32_t( m_normMatrices2.size() );     assert( m_normMatrices2.size() < maxAllowedSize );
  m_uboData.m_normMatrices2_capacity = uint32_t( m_normMatrices2.capacity() ); assert( m_normMatrices2.capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_CatmulClarkHeaders_size     = uint32_t( m_pAccelStruct_m_CatmulClarkHeaders->size() );     assert( m_pAccelStruct_m_CatmulClarkHeaders->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_CatmulClarkHeaders_capacity = uint32_t( m_pAccelStruct_m_CatmulClarkHeaders->capacity() ); assert( m_pAccelStruct_m_CatmulClarkHeaders->capacity() < maxAllowedSize );
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
  m_uboData.m_pdfLightData_size     = uint32_t( m_pdfLightData.size() );     assert( m_pdfLightData.size() < maxAllowedSize );
  m_uboData.m_pdfLightData_capacity = uint32_t( m_pdfLightData.capacity() ); assert( m_pdfLightData.capacity() < maxAllowedSize );
  m_uboData.m_precomp_coat_transmittance_size     = uint32_t( m_precomp_coat_transmittance.size() );     assert( m_precomp_coat_transmittance.size() < maxAllowedSize );
  m_uboData.m_precomp_coat_transmittance_capacity = uint32_t( m_precomp_coat_transmittance.capacity() ); assert( m_precomp_coat_transmittance.capacity() < maxAllowedSize );
  m_uboData.m_precomp_thin_films_size     = uint32_t( m_precomp_thin_films.size() );     assert( m_precomp_thin_films.size() < maxAllowedSize );
  m_uboData.m_precomp_thin_films_capacity = uint32_t( m_precomp_thin_films.capacity() ); assert( m_precomp_thin_films.capacity() < maxAllowedSize );
  m_uboData.m_randomGens_size     = uint32_t( m_randomGens.size() );     assert( m_randomGens.size() < maxAllowedSize );
  m_uboData.m_randomGens_capacity = uint32_t( m_randomGens.capacity() ); assert( m_randomGens.capacity() < maxAllowedSize );
  m_uboData.m_remapInst_size     = uint32_t( m_remapInst.size() );     assert( m_remapInst.size() < maxAllowedSize );
  m_uboData.m_remapInst_capacity = uint32_t( m_remapInst.capacity() ); assert( m_remapInst.capacity() < maxAllowedSize );
  m_uboData.m_spec_offset_sz_size     = uint32_t( m_spec_offset_sz.size() );     assert( m_spec_offset_sz.size() < maxAllowedSize );
  m_uboData.m_spec_offset_sz_capacity = uint32_t( m_spec_offset_sz.capacity() ); assert( m_spec_offset_sz.capacity() < maxAllowedSize );
  m_uboData.m_spec_tex_ids_wavelengths_size     = uint32_t( m_spec_tex_ids_wavelengths.size() );     assert( m_spec_tex_ids_wavelengths.size() < maxAllowedSize );
  m_uboData.m_spec_tex_ids_wavelengths_capacity = uint32_t( m_spec_tex_ids_wavelengths.capacity() ); assert( m_spec_tex_ids_wavelengths.capacity() < maxAllowedSize );
  m_uboData.m_spec_tex_offset_sz_size     = uint32_t( m_spec_tex_offset_sz.size() );     assert( m_spec_tex_offset_sz.size() < maxAllowedSize );
  m_uboData.m_spec_tex_offset_sz_capacity = uint32_t( m_spec_tex_offset_sz.capacity() ); assert( m_spec_tex_offset_sz.capacity() < maxAllowedSize );
  m_uboData.m_spec_values_size     = uint32_t( m_spec_values.size() );     assert( m_spec_values.size() < maxAllowedSize );
  m_uboData.m_spec_values_capacity = uint32_t( m_spec_values.capacity() ); assert( m_spec_values.capacity() < maxAllowedSize );
  m_uboData.m_triIndices_size     = uint32_t( m_triIndices.size() );     assert( m_triIndices.size() < maxAllowedSize );
  m_uboData.m_triIndices_capacity = uint32_t( m_triIndices.capacity() ); assert( m_triIndices.capacity() < maxAllowedSize );
  m_uboData.m_vNorm4f_size     = uint32_t( m_vNorm4f.size() );     assert( m_vNorm4f.size() < maxAllowedSize );
  m_uboData.m_vNorm4f_capacity = uint32_t( m_vNorm4f.capacity() ); assert( m_vNorm4f.capacity() < maxAllowedSize );
  m_uboData.m_vTang4f_size     = uint32_t( m_vTang4f.size() );     assert( m_vTang4f.size() < maxAllowedSize );
  m_uboData.m_vTang4f_capacity = uint32_t( m_vTang4f.capacity() ); assert( m_vTang4f.capacity() < maxAllowedSize );
  m_uboData.m_vertOffset_size     = uint32_t( m_vertOffset.size() );     assert( m_vertOffset.size() < maxAllowedSize );
  m_uboData.m_vertOffset_capacity = uint32_t( m_vertOffset.capacity() ); assert( m_vertOffset.capacity() < maxAllowedSize );
  a_pCopyEngine->UpdateBuffer(m_classDataBuffer, 0, &m_uboData, sizeof(m_uboData));
}

void Integrator_Generated::ReadPlainMembers(std::shared_ptr<vk_utils::ICopyEngine> a_pCopyEngine)
{
  a_pCopyEngine->ReadBuffer(m_classDataBuffer, 0, &m_uboData, sizeof(m_uboData));
  auto pUnderlyingImpl = dynamic_cast<BVHRT*>(m_pAccelStruct->UnderlyingImpl(0));
  m_projInv = m_uboData.m_projInv;
  m_worldViewInv = m_uboData.m_worldViewInv;
  m_camRespoceRGB = m_uboData.m_camRespoceRGB;
  m_envColor = m_uboData.m_envColor;
  m_envSamRow0 = m_uboData.m_envSamRow0;
  m_envSamRow1 = m_uboData.m_envSamRow1;
  m_physSize = m_uboData.m_physSize;
  pUnderlyingImpl->coctree_v3_header = m_uboData.m_pAccelStruct_coctree_v3_header;
  pUnderlyingImpl->m_preset = m_uboData.m_pAccelStruct_m_preset;
  memcpy(m_camResponseSpectrumId, m_uboData.m_camResponseSpectrumId, sizeof(m_camResponseSpectrumId));
  m_camLensRadius = m_uboData.m_camLensRadius;
  m_camResponseType = m_uboData.m_camResponseType;
  m_camTargetDist = m_uboData.m_camTargetDist;
  m_disableImageContrib = m_uboData.m_disableImageContrib;
  m_enableOpticSim = m_uboData.m_enableOpticSim;
  m_envCamBackId = m_uboData.m_envCamBackId;
  m_envEnableSam = m_uboData.m_envEnableSam;
  m_envLightId = m_uboData.m_envLightId;
  m_envTexId = m_uboData.m_envTexId;
  m_exposureMult = m_uboData.m_exposureMult;
  m_fbHeight = m_uboData.m_fbHeight;
  m_fbWidth = m_uboData.m_fbWidth;
  m_intergatorType = m_uboData.m_intergatorType;
  m_maxThreadId = m_uboData.m_maxThreadId;
  m_renderLayer = m_uboData.m_renderLayer;
  m_spectral_mode = m_uboData.m_spectral_mode;
  m_tileSize = m_uboData.m_tileSize;
  m_traceDepth = m_uboData.m_traceDepth;
  m_winHeight = m_uboData.m_winHeight;
  m_winStartX = m_uboData.m_winStartX;
  m_winStartY = m_uboData.m_winStartY;
  m_winWidth = m_uboData.m_winWidth;
  all_references.resize(m_uboData.all_references_size);
  m_allRemapLists.resize(m_uboData.m_allRemapLists_size);
  m_allRemapListsOffsets.resize(m_uboData.m_allRemapListsOffsets_size);
  m_cie_x.resize(m_uboData.m_cie_x_size);
  m_cie_y.resize(m_uboData.m_cie_y_size);
  m_cie_z.resize(m_uboData.m_cie_z_size);
  m_films_eta_k_vec.resize(m_uboData.m_films_eta_k_vec_size);
  m_films_spec_id_vec.resize(m_uboData.m_films_spec_id_vec_size);
  m_instIdToLightInstId.resize(m_uboData.m_instIdToLightInstId_size);
  m_lights.resize(m_uboData.m_lights_size);
  m_lines.resize(m_uboData.m_lines_size);
  m_matIdByPrimId.resize(m_uboData.m_matIdByPrimId_size);
  m_matIdOffsets.resize(m_uboData.m_matIdOffsets_size);
  m_materials.resize(m_uboData.m_materials_size);
  m_normMatrices.resize(m_uboData.m_normMatrices_size);
  m_normMatrices2.resize(m_uboData.m_normMatrices2_size);
  m_pAccelStruct_m_CatmulClarkHeaders->resize(m_uboData.m_pAccelStruct_m_CatmulClarkHeaders_size);
  m_pAccelStruct_m_NURBSData->resize(m_uboData.m_pAccelStruct_m_NURBSData_size);
  m_pAccelStruct_m_NURBSHeaders->resize(m_uboData.m_pAccelStruct_m_NURBSHeaders_size);
  m_pAccelStruct_m_NURBS_approxes->resize(m_uboData.m_pAccelStruct_m_NURBS_approxes_size);
  m_pAccelStruct_m_RibbonHeaders->resize(m_uboData.m_pAccelStruct_m_RibbonHeaders_size);
  m_pAccelStruct_m_SdfCompactOctreeRotModifiers->resize(m_uboData.m_pAccelStruct_m_SdfCompactOctreeRotModifiers_size);
  m_pAccelStruct_m_SdfCompactOctreeV2Data->resize(m_uboData.m_pAccelStruct_m_SdfCompactOctreeV2Data_size);
  m_pAccelStruct_m_SdfCompactOctreeV3Data->resize(m_uboData.m_pAccelStruct_m_SdfCompactOctreeV3Data_size);
  m_pAccelStruct_m_SdfFrameOctreeNodes->resize(m_uboData.m_pAccelStruct_m_SdfFrameOctreeNodes_size);
  m_pAccelStruct_m_SdfFrameOctreeRoots->resize(m_uboData.m_pAccelStruct_m_SdfFrameOctreeRoots_size);
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
  m_pdfLightData.resize(m_uboData.m_pdfLightData_size);
  m_precomp_coat_transmittance.resize(m_uboData.m_precomp_coat_transmittance_size);
  m_precomp_thin_films.resize(m_uboData.m_precomp_thin_films_size);
  m_randomGens.resize(m_uboData.m_randomGens_size);
  m_remapInst.resize(m_uboData.m_remapInst_size);
  m_spec_offset_sz.resize(m_uboData.m_spec_offset_sz_size);
  m_spec_tex_ids_wavelengths.resize(m_uboData.m_spec_tex_ids_wavelengths_size);
  m_spec_tex_offset_sz.resize(m_uboData.m_spec_tex_offset_sz_size);
  m_spec_values.resize(m_uboData.m_spec_values_size);
  m_triIndices.resize(m_uboData.m_triIndices_size);
  m_vNorm4f.resize(m_uboData.m_vNorm4f_size);
  m_vTang4f.resize(m_uboData.m_vTang4f_size);
  m_vertOffset.resize(m_uboData.m_vertOffset_size);
}

void Integrator_Generated::UpdateVectorMembers(std::shared_ptr<vk_utils::ICopyEngine> a_pCopyEngine)
{
  if(all_references.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.all_referencesBuffer, 0, all_references.data(), all_references.size()*sizeof(AllBufferReferences) );
  if(m_allRemapLists.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_allRemapListsBuffer, 0, m_allRemapLists.data(), m_allRemapLists.size()*sizeof(int) );
  if(m_allRemapListsOffsets.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_allRemapListsOffsetsBuffer, 0, m_allRemapListsOffsets.data(), m_allRemapListsOffsets.size()*sizeof(int) );
  if(m_cie_x.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_cie_xBuffer, 0, m_cie_x.data(), m_cie_x.size()*sizeof(float) );
  if(m_cie_y.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_cie_yBuffer, 0, m_cie_y.data(), m_cie_y.size()*sizeof(float) );
  if(m_cie_z.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_cie_zBuffer, 0, m_cie_z.data(), m_cie_z.size()*sizeof(float) );
  if(m_films_eta_k_vec.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_films_eta_k_vecBuffer, 0, m_films_eta_k_vec.data(), m_films_eta_k_vec.size()*sizeof(float) );
  if(m_films_spec_id_vec.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_films_spec_id_vecBuffer, 0, m_films_spec_id_vec.data(), m_films_spec_id_vec.size()*sizeof(unsigned int) );
  if(m_instIdToLightInstId.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_instIdToLightInstIdBuffer, 0, m_instIdToLightInstId.data(), m_instIdToLightInstId.size()*sizeof(unsigned int) );
  if(m_lights.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_lightsBuffer, 0, m_lights.data(), m_lights.size()*sizeof(struct LightSource) );
  if(m_lines.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_linesBuffer, 0, m_lines.data(), m_lines.size()*sizeof(struct Integrator::LensElementInterface) );
  if(m_matIdByPrimId.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_matIdByPrimIdBuffer, 0, m_matIdByPrimId.data(), m_matIdByPrimId.size()*sizeof(unsigned int) );
  if(m_matIdOffsets.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_matIdOffsetsBuffer, 0, m_matIdOffsets.data(), m_matIdOffsets.size()*sizeof(unsigned int) );
  if(m_materials.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_materialsBuffer, 0, m_materials.data(), m_materials.size()*sizeof(struct Material) );
  if(m_normMatrices.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_normMatricesBuffer, 0, m_normMatrices.data(), m_normMatrices.size()*sizeof(struct LiteMath::float4x4) );
  if(m_normMatrices2.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_normMatrices2Buffer, 0, m_normMatrices2.data(), m_normMatrices2.size()*sizeof(struct LiteMath::float4x4) );
  if(m_pAccelStruct_m_CatmulClarkHeaders->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_CatmulClarkHeadersBuffer, 0, m_pAccelStruct_m_CatmulClarkHeaders->data(), m_pAccelStruct_m_CatmulClarkHeaders->size()*sizeof(struct CatmulClarkHeader) );
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
  if(m_pdfLightData.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pdfLightDataBuffer, 0, m_pdfLightData.data(), m_pdfLightData.size()*sizeof(float) );
  if(m_precomp_coat_transmittance.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_precomp_coat_transmittanceBuffer, 0, m_precomp_coat_transmittance.data(), m_precomp_coat_transmittance.size()*sizeof(float) );
  if(m_precomp_thin_films.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_precomp_thin_filmsBuffer, 0, m_precomp_thin_films.data(), m_precomp_thin_films.size()*sizeof(float) );
  if(m_randomGens.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_randomGensBuffer, 0, m_randomGens.data(), m_randomGens.size()*sizeof(struct RandomGenT) );
  if(m_remapInst.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_remapInstBuffer, 0, m_remapInst.data(), m_remapInst.size()*sizeof(int) );
  if(m_spec_offset_sz.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_spec_offset_szBuffer, 0, m_spec_offset_sz.data(), m_spec_offset_sz.size()*sizeof(struct LiteMath::uint2) );
  if(m_spec_tex_ids_wavelengths.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_spec_tex_ids_wavelengthsBuffer, 0, m_spec_tex_ids_wavelengths.data(), m_spec_tex_ids_wavelengths.size()*sizeof(struct LiteMath::uint2) );
  if(m_spec_tex_offset_sz.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_spec_tex_offset_szBuffer, 0, m_spec_tex_offset_sz.data(), m_spec_tex_offset_sz.size()*sizeof(struct LiteMath::uint2) );
  if(m_spec_values.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_spec_valuesBuffer, 0, m_spec_values.data(), m_spec_values.size()*sizeof(float) );
  if(m_triIndices.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_triIndicesBuffer, 0, m_triIndices.data(), m_triIndices.size()*sizeof(unsigned int) );
  if(m_vNorm4f.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_vNorm4fBuffer, 0, m_vNorm4f.data(), m_vNorm4f.size()*sizeof(struct LiteMath::float4) );
  if(m_vTang4f.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_vTang4fBuffer, 0, m_vTang4f.data(), m_vTang4f.size()*sizeof(struct LiteMath::float4) );
  if(m_vertOffset.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_vertOffsetBuffer, 0, m_vertOffset.data(), m_vertOffset.size()*sizeof(unsigned int) );
}

void Integrator_Generated::Update_m_lights(size_t a_first, size_t a_size)
{
  if(m_lights.size() != 0 && m_pLastCopyHelper != nullptr)
    m_pLastCopyHelper->UpdateBuffer(m_vdata.m_lightsBuffer, a_first*sizeof(struct LightSource), m_lights.data() + a_first, a_size*sizeof(struct LightSource) );
}
void Integrator_Generated::Update_m_matIdOffsets()
{
  if(m_matIdOffsets.size() != 0 && m_pLastCopyHelper != nullptr)
    m_pLastCopyHelper->UpdateBuffer(m_vdata.m_matIdOffsetsBuffer, 0, m_matIdOffsets.data(), m_matIdOffsets.size()*sizeof(unsigned int) );
}
void Integrator_Generated::Update_m_materials(size_t a_first, size_t a_size)
{
  if(m_materials.size() != 0 && m_pLastCopyHelper != nullptr)
    m_pLastCopyHelper->UpdateBuffer(m_vdata.m_materialsBuffer, a_first*sizeof(struct Material), m_materials.data() + a_first, a_size*sizeof(struct Material) );
}

void Integrator_Generated::UpdateTextureMembers(std::shared_ptr<vk_utils::ICopyEngine> a_pCopyEngine)
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
    throw std::runtime_error("Integrator_Generated::UpdateTextureMembers: failed to begin command buffer!");
  vkCmdPipelineBarrier(cmdBuff,VK_PIPELINE_STAGE_TRANSFER_BIT,VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,0,0,nullptr,0,nullptr,uint32_t(barriers.size()),barriers.data());
  vkEndCommandBuffer(cmdBuff);

  vk_utils::executeCommandBufferNow(cmdBuff, transferQueue, device);
}

void Integrator_Generated::RayTraceMegaCmd(uint tid, uint channels, float* out_color)
{
  uint32_t blockSizeX = 256;
  uint32_t blockSizeY = 1;
  uint32_t blockSizeZ = 1;

  struct KernelArgsPC
  {
    uint m_channels;
    uint32_t m_sizeX;
    uint32_t m_sizeY;
    uint32_t m_sizeZ;
    uint32_t m_tFlags;
  } pcData;

  uint32_t sizeX  = uint32_t(tid);
  uint32_t sizeY  = uint32_t(1);
  uint32_t sizeZ  = uint32_t(1);

  pcData.m_sizeX  = tid;
  pcData.m_sizeY  = 1;
  pcData.m_sizeZ  = 1;
  pcData.m_tFlags = m_currThreadFlags;
  pcData.m_channels = channels;
  vkCmdPushConstants(m_currCmdBuffer, RayTraceMegaLayout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(KernelArgsPC), &pcData);
  
  vkCmdBindPipeline(m_currCmdBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, RayTraceMegaPipeline);
  vkCmdDispatch    (m_currCmdBuffer, (sizeX + blockSizeX - 1) / blockSizeX, (sizeY + blockSizeY - 1) / blockSizeY, (sizeZ + blockSizeZ - 1) / blockSizeZ);
}

void Integrator_Generated::CastSingleRayMegaCmd(uint tid, float* out_color)
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

  uint32_t sizeX  = uint32_t(tid);
  uint32_t sizeY  = uint32_t(1);
  uint32_t sizeZ  = uint32_t(1);

  pcData.m_sizeX  = tid;
  pcData.m_sizeY  = 1;
  pcData.m_sizeZ  = 1;
  pcData.m_tFlags = m_currThreadFlags;
  vkCmdPushConstants(m_currCmdBuffer, CastSingleRayMegaLayout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(KernelArgsPC), &pcData);
  
  vkCmdBindPipeline(m_currCmdBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, CastSingleRayMegaPipeline);
  vkCmdDispatch    (m_currCmdBuffer, (sizeX + blockSizeX - 1) / blockSizeX, (sizeY + blockSizeY - 1) / blockSizeY, (sizeZ + blockSizeZ - 1) / blockSizeZ);
}

void Integrator_Generated::PackXYMegaCmd(uint tidX, uint tidY)
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

void Integrator_Generated::PathTraceFromInputRaysMegaCmd(uint tid, uint channels, const RayPosAndW* in_rayPosAndNear, const RayDirAndT* in_rayDirAndFar, float* out_color)
{
  uint32_t blockSizeX = 256;
  uint32_t blockSizeY = 1;
  uint32_t blockSizeZ = 1;

  struct KernelArgsPC
  {
    uint m_channels;
    uint32_t m_sizeX;
    uint32_t m_sizeY;
    uint32_t m_sizeZ;
    uint32_t m_tFlags;
  } pcData;

  uint32_t sizeX  = uint32_t(tid);
  uint32_t sizeY  = uint32_t(1);
  uint32_t sizeZ  = uint32_t(1);

  pcData.m_sizeX  = tid;
  pcData.m_sizeY  = 1;
  pcData.m_sizeZ  = 1;
  pcData.m_tFlags = m_currThreadFlags;
  pcData.m_channels = channels;
  vkCmdPushConstants(m_currCmdBuffer, PathTraceFromInputRaysMegaLayout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(KernelArgsPC), &pcData);
  
  vkCmdBindPipeline(m_currCmdBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, PathTraceFromInputRaysMegaPipeline);
  vkCmdDispatch    (m_currCmdBuffer, (sizeX + blockSizeX - 1) / blockSizeX, (sizeY + blockSizeY - 1) / blockSizeY, (sizeZ + blockSizeZ - 1) / blockSizeZ);
}

void Integrator_Generated::PathTraceMegaCmd(uint tid, uint channels, float* out_color)
{
  uint32_t blockSizeX = 256;
  uint32_t blockSizeY = 1;
  uint32_t blockSizeZ = 1;

  struct KernelArgsPC
  {
    uint m_channels;
    uint32_t m_sizeX;
    uint32_t m_sizeY;
    uint32_t m_sizeZ;
    uint32_t m_tFlags;
  } pcData;

  uint32_t sizeX  = uint32_t(tid);
  uint32_t sizeY  = uint32_t(1);
  uint32_t sizeZ  = uint32_t(1);

  pcData.m_sizeX  = tid;
  pcData.m_sizeY  = 1;
  pcData.m_sizeZ  = 1;
  pcData.m_tFlags = m_currThreadFlags;
  pcData.m_channels = channels;
  vkCmdPushConstants(m_currCmdBuffer, PathTraceMegaLayout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(KernelArgsPC), &pcData);
  
  vkCmdBindPipeline(m_currCmdBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, PathTraceMegaPipeline);
  vkCmdDispatch    (m_currCmdBuffer, (sizeX + blockSizeX - 1) / blockSizeX, (sizeY + blockSizeY - 1) / blockSizeY, (sizeZ + blockSizeZ - 1) / blockSizeZ);
}

void Integrator_Generated::NaivePathTraceMegaCmd(uint tid, uint channels, float* out_color)
{
  uint32_t blockSizeX = 256;
  uint32_t blockSizeY = 1;
  uint32_t blockSizeZ = 1;

  struct KernelArgsPC
  {
    uint m_channels;
    uint32_t m_sizeX;
    uint32_t m_sizeY;
    uint32_t m_sizeZ;
    uint32_t m_tFlags;
  } pcData;

  uint32_t sizeX  = uint32_t(tid);
  uint32_t sizeY  = uint32_t(1);
  uint32_t sizeZ  = uint32_t(1);

  pcData.m_sizeX  = tid;
  pcData.m_sizeY  = 1;
  pcData.m_sizeZ  = 1;
  pcData.m_tFlags = m_currThreadFlags;
  pcData.m_channels = channels;
  vkCmdPushConstants(m_currCmdBuffer, NaivePathTraceMegaLayout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(KernelArgsPC), &pcData);
  
  vkCmdBindPipeline(m_currCmdBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, NaivePathTraceMegaPipeline);
  vkCmdDispatch    (m_currCmdBuffer, (sizeX + blockSizeX - 1) / blockSizeX, (sizeY + blockSizeY - 1) / blockSizeY, (sizeZ + blockSizeZ - 1) / blockSizeZ);
}


void Integrator_Generated::copyKernelFloatCmd(uint32_t length)
{
  uint32_t blockSizeX = MEMCPY_BLOCK_SIZE;

  vkCmdBindPipeline(m_currCmdBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, copyKernelFloatPipeline);
  vkCmdPushConstants(m_currCmdBuffer, copyKernelFloatLayout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(uint32_t), &length);
  vkCmdDispatch(m_currCmdBuffer, (length + blockSizeX - 1) / blockSizeX, 1, 1);
}

void Integrator_Generated::matMulTransposeCmd(uint32_t A_offset, uint32_t B_offset, uint32_t C_offset, uint32_t A_col_len, uint32_t B_col_len, uint32_t A_row_len)
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

VkBufferMemoryBarrier Integrator_Generated::BarrierForClearFlags(VkBuffer a_buffer)
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

VkBufferMemoryBarrier Integrator_Generated::BarrierForSingleBuffer(VkBuffer a_buffer)
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

void Integrator_Generated::BarriersForSeveralBuffers(VkBuffer* a_inBuffers, VkBufferMemoryBarrier* a_outBarriers, uint32_t a_buffersNum)
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

void Integrator_Generated::RayTraceCmd(VkCommandBuffer a_commandBuffer, uint tid, uint channels, float* out_color)
{
  VkPipelineStageFlagBits prevStageBits = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
  m_currCmdBuffer = a_commandBuffer;
  VkMemoryBarrier memoryBarrier = { VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr, VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT };
  vkCmdBindDescriptorSets(a_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, RayTraceMegaLayout, 0, 1, &m_allGeneratedDS[0], 0, nullptr);
  RayTraceMegaCmd(tid, channels, out_color);
  vkCmdPipelineBarrier(m_currCmdBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1, &memoryBarrier, 0, nullptr, 0, nullptr);
}

void Integrator_Generated::CastSingleRayCmd(VkCommandBuffer a_commandBuffer, uint tid, float* out_color)
{
  VkPipelineStageFlagBits prevStageBits = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
  m_currCmdBuffer = a_commandBuffer;
  VkMemoryBarrier memoryBarrier = { VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr, VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT };
  vkCmdBindDescriptorSets(a_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, CastSingleRayMegaLayout, 0, 1, &m_allGeneratedDS[1], 0, nullptr);
  CastSingleRayMegaCmd(tid, out_color);
  vkCmdPipelineBarrier(m_currCmdBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1, &memoryBarrier, 0, nullptr, 0, nullptr);
}

void Integrator_Generated::PackXYCmd(VkCommandBuffer a_commandBuffer, uint tidX, uint tidY)
{
  VkPipelineStageFlagBits prevStageBits = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
  m_currCmdBuffer = a_commandBuffer;
  VkMemoryBarrier memoryBarrier = { VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr, VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT };
  vkCmdBindDescriptorSets(a_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, PackXYMegaLayout, 0, 1, &m_allGeneratedDS[2], 0, nullptr);
  PackXYMegaCmd(tidX, tidY);
  vkCmdPipelineBarrier(m_currCmdBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1, &memoryBarrier, 0, nullptr, 0, nullptr);
}

void Integrator_Generated::PathTraceFromInputRaysCmd(VkCommandBuffer a_commandBuffer, uint tid, uint channels, const RayPosAndW* in_rayPosAndNear, const RayDirAndT* in_rayDirAndFar, float* out_color)
{
  VkPipelineStageFlagBits prevStageBits = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
  m_currCmdBuffer = a_commandBuffer;
  VkMemoryBarrier memoryBarrier = { VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr, VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT };
  vkCmdBindDescriptorSets(a_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, PathTraceFromInputRaysMegaLayout, 0, 1, &m_allGeneratedDS[3], 0, nullptr);
  PathTraceFromInputRaysMegaCmd(tid, channels, in_rayPosAndNear, in_rayDirAndFar, out_color);
  vkCmdPipelineBarrier(m_currCmdBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1, &memoryBarrier, 0, nullptr, 0, nullptr);
}

void Integrator_Generated::PathTraceCmd(VkCommandBuffer a_commandBuffer, uint tid, uint channels, float* out_color)
{
  VkPipelineStageFlagBits prevStageBits = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
  m_currCmdBuffer = a_commandBuffer;
  VkMemoryBarrier memoryBarrier = { VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr, VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT };
  vkCmdBindDescriptorSets(a_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, PathTraceMegaLayout, 0, 1, &m_allGeneratedDS[4], 0, nullptr);
  PathTraceMegaCmd(tid, channels, out_color);
  vkCmdPipelineBarrier(m_currCmdBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1, &memoryBarrier, 0, nullptr, 0, nullptr);
}

void Integrator_Generated::NaivePathTraceCmd(VkCommandBuffer a_commandBuffer, uint tid, uint channels, float* out_color)
{
  VkPipelineStageFlagBits prevStageBits = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
  m_currCmdBuffer = a_commandBuffer;
  VkMemoryBarrier memoryBarrier = { VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr, VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT };
  vkCmdBindDescriptorSets(a_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, NaivePathTraceMegaLayout, 0, 1, &m_allGeneratedDS[5], 0, nullptr);
  NaivePathTraceMegaCmd(tid, channels, out_color);
  vkCmdPipelineBarrier(m_currCmdBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1, &memoryBarrier, 0, nullptr, 0, nullptr);
}



void Integrator_Generated::RayTraceBlock(uint tid, uint channels, float* out_color, uint32_t a_numPasses)
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
  if(RayTrace_local.needToClearOutput)
    outFlags |= VK_IMAGE_USAGE_TRANSFER_DST_BIT;
  std::vector<VkBuffer> buffers;
  std::vector<VkImage>  images2;
  std::vector<vk_utils::VulkanImageMem*> images;
  auto beforeCreateObjects = std::chrono::high_resolution_clock::now();
  VkBuffer out_colorGPU = vk_utils::createBuffer(device, tid*channels*sizeof(float ), outFlags);
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
  m_exTimeRayTrace.msAPIOverhead = std::chrono::duration_cast<std::chrono::microseconds>(afterCreateObjects - beforeCreateObjects).count()/1000.f;

  auto afterCopy2 = std::chrono::high_resolution_clock::now(); // just declare it here, replace value later

  auto afterInitBuffers = std::chrono::high_resolution_clock::now();
  m_exTimeRayTrace.msAPIOverhead += std::chrono::duration_cast<std::chrono::microseconds>(afterInitBuffers - afterCreateObjects).count()/1000.f;

  auto beforeSetInOut = std::chrono::high_resolution_clock::now();
  this->SetVulkanInOutFor_RayTrace(out_colorGPU, 0);

  // (3) copy input data to GPU
  //
  auto beforeCopy = std::chrono::high_resolution_clock::now();
  m_exTimeRayTrace.msAPIOverhead += std::chrono::duration_cast<std::chrono::microseconds>(beforeCopy - beforeSetInOut).count()/1000.f;
  auto afterCopy = std::chrono::high_resolution_clock::now();
  m_exTimeRayTrace.msCopyToGPU = std::chrono::duration_cast<std::chrono::microseconds>(afterCopy - beforeCopy).count()/1000.f;
  //
  m_exTimeRayTrace.msExecuteOnGPU = 0;
  //// (3.1) clear all outputs if we are in RTV pattern
  //
  if(RayTrace_local.needToClearOutput)
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
    m_exTimeRayTrace.msExecuteOnGPU  += std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count()/1000.f;
  }

  // (4) now execute algorithm on GPU
  //
  {
    VkCommandBuffer commandBuffer = vk_utils::createCommandBuffer(device, commandPool);
    VkCommandBufferBeginInfo beginCommandBufferInfo = {};
    beginCommandBufferInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginCommandBufferInfo.flags = VK_COMMAND_BUFFER_USAGE_SIMULTANEOUS_USE_BIT;
    vkBeginCommandBuffer(commandBuffer, &beginCommandBufferInfo);
    RayTraceCmd(commandBuffer, tid, channels, out_color);
    vkEndCommandBuffer(commandBuffer);
    auto start = std::chrono::high_resolution_clock::now();
    if(a_numPasses > 1)
      ProgressBarStart();
    for(uint32_t pass = 0; pass < a_numPasses; pass++) {
      vk_utils::executeCommandBufferNow(commandBuffer, computeQueue, device);
      if((pass != 0) && (pass % 256 == 0))
        ProgressBarAccum(256.0f/float(a_numPasses));
    }
    if(a_numPasses > 1)
      ProgressBarDone();
    vkFreeCommandBuffers(device, commandPool, 1, &commandBuffer);
    auto stop = std::chrono::high_resolution_clock::now();
    m_exTimeRayTrace.msExecuteOnGPU += std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count()/1000.f;
  }

  // (5) copy output data to CPU
  //
  auto beforeCopy2 = std::chrono::high_resolution_clock::now();
  pCopyHelper->ReadBuffer(out_colorGPU, 0, out_color, tid*channels*sizeof(float ));
  this->ReadPlainMembers(pCopyHelper);
  afterCopy2 = std::chrono::high_resolution_clock::now();
  m_exTimeRayTrace.msCopyFromGPU = std::chrono::duration_cast<std::chrono::microseconds>(afterCopy2 - beforeCopy2).count()/1000.f;

  // (6) free resources
  //
  vkDestroyBuffer(device, out_colorGPU, nullptr);
  if(buffersMem != VK_NULL_HANDLE)
    vkFreeMemory(device, buffersMem, nullptr);
  if(imagesMem != VK_NULL_HANDLE)
    vkFreeMemory(device, imagesMem, nullptr);

  m_exTimeRayTrace.msAPIOverhead += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - afterCopy2).count()/1000.f;
}

void Integrator_Generated::CastSingleRayBlock(uint tid, float* out_color, uint32_t a_numPasses)
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
  if(CastSingleRay_local.needToClearOutput)
    outFlags |= VK_IMAGE_USAGE_TRANSFER_DST_BIT;
  std::vector<VkBuffer> buffers;
  std::vector<VkImage>  images2;
  std::vector<vk_utils::VulkanImageMem*> images;
  auto beforeCreateObjects = std::chrono::high_resolution_clock::now();
  VkBuffer out_colorGPU = vk_utils::createBuffer(device, tid*4*sizeof(float ), outFlags);
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
  m_exTimeCastSingleRay.msAPIOverhead = std::chrono::duration_cast<std::chrono::microseconds>(afterCreateObjects - beforeCreateObjects).count()/1000.f;

  auto afterCopy2 = std::chrono::high_resolution_clock::now(); // just declare it here, replace value later

  auto afterInitBuffers = std::chrono::high_resolution_clock::now();
  m_exTimeCastSingleRay.msAPIOverhead += std::chrono::duration_cast<std::chrono::microseconds>(afterInitBuffers - afterCreateObjects).count()/1000.f;

  auto beforeSetInOut = std::chrono::high_resolution_clock::now();
  this->SetVulkanInOutFor_CastSingleRay(out_colorGPU, 0);

  // (3) copy input data to GPU
  //
  auto beforeCopy = std::chrono::high_resolution_clock::now();
  m_exTimeCastSingleRay.msAPIOverhead += std::chrono::duration_cast<std::chrono::microseconds>(beforeCopy - beforeSetInOut).count()/1000.f;
  auto afterCopy = std::chrono::high_resolution_clock::now();
  m_exTimeCastSingleRay.msCopyToGPU = std::chrono::duration_cast<std::chrono::microseconds>(afterCopy - beforeCopy).count()/1000.f;
  //
  m_exTimeCastSingleRay.msExecuteOnGPU = 0;
  //// (3.1) clear all outputs if we are in RTV pattern
  //
  if(CastSingleRay_local.needToClearOutput)
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
    m_exTimeCastSingleRay.msExecuteOnGPU  += std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count()/1000.f;
  }

  // (4) now execute algorithm on GPU
  //
  {
    VkCommandBuffer commandBuffer = vk_utils::createCommandBuffer(device, commandPool);
    VkCommandBufferBeginInfo beginCommandBufferInfo = {};
    beginCommandBufferInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginCommandBufferInfo.flags = VK_COMMAND_BUFFER_USAGE_SIMULTANEOUS_USE_BIT;
    vkBeginCommandBuffer(commandBuffer, &beginCommandBufferInfo);
    CastSingleRayCmd(commandBuffer, tid, out_color);
    vkEndCommandBuffer(commandBuffer);
    auto start = std::chrono::high_resolution_clock::now();
    if(a_numPasses > 1)
      ProgressBarStart();
    for(uint32_t pass = 0; pass < a_numPasses; pass++) {
      vk_utils::executeCommandBufferNow(commandBuffer, computeQueue, device);
      if((pass != 0) && (pass % 256 == 0))
        ProgressBarAccum(256.0f/float(a_numPasses));
    }
    if(a_numPasses > 1)
      ProgressBarDone();
    vkFreeCommandBuffers(device, commandPool, 1, &commandBuffer);
    auto stop = std::chrono::high_resolution_clock::now();
    m_exTimeCastSingleRay.msExecuteOnGPU += std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count()/1000.f;
  }

  // (5) copy output data to CPU
  //
  auto beforeCopy2 = std::chrono::high_resolution_clock::now();
  pCopyHelper->ReadBuffer(out_colorGPU, 0, out_color, tid*4*sizeof(float ));
  this->ReadPlainMembers(pCopyHelper);
  afterCopy2 = std::chrono::high_resolution_clock::now();
  m_exTimeCastSingleRay.msCopyFromGPU = std::chrono::duration_cast<std::chrono::microseconds>(afterCopy2 - beforeCopy2).count()/1000.f;

  // (6) free resources
  //
  vkDestroyBuffer(device, out_colorGPU, nullptr);
  if(buffersMem != VK_NULL_HANDLE)
    vkFreeMemory(device, buffersMem, nullptr);
  if(imagesMem != VK_NULL_HANDLE)
    vkFreeMemory(device, imagesMem, nullptr);

  m_exTimeCastSingleRay.msAPIOverhead += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - afterCopy2).count()/1000.f;
}

void Integrator_Generated::PackXYBlock(uint tidX, uint tidY, uint32_t a_numPasses)
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
    PackXYCmd(commandBuffer, tidX, tidY);
    vkEndCommandBuffer(commandBuffer);
    auto start = std::chrono::high_resolution_clock::now();
    if(a_numPasses > 1)
      ProgressBarStart();
    for(uint32_t pass = 0; pass < a_numPasses; pass++) {
      vk_utils::executeCommandBufferNow(commandBuffer, computeQueue, device);
      if((pass != 0) && (pass % 256 == 0))
        ProgressBarAccum(256.0f/float(a_numPasses));
    }
    if(a_numPasses > 1)
      ProgressBarDone();
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

void Integrator_Generated::PathTraceFromInputRaysBlock(uint tid, uint channels, const RayPosAndW* in_rayPosAndNear, const RayDirAndT* in_rayDirAndFar, float* out_color, uint32_t a_numPasses)
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
  if(PathTraceFromInputRays_local.needToClearOutput)
    outFlags |= VK_IMAGE_USAGE_TRANSFER_DST_BIT;
  std::vector<VkBuffer> buffers;
  std::vector<VkImage>  images2;
  std::vector<vk_utils::VulkanImageMem*> images;
  auto beforeCreateObjects = std::chrono::high_resolution_clock::now();
  VkBuffer in_rayPosAndNearGPU = vk_utils::createBuffer(device, tid*channels*sizeof(const RayPosAndW ), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  buffers.push_back(in_rayPosAndNearGPU);
  VkBuffer in_rayDirAndFarGPU = vk_utils::createBuffer(device, tid*channels*sizeof(const RayDirAndT ), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  buffers.push_back(in_rayDirAndFarGPU);
  VkBuffer out_colorGPU = vk_utils::createBuffer(device, tid*channels*sizeof(float ), outFlags);
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
  m_exTimePathTraceFromInputRays.msAPIOverhead = std::chrono::duration_cast<std::chrono::microseconds>(afterCreateObjects - beforeCreateObjects).count()/1000.f;

  auto afterCopy2 = std::chrono::high_resolution_clock::now(); // just declare it here, replace value later

  auto afterInitBuffers = std::chrono::high_resolution_clock::now();
  m_exTimePathTraceFromInputRays.msAPIOverhead += std::chrono::duration_cast<std::chrono::microseconds>(afterInitBuffers - afterCreateObjects).count()/1000.f;

  auto beforeSetInOut = std::chrono::high_resolution_clock::now();
  this->SetVulkanInOutFor_PathTraceFromInputRays(in_rayPosAndNearGPU, 0, in_rayDirAndFarGPU, 0, out_colorGPU, 0);

  // (3) copy input data to GPU
  //
  auto beforeCopy = std::chrono::high_resolution_clock::now();
  m_exTimePathTraceFromInputRays.msAPIOverhead += std::chrono::duration_cast<std::chrono::microseconds>(beforeCopy - beforeSetInOut).count()/1000.f;
  pCopyHelper->UpdateBuffer(in_rayPosAndNearGPU, 0, in_rayPosAndNear, tid*channels*sizeof(const RayPosAndW ));
  pCopyHelper->UpdateBuffer(in_rayDirAndFarGPU, 0, in_rayDirAndFar, tid*channels*sizeof(const RayDirAndT ));
  auto afterCopy = std::chrono::high_resolution_clock::now();
  m_exTimePathTraceFromInputRays.msCopyToGPU = std::chrono::duration_cast<std::chrono::microseconds>(afterCopy - beforeCopy).count()/1000.f;
  //
  m_exTimePathTraceFromInputRays.msExecuteOnGPU = 0;
  //// (3.1) clear all outputs if we are in RTV pattern
  //
  if(PathTraceFromInputRays_local.needToClearOutput)
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
    m_exTimePathTraceFromInputRays.msExecuteOnGPU  += std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count()/1000.f;
  }

  // (4) now execute algorithm on GPU
  //
  {
    VkCommandBuffer commandBuffer = vk_utils::createCommandBuffer(device, commandPool);
    VkCommandBufferBeginInfo beginCommandBufferInfo = {};
    beginCommandBufferInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginCommandBufferInfo.flags = VK_COMMAND_BUFFER_USAGE_SIMULTANEOUS_USE_BIT;
    vkBeginCommandBuffer(commandBuffer, &beginCommandBufferInfo);
    PathTraceFromInputRaysCmd(commandBuffer, tid, channels, in_rayPosAndNear, in_rayDirAndFar, out_color);
    vkEndCommandBuffer(commandBuffer);
    auto start = std::chrono::high_resolution_clock::now();
    if(a_numPasses > 1)
      ProgressBarStart();
    for(uint32_t pass = 0; pass < a_numPasses; pass++) {
      vk_utils::executeCommandBufferNow(commandBuffer, computeQueue, device);
      if((pass != 0) && (pass % 256 == 0))
        ProgressBarAccum(256.0f/float(a_numPasses));
    }
    if(a_numPasses > 1)
      ProgressBarDone();
    vkFreeCommandBuffers(device, commandPool, 1, &commandBuffer);
    auto stop = std::chrono::high_resolution_clock::now();
    m_exTimePathTraceFromInputRays.msExecuteOnGPU += std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count()/1000.f;
  }

  // (5) copy output data to CPU
  //
  auto beforeCopy2 = std::chrono::high_resolution_clock::now();
  pCopyHelper->ReadBuffer(out_colorGPU, 0, out_color, tid*channels*sizeof(float ));
  this->ReadPlainMembers(pCopyHelper);
  afterCopy2 = std::chrono::high_resolution_clock::now();
  m_exTimePathTraceFromInputRays.msCopyFromGPU = std::chrono::duration_cast<std::chrono::microseconds>(afterCopy2 - beforeCopy2).count()/1000.f;

  // (6) free resources
  //
  vkDestroyBuffer(device, in_rayPosAndNearGPU, nullptr);
  vkDestroyBuffer(device, in_rayDirAndFarGPU, nullptr);
  vkDestroyBuffer(device, out_colorGPU, nullptr);
  if(buffersMem != VK_NULL_HANDLE)
    vkFreeMemory(device, buffersMem, nullptr);
  if(imagesMem != VK_NULL_HANDLE)
    vkFreeMemory(device, imagesMem, nullptr);

  m_exTimePathTraceFromInputRays.msAPIOverhead += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - afterCopy2).count()/1000.f;
}

void Integrator_Generated::PathTraceBlock(uint tid, uint channels, float* out_color, uint32_t a_numPasses)
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
  if(PathTrace_local.needToClearOutput)
    outFlags |= VK_IMAGE_USAGE_TRANSFER_DST_BIT;
  std::vector<VkBuffer> buffers;
  std::vector<VkImage>  images2;
  std::vector<vk_utils::VulkanImageMem*> images;
  auto beforeCreateObjects = std::chrono::high_resolution_clock::now();
  VkBuffer out_colorGPU = vk_utils::createBuffer(device, tid*channels*sizeof(float ), outFlags);
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
  m_exTimePathTrace.msAPIOverhead = std::chrono::duration_cast<std::chrono::microseconds>(afterCreateObjects - beforeCreateObjects).count()/1000.f;

  auto afterCopy2 = std::chrono::high_resolution_clock::now(); // just declare it here, replace value later

  auto afterInitBuffers = std::chrono::high_resolution_clock::now();
  m_exTimePathTrace.msAPIOverhead += std::chrono::duration_cast<std::chrono::microseconds>(afterInitBuffers - afterCreateObjects).count()/1000.f;

  auto beforeSetInOut = std::chrono::high_resolution_clock::now();
  this->SetVulkanInOutFor_PathTrace(out_colorGPU, 0);

  // (3) copy input data to GPU
  //
  auto beforeCopy = std::chrono::high_resolution_clock::now();
  m_exTimePathTrace.msAPIOverhead += std::chrono::duration_cast<std::chrono::microseconds>(beforeCopy - beforeSetInOut).count()/1000.f;
  auto afterCopy = std::chrono::high_resolution_clock::now();
  m_exTimePathTrace.msCopyToGPU = std::chrono::duration_cast<std::chrono::microseconds>(afterCopy - beforeCopy).count()/1000.f;
  //
  m_exTimePathTrace.msExecuteOnGPU = 0;
  //// (3.1) clear all outputs if we are in RTV pattern
  //
  if(PathTrace_local.needToClearOutput)
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
    m_exTimePathTrace.msExecuteOnGPU  += std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count()/1000.f;
  }

  // (4) now execute algorithm on GPU
  //
  {
    VkCommandBuffer commandBuffer = vk_utils::createCommandBuffer(device, commandPool);
    VkCommandBufferBeginInfo beginCommandBufferInfo = {};
    beginCommandBufferInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginCommandBufferInfo.flags = VK_COMMAND_BUFFER_USAGE_SIMULTANEOUS_USE_BIT;
    vkBeginCommandBuffer(commandBuffer, &beginCommandBufferInfo);
    PathTraceCmd(commandBuffer, tid, channels, out_color);
    vkEndCommandBuffer(commandBuffer);
    auto start = std::chrono::high_resolution_clock::now();
    if(a_numPasses > 1)
      ProgressBarStart();
    for(uint32_t pass = 0; pass < a_numPasses; pass++) {
      vk_utils::executeCommandBufferNow(commandBuffer, computeQueue, device);
      if((pass != 0) && (pass % 256 == 0))
        ProgressBarAccum(256.0f/float(a_numPasses));
    }
    if(a_numPasses > 1)
      ProgressBarDone();
    vkFreeCommandBuffers(device, commandPool, 1, &commandBuffer);
    auto stop = std::chrono::high_resolution_clock::now();
    m_exTimePathTrace.msExecuteOnGPU += std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count()/1000.f;
  }

  // (5) copy output data to CPU
  //
  auto beforeCopy2 = std::chrono::high_resolution_clock::now();
  pCopyHelper->ReadBuffer(out_colorGPU, 0, out_color, tid*channels*sizeof(float ));
  this->ReadPlainMembers(pCopyHelper);
  afterCopy2 = std::chrono::high_resolution_clock::now();
  m_exTimePathTrace.msCopyFromGPU = std::chrono::duration_cast<std::chrono::microseconds>(afterCopy2 - beforeCopy2).count()/1000.f;

  // (6) free resources
  //
  vkDestroyBuffer(device, out_colorGPU, nullptr);
  if(buffersMem != VK_NULL_HANDLE)
    vkFreeMemory(device, buffersMem, nullptr);
  if(imagesMem != VK_NULL_HANDLE)
    vkFreeMemory(device, imagesMem, nullptr);

  m_exTimePathTrace.msAPIOverhead += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - afterCopy2).count()/1000.f;
}

void Integrator_Generated::NaivePathTraceBlock(uint tid, uint channels, float* out_color, uint32_t a_numPasses)
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
  if(NaivePathTrace_local.needToClearOutput)
    outFlags |= VK_IMAGE_USAGE_TRANSFER_DST_BIT;
  std::vector<VkBuffer> buffers;
  std::vector<VkImage>  images2;
  std::vector<vk_utils::VulkanImageMem*> images;
  auto beforeCreateObjects = std::chrono::high_resolution_clock::now();
  VkBuffer out_colorGPU = vk_utils::createBuffer(device, tid*channels*sizeof(float ), outFlags);
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
  m_exTimeNaivePathTrace.msAPIOverhead = std::chrono::duration_cast<std::chrono::microseconds>(afterCreateObjects - beforeCreateObjects).count()/1000.f;

  auto afterCopy2 = std::chrono::high_resolution_clock::now(); // just declare it here, replace value later

  auto afterInitBuffers = std::chrono::high_resolution_clock::now();
  m_exTimeNaivePathTrace.msAPIOverhead += std::chrono::duration_cast<std::chrono::microseconds>(afterInitBuffers - afterCreateObjects).count()/1000.f;

  auto beforeSetInOut = std::chrono::high_resolution_clock::now();
  this->SetVulkanInOutFor_NaivePathTrace(out_colorGPU, 0);

  // (3) copy input data to GPU
  //
  auto beforeCopy = std::chrono::high_resolution_clock::now();
  m_exTimeNaivePathTrace.msAPIOverhead += std::chrono::duration_cast<std::chrono::microseconds>(beforeCopy - beforeSetInOut).count()/1000.f;
  auto afterCopy = std::chrono::high_resolution_clock::now();
  m_exTimeNaivePathTrace.msCopyToGPU = std::chrono::duration_cast<std::chrono::microseconds>(afterCopy - beforeCopy).count()/1000.f;
  //
  m_exTimeNaivePathTrace.msExecuteOnGPU = 0;
  //// (3.1) clear all outputs if we are in RTV pattern
  //
  if(NaivePathTrace_local.needToClearOutput)
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
    m_exTimeNaivePathTrace.msExecuteOnGPU  += std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count()/1000.f;
  }

  // (4) now execute algorithm on GPU
  //
  {
    VkCommandBuffer commandBuffer = vk_utils::createCommandBuffer(device, commandPool);
    VkCommandBufferBeginInfo beginCommandBufferInfo = {};
    beginCommandBufferInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginCommandBufferInfo.flags = VK_COMMAND_BUFFER_USAGE_SIMULTANEOUS_USE_BIT;
    vkBeginCommandBuffer(commandBuffer, &beginCommandBufferInfo);
    NaivePathTraceCmd(commandBuffer, tid, channels, out_color);
    vkEndCommandBuffer(commandBuffer);
    auto start = std::chrono::high_resolution_clock::now();
    if(a_numPasses > 1)
      ProgressBarStart();
    for(uint32_t pass = 0; pass < a_numPasses; pass++) {
      vk_utils::executeCommandBufferNow(commandBuffer, computeQueue, device);
      if((pass != 0) && (pass % 256 == 0))
        ProgressBarAccum(256.0f/float(a_numPasses));
    }
    if(a_numPasses > 1)
      ProgressBarDone();
    vkFreeCommandBuffers(device, commandPool, 1, &commandBuffer);
    auto stop = std::chrono::high_resolution_clock::now();
    m_exTimeNaivePathTrace.msExecuteOnGPU += std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count()/1000.f;
  }

  // (5) copy output data to CPU
  //
  auto beforeCopy2 = std::chrono::high_resolution_clock::now();
  pCopyHelper->ReadBuffer(out_colorGPU, 0, out_color, tid*channels*sizeof(float ));
  this->ReadPlainMembers(pCopyHelper);
  afterCopy2 = std::chrono::high_resolution_clock::now();
  m_exTimeNaivePathTrace.msCopyFromGPU = std::chrono::duration_cast<std::chrono::microseconds>(afterCopy2 - beforeCopy2).count()/1000.f;

  // (6) free resources
  //
  vkDestroyBuffer(device, out_colorGPU, nullptr);
  if(buffersMem != VK_NULL_HANDLE)
    vkFreeMemory(device, buffersMem, nullptr);
  if(imagesMem != VK_NULL_HANDLE)
    vkFreeMemory(device, imagesMem, nullptr);

  m_exTimeNaivePathTrace.msAPIOverhead += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - afterCopy2).count()/1000.f;
}



void Integrator_Generated::GetExecutionTime(const char* a_funcName, float a_out[4])
{
  vk_utils::ExecTime res = {};
  if(std::string(a_funcName) == "RayTrace" || std::string(a_funcName) == "RayTraceBlock")
    res = m_exTimeRayTrace;
  if(std::string(a_funcName) == "CastSingleRay" || std::string(a_funcName) == "CastSingleRayBlock")
    res = m_exTimeCastSingleRay;
  if(std::string(a_funcName) == "PackXY" || std::string(a_funcName) == "PackXYBlock")
    res = m_exTimePackXY;
  if(std::string(a_funcName) == "PathTraceFromInputRays" || std::string(a_funcName) == "PathTraceFromInputRaysBlock")
    res = m_exTimePathTraceFromInputRays;
  if(std::string(a_funcName) == "PathTrace" || std::string(a_funcName) == "PathTraceBlock")
    res = m_exTimePathTrace;
  if(std::string(a_funcName) == "NaivePathTrace" || std::string(a_funcName) == "NaivePathTraceBlock")
    res = m_exTimeNaivePathTrace;
  a_out[0] = res.msExecuteOnGPU;
  a_out[1] = res.msCopyToGPU;
  a_out[2] = res.msCopyFromGPU;
  a_out[3] = res.msAPIOverhead;
}

