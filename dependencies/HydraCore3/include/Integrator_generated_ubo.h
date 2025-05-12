#ifndef Integrator_gpu_UBO_H
#define Integrator_gpu_UBO_H

#ifndef GLSL
#define LAYOUT_STD140
#include "LiteMath.h"
using LiteMath::uint;
typedef LiteMath::float4x4 mat4;
typedef LiteMath::float2   vec2;
typedef LiteMath::float3   vec3;
typedef LiteMath::float4   vec4;
typedef LiteMath::int2     ivec2;
typedef LiteMath::int3     ivec3;
typedef LiteMath::int4     ivec4;
typedef LiteMath::uint2    uvec2;
typedef LiteMath::uint3    uvec3;
typedef LiteMath::uint4    uvec4;
#else
#define M_PI          3.14159265358979323846f
#define M_TWOPI       6.28318530717958647692f
#define INV_PI        0.31830988618379067154f
#define INV_TWOPI     0.15915494309189533577f
#endif

struct Integrator_Generated_UBO_Data
{
  mat4 m_projInv; 
  mat4 m_worldViewInv; 
  vec4 m_camRespoceRGB; 
  vec4 m_envColor; 
  vec4 m_envSamRow0; 
  vec4 m_envSamRow1; 
  vec2 m_physSize; 
  COctreeV3Header m_pAccelStruct_coctree_v3_header; 
  MultiRenderPreset m_pAccelStruct_m_preset; 
  int m_camResponseSpectrumId[3]; 
  float m_camLensRadius; 
  int m_camResponseType; 
  float m_camTargetDist; 
  uint m_disableImageContrib; 
  uint m_enableOpticSim; 
  uint m_envCamBackId; 
  uint m_envEnableSam; 
  uint m_envLightId; 
  uint m_envTexId; 
  float m_exposureMult; 
  int m_fbHeight; 
  int m_fbWidth; 
  uint m_intergatorType; 
  uint m_maxThreadId; 
  uint m_renderLayer; 
  int m_spectral_mode; 
  uint m_tileSize; 
  uint m_traceDepth; 
  int m_winHeight; 
  int m_winStartX; 
  int m_winStartY; 
  int m_winWidth; 
  uint all_references_capacity; 
  uint all_references_size; 
  uint m_allRemapListsOffsets_capacity; 
  uint m_allRemapListsOffsets_size; 
  uint m_allRemapLists_capacity; 
  uint m_allRemapLists_size; 
  uint m_cie_x_capacity; 
  uint m_cie_x_size; 
  uint m_cie_y_capacity; 
  uint m_cie_y_size; 
  uint m_cie_z_capacity; 
  uint m_cie_z_size; 
  uint m_films_eta_k_vec_capacity; 
  uint m_films_eta_k_vec_size; 
  uint m_films_spec_id_vec_capacity; 
  uint m_films_spec_id_vec_size; 
  uint m_instIdToLightInstId_capacity; 
  uint m_instIdToLightInstId_size; 
  uint m_lights_capacity; 
  uint m_lights_size; 
  uint m_lines_capacity; 
  uint m_lines_size; 
  uint m_matIdByPrimId_capacity; 
  uint m_matIdByPrimId_size; 
  uint m_matIdOffsets_capacity; 
  uint m_matIdOffsets_size; 
  uint m_materials_capacity; 
  uint m_materials_size; 
  uint m_normMatrices2_capacity; 
  uint m_normMatrices2_size; 
  uint m_normMatrices_capacity; 
  uint m_normMatrices_size; 
  uint m_pAccelStruct_capacity; 
  uint m_pAccelStruct_m_CatmulClarkHeaders_capacity; 
  uint m_pAccelStruct_m_CatmulClarkHeaders_size; 
  uint m_pAccelStruct_m_NURBSData_capacity; 
  uint m_pAccelStruct_m_NURBSData_size; 
  uint m_pAccelStruct_m_NURBSHeaders_capacity; 
  uint m_pAccelStruct_m_NURBSHeaders_size; 
  uint m_pAccelStruct_m_NURBS_approxes_capacity; 
  uint m_pAccelStruct_m_NURBS_approxes_size; 
  uint m_pAccelStruct_m_RibbonHeaders_capacity; 
  uint m_pAccelStruct_m_RibbonHeaders_size; 
  uint m_pAccelStruct_m_SdfCompactOctreeRotModifiers_capacity; 
  uint m_pAccelStruct_m_SdfCompactOctreeRotModifiers_size; 
  uint m_pAccelStruct_m_SdfCompactOctreeV2Data_capacity; 
  uint m_pAccelStruct_m_SdfCompactOctreeV2Data_size; 
  uint m_pAccelStruct_m_SdfCompactOctreeV3Data_capacity; 
  uint m_pAccelStruct_m_SdfCompactOctreeV3Data_size; 
  uint m_pAccelStruct_m_SdfFrameOctreeNodes_capacity; 
  uint m_pAccelStruct_m_SdfFrameOctreeNodes_size; 
  uint m_pAccelStruct_m_SdfFrameOctreeRoots_capacity; 
  uint m_pAccelStruct_m_SdfFrameOctreeRoots_size; 
  uint m_pAccelStruct_m_SdfSBSDataF_capacity; 
  uint m_pAccelStruct_m_SdfSBSDataF_size; 
  uint m_pAccelStruct_m_SdfSBSData_capacity; 
  uint m_pAccelStruct_m_SdfSBSData_size; 
  uint m_pAccelStruct_m_SdfSBSHeaders_capacity; 
  uint m_pAccelStruct_m_SdfSBSHeaders_size; 
  uint m_pAccelStruct_m_SdfSBSNodes_capacity; 
  uint m_pAccelStruct_m_SdfSBSNodes_size; 
  uint m_pAccelStruct_m_SdfSBSRoots_capacity; 
  uint m_pAccelStruct_m_SdfSBSRoots_size; 
  uint m_pAccelStruct_m_SdfSVSNodes_capacity; 
  uint m_pAccelStruct_m_SdfSVSNodes_size; 
  uint m_pAccelStruct_m_SdfSVSRoots_capacity; 
  uint m_pAccelStruct_m_SdfSVSRoots_size; 
  uint m_pAccelStruct_m_abstractObjectPtrs_capacity; 
  uint m_pAccelStruct_m_abstractObjectPtrs_size; 
  uint m_pAccelStruct_m_allNodePairs_capacity; 
  uint m_pAccelStruct_m_allNodePairs_size; 
  uint m_pAccelStruct_m_geomData_capacity; 
  uint m_pAccelStruct_m_geomData_size; 
  uint m_pAccelStruct_m_indices_capacity; 
  uint m_pAccelStruct_m_indices_size; 
  uint m_pAccelStruct_m_instanceData_capacity; 
  uint m_pAccelStruct_m_instanceData_size; 
  uint m_pAccelStruct_m_nodesTLAS_capacity; 
  uint m_pAccelStruct_m_nodesTLAS_size; 
  uint m_pAccelStruct_m_origNodes_capacity; 
  uint m_pAccelStruct_m_origNodes_size; 
  uint m_pAccelStruct_m_primIdCount_capacity; 
  uint m_pAccelStruct_m_primIdCount_size; 
  uint m_pAccelStruct_m_primIndices_capacity; 
  uint m_pAccelStruct_m_primIndices_size; 
  uint m_pAccelStruct_m_vertNorm_capacity; 
  uint m_pAccelStruct_m_vertNorm_size; 
  uint m_pAccelStruct_m_vertPos_capacity; 
  uint m_pAccelStruct_m_vertPos_size; 
  uint m_pAccelStruct_size; 
  uint m_pAccelStruct_startEnd_capacity; 
  uint m_pAccelStruct_startEnd_size; 
  uint m_packedXY_capacity; 
  uint m_packedXY_size; 
  uint m_pdfLightData_capacity; 
  uint m_pdfLightData_size; 
  uint m_precomp_coat_transmittance_capacity; 
  uint m_precomp_coat_transmittance_size; 
  uint m_precomp_thin_films_capacity; 
  uint m_precomp_thin_films_size; 
  uint m_randomGens_capacity; 
  uint m_randomGens_size; 
  uint m_remapInst_capacity; 
  uint m_remapInst_size; 
  uint m_spec_offset_sz_capacity; 
  uint m_spec_offset_sz_size; 
  uint m_spec_tex_ids_wavelengths_capacity; 
  uint m_spec_tex_ids_wavelengths_size; 
  uint m_spec_tex_offset_sz_capacity; 
  uint m_spec_tex_offset_sz_size; 
  uint m_spec_values_capacity; 
  uint m_spec_values_size; 
  uint m_textures_capacity; 
  uint m_textures_size; 
  uint m_triIndices_capacity; 
  uint m_triIndices_size; 
  uint m_vNorm4f_capacity; 
  uint m_vNorm4f_size; 
  uint m_vTang4f_capacity; 
  uint m_vTang4f_size; 
  uint m_vertOffset_capacity; 
  uint m_vertOffset_size; 
  uint dummy_last;
};

#endif

