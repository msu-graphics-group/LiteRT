#ifndef MultiRenderer_gpu_UBO_H
#define MultiRenderer_gpu_UBO_H

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

struct MultiRenderer_gpu_rq_UBO_Data
{
  mat4 m_projInv; 
  mat4 m_worldViewInv; 
  COctreeV3Header m_pAccelStruct_coctree_v3_header; 
  MultiRenderPreset m_pAccelStruct_m_preset; 
  MultiRenderPreset m_preset; 
  uint m_height; 
  uint m_packedXY_height; 
  uint m_packedXY_width; 
  uint m_seed; 
  uint m_width; 
  uint all_references_capacity; 
  uint all_references_size; 
  uint m_geomOffsets_capacity; 
  uint m_geomOffsets_size; 
  uint m_indices_capacity; 
  uint m_indices_size; 
  uint m_instanceTransformInvTransposed_capacity; 
  uint m_instanceTransformInvTransposed_size; 
  uint m_lights_capacity; 
  uint m_lights_size; 
  uint m_matIdOffsets_capacity; 
  uint m_matIdOffsets_size; 
  uint m_matIdbyPrimId_capacity; 
  uint m_matIdbyPrimId_size; 
  uint m_materials_capacity; 
  uint m_materials_size; 
  uint m_normals_capacity; 
  uint m_normals_size; 
  uint m_pAccelStruct_capacity; 
  uint m_pAccelStruct_m_CatmulClarkHeaders_capacity; 
  uint m_pAccelStruct_m_CatmulClarkHeaders_size; 
  uint m_pAccelStruct_m_GraphicsPrimHeaders_capacity; 
  uint m_pAccelStruct_m_GraphicsPrimHeaders_size; 
  uint m_pAccelStruct_m_GraphicsPrimPoints_capacity; 
  uint m_pAccelStruct_m_GraphicsPrimPoints_size; 
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
  uint m_pAccelStruct_m_SdfFrameOctreeTexNodes_capacity; 
  uint m_pAccelStruct_m_SdfFrameOctreeTexNodes_size; 
  uint m_pAccelStruct_m_SdfFrameOctreeTexRoots_capacity; 
  uint m_pAccelStruct_m_SdfFrameOctreeTexRoots_size; 
  uint m_pAccelStruct_m_SdfGridData_capacity; 
  uint m_pAccelStruct_m_SdfGridData_size; 
  uint m_pAccelStruct_m_SdfGridOffsets_capacity; 
  uint m_pAccelStruct_m_SdfGridOffsets_size; 
  uint m_pAccelStruct_m_SdfGridSizes_capacity; 
  uint m_pAccelStruct_m_SdfGridSizes_size; 
  uint m_pAccelStruct_m_SdfSBSAdaptDataF_capacity; 
  uint m_pAccelStruct_m_SdfSBSAdaptDataF_size; 
  uint m_pAccelStruct_m_SdfSBSAdaptData_capacity; 
  uint m_pAccelStruct_m_SdfSBSAdaptData_size; 
  uint m_pAccelStruct_m_SdfSBSAdaptHeaders_capacity; 
  uint m_pAccelStruct_m_SdfSBSAdaptHeaders_size; 
  uint m_pAccelStruct_m_SdfSBSAdaptNodes_capacity; 
  uint m_pAccelStruct_m_SdfSBSAdaptNodes_size; 
  uint m_pAccelStruct_m_SdfSBSAdaptRoots_capacity; 
  uint m_pAccelStruct_m_SdfSBSAdaptRoots_size; 
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
  uint m_textures_capacity; 
  uint m_textures_size; 
  uint m_vertices_capacity; 
  uint m_vertices_size; 
  uint dummy_last;
};

#endif

