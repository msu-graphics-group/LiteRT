#ifndef MultiRenderer_UBO_H
#define MultiRenderer_UBO_H

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

struct MultiRenderer_GPU_UBO_Data
{
  mat4 m_projInv; 
  mat4 m_worldViewInv; 
  RenderPreset m_presets; 
  uint m_height; 
  uint m_width; 
  uint m_pAccelStruct_capacity; 
  uint m_pAccelStruct_m_ConjIndices_capacity; 
  uint m_pAccelStruct_m_ConjIndices_size; 
  uint m_pAccelStruct_m_SdfConjunctions_capacity; 
  uint m_pAccelStruct_m_SdfConjunctions_size; 
  uint m_pAccelStruct_m_SdfGridData_capacity; 
  uint m_pAccelStruct_m_SdfGridData_size; 
  uint m_pAccelStruct_m_SdfGridOffsets_capacity; 
  uint m_pAccelStruct_m_SdfGridOffsets_size; 
  uint m_pAccelStruct_m_SdfGridSizes_capacity; 
  uint m_pAccelStruct_m_SdfGridSizes_size; 
  uint m_pAccelStruct_m_SdfNeuralProperties_capacity; 
  uint m_pAccelStruct_m_SdfNeuralProperties_size; 
  uint m_pAccelStruct_m_SdfObjects_capacity; 
  uint m_pAccelStruct_m_SdfObjects_size; 
  uint m_pAccelStruct_m_SdfOctreeNodes_capacity; 
  uint m_pAccelStruct_m_SdfOctreeNodes_size; 
  uint m_pAccelStruct_m_SdfOctreeRoots_capacity; 
  uint m_pAccelStruct_m_SdfOctreeRoots_size; 
  uint m_pAccelStruct_m_SdfParameters_capacity; 
  uint m_pAccelStruct_m_SdfParameters_size; 
  uint m_pAccelStruct_m_allNodePairs_capacity; 
  uint m_pAccelStruct_m_allNodePairs_size; 
  uint m_pAccelStruct_m_bvhOffsets_capacity; 
  uint m_pAccelStruct_m_bvhOffsets_size; 
  uint m_pAccelStruct_m_geomIdByInstId_capacity; 
  uint m_pAccelStruct_m_geomIdByInstId_size; 
  uint m_pAccelStruct_m_geomOffsets_capacity; 
  uint m_pAccelStruct_m_geomOffsets_size; 
  uint m_pAccelStruct_m_geomTypeByGeomId_capacity; 
  uint m_pAccelStruct_m_geomTypeByGeomId_size; 
  uint m_pAccelStruct_m_indices_capacity; 
  uint m_pAccelStruct_m_indices_size; 
  uint m_pAccelStruct_m_instMatricesInv_capacity; 
  uint m_pAccelStruct_m_instMatricesInv_size; 
  uint m_pAccelStruct_m_nodesTLAS_capacity; 
  uint m_pAccelStruct_m_nodesTLAS_size; 
  uint m_pAccelStruct_m_primIndices_capacity; 
  uint m_pAccelStruct_m_primIndices_size; 
  uint m_pAccelStruct_m_vertPos_capacity; 
  uint m_pAccelStruct_m_vertPos_size; 
  uint m_pAccelStruct_size; 
  uint m_packedXY_capacity; 
  uint m_packedXY_size; 
  uint dummy_last;
};

#endif

