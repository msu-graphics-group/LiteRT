#pragma once
#include <vector>
#include "LiteMath.h"

namespace dr
{
  using LiteMath::float3;
  static constexpr uint32_t INVALID_INDEX = 0xFFFFFFFF;
  struct PDColor
  {
    uint32_t index;
    float value;
  };

  struct PDDist
  {
    float3 dDiffuse;
    uint32_t index;
    float3 dNorm;
    uint32_t _pad;
  };

  struct CRT_HitDR 
  {
    float    t;         ///< intersection distance from ray origin to object
    uint32_t primId; 
    uint32_t instId;
    uint32_t geomId;    ///< use 4 most significant bits for geometry type; thay are zero for triangles 
    float3   color;
    uint32_t _pad0;
    float3   normal;
    uint32_t _pad1;

    PDColor dDiffuse_dSc[8]; //8 color points, PDs for diffuse (PDs for R,G,B are the same)
    PDDist  dDiffuseNormal_dSd[8]; //8 distance points, PDs for diffuse and normal
    //    dNorm_dSc[8]; is zero, normal vector does not depend on color

  };
}