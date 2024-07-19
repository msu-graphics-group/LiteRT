#pragma once
#include <vector>
#include "LiteMath.h"

namespace dr
{
  struct PD
  {
    static constexpr uint32_t INVALID_INDEX = 0xFFFFFFFF;
    uint32_t index;
    uint32_t size;
    float value;
  };

  struct CRT_HitDR 
  {
    float    t;         ///< intersection distance from ray origin to object
    uint32_t primId; 
    uint32_t instId;
    uint32_t geomId;    ///< use 4 most significant bits for geometry type; thay are zero for triangles 
    float    coords[4]; ///< custom intersection data; for triangles coords[0] and coords[1] stores baricentric coords (u,v)
                        // coords[2] and coords[3] stores normal.xy

    PD dDiffuse_dS[8]; //8 points, PDs for R,G,B are the same
  };
}