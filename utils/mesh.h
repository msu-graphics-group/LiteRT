#pragma once
#include "LiteScene/cmesh4.h"

namespace cmesh4
{
  using LiteMath::float4;
  using LiteMath::float3;

  void get_bbox(const cmesh4::SimpleMesh &mesh, float3 *min_pos, float3 *max_pos);

  //rescales mesh with constant scale to fit it inside the given box, 
  void rescale_mesh(cmesh4::SimpleMesh &mesh, float3 min_pos, float3 max_pos);
}