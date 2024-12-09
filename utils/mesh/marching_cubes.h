#pragma once
#include <functional>
#include "LiteScene/cmesh4.h"

namespace cmesh4
{
  using LiteMath::float4;
  using LiteMath::float3;
  using LiteMath::uint3;
  using DensityFunction = std::function<float(const float3 &)>;
  using MultithreadedDensityFunction = std::function<float(const float3 &, unsigned idx)>;

  struct MarchingCubesSettings
  {
    float3 min_pos = float3(-1,-1,-1);
    float3 max_pos = float3(1,1,1);

    LiteMath::uint3 size = LiteMath::uint3(128, 128, 128);
    float iso_level = 0.5f;
  };

  cmesh4::SimpleMesh create_mesh_marching_cubes(MarchingCubesSettings settings, MultithreadedDensityFunction sdf, unsigned max_threads);
}