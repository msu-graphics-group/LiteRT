#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>
#include <string>
#include <memory>

#include "LiteMath.h"

using LiteMath::cross;
using LiteMath::dot;
using LiteMath::float2;
using LiteMath::float3;
using LiteMath::float4;
using LiteMath::float4x4;
using LiteMath::int2;
using LiteMath::inverse4x4;
using LiteMath::normalize;
using LiteMath::sign;
using LiteMath::to_float3;
using LiteMath::uint;
using LiteMath::uint2;
using LiteMath::uint3;
using LiteMath::uint4;
using LiteMath::Box4f;

#include "../ISceneObject.h"
#include "../raytrace_common.h"

static const uint32_t NEURALRT_BSIZE = 1;

class NeuralRT
{
public:
  NeuralRT();

  uint32_t AddGeom_NeuralSdf(NeuralProperties neural_properties, float *data, BuildOptions a_qualityLevel = BUILD_HIGH);
  
  void Render(uint32_t* imageData, uint32_t a_width, uint32_t a_height, 
              const LiteMath::float4x4& a_worldView, const LiteMath::float4x4& a_proj, int a_passNum = 1);

protected:
  template<uint bsize> 
  void kernelBE1D_SphereTracing(uint blockNum); 

  std::vector<NeuralProperties> m_SdfNeuralProperties;
  std::vector<float> m_SdfNeuralData;

  std::vector<uint32_t> m_ImageData;

  uint32_t m_width;
  uint32_t m_height;

  LiteMath::float4x4 m_proj;
  LiteMath::float4x4 m_worldView;
  LiteMath::float4x4 m_projInv;
  LiteMath::float4x4 m_worldViewInv;
};