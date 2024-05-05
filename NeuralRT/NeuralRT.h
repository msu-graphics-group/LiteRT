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

static constexpr unsigned NEURALRT_BSIZE = 8;
static constexpr unsigned NEURALRT_LAYER_SIZE = 64;

//enum NeuralRTRenderType
static constexpr unsigned NEURALRT_RENDER_SIMPLE        = 0;
static constexpr unsigned NEURALRT_RENDER_BLOCKED       = 1;
static constexpr unsigned NEURALRT_RENDER_COOP_MATRICES = 2;

class NeuralRT
{
public:
  NeuralRT();

  uint32_t AddGeom_NeuralSdf(NeuralProperties neural_properties, float *data, BuildOptions a_qualityLevel = BUILD_HIGH);
  
  virtual void Render(uint32_t* imageData [[size("a_width*a_height")]], uint32_t a_width, uint32_t a_height, 
                      const LiteMath::float4x4& a_worldView, const LiteMath::float4x4& a_proj, 
                      uint32_t a_renderType = NEURALRT_RENDER_SIMPLE, int a_passNum = 1);

  virtual void CommitDeviceData() {}                                       // will be overriden in generated class
  virtual void GetExecutionTime(const char* a_funcName, float a_out[4]) {} // will be overriden in generated class
protected:
  template<uint bsize> 
  void kernelBE1D_BlockedSphereTracing(uint32_t* imageData, uint blockNum); 

  template<uint bsize> 
  void kernelBE1D_CoopMatricesSphereTracing(uint32_t* imageData, uint blockNum); 

  virtual void kernel1D_SimpleSphereTracing(uint32_t* imageData, uint blockNum);

  virtual void Render_internal(uint32_t* imageData [[size("a_width*a_height")]], uint32_t a_width, uint32_t a_height, 
                               uint32_t a_renderType, int a_passNum);

  std::vector<NeuralProperties> m_SdfNeuralProperties;
  std::vector<float> m_SdfNeuralData;

  uint32_t m_width;
  uint32_t m_height;

  LiteMath::float4x4 m_projInv;
  LiteMath::float4x4 m_worldViewInv;
};

std::shared_ptr<NeuralRT> CreateNeuralRT(const char* a_name);