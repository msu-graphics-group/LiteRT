#ifndef SPECTRUM_H
#define SPECTRUM_H

#include <vector>
#include <filesystem>
#include "LiteMath.h"
#include "include/cglobals.h"
#ifndef __OPENCL_VERSION__
using namespace LiteMath;
#endif

#ifndef KERNEL_SLICER
struct Spectrum
{
  float Sample(float lambda) const;
  //std::vector<float> Resample(int channels, float lambdaOffs = 0.0f);
  std::vector<float> ResampleUniform();

  // sorted by wavelength
  std::vector<float> wavelengths; 
  std::vector<float> values;
  uint32_t id = 0;
};
#endif

inline uint32_t BinarySearch(const float* array, size_t array_sz, float val) 
{
  int32_t last = (int32_t)array_sz - 2, first = 1;
  while (last > 0) 
  {
    uint32_t half = (uint32_t)last >> 1, 
    middle = first + half;
    bool predResult = array[middle] <= val;
    first = predResult ? int32_t(middle + 1) : first;
    last = predResult ? last - int32_t(half + 1) : int32_t(half);
  }
  return (uint32_t)clamp(int32_t(first - 1), 0, int32_t(array_sz - 2));
}


// temporary?
inline uint32_t BinarySearchU2(const uint2* array, uint32_t array_sz, float val) 
{
  int32_t last = (int32_t)array_sz - 2, first = 1;
  while (last > 0) 
  {
    uint32_t half = (uint32_t)last >> 1, 
    middle = first + half;
    bool predResult = float(array[middle].y) <= val;
    first = predResult ? int(middle + 1) : first;
    last = predResult ? last - int(half + 1) : int(half);
  }
  return (uint32_t)clamp(int(first - 1), 0, int(array_sz - 2));
}

// "stratified" sample wavelengths in [a, b] with random number u
static inline float4 SampleWavelengths(float u, float a, float b) 
{
  // pdf is 1.0f / (b - a)
  float4 res;

  res[0] = lerp(a, b, u);

  float delta = (b - a) / SPECTRUM_SAMPLE_SZ;
  for (uint32_t i = 1; i < SPECTRUM_SAMPLE_SZ; ++i) 
  {
      res[i] = res[i - 1] + delta;
      if (res[i] > b)
        res[i] = a + (res[i] - b);
  }

  return res;
}

//static inline float4 SampleSpectrum(const float* a_spec_wavelengths, const float* a_spec_values, float4 a_wavelengths, uint32_t a_sz)
//{
//  float4 sampleSpec = float4(0,0,0,0);
//  const uint spectralSamples = uint(sizeof(a_wavelengths.M) / sizeof(a_wavelengths.M[0])); 
//  for(uint i = 0; i < spectralSamples; ++i)
//  {
//    if (a_sz == 0 || a_wavelengths[i] < a_spec_wavelengths[0] || a_wavelengths[i] > a_spec_wavelengths[a_sz - 1])
//    {
//      sampleSpec[i] = 0.0f;
//    }
//    else
//    {
//      int last = (int)a_sz - 2, first = 1;
//      while (last > 0) 
//      {
//        int half = last >> 1, 
//        middle = first + half;
//        bool predResult = a_spec_wavelengths[middle] <= a_wavelengths[i];
//        first = predResult ? int(middle + 1) : first;
//        last = predResult ? last - int(half + 1) : int(half);
//      }
//      int o = clamp(int(first - 1), 0, int(a_sz - 2));
//
//      float t = (a_wavelengths[i] - a_spec_wavelengths[o]) / (a_spec_wavelengths[o + 1] - a_spec_wavelengths[o]);
//      sampleSpec[i] =  lerp(a_spec_values[o], a_spec_values[o + 1], t);
//    } 
//  }
//  return sampleSpec;
//}

static inline float4 SampleUniformSpectrum(const float* a_spec_values, float4 a_wavelengths, uint32_t a_sz)
{
  const int  WAVESN = int(LAMBDA_MAX-LAMBDA_MIN);
  const int4 index1 = int4(min(max(a_wavelengths - float4(LAMBDA_MIN), float4(0.0f)), float4(WAVESN - 1)));   
  const int4 index2 = min(index1 + int4(1), int4(WAVESN-1));

  // const float4 mask = {index1.x >= WAVESN ? 0 : 1, index1.y >= WAVESN ? 0 : 1, index1.z >= WAVESN ? 0 : 1, index1.w >= WAVESN ? 0 : 1};
  

  const float4 x1 = float4(LAMBDA_MIN) + float4(index1);
  const float4 y1 = float4(a_spec_values[index1[0]], a_spec_values[index1[1]], a_spec_values[index1[2]], a_spec_values[index1[3]]); // TODO: reorder mem access for better cache: (index1[0], index2[0])
  const float4 y2 = float4(a_spec_values[index2[0]], a_spec_values[index2[1]], a_spec_values[index2[2]], a_spec_values[index2[3]]); // TODO: reorder mem access for better cache: (index1[1], index2[1])

  auto res = (y1 + (a_wavelengths - x1) * (y2 - y1));
  // if(std::isinf(res.x) || std::isnan(res.x) || res.x < 0)
  // {
  //   int a = 2;
  // }

  return res;
}

//static inline float4 SampleCIE(const float4 &lambda, const float* cie, float a = LAMBDA_MIN, float b = LAMBDA_MAX)
//{
//  float4 res;
//
//  for (uint32_t i = 0; i < SPECTRUM_SAMPLE_SZ; ++i) 
//  {
//    uint32_t offset = uint32_t(float(std::floor(lambda[i] + 0.5f)) - a);
//    if (offset < 0 || offset >= nCIESamples)
//      res[i] = 0;
//    else
//      res[i] = cie[offset];
//  }
//  return res;
//}

static inline float SpectrumAverage(float4 spec) 
{
  float sum = spec[0];
  for (uint32_t i = 1; i < SPECTRUM_SAMPLE_SZ; ++i)
    sum += spec[i];
  return sum / SPECTRUM_SAMPLE_SZ;
}

static inline float3 SpectrumToXYZ(float4 spec, float4 lambda, float lambda_min, float lambda_max,
                                   const float* a_CIE_X, const float* a_CIE_Y, const float* a_CIE_Z, bool terminate_waves) 
{
  float4 pdf = float4(1.0f / (lambda_max - lambda_min));
  const float CIE_Y_integral = 106.856895f;
  const uint32_t nCIESamples = 471;

  if(terminate_waves)
  {
    pdf[0] /= SPECTRUM_SAMPLE_SZ;
    for(uint32_t i = 1; i < SPECTRUM_SAMPLE_SZ; ++i)
    {
      pdf[i] = 0.0f;
    }
  }
  
  for (uint32_t i = 0; i < SPECTRUM_SAMPLE_SZ; ++i)
  {
    spec[i] = (pdf[i] != 0) ? spec[i] / pdf[i] : 0.0f;
  }
  
  //float4 X = SampleCIE(lambda, a_CIE_X, lambda_min, lambda_max);
  //float4 Y = SampleCIE(lambda, a_CIE_Y, lambda_min, lambda_max);
  //float4 Z = SampleCIE(lambda, a_CIE_Z, lambda_min, lambda_max);
  float4 X,Y,Z; 
  for (uint32_t i = 0; i < SPECTRUM_SAMPLE_SZ; ++i) 
  {
    uint32_t offset = uint32_t(float(std::floor(lambda[i] + 0.5f)) - lambda_min);
  
    if (offset >= nCIESamples)
      X[i] = 0;
    else
      X[i] = a_CIE_X[offset];
  
    if (offset >= nCIESamples)
      Y[i] = 0;
    else
      Y[i] = a_CIE_Y[offset];
  
    if (offset >= nCIESamples)
      Z[i] = 0;
    else
      Z[i] = a_CIE_Z[offset];
  }

  for (uint32_t i = 0; i < SPECTRUM_SAMPLE_SZ; ++i)
  {
    X[i] *= spec[i];
    Y[i] *= spec[i];
    Z[i] *= spec[i];
  }

  float x = SpectrumAverage(X) / CIE_Y_integral;
  float y = SpectrumAverage(Y) / CIE_Y_integral;
  float z = SpectrumAverage(Z) / CIE_Y_integral;

  return float3{x ,y, z};
}

// 2° standard colorimetric observer
inline LiteMath::float3 XYZToRGB(LiteMath::float3 xyz)
{
  LiteMath::float3 rgb;
  rgb[0] = +3.240479f * xyz[0] - 1.537150f * xyz[1] - 0.498535f * xyz[2];
  rgb[1] = -0.969256f * xyz[0] + 1.875991f * xyz[1] + 0.041556f * xyz[2];
  rgb[2] = +0.055648f * xyz[0] - 0.204043f * xyz[1] + 1.057311f * xyz[2];

  return rgb;
}

std::vector<float> Get_CIE_lambda();
std::vector<float> Get_CIE_X();
std::vector<float> Get_CIE_Y();
std::vector<float> Get_CIE_Z();

#endif