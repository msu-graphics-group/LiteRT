#pragma once

#include <array>
#include <vector>
#include "include/cmaterial.h"

void SetMiPlastic(Material* material, float int_ior, float ext_ior, float4 diffuse_reflectance, float4 specular_reflectance = float4(1));

namespace mi
{
  float fresnel_diffuse_reflectance(float eta);

  struct CoatPrecomputed
  {
    std::array<float, MI_ROUGH_TRANSMITTANCE_RES> transmittance;
    float internal_reflectance;
    float specular_sampling_weight;
  };
  CoatPrecomputed fresnel_coat_precompute(float alpha, float int_ior, float ext_ior, float4 diffuse_reflectance, float4 specular_reflectance,
                                          bool is_spectral, const std::vector<float>& reflectance_spectrum);
}