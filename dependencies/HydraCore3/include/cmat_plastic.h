#pragma once
#include "cglobals.h"
#include "crandom.h"
#include "cmaterial.h"


static inline void plasticSampleAndEval(const Material* a_materials, float4 a_reflSpec, float4 rands,
                                        float3 v, float3 n, float2 tc, BsdfSample* pRes, const float* transmittance)
{
  const float alpha         = a_materials[0].data[PLASTIC_ROUGHNESS];
  const float eta           = a_materials[0].data[PLASTIC_IOR_RATIO];
  const float spec_weight   = a_materials[0].data[PLASTIC_SPEC_SAMPLE_WEIGHT];
  const uint  nonlinear     = a_materials[0].nonlinear;
  const float internal_refl = a_materials[0].data[PLASTIC_PRECOMP_REFLECTANCE];
  const float2 alpha2       = {alpha, alpha};

  float3 s = n;
  float3 t = n;
  CoordinateSystemV2(n, &s, &t);
  
  const float3 wi = float3(dot(v, s), dot(v, t), dot(v, n));
  if(wi.z <= 0)
    return;

  const float cos_theta_i = std::max(wi.z, EPSILON_32);

  // float t_i = lerp_gather(transmittance, cos_theta_i, MI_ROUGH_TRANSMITTANCE_RES);
  float t_i = 0.0f;
  {
    float x = cos_theta_i;
    x *= float(MI_ROUGH_TRANSMITTANCE_RES - 1);
    uint32_t index = std::min(uint32_t(x), uint32_t(MI_ROUGH_TRANSMITTANCE_RES - 2));

    float v0 = transmittance[index];
    float v1 = transmittance[index + 1];

    t_i = lerp(v0, v1, x - float(index));
  }

  float prob_specular = (1.f - t_i) * spec_weight;
  float prob_diffuse  = t_i * (1.f - spec_weight);

  if(prob_diffuse != 0.0f && prob_specular != 0.0f)
  {
    prob_specular = prob_specular / (prob_specular + prob_diffuse);
    prob_diffuse  = 1.f - prob_specular;
  }
  else
  {
    prob_diffuse  = 1.0f;
    prob_specular = 0.0f;
  }

  const bool sample_specular = rands.z < prob_specular;
  const bool sample_diffuse  = !sample_specular;

  float3 wo {0.0f, 0.0f, 0.0f};
  if(sample_specular)
  {
    const float3 wm = to_float3(sample_visible_normal(wi, {rands.x, rands.y}, alpha2));
    wo = reflect((-1.0f) * wi, wm);
  }

  if(sample_diffuse)
  {
    wo = square_to_cosine_hemisphere({rands.x, rands.y});
  }

  if(cos_theta_i * wo.z <= 0)
  {
    return;
  }

  const float cos_theta_o = std::max(wo.z, EPSILON_32);

  float3 H = normalize(wo + wi);
  float  D = eval_microfacet(H, alpha2, 1);

  float pdf = D * smith_g1(wi, H, alpha2) / (4.f * cos_theta_i);
  pdf *= prob_specular;
  pdf += prob_diffuse * INV_PI * cos_theta_o;

  const float F = FrDielectric(dot(wi, H), eta); 
  float G = microfacet_G(wi, wo, H, alpha2);
  float val = F * D * G / (4.f * cos_theta_i * cos_theta_o);

  // float t_o = lerp_gather(transmittance, cos_theta_o, MI_ROUGH_TRANSMITTANCE_RES); 
  float t_o = 0.0f;
  {
    float x = cos_theta_o;
    x *= float(MI_ROUGH_TRANSMITTANCE_RES - 1);
    uint32_t index = std::min(uint32_t(x), uint32_t(MI_ROUGH_TRANSMITTANCE_RES - 2));

    float v0 = transmittance[index];
    float v1 = transmittance[index + 1];

    t_o = lerp(v0, v1, x - float(index));
  }

  float4 diffuse = a_reflSpec / (1.f - (nonlinear > 0 ? (a_reflSpec * internal_refl) : float4(internal_refl)));
  const float inv_eta_2 = 1.f / (eta * eta);

  pRes->dir   = normalize(wo.x * s + wo.y * t + wo.z * n);
  pRes->val   = float4(val) + diffuse * (INV_PI * inv_eta_2 * /*cos_theta_o **/ t_i * t_o );
  pRes->pdf   = pdf;
  pRes->flags = RAY_FLAG_HAS_NON_SPEC;
}


static void plasticEval(const Material* a_materials, float4 a_reflSpec, float3 l, float3 v, float3 n, float2 tc, 
                        BsdfEval* pRes, const float* transmittance)
{
  const float alpha     = a_materials[0].data[PLASTIC_ROUGHNESS];
  const float eta       = a_materials[0].data[PLASTIC_IOR_RATIO];
  const float spec_weight = a_materials[0].data[PLASTIC_SPEC_SAMPLE_WEIGHT];
  const uint  nonlinear   = a_materials[0].nonlinear;
  const float internal_refl = a_materials[0].data[PLASTIC_PRECOMP_REFLECTANCE];

  const float2 alpha2 {alpha, alpha};
  
  float3 s = n;
  float3 t = n;
  CoordinateSystemV2(n, &s, &t);
  
  const float3 wo = float3(dot(l, s), dot(l, t), dot(l, n));
  const float3 wi = float3(dot(v, s), dot(v, t), dot(v, n));
  if(wi.z * wo.z <= 0)
  {
    return;
  }
  const float cos_theta_i = std::max(wi.z, EPSILON_32);
  const float cos_theta_o = std::max(wo.z, EPSILON_32);

  // float t_i = lerp_gather(transmittance, cos_theta_i, MI_ROUGH_TRANSMITTANCE_RES);
  float t_i = 0.0f;
  {
    float x = cos_theta_i;
    x *= float(MI_ROUGH_TRANSMITTANCE_RES - 1);
    uint32_t index = std::min(uint32_t(x), uint32_t(MI_ROUGH_TRANSMITTANCE_RES - 2));

    float v0 = transmittance[index];
    float v1 = transmittance[index + 1];

    t_i = lerp(v0, v1, x - float(index));
  }

  float prob_specular = (1.f - t_i) * spec_weight;
  float prob_diffuse  = t_i * (1.f - spec_weight);

  if(prob_diffuse != 0.0f && prob_specular != 0.0f)
  {
    prob_specular = prob_specular / (prob_specular + prob_diffuse);
    prob_diffuse  = 1.f - prob_specular;
  }
  else
  {
    prob_diffuse  = 1.0f;
    prob_specular = 0.0f;
  }

 
  float3 H = normalize(wo + wi);
  float  D = eval_microfacet(H, alpha2, 1);
  float smith_g1_wi = smith_g1(wi, H, alpha2);

  float pdf = D * smith_g1_wi / (4.f * cos_theta_i);
  pdf *= prob_specular;
  pdf += prob_diffuse * INV_PI * cos_theta_o;


  const float F = FrDielectric(dot(wi, H), eta); 
  float G = smith_g1(wo, H, alpha2) * smith_g1_wi;
  float val = F * D * G / (4.f * cos_theta_i * cos_theta_o);

  // float t_o = lerp_gather(transmittance, cos_theta_o, MI_ROUGH_TRANSMITTANCE_RES); 
  float t_o = 0.0f;
  {
    float x = cos_theta_o;
    x *= float(MI_ROUGH_TRANSMITTANCE_RES - 1);
    uint32_t index = std::min(uint32_t(x), uint32_t(MI_ROUGH_TRANSMITTANCE_RES - 2));

    float v0 = transmittance[index];
    float v1 = transmittance[index + 1];

    t_o = lerp(v0, v1, x - float(index));
  }

  float4 diffuse = a_reflSpec / (1.f - (nonlinear > 0 ? (a_reflSpec * internal_refl) : float4(internal_refl)));
  const float inv_eta_2 = 1.f / (eta * eta);

  pRes->val   = float4(val) + diffuse * (INV_PI * inv_eta_2 * /*cos_theta_o **/ t_i * t_o );
  pRes->pdf   = pdf;
}