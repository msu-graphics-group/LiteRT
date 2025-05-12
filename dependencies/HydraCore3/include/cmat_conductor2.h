#pragma once
#include "cglobals.h"
#include "crandom.h"
#include "cmaterial.h"
#include "../spectrum.h"


// mitsuba impl. based variant using "sample visible normals" from
// Eric Heitz and Eugene D'Eon. Importance sampling microfacet-based bsdfs using the distribution of visible normals. Computer Graphics Forum, June 2014.

static inline void conductorRoughSampleAndEval_V2(const Material* a_materials, const float4 etaSpec, const float4 kSpec, 
                                               float4 rands, float3 v, float3 n, float2 tc, float3 alpha_tex, 
                                               BsdfSample* pRes)
{
  if(v.z == 0)
    return;

  const float2 alpha = float2(min(a_materials[0].data[CONDUCTOR_ROUGH_U], alpha_tex.x), 
                              min(a_materials[0].data[CONDUCTOR_ROUGH_V], alpha_tex.y));

  float3 s, t = n;
  CoordinateSystemV2(n, &s, &t);
  float3 wi = float3(dot(v, s), dot(v, t), dot(v, n));
  if(wi.z <= 0)
    return;

  const float cos_theta_i = std::max(wi.z, EPSILON_32);

  const float4 wm_pdf = sample_visible_normal(wi, {rands.x, rands.y}, alpha);
  const float3 wm = to_float3(wm_pdf);
  const float3 wo = reflect((-1.0f) * wi, wm);
  const float cos_theta_o = std::max(wo.z, EPSILON_32);
  if(wo.z <= 0 || wm_pdf.w == 0.0f) // not in the same hemisphere
  {
    return;
  }

  float G1 = smith_g1(wo, wm, alpha);
  float4 val;
  for(uint32_t i = 0; i < SPECTRUM_SAMPLE_SZ; ++i)
  {
    val[i] = G1 * FrComplexConductor(std::abs(dot(wi, wm)), complex{etaSpec[i], kSpec[i]});
  }

  pRes->dir   = normalize(wo.x * s + wo.y * t + wo.z * n);
  pRes->pdf   = wm_pdf.w / (4.0f * std::abs(dot(wo, wm)));
  pRes->flags = RAY_FLAG_HAS_NON_SPEC;

  pRes->val   = pRes->pdf * val / cos_theta_o; 
}


static void conductorRoughEval_V2(const Material* a_materials, const float4 etaSpec, const float4 kSpec, 
                               float3 l, float3 v, float3 n, float2 tc, float3 alpha_tex, 
                               BsdfEval* pRes)
{
  const float2 alpha = float2(min(a_materials[0].data[CONDUCTOR_ROUGH_U], alpha_tex.x), 
                              min(a_materials[0].data[CONDUCTOR_ROUGH_V], alpha_tex.y));

  float3 s, t = n;
  CoordinateSystemV2(n, &s, &t);

  // v = (-1.0f) * v;
  const float3 wo = float3(dot(l, s), dot(l, t), dot(l, n));
  const float3 wi = float3(dot(v, s), dot(v, t), dot(v, n));

  if(wo.z * wi.z < 0.0f)
    return;

  const float cos_theta_i = std::max(wi.z, EPSILON_32);
  const float cos_theta_o = std::max(wo.z, EPSILON_32);

  float3 H = normalize(wo + wi);
  float  D = eval_microfacet(H, alpha, 1);
  float  G = microfacet_G(wi, wo, H, alpha);
  
  const float res = D * G / (4.f * cos_theta_i * cos_theta_o);
  float4 val;
  for(uint32_t i = 0; i < SPECTRUM_SAMPLE_SZ; ++i)
  {
    val[i] = res * FrComplexConductor(std::abs(dot(wi, H)), complex{etaSpec[i], kSpec[i]});
  }

  pRes->val = val;
  pRes->pdf = D * smith_g1(wi, H, alpha) / (4.f * cos_theta_i);
}