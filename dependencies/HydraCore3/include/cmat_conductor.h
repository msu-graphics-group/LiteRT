#pragma once
#include "cglobals.h"
#include "crandom.h"
#include "cmaterial.h"
#include "../spectrum.h"


static inline void conductorSmoothSampleAndEval(const Material* a_materials, const float4 etaSpec, const float4 kSpec,
                                                float4 rands, float3 v, float3 n, float2 tc,
                                                BsdfSample* pRes)
{
  const float4 rgb_reflectance = a_materials[0].colors[CONDUCTOR_COLOR];
  const float3 pefReflDir = reflect((-1.0f)*v, n);
  const float cosThetaOut = dot(pefReflDir, n);
  float3 dir              = pefReflDir;
  float  pdf              = 1.0f;
  
  float4 val;
  for(uint32_t i = 0; i < SPECTRUM_SAMPLE_SZ; ++i)
  {
    val[i] = FrComplexConductor(cosThetaOut, complex{etaSpec[i], kSpec[i]});
    val[i] = (cosThetaOut <= 1e-6f) ? 0.0f : (val[i] / std::max(cosThetaOut, 1e-6f));  
  }
  
  pRes->val = val * rgb_reflectance; 
  pRes->dir = dir;
  pRes->pdf = pdf;
  pRes->flags = RAY_EVENT_S;
}


static void conductorSmoothEval(const Material* a_materials, float4 wavelengths, float3 l, float3 v, float3 n, float2 tc,
                                BsdfEval* pRes)
{
  for (uint32_t i = 0; i < SPECTRUM_SAMPLE_SZ; ++i) 
  {
    pRes->val.M[i] = 0.0f;
  }
  pRes->pdf = 0.0f;
}


static float conductorRoughEvalInternal(float3 wo, float3 wi, float3 wm, float2 alpha, complex ior)
{
  if(wo.z * wi.z < 0) // not in the same hemisphere
  {
    return 0.0f;
  }

  float cosTheta_o = AbsCosTheta(wo);
  float cosTheta_i = AbsCosTheta(wi);
  if (cosTheta_i == 0 || cosTheta_o == 0)
    return 0.0f;

  float F = FrComplexConductor(std::abs(dot(wo, wm)), ior);
  float val = trD(wm, alpha) * F * trG(wo, wi, alpha) / (4.0f * cosTheta_i * cosTheta_o);

  return val;
}


static inline void conductorRoughSampleAndEval(const Material* a_materials, const float4 etaSpec, const float4 kSpec, 
                                               float4 rands, float3 v, float3 n, float2 tc, float3 alpha_tex, 
                                               BsdfSample* pRes)
{
  if(v.z == 0)
    return;

  const float4 rgb_reflectance = a_materials[0].colors[CONDUCTOR_COLOR];

  const float2 alpha = float2(min(a_materials[0].data[CONDUCTOR_ROUGH_U], alpha_tex.x), 
                              min(a_materials[0].data[CONDUCTOR_ROUGH_V], alpha_tex.y));

  float3 nx, ny, nz = n;
  CoordinateSystemV2(nz, &nx, &ny);
  const float3 wo = float3(dot(v, nx), dot(v, ny), dot(v, nz));
  if(wo.z == 0)
    return;

  if(wo.z == 0)
    return;

  float3 wm = trSample(wo, float2(rands.x, rands.y), alpha);
  float3 wi = reflect((-1.0f) * wo, wm);

  if(wo.z * wi.z < 0) // not in the same hemisphere
  {
    return;
  }

  float4 val;
  for(uint32_t i = 0; i < SPECTRUM_SAMPLE_SZ; ++i)
  {
    val[i] = conductorRoughEvalInternal(wo, wi, wm, alpha, complex{etaSpec[i], kSpec[i]});
  }

  pRes->val   = val * rgb_reflectance; 
  pRes->dir   = normalize(wi.x * nx + wi.y * ny + wi.z * nz);
  pRes->pdf   = trPDF(wo, wm, alpha) / (4.0f * std::abs(dot(wo, wm)));
  pRes->flags = RAY_FLAG_HAS_NON_SPEC;
}


static void conductorRoughEval(const Material* a_materials, const float4 etaSpec, const float4 kSpec, 
                               float3 l, float3 v, float3 n, float2 tc, float3 alpha_tex, 
                               BsdfEval* pRes)
{
  const float2 alpha = float2(min(a_materials[0].data[CONDUCTOR_ROUGH_U], alpha_tex.x), 
                              min(a_materials[0].data[CONDUCTOR_ROUGH_V], alpha_tex.y));

  const float4 rgb_reflectance = a_materials[0].colors[CONDUCTOR_COLOR];

  float3 nx, ny, nz = n;
  CoordinateSystemV2(nz, &nx, &ny);

  // v = (-1.0f) * v;
  const float3 wo = float3(dot(v, nx), dot(v, ny), dot(v, nz));
  const float3 wi = float3(dot(l, nx), dot(l, ny), dot(l, nz));

  if(wo.z * wi.z < 0.0f)
    return;

  float3 wm = wo + wi;
  if (dot(wm, wm) == 0)
      return;

  wm = normalize(wm);
  float4 val;
  for(uint32_t i = 0; i < SPECTRUM_SAMPLE_SZ; ++i)
  {
    val[i] = conductorRoughEvalInternal(wo, wi, wm, alpha, complex{etaSpec[i], kSpec[i]});
  }

  pRes->val = val * rgb_reflectance;

  wm        = FaceForward(wm, float3(0.0f, 0.0f, 1.0f));
  pRes->pdf = trPDF(wo, wm, alpha) / (4.0f * std::abs(dot(wo, wm)));
}