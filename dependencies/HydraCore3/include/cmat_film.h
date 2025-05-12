#pragma once
#include "cglobals.h"
#include "crandom.h"
#include "cmaterial.h"
#include "../spectrum.h"
#include "airy_reflectance.h"
#include <iostream>

static inline void filmSmoothSampleAndEval(const Material* a_materials, 
        const float extIOR, const complex filmIOR, const complex intIOR, const float thickness, const float4 a_wavelengths, const float _extIOR,
        float4 rands, float3 v, float3 n, float2 tc, BsdfSample* pRes, const float* precomputed_data, const bool spectral_mode, const bool precomputed)
{
  const uint transparFlag = as_uint((a_materials[0].data[FILM_TRANSPARENT]));
  bool reversed = false;
  uint32_t refl_offset;
  uint32_t refr_offset;
  if ((pRes->flags & RAY_FLAG_HAS_INV_NORMAL) != 0) // inside of object
  {
    n = -1 * n;
  }
  if (dot(n, v) < 0.f && intIOR.im < 0.001f)
  {
    reversed = true;
    refl_offset = FILM_ANGLE_RES * 2;
    refr_offset = FILM_ANGLE_RES * 3;
  }
  else
  {
    refl_offset = 0;
    refr_offset = FILM_ANGLE_RES;
  }

  float3 s, t = n;
  CoordinateSystemV2(n, &s, &t);
  float3 wi = float3(dot(v, s), dot(v, t), dot(v, n));

  float cosThetaI = clamp(std::abs(wi.z), 0.0001f, 1.0f);
  float ior = intIOR.re / extIOR;
  float4 R = float4(0.0f), T = float4(0.0f);

  if (spectral_mode)
  {
    if (precomputed)
    {
      float w = clamp((a_wavelengths[0] - LAMBDA_MIN) / (LAMBDA_MAX - LAMBDA_MIN), 0.f, 1.f);
      float theta = clamp(std::acos(cosThetaI) * 2.f / M_PI, 0.f, 1.f);
      w *= FILM_LENGTH_RES - 1;
      theta *= FILM_ANGLE_RES - 1;
      uint32_t index1 = std::min(uint32_t(w), uint32_t(FILM_LENGTH_RES - 2));
      uint32_t index2 = std::min(uint32_t(theta), uint32_t(FILM_ANGLE_RES - 2));

      float alpha = w - float(index1);
      float beta = theta - float(index2);

      float v0 = lerp(precomputed_data[refl_offset * FILM_LENGTH_RES + index1 * FILM_ANGLE_RES + index2], precomputed_data[refl_offset * FILM_LENGTH_RES + (index1 + 1) * FILM_ANGLE_RES + index2], alpha);
      float v1 = lerp(precomputed_data[refl_offset * FILM_LENGTH_RES + index1 * FILM_ANGLE_RES + index2 + 1], precomputed_data[refl_offset * FILM_LENGTH_RES + (index1 + 1) * FILM_ANGLE_RES + index2 + 1], alpha);
      R[0] = lerp(v0, v1, beta);

      v0 = lerp(precomputed_data[refr_offset * FILM_LENGTH_RES + index1 * FILM_ANGLE_RES + index2], precomputed_data[refr_offset * FILM_LENGTH_RES + (index1 + 1) * FILM_ANGLE_RES + index2], alpha);
      v1 = lerp(precomputed_data[refr_offset * FILM_LENGTH_RES + index1 * FILM_ANGLE_RES + index2 + 1], precomputed_data[refr_offset * FILM_LENGTH_RES + (index1 + 1) * FILM_ANGLE_RES + index2 + 1], alpha);
      T[0] = lerp(v0, v1, beta);
    }
    else 
    {
      if (!reversed)
      {
        auto result = FrFilm(cosThetaI, extIOR, filmIOR, intIOR, thickness, a_wavelengths[0]);
        R[0] = result.refl;
        T[0] = result.refr;
      }
      else
      {
        auto result = FrFilm(cosThetaI, intIOR, filmIOR, extIOR, thickness, a_wavelengths[0]);
        R[0] = result.refl;
        T[0] = result.refr;
      }
    }
  }
  else
  {
    float theta = clamp(std::acos(cosThetaI) * 2.f / M_PI, 0.f, 1.f);
    theta *= FILM_ANGLE_RES - 1;
    if (as_uint((a_materials[0].data[FILM_THICKNESS_MAP])) == 1u)
    {
      float thickness_min = a_materials[0].data[FILM_THICKNESS_MIN];
      float thickness_max = a_materials[0].data[FILM_THICKNESS_MAX];
      float t = clamp((thickness - thickness_min) / (thickness_max - thickness_min), 0.f, 1.f);
      t *= FILM_THICKNESS_RES - 1;
      uint32_t index1 = std::min(uint32_t(t), uint32_t(FILM_THICKNESS_RES - 2));
      uint32_t index2 = std::min(uint32_t(theta), uint32_t(FILM_ANGLE_RES - 2));

      float alpha = t - float(index1);
      float beta = theta - float(index2);

      uint a = (refl_offset * FILM_THICKNESS_RES + index1 * FILM_ANGLE_RES + index2) * 3;
      uint b = (refl_offset * FILM_THICKNESS_RES + (index1 + 1) * FILM_ANGLE_RES + index2) * 3;
      uint c = (refl_offset * FILM_THICKNESS_RES + index1 * FILM_ANGLE_RES + index2 + 1) * 3;
      uint d = (refl_offset * FILM_THICKNESS_RES + (index1 + 1) * FILM_ANGLE_RES + index2 + 1) * 3;

      float v0 = lerp(precomputed_data[a], precomputed_data[b], alpha);
      float v1 = lerp(precomputed_data[c], precomputed_data[d], alpha);
      R[0] = lerp(v0, v1, beta);

      v0 = lerp(precomputed_data[a + 1], precomputed_data[b + 1], alpha);
      v1 = lerp(precomputed_data[c + 1], precomputed_data[d + 1], alpha);
      R[1] = lerp(v0, v1, beta);

      v0 = lerp(precomputed_data[a + 2], precomputed_data[b + 2], alpha);
      v1 = lerp(precomputed_data[c + 2], precomputed_data[d + 2], alpha);
      R[2] = lerp(v0, v1, beta);

      a = (refr_offset * FILM_THICKNESS_RES + index1 * FILM_ANGLE_RES + index2) * 3;
      b = (refr_offset * FILM_THICKNESS_RES + (index1 + 1) * FILM_ANGLE_RES + index2) * 3;
      c = (refr_offset * FILM_THICKNESS_RES + index1 * FILM_ANGLE_RES + index2 + 1) * 3;
      d = (refr_offset * FILM_THICKNESS_RES + (index1 + 1) * FILM_ANGLE_RES + index2 + 1) * 3;

      v0 = lerp(precomputed_data[a], precomputed_data[b], alpha);
      v1 = lerp(precomputed_data[c], precomputed_data[d], alpha);
      T[0] = lerp(v0, v1, beta);

      v0 = lerp(precomputed_data[a + 1], precomputed_data[b + 1], alpha);
      v1 = lerp(precomputed_data[c + 1], precomputed_data[d + 1], alpha);
      T[1] = lerp(v0, v1, beta);

      v0 = lerp(precomputed_data[a + 2], precomputed_data[b + 2], alpha);
      v1 = lerp(precomputed_data[c + 2], precomputed_data[d + 2], alpha);
      T[2] = lerp(v0, v1, beta);
    }
    else
    {
      uint32_t index = std::min(uint32_t(theta), uint32_t(FILM_ANGLE_RES - 2));

      float alpha = theta - float(index);

      R[0] = lerp(precomputed_data[(refl_offset + index) * 3], precomputed_data[(refl_offset + index + 1) * 3], alpha);
      R[1] = lerp(precomputed_data[(refl_offset + index) * 3 + 1], precomputed_data[(refl_offset + index + 1) * 3 + 1], alpha);
      R[2] = lerp(precomputed_data[(refl_offset + index) * 3 + 2], precomputed_data[(refl_offset + index + 1) * 3 + 2], alpha);

      T[0] = lerp(precomputed_data[(refr_offset + index) * 3], precomputed_data[(refr_offset + index + 1) * 3], alpha);
      T[1] = lerp(precomputed_data[(refr_offset + index) * 3 + 1], precomputed_data[(refr_offset + index + 1) * 3 + 1], alpha);
      T[2] = lerp(precomputed_data[(refr_offset + index) * 3 + 2], precomputed_data[(refr_offset + index + 1) * 3 + 2], alpha);
    }
  }

  if (intIOR.im > 0.001f || transparFlag == 0)
  {
    float3 wo = float3(-wi.x, -wi.y, wi.z);
    pRes->val = R;
    pRes->pdf = 1.f;
    pRes->dir = normalize(wo.x * s + wo.y * t + wo.z * n);
    pRes->flags |= RAY_EVENT_S;
    pRes->ior = _extIOR;
  }
  else
  {
    if (rands.x * (sum(R) + sum(T)) < sum(R))
    {
      float3 wo = float3(-wi.x, -wi.y, wi.z);
      pRes->val = R;
      pRes->pdf = sum(R) / (sum(R) + sum(T));
      pRes->dir = normalize(wo.x * s + wo.y * t + wo.z * n);
      pRes->flags |= RAY_EVENT_S;
      pRes->ior = _extIOR;
    }
    else
    {
      float4 fr = FrDielectricDetailedV2(wi.z, ior);
      const float cosThetaT = fr.y;
      const float eta_ti = fr.w;  

      float3 wo = refract(wi, cosThetaT, eta_ti);
      pRes->val = T;
      pRes->pdf = sum(T) / (sum(R) + sum(T));
      pRes->dir = normalize(wo.x * s + wo.y * t + wo.z * n);
      pRes->flags |= (RAY_EVENT_S | RAY_EVENT_T);
      pRes->ior = (_extIOR == intIOR.re) ? extIOR : intIOR.re;
    }
  }

  pRes->val /= std::max(std::abs(dot(pRes->dir, n)), 1e-6f);
}

static inline void filmRoughSampleAndEval(const Material* a_materials, 
        const float extIOR, const complex filmIOR, const complex intIOR, const float thickness, const float4 a_wavelengths, const float _extIOR,
        float4 rands, float3 v, float3 n, float2 tc, float3 alpha_tex, BsdfSample* pRes, const float* precomputed_data, const bool spectral_mode, const bool precomputed)
{
  const uint transparFlag = as_uint(a_materials[0].data[FILM_TRANSPARENT]);
  bool reversed = false;
  uint32_t refl_offset;
  uint32_t refr_offset;
  if ((pRes->flags & RAY_FLAG_HAS_INV_NORMAL) != 0) // inside of object
  {
    n = -1 * n;
  }

  if (dot(v, n) < 0.f && intIOR.im < 0.001f)
  {
    reversed = true;
    refl_offset = FILM_ANGLE_RES * 2;
    refr_offset = FILM_ANGLE_RES * 3;
  }
  else
  {
    refl_offset = 0;
    refr_offset = FILM_ANGLE_RES;
  }

  const float2 alpha = float2(min(a_materials[0].data[FILM_ROUGH_V], alpha_tex.x), 
                              min(a_materials[0].data[FILM_ROUGH_U], alpha_tex.y));

  float3 s, t = n;
  CoordinateSystemV2(n, &s, &t);
  float3 wi = float3(dot(v, s), dot(v, t), dot(v, n));

  float ior = intIOR.re / extIOR;
  if (reversed)
  {
    wi = -1 * wi;
    ior = 1.f / ior;
  }

  const float3 wm = trSample(wi, float2(rands.x, rands.y), alpha);

  float cosThetaI = clamp(std::abs(dot(wi, wm)), 0.00001f, 1.0f);
  float4 R = float4(0.0f), T = float4(0.0f);

  if (spectral_mode)
  {
    if (precomputed)
    {
      float w = clamp((a_wavelengths[0] - LAMBDA_MIN) / (LAMBDA_MAX - LAMBDA_MIN), 0.f, 1.f);
      float theta = clamp(std::acos(cosThetaI) * 2.f / M_PI, 0.f, 1.f);
      w *= FILM_LENGTH_RES - 1;
      theta *= FILM_ANGLE_RES - 1;
      uint32_t index1 = std::min(uint32_t(w), uint32_t(FILM_LENGTH_RES - 2));
      uint32_t index2 = std::min(uint32_t(theta), uint32_t(FILM_ANGLE_RES - 2));

      float alpha = w - float(index1);
      float beta = theta - float(index2);

      float v0 = lerp(precomputed_data[refl_offset * FILM_LENGTH_RES + index1 * FILM_ANGLE_RES + index2], precomputed_data[refl_offset * FILM_LENGTH_RES + (index1 + 1) * FILM_ANGLE_RES + index2], alpha);
      float v1 = lerp(precomputed_data[refl_offset * FILM_LENGTH_RES + index1 * FILM_ANGLE_RES + index2 + 1], precomputed_data[refl_offset * FILM_LENGTH_RES + (index1 + 1) * FILM_ANGLE_RES + index2 + 1], alpha);
      R[0] = lerp(v0, v1, beta);

      v0 = lerp(precomputed_data[refr_offset * FILM_LENGTH_RES + index1 * FILM_ANGLE_RES + index2], precomputed_data[refr_offset * FILM_LENGTH_RES + (index1 + 1) * FILM_ANGLE_RES + index2], alpha);
      v1 = lerp(precomputed_data[refr_offset * FILM_LENGTH_RES + index1 * FILM_ANGLE_RES + index2 + 1], precomputed_data[refr_offset * FILM_LENGTH_RES + (index1 + 1) * FILM_ANGLE_RES + index2 + 1], alpha);
      T[0] = lerp(v0, v1, beta);
    }
    else 
    {
      if (!reversed)
      {
        auto result = FrFilm(cosThetaI, extIOR, filmIOR, intIOR, thickness, a_wavelengths[0]);
        R[0] = result.refl;
        T[0] = result.refr;
      }
      else
      {
        auto result = FrFilm(cosThetaI, intIOR, filmIOR, extIOR, thickness, a_wavelengths[0]);
        R[0] = result.refl;
        T[0] = result.refr;
      }
    }
  }
  else
  {
    float theta = clamp(std::acos(cosThetaI) * 2.f / M_PI, 0.f, 1.f);
    theta *= FILM_ANGLE_RES - 1;
    if (as_uint((a_materials[0].data[FILM_THICKNESS_MAP])) == 1u)
    {
      float thickness_min = a_materials[0].data[FILM_THICKNESS_MIN];
      float thickness_max = a_materials[0].data[FILM_THICKNESS_MAX];
      float t = clamp((thickness - thickness_min) / (thickness_max - thickness_min), 0.f, 1.f);
      t *= FILM_THICKNESS_RES - 1;
      uint32_t index1 = std::min(uint32_t(t), uint32_t(FILM_THICKNESS_RES - 2));
      uint32_t index2 = std::min(uint32_t(theta), uint32_t(FILM_ANGLE_RES - 2));

      float alpha = t - float(index1);
      float beta = theta - float(index2);

      uint a = (refl_offset * FILM_THICKNESS_RES + index1 * FILM_ANGLE_RES + index2) * 3;
      uint b = (refl_offset * FILM_THICKNESS_RES + (index1 + 1) * FILM_ANGLE_RES + index2) * 3;
      uint c = (refl_offset * FILM_THICKNESS_RES + index1 * FILM_ANGLE_RES + index2 + 1) * 3;
      uint d = (refl_offset * FILM_THICKNESS_RES + (index1 + 1) * FILM_ANGLE_RES + index2 + 1) * 3;

      float v0 = lerp(precomputed_data[a], precomputed_data[b], alpha);
      float v1 = lerp(precomputed_data[c], precomputed_data[d], alpha);
      R[0] = lerp(v0, v1, beta);

      v0 = lerp(precomputed_data[a + 1], precomputed_data[b + 1], alpha);
      v1 = lerp(precomputed_data[c + 1], precomputed_data[d + 1], alpha);
      R[1] = lerp(v0, v1, beta);

      v0 = lerp(precomputed_data[a + 2], precomputed_data[b + 2], alpha);
      v1 = lerp(precomputed_data[c + 2], precomputed_data[d + 2], alpha);
      R[2] = lerp(v0, v1, beta);

      a = (refr_offset * FILM_THICKNESS_RES + index1 * FILM_ANGLE_RES + index2) * 3;
      b = (refr_offset * FILM_THICKNESS_RES + (index1 + 1) * FILM_ANGLE_RES + index2) * 3;
      c = (refr_offset * FILM_THICKNESS_RES + index1 * FILM_ANGLE_RES + index2 + 1) * 3;
      d = (refr_offset * FILM_THICKNESS_RES + (index1 + 1) * FILM_ANGLE_RES + index2 + 1) * 3;

      v0 = lerp(precomputed_data[a], precomputed_data[b], alpha);
      v1 = lerp(precomputed_data[c], precomputed_data[d], alpha);
      T[0] = lerp(v0, v1, beta);

      v0 = lerp(precomputed_data[a + 1], precomputed_data[b + 1], alpha);
      v1 = lerp(precomputed_data[c + 1], precomputed_data[d + 1], alpha);
      T[1] = lerp(v0, v1, beta);

      v0 = lerp(precomputed_data[a + 2], precomputed_data[b + 2], alpha);
      v1 = lerp(precomputed_data[c + 2], precomputed_data[d + 2], alpha);
      T[2] = lerp(v0, v1, beta);
    }
    else
    {
      uint32_t index = std::min(uint32_t(theta), uint32_t(FILM_ANGLE_RES - 2));

      float alpha = theta - float(index);

      R[0] = lerp(precomputed_data[(refl_offset + index) * 3], precomputed_data[(refl_offset + index + 1) * 3], alpha);
      R[1] = lerp(precomputed_data[(refl_offset + index) * 3 + 1], precomputed_data[(refl_offset + index + 1) * 3 + 1], alpha);
      R[2] = lerp(precomputed_data[(refl_offset + index) * 3 + 2], precomputed_data[(refl_offset + index + 1) * 3 + 2], alpha);

      T[0] = lerp(precomputed_data[(refr_offset + index) * 3], precomputed_data[(refr_offset + index + 1) * 3], alpha);
      T[1] = lerp(precomputed_data[(refr_offset + index) * 3 + 1], precomputed_data[(refr_offset + index + 1) * 3 + 1], alpha);
      T[2] = lerp(precomputed_data[(refr_offset + index) * 3 + 2], precomputed_data[(refr_offset + index + 1) * 3 + 2], alpha);
    }
  }
  
  if (intIOR.im > 0.001f || transparFlag == 0)
  {
    float3 wo = reflect((-1.0f) * wi, wm);
    if (wi.z < 0.f || wo.z <= 0.f)
    {
      return;
    }
    const float cos_theta_i = std::max(wi.z, EPSILON_32);
    const float cos_theta_o = std::max(wo.z, EPSILON_32);
    pRes->pdf = trPDF(wi, wm, alpha) / (4.0f * std::abs(dot(wi, wm)));
    pRes->val = trD(wm, alpha) * microfacet_G(wi, wo, wm, alpha) * R / (4.0f * cos_theta_i * cos_theta_o);
    if (reversed)
    {
      wo = -1 * wo;
    }
    pRes->dir = normalize(wo.x * s + wo.y * t + wo.z * n);
    pRes->flags = RAY_FLAG_HAS_NON_SPEC;
    pRes->ior = _extIOR;
  }
  else
  {
    if (rands.w * (sum(R) + sum(T)) < sum(R))
    {
      float3 wo = reflect((-1.0f) * wi, wm);
      if (wi.z < 0.f || wo.z <= 0.f)
      {
        return;
      }
      const float cos_theta_i = std::max(wi.z, EPSILON_32);
      const float cos_theta_o = std::max(wo.z, EPSILON_32);
      pRes->pdf = trPDF(wi, wm, alpha) / (4.0f * std::abs(dot(wi, wm))) * sum(R) / (sum(R) + sum(T));
      pRes->val = trD(wm, alpha) * microfacet_G(wi, wo, wm, alpha) * R / (4.0f * cos_theta_i * cos_theta_o);
      if (reversed)
      {
        wo = -1 * wo;
      }
      pRes->dir = normalize(wo.x * s + wo.y * t + wo.z * n);
      pRes->flags = RAY_FLAG_HAS_NON_SPEC;
      pRes->ior = _extIOR;
    }
    else
    {
      float4 fr = FrDielectricDetailedV2(dot(wi, wm), ior);
      const float cosThetaT = fr.y;
      const float eta_it = fr.z;
      const float eta_ti = fr.w;  

      float3 ws, wt;
      CoordinateSystemV2(wm, &ws, &wt);
      const float3 local_wi = {dot(ws, wi), dot(wt, wi), dot(wm, wi)};
      const float3 local_wo = refract(local_wi, cosThetaT, eta_ti);
      float3 wo = local_wo.x * ws + local_wo.y * wt + local_wo.z * wm;
      if (wo.z > 0.f)
      {
        return;
      }
      const float cos_theta_i = std::max(wi.z, EPSILON_32);
      const float cos_theta_o = std::min(wo.z, -EPSILON_32);
      if (std::abs(eta_it - 1.f) <= 1e-6f)
      {
        pRes->pdf = trPDF(wi, wm, alpha) / (4.0f * std::abs(dot(wi, wm))) * sum(T) / (sum(R) + sum(T));
        pRes->val = trD(wm, alpha) * microfacet_G(wi, wo, wm, alpha) * T / (4.0f * -cos_theta_i * cos_theta_o);
      }
      else
      {
        float denom = sqr(dot(wo, wm) + dot(wi, wm) / eta_it);
        float dwm_dwi = std::abs(dot(wo, wm)) / denom;
        pRes->pdf = trPDF(wi, wm, alpha) * dwm_dwi * sum(T) / (sum(R) + sum(T));
        pRes->val = trD(wm, alpha) * microfacet_G(wi, wo, wm, alpha) * T * std::abs(dot(wi, wm) * dot(wo, wm) / (cos_theta_i * cos_theta_o * denom));
      }
      if (reversed)
      {
        wo = -1 * wo;
      }
      pRes->dir = normalize(wo.x * s + wo.y * t + wo.z * n);
      pRes->flags = RAY_FLAG_HAS_NON_SPEC;
      pRes->ior = (_extIOR == intIOR.re) ? extIOR : intIOR.re;
    }
  }
}


static void filmRoughEval(const Material* a_materials, 
        const float extIOR, const complex filmIOR, const complex intIOR, const float thickness, const float4 a_wavelengths, float3 l, float3 v, float3 n, float2 tc,
        float3 alpha_tex, BsdfEval* pRes, const float* precomputed_data, const bool spectral_mode, const bool precomputed)
{
  if (intIOR.im < 0.001f)
  {
    return;
  }

  uint32_t refl_offset;
  uint32_t refr_offset;

  bool reversed = false;
  if (dot(v, n) < 0.f && intIOR.im < 0.001f)
  {
    reversed = true;
    refl_offset = FILM_ANGLE_RES * 2;
    refr_offset = FILM_ANGLE_RES * 3;
  }
  else
  {
    refl_offset = 0;
    refr_offset = FILM_ANGLE_RES;
  }

  const float2 alpha = float2(std::min(a_materials[0].data[FILM_ROUGH_V], alpha_tex.x), 
                              std::min(a_materials[0].data[FILM_ROUGH_U], alpha_tex.y));

  float3 s, t = n;
  CoordinateSystemV2(n, &s, &t);
  const float3 wo = float3(dot(l, s), dot(l, t), dot(l, n));
  const float3 wi = float3(dot(v, s), dot(v, t), dot(v, n));
  const float3 wm = normalize(wo + wi);

  if (wi.z * wo.z < 0.f)
  {
    return;
  }

  float ior = intIOR.re / extIOR;
  if (reversed)
  {
    ior = 1.f / ior;
  }

  float cosThetaI = clamp(std::abs(dot(wo, wm)), 0.00001f, 1.0f);
  
  float4 R = float4(0.0f);
  if (spectral_mode)
  {
    if (precomputed)
    {
      float w = clamp((a_wavelengths[0] - LAMBDA_MIN) / (LAMBDA_MAX - LAMBDA_MIN), 0.f, 1.f);
      float theta = clamp(std::acos(cosThetaI) * 2.f / M_PI, 0.f, 1.f);
      //result.refl = lerp_gather_2d(reflectance, w, theta, FILM_LENGTH_RES, FILM_ANGLE_RES);
      w *= FILM_LENGTH_RES - 1;
      theta *= FILM_ANGLE_RES - 1;
      uint32_t index1 = std::min(uint32_t(w), uint32_t(FILM_LENGTH_RES - 2));
      uint32_t index2 = std::min(uint32_t(theta), uint32_t(FILM_ANGLE_RES - 2));

      float alpha = w - float(index1);
      float beta = theta - float(index2);

      float v0 = lerp(precomputed_data[refl_offset * FILM_LENGTH_RES + index1 * FILM_ANGLE_RES + index2], precomputed_data[refl_offset * FILM_LENGTH_RES + (index1 + 1) * FILM_ANGLE_RES + index2], alpha);
      float v1 = lerp(precomputed_data[refl_offset * FILM_LENGTH_RES + index1 * FILM_ANGLE_RES + index2 + 1], precomputed_data[refl_offset * FILM_LENGTH_RES + (index1 + 1) * FILM_ANGLE_RES + index2 + 1], alpha);
      R[0] = lerp(v0, v1, beta);
    }
    else
    {
      if (!reversed)
      {
        R[0] = FrFilmRefl(cosThetaI, extIOR, filmIOR, intIOR, thickness, a_wavelengths[0]);
      }
      else
      {
        R[0] = FrFilmRefl(cosThetaI, intIOR, filmIOR, extIOR, thickness, a_wavelengths[0]); 
      }
    }
  }
  else
  {
    float theta = clamp(std::acos(cosThetaI) * 2.f / M_PI, 0.f, 1.f);
    theta *= FILM_ANGLE_RES - 1;
    if (as_uint((a_materials[0].data[FILM_THICKNESS_MAP])) == 1u)
    {
      float thickness_min = a_materials[0].data[FILM_THICKNESS_MIN];
      float thickness_max = a_materials[0].data[FILM_THICKNESS_MAX];
      float t = clamp((thickness - thickness_min) / (thickness_max - thickness_min), 0.f, 1.f);
      t *= FILM_THICKNESS_RES - 1;
      uint32_t index1 = std::min(uint32_t(t), uint32_t(FILM_THICKNESS_RES - 2));
      uint32_t index2 = std::min(uint32_t(theta), uint32_t(FILM_ANGLE_RES - 2));

      float alpha = t - float(index1);
      float beta = theta - float(index2);

      uint a = (refl_offset * FILM_THICKNESS_RES + index1 * FILM_ANGLE_RES + index2) * 3;
      uint b = (refl_offset * FILM_THICKNESS_RES + (index1 + 1) * FILM_ANGLE_RES + index2) * 3;
      uint c = (refl_offset * FILM_THICKNESS_RES + index1 * FILM_ANGLE_RES + index2 + 1) * 3;
      uint d = (refl_offset * FILM_THICKNESS_RES + (index1 + 1) * FILM_ANGLE_RES + index2 + 1) * 3;

      float v0 = lerp(precomputed_data[a], precomputed_data[b], alpha);
      float v1 = lerp(precomputed_data[c], precomputed_data[d], alpha);
      R[0] = lerp(v0, v1, beta);

      v0 = lerp(precomputed_data[a + 1], precomputed_data[b + 1], alpha);
      v1 = lerp(precomputed_data[c + 1], precomputed_data[d + 1], alpha);
      R[1] = lerp(v0, v1, beta);

      v0 = lerp(precomputed_data[a + 2], precomputed_data[b + 2], alpha);
      v1 = lerp(precomputed_data[c + 2], precomputed_data[d + 2], alpha);
      R[2] = lerp(v0, v1, beta);
    }
    else
    {
      uint32_t index = std::min(uint32_t(theta), uint32_t(FILM_ANGLE_RES - 2));

      float alpha = theta - float(index);

      R[0] = lerp(precomputed_data[(refl_offset + index) * 3], precomputed_data[(refl_offset + index + 1) * 3], alpha);
      R[1] = lerp(precomputed_data[(refl_offset + index) * 3 + 1], precomputed_data[(refl_offset + index + 1) * 3 + 1], alpha);
      R[2] = lerp(precomputed_data[(refl_offset + index) * 3 + 2], precomputed_data[(refl_offset + index + 1) * 3 + 2], alpha);
    }
  }

  const float cos_theta_i = std::max(wi.z, EPSILON_32);
  const float cos_theta_o = std::max(wo.z, EPSILON_32);

  float D = eval_microfacet(wm, alpha, 1);
  float G = microfacet_G(wi, wo, wm, alpha);
  pRes->pdf = trPDF(wi, wm, alpha) / (4.0f * std::abs(dot(wi, wm)));
  pRes->val = trD(wm, alpha) * microfacet_G(wi, wo, wm, alpha) * R / (4.0f * cos_theta_i * cos_theta_o);
}