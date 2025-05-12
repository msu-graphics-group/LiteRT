#ifndef RTC_MATERIAL
#define RTC_MATERIAL

#include "cglobals.h"
#include <algorithm>

struct BsdfSample
{
  float4 val;
  float3 dir;
  float  pdf; 
  uint   flags;
  float  ior;
};

struct BsdfEval
{
  float4 val;
  float  pdf; 
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////

enum GLTF_COMPOMENT { GLTF_COMPONENT_LAMBERT   = 1, 
                      GLTF_COMPONENT_COAT      = 2,
                      GLTF_COMPONENT_METAL     = 4,
                      GLTF_METAL_PERF_MIRROR   = 8, 
                      GLTF_COMPONENT_ORENNAYAR = 16,
                      FLAG_NMAP_INVERT_X       = 32,
                      FLAG_NMAP_INVERT_Y       = 64,
                      FLAG_NMAP_SWAP_XY        = 128,
                      FLAG_FOUR_TEXTURES       = 256,
                      FLAG_PACK_FOUR_PARAMS_IN_TEXTURE = 512,
                      FLAG_INVERT_GLOSINESS    = 1024, }; // bit fields

enum MATERIAL_TYPES { MAT_TYPE_GLTF          = 1,
                      MAT_TYPE_GLASS         = 2,
                      MAT_TYPE_CONDUCTOR     = 3,
                      MAT_TYPE_DIFFUSE       = 4,
                      MAT_TYPE_PLASTIC       = 5,
                      MAT_TYPE_BLEND         = 6,
                      MAT_TYPE_DIELECTRIC    = 7,
                      MAT_TYPE_THIN_FILM     = 8,
                      MAT_TYPE_LIGHT_SOURCE  = 0xEFFFFFFF };

enum MATERIAL_EVENT {
  RAY_EVENT_S         = 1,  ///< Indicates Specular reflection or refraction  (additionally check for RAY_EVENT_T)
  RAY_EVENT_D         = 2,  ///< Indicates Diffuse  reflection or translucent (additionally check for RAY_EVENT_T)
  RAY_EVENT_G         = 4,  ///< Indicates Glossy   reflection or refraction  (additionally check for RAY_EVENT_T)
  RAY_EVENT_T         = 8,  ///< Indicates Transparency or refraction. 
  RAY_EVENT_V         = 16, ///< Indicates Volume scattering, not used for a while
  RAY_EVENT_TOUT      = 32, ///< Indicates Transparency Outside of water or glass et c. (old RAY_IS_INSIDE_TRANSPARENT_OBJECT = 128)
  RAY_EVENT_TNINGLASS = 64
};

////////////////////////////////
// Indexes for materials
// 
// Custom for all materials

// GLTF
// The BRDF of the metallic-roughness material is a linear interpolation of a metallic BRDF and a dielectric BRDF. 
// The BRDFs **share** the parameters for roughness and base color.
// colors
static constexpr uint GLTF_COLOR_BASE             = 0;  ///< color for both lambert and emissive lights; baseColor.w store emission
static constexpr uint GLTF_COLOR_COAT             = 1;  ///< in our implementation we allow different color for coating (fresnel) and diffuse
static constexpr uint GLTF_COLOR_METAL            = 2;  ///< in our implementation we allow different color for metals and diffuse
static constexpr uint GLTF_COLOR_LAST_IND         = GLTF_COLOR_METAL;

// custom                                               
static constexpr uint GLTF_FLOAT_MI_FDR_INT       = 0; ///< ScalarFloat m_fdr_int;
static constexpr uint GLTF_FLOAT_MI_FDR_EXT       = 1; ///< ScalarFloat m_fdr_ext;
static constexpr uint GLTF_FLOAT_MI_SSW           = 2; ///< Float m_specular_sampling_weight;
static constexpr uint GLTF_FLOAT_ALPHA            = 3; ///< blend factor between dielectric and metal reflection : alpha*baseColor + (1.0f-alpha)*baseColor
static constexpr uint GLTF_FLOAT_GLOSINESS        = 4; ///< material glosiness or intensity for lights, take color from baseColor
static constexpr uint GLTF_FLOAT_IOR              = 5; ///< index of refraction for reflection Fresnel
static constexpr uint GLTF_FLOAT_ROUGH_ORENNAYAR  = 6; ///< roughness for Oren-Nayar
static constexpr uint GLTF_FLOAT_REFL_COAT        = 7; ///< reflection magnitude for coat
static constexpr uint GLTF_CUSTOM_LAST_IND        = GLTF_FLOAT_REFL_COAT;

// GLASS
// colors
static constexpr uint GLASS_COLOR_REFLECT         = 0;    
static constexpr uint GLASS_COLOR_TRANSP          = 1;  
static constexpr uint GLASS_COLOR_LAST_IND        = GLASS_COLOR_TRANSP;

// custom 
static constexpr uint GLASS_FLOAT_GLOSS_REFLECT   = 0;
static constexpr uint GLASS_FLOAT_GLOSS_TRANSP    = 1;
static constexpr uint GLASS_FLOAT_IOR             = 2;
static constexpr uint GLASS_CUSTOM_LAST_IND       = GLASS_FLOAT_IOR;

// DIELECTRIC
// colors, for physical realism, color parameters should never be touched
static constexpr uint DIELECTRIC_COLOR_REFLECT    = 0;    
static constexpr uint DIELECTRIC_COLOR_TRANSMIT   = 1;  
static constexpr uint DIELECTRIC_COLOR_LAST_IND   = DIELECTRIC_COLOR_TRANSMIT;

// custom 
static constexpr uint DIELECTRIC_ETA_EXT          = 0;
static constexpr uint DIELECTRIC_ETA_INT          = 1;
static constexpr uint DIELECTRIC_ETA_INT_SPECID   = 2;
static constexpr uint DIELECTRIC_CUSTOM_LAST_IND  = DIELECTRIC_ETA_INT_SPECID;

// EMISSION
// colors
static constexpr uint EMISSION_COLOR              = 0;    
static constexpr uint EMISSION_COLOR_LAST_IND     = EMISSION_COLOR;

// custom 
static constexpr uint EMISSION_MULT               = 0;

// Conductor
// colors
static constexpr uint CONDUCTOR_COLOR             = 0;
static constexpr uint CONDUCTOR_COLOR_LAST_IND    = CONDUCTOR_COLOR;

// custom
static constexpr uint CONDUCTOR_ROUGH_U           = 0;
static constexpr uint CONDUCTOR_ROUGH_V           = 1;
static constexpr uint CONDUCTOR_ETA               = 2;
static constexpr uint CONDUCTOR_K                 = 3;
static constexpr uint CONDUCTOR_CUSTOM_LAST_IND   = CONDUCTOR_K;

// Plastic (mitsuba)
// colors
static constexpr uint PLASTIC_COLOR             = 0;
static constexpr uint PLASTIC_COLOR_LAST_IND    = PLASTIC_COLOR;

// custom
static constexpr uint PLASTIC_ROUGHNESS           = 0;
static constexpr uint PLASTIC_IOR_RATIO           = 1;
static constexpr uint PLASTIC_SPEC_SAMPLE_WEIGHT  = 2;
static constexpr uint PLASTIC_PRECOMP_REFLECTANCE = 3;
static constexpr uint PLASTIC_CUSTOM_LAST_IND     = PLASTIC_PRECOMP_REFLECTANCE;


// Simple diffuse
// colors
static constexpr uint DIFFUSE_COLOR             = 0;
static constexpr uint DIFFUSE_COLOR_LAST_IND    = DIFFUSE_COLOR;

// custom
static constexpr uint DIFFUSE_ROUGHNESS         = 0;
static constexpr uint DIFFUSE_CUSTOM_LAST_IND   = DIFFUSE_ROUGHNESS;


// Blend material
// colors
static constexpr uint BLEND_COLOR_LAST_IND = 0;

// custom
static constexpr uint BLEND_WEIGHT          = 0;
static constexpr uint BLEND_CUSTOM_LAST_IND = BLEND_WEIGHT;

// Conductor
// colors
static constexpr uint FILM_COLOR             = 0;
static constexpr uint FILM_COLOR_LAST_IND    = FILM_COLOR;

// custom
static constexpr uint FILM_ROUGH_U           = 0;
static constexpr uint FILM_ROUGH_V           = 1;
static constexpr uint FILM_PRECOMP_FLAG      = 2;
static constexpr uint FILM_PRECOMP_OFFSET    = 3;
static constexpr uint FILM_ETA_OFFSET        = 4;
static constexpr uint FILM_K_OFFSET          = 5;
static constexpr uint FILM_ETA_SPECID_OFFSET = 6;
static constexpr uint FILM_K_SPECID_OFFSET   = 7;
static constexpr uint FILM_ETA_EXT           = 8;
static constexpr uint FILM_THICKNESS_OFFSET  = 9;
static constexpr uint FILM_THICKNESS_MIN     = 10;
static constexpr uint FILM_THICKNESS_MAX     = 11;
static constexpr uint FILM_THICKNESS_MAP     = 12;
static constexpr uint FILM_THICKNESS         = 13;
static constexpr uint FILM_LAYERS_COUNT      = 14;
static constexpr uint FILM_TRANSPARENT       = 15;
static constexpr uint FILM_CUSTOM_LAST_IND   = FILM_TRANSPARENT;


// The size is taken according to the largest indexes
static constexpr uint COLOR_DATA_SIZE  = 4;  // std::max(std::max(GLTF_COLOR_LAST_IND, GLASS_COLOR_LAST_IND), CONDUCTOR_COLOR_LAST_IND) + 1;
static constexpr uint CUSTOM_DATA_SIZE = 16; // std::max(std::max(GLTF_CUSTOM_LAST_IND, GLASS_CUSTOM_LAST_IND), CONDUCTOR_CUSTOM_LAST_IND) + 1;

struct Material
{
  uint mtype;
  uint cflags;
  uint lightId;
  uint nonlinear;
  
  uint texid[4];
  uint spdid[4];
  uint datai[4];

  float4 colors[COLOR_DATA_SIZE]; ///< colors data
  float4 row0[4];                 ///< texture matrix
  float4 row1[4];                 ///< texture matrix
      
  float  data[CUSTOM_DATA_SIZE]; ///< float, uint and custom data. Read uint: uint x = as_uint(data[INDEX]), write: data[INDEX] = as_float(x)
};


static inline float safe_sqrt(float val)
{
  return std::sqrt(std::max(val, 0.0f));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Lambert BRDF
//////////////////////////////////////////////////////////////////////////////////////////////////////////

static inline float3 lambertSample(const float2 rands, const float3 v, const float3 n)
{
  return MapSampleToCosineDistribution(rands.x, rands.y, n, n, 1.0f);
}

static inline float lambertEvalPDF(float3 l, float3 v, float3 n) 
{ 
  return std::abs(dot(l, n)) * INV_PI;
}

static inline float lambertEvalBSDF(float3 l, float3 v, float3 n)
{
  return INV_PI;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// PBRT routine
//////////////////////////////////////////////////////////////////////////////////////////////////////////

static inline float cosPhiPBRT(const float3 w, const float sintheta)
{
  if (sintheta == 0.0f)
    return 1.0f;
  else
    return clamp(w.x / sintheta, -1.0f, 1.0f);
}

static inline float sinPhiPBRT(const float3 w, const float sintheta)
{
  if (sintheta == 0.0f)
    return 0.0f;
  else
    return clamp(w.y / sintheta, -1.0f, 1.0f);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Oren-Nayar BRDF from PBRT
//////////////////////////////////////////////////////////////////////////////////////////////////////////

static inline float orennayarFunc(const float3 a_l, const float3 a_v, const float3 a_n, const float a_roughness)
{
  const float cosTheta_wi = dot(a_l, a_n);
  const float cosTheta_wo = dot(a_v, a_n);

  const float sinTheta_wi = safe_sqrt(1.0f - cosTheta_wi * cosTheta_wi);
  const float sinTheta_wo = safe_sqrt(1.0f - cosTheta_wo * cosTheta_wo);

  const float sigma  = a_roughness * M_PI * 0.5f; //Radians(sig)
  const float sigma2 = sigma * sigma;
  const float A      = 1.0f - (sigma2 / (2.0f * (sigma2 + 0.33f)));
  const float B      = 0.45f * sigma2 / (sigma2 + 0.09f);

  ///////////////////////////////////////////////////////////////////////////// to PBRT coordinate system
  // wo = a_v = -ray_dir
  // wi = a_l = newDir
  //
  float3 nx, ny, nz = a_n;
  CoordinateSystemV2(nz, &nx, &ny);

  ///////////////////////////////////////////////////////////////////////////// to PBRT coordinate system

  // Compute cosine term of Oren-Nayar model
  float maxcos = 0.0f;

  if (sinTheta_wi > 1e-4f && sinTheta_wo > 1e-4f)
  {
    const float3 wo     = float3(-dot(a_v, nx), -dot(a_v, ny), -dot(a_v, nz));
    const float3 wi     = float3(-dot(a_l, nx), -dot(a_l, ny), -dot(a_l, nz));
    const float sinphii = sinPhiPBRT(wi, sinTheta_wi);
    const float cosphii = cosPhiPBRT(wi, sinTheta_wi);
    const float sinphio = sinPhiPBRT(wo, sinTheta_wo);
    const float cosphio = cosPhiPBRT(wo, sinTheta_wo);
    const float dcos    = cosphii * cosphio + sinphii * sinphio;
    maxcos              = std::max(0.0f, dcos);
  }

  // Compute sine and tangent terms of Oren-Nayar model
  float sinalpha = 0.0f, tanbeta = 0.0f;

  if (std::abs(cosTheta_wi) > std::abs(cosTheta_wo))
  {
    sinalpha = sinTheta_wo;
    tanbeta  = sinTheta_wi / std::max(std::abs(cosTheta_wi), DEPSILON);
  }
  else
  {
    sinalpha = sinTheta_wi;
    tanbeta  = sinTheta_wo / std::max(std::abs(cosTheta_wo), DEPSILON);
  }

  return (A + B * maxcos * sinalpha * tanbeta);
}


static inline float orennayarEvalPDF(const float3 l, const float3 n)
{
  return std::abs(dot(l, n)) * INV_PI;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

static inline float3 SphericalDirectionPBRT(const float sintheta, const float costheta, const float phi) 
{ 
  return float3(sintheta * std::cos(phi), sintheta * std::sin(phi), costheta); 
}

static inline float GGX_Distribution(const float cosThetaNH, const float alpha)
{
  const float alpha2 = alpha * alpha;
  const float NH_sqr = clamp(cosThetaNH * cosThetaNH, 0.0f, 1.0f);
  const float den    = NH_sqr * alpha2 + (1.0f - NH_sqr);
  return alpha2 / std::max((float)(M_PI) * den * den, 1e-6f);
}

static inline float GGX_GeomShadMask(const float cosThetaN, const float alpha)
{
  // Height - Correlated G.
  //const float tanNV      = sqrt(1.0f - cosThetaN * cosThetaN) / cosThetaN;
  //const float a          = 1.0f / (alpha * tanNV);
  //const float lambda     = (-1.0f + sqrt(1.0f + 1.0f / (a*a))) / 2.0f;
  //const float G          = 1.0f / (1.0f + lambda);

  // Optimized and equal to the commented-out formulas on top.
  const float cosTheta_sqr = clamp(cosThetaN * cosThetaN, 0.0f, 1.0f);
  const float tan2         = (1.0f - cosTheta_sqr) / std::max(cosTheta_sqr, 1e-6f);
  const float GP           = 2.0f / (1.0f + safe_sqrt(1.0f + alpha * alpha * tan2));
  return GP;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

static inline float3 ggxSample(const float2 rands, const float3 v, const float3 n, const float roughness)
{
  const float roughSqr = roughness * roughness;
    
  float3 nx, ny, nz = n;
  CoordinateSystemV2(nz, &nx, &ny);
    
  const float3 wo       = float3(dot(v, nx), dot(v, ny), dot(v, nz));
  const float phi       = rands.x * M_TWOPI;
  const float cosTheta  = clamp(safe_sqrt((1.0f - rands.y) / (1.0f + roughSqr * roughSqr * rands.y - rands.y)), 0.0f, 1.0f);
  const float sinTheta  = safe_sqrt(1.0f - cosTheta * cosTheta);
  const float3 wh       = SphericalDirectionPBRT(sinTheta, cosTheta, phi);
    
  const float3 wi = 2.0f * dot(wo, wh) * wh - wo;      // Compute incident direction by reflecting about wm  
  return normalize(wi.x * nx + wi.y * ny + wi.z * nz); // back to normal coordinate system
}

static inline float ggxEvalPDF(const float3 l, const float3 v, const float3 n, const float roughness)
{ 
  const float dotNV = dot(n, v);
  const float dotNL = dot(n, l);
  if (dotNV < 1e-6f || dotNL < 1e-6f)
    return 1.0f;

  const float  roughSqr  = roughness * roughness;
    
  const float3 h    = normalize(v + l); // half vector.
  const float dotNH = dot(n, h);
  const float dotHV = dot(h, v);
  const float D     = GGX_Distribution(dotNH, roughSqr);
  return  D * dotNH / (4.0f * std::max(dotHV,1e-6f));
}

static inline float ggxEvalBSDF(const float3 l, const float3 v, const float3 n, const float roughness)
{
  if(std::abs(dot(l, n)) < 1e-5f)
    return 0.0f; 
 
  const float dotNV = dot(n, v);  
  const float dotNL = dot(n, l);
  if (dotNV < 1e-6f || dotNL < 1e-6f)
    return 0.0f; 

  const float  roughSqr = roughness * roughness;
  const float3 h    = normalize(v + l); // half vector.
  const float dotNH = dot(n, h);
  const float D     = GGX_Distribution(dotNH, roughSqr);
  const float G     = GGX_GeomShadMask(dotNV, roughSqr)*GGX_GeomShadMask(dotNL, roughSqr);      

  return (D * G / std::max(4.0f * dotNV * dotNL, 1e-6f));  // Pass single-scattering
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Trowbridge-Reitz from PBRT-v4
// pbrt is Copyright(c) 1998-2020 Matt Pharr, Wenzel Jakob, and Greg Humphreys.
// The pbrt source code is licensed under the Apache License, Version 2.0.
// SPDX: Apache-2.0

static inline float CosTheta(float3 w) 
{
  return w.z;
}
static inline float Cos2Theta(float3 w) 
{
  return w.z * w.z;
}
static inline float AbsCosTheta(float3 w) 
{
  return std::abs(w.z);
}

static inline float Sin2Theta(float3 w) 
{
  return std::max(0.0f, 1.0f - Cos2Theta(w));
}
static inline float SinTheta(float3 w) 
{
  return safe_sqrt(Sin2Theta(w));
}

static inline float TanTheta(float3 w) 
{
  return SinTheta(w) / CosTheta(w);
}
static inline float Tan2Theta(float3 w) 
{
  return Sin2Theta(w) / Cos2Theta(w);
}

static inline float CosPhi(float3 w) 
{
  float sinTheta = SinTheta(w);
  return (sinTheta == 0) ? 1 : clamp(w.x / sinTheta, -1.0f, 1.0f);
}

static inline float SinPhi(float3 w) 
{
  float sinTheta = SinTheta(w);
  return (sinTheta == 0) ? 0 : clamp(w.y / sinTheta, -1.0f, 1.0f);
}

static inline float3 FaceForward(float3 v, float3 n2) 
{
    return (dot(v, n2) < 0.f) ? (-1.0f) * v : v;
}

static inline float2 SampleUniformDiskPolar(float2 u) 
{
  float r = safe_sqrt(u[0]);
  float theta = M_TWOPI * u[1];
  return {r * std::cos(theta), r * std::sin(theta)};
}

static inline float trD(float3 wm, float2 alpha)  
{
  float tan2Theta = Tan2Theta(wm);
  if (std::isinf(tan2Theta))
      return 0;
  float cos4Theta = Cos2Theta(wm) * Cos2Theta(wm);
  if (cos4Theta < 1e-16f)
      return 0;
  float e = tan2Theta * ((CosPhi(wm) / alpha.x) * (CosPhi(wm) / alpha.x) + (SinPhi(wm) / alpha.y) * (SinPhi(wm) / alpha.y));
  return 1.0f / (M_PI * alpha.x * alpha.y * cos4Theta * (1 + e) * (1 + e));
}

static inline bool trEffectivelySmooth(float2 alpha) 
{ 
  return std::max(alpha.x, alpha.y) < 1e-3f; 
}

static inline float trLambda(float3 w, float2 alpha)  
{
  float tan2Theta = Tan2Theta(w);
  if (std::isinf(tan2Theta))
    return 0;
  float alpha2 = (CosPhi(w) * alpha.x) * (CosPhi(w) * alpha.x) + (SinPhi(w) * alpha.y) * (SinPhi(w) * alpha.y);
  return (safe_sqrt(1.0f + alpha2 * tan2Theta) - 1.0f) / 2.0f;
}

static inline float trG1(float3 w, float2 alpha) 
{ 
  return 1.0f / (1.0f + trLambda(w, alpha)); 
}

static inline float trG(float3 wo, float3 wi, float2 alpha) 
{ 
  return 1.0f / (1.0f + trLambda(wo, alpha) + trLambda(wi, alpha)); 
}

static inline float trD(float3 w, float3 wm, float2 alpha) 
{
  return trG1(w, alpha) / AbsCosTheta(w) * trD(wm, alpha) * std::abs(dot(w, wm));
}

static inline float trPDF(float3 w, float3 wm, float2 alpha) 
{ 
  return trD(w, wm, alpha); 
}

static inline float3 trSample(float3 wo, float2 rands, float2 alpha)  
{
  // Transform _w_ to hemispherical configuration
  float3 wh = normalize(float3(alpha.x * wo.x, alpha.y * wo.y, wo.z));
  if (wh.z < 0)
  {
    wh = (-1.0f) * wh;
  }

  // Find orthonormal basis for visible normal sampling
  float3 T1 = (wh.z < 0.99999f) ? normalize(cross(float3(0, 0, 1), wh)) : float3(1, 0, 0);
  float3 T2 = cross(wh, T1);

  // Generate uniformly distributed points on the unit disk
  float2 p = SampleUniformDiskPolar(rands);

  // Warp hemispherical projection for visible normal sampling
  float h = safe_sqrt(1 - p.x * p.x);
  p.y = LiteMath::lerp(h, p.y, (1 + wh.z) / 2);

  // Reproject to hemisphere and transform normal to ellipsoid configuration
  float pz = safe_sqrt(1.0f - dot(p, p));
  float3 nh = p.x * T1 + p.y * T2 + pz * wh;
  return normalize(float3(alpha.x * nh.x, alpha.y * nh.y, std::max(1e-6f, nh.z)));
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

static inline float FrDielectricPBRT(float cosThetaI, float etaI, float etaT) 
{
  cosThetaI = clamp(cosThetaI, -1.0f, 1.0f);
  // Potentially swap indices of refraction
  bool entering = cosThetaI > 0.0f;
  if (!entering) 
  {
    const float tmp = etaI;
    etaI = etaT;
    etaT = tmp;
    cosThetaI = std::abs(cosThetaI);
  }

  // Compute _cosThetaT_ using Snell's law
  float sinThetaI = safe_sqrt(1.0f - cosThetaI * cosThetaI);
  float sinThetaT = etaI / etaT * sinThetaI;

  // Handle total internal reflection
  if (sinThetaT >= 1.0f) 
    return 1.0f;

  const float cosThetaT = safe_sqrt(1.0f - sinThetaT * sinThetaT);
  const float Rparl     = ((etaT * cosThetaI) - (etaI * cosThetaT)) / ((etaT * cosThetaI) + (etaI * cosThetaT));
  const float Rperp     = ((etaI * cosThetaI) - (etaT * cosThetaT)) / ((etaI * cosThetaI) + (etaT * cosThetaT));
  return 0.5f*(Rparl * Rparl + Rperp * Rperp);
}

static inline float FrDielectric(float cosTheta_i, float eta) 
{
  cosTheta_i = clamp(cosTheta_i, -1.0f, 1.0f);
  
  if (cosTheta_i < 0.0f) 
  {
      eta = 1.0f / eta;
      cosTheta_i = -cosTheta_i;
  }

  float sin2Theta_i = 1.0f - cosTheta_i * cosTheta_i;
  float sin2Theta_t = sin2Theta_i / (eta * eta);
  if (sin2Theta_t >= 1.0f)
      return 1.f;
  float cosTheta_t = safe_sqrt(1.0f - sin2Theta_t);

  float r_parl = (eta * cosTheta_i - cosTheta_t) / (eta * cosTheta_i + cosTheta_t);
  float r_perp = (cosTheta_i - eta * cosTheta_t) / (cosTheta_i + eta * cosTheta_t);
  return (r_parl * r_parl + r_perp * r_perp) / 2.0f;
}

static inline float FresnelMitsuba(float cos_theta_i, float eta) 
{
  auto outside_mask = cos_theta_i >= 0.f;

  float rcp_eta = 1.0f / eta,
        eta_it = outside_mask ? eta : rcp_eta,
        eta_ti = outside_mask ? rcp_eta : eta;

  /* Using Snell's law, calculate the squared sine of the
      angle between the surface normal and the transmitted ray */
  float cos_theta_t_sqr = -(-cos_theta_i * cos_theta_i + 1.f) * eta_ti * eta_ti + 1.f;

  /* Find the absolute cosines of the incident/transmitted rays */
  float cos_theta_i_abs = std::abs(cos_theta_i);
  float cos_theta_t_abs = safe_sqrt(cos_theta_t_sqr);

  auto index_matched = eta == 1.f;
  auto special_case  = index_matched || cos_theta_i_abs == 0.f;

  float r_sc = index_matched ? 0.f : 1.f;

  /* Amplitudes of reflected waves */
  float a_s = (-eta_it * cos_theta_t_abs + cos_theta_i_abs) / (eta_it * cos_theta_t_abs + cos_theta_i_abs);

  float a_p = (-eta_it * cos_theta_i_abs + cos_theta_t_abs) / (eta_it * cos_theta_i_abs + cos_theta_t_abs);

  float r = 0.5f * (a_s * a_s + a_p * a_p);

  r = special_case ? r_sc : r;

  /* Adjust the sign of the transmitted direction */
  //float cos_theta_t = cos_theta_i >= 0 ? -cos_theta_t_abs: cos_theta_t_abs;
  return r;
}

static inline float4 FrDielectricDetailed(float cosTheta_i, float eta) 
{
  cosTheta_i = clamp(cosTheta_i, -1.0f, 1.0f);
  
  if (cosTheta_i < 0.0f) 
  {
      eta = 1.0f / eta;
      cosTheta_i = -cosTheta_i;
  }
  float r = 0.f;

  float sin2Theta_i = 1.0f - cosTheta_i * cosTheta_i;
  float sin2Theta_t = sin2Theta_i / (eta * eta);
  if (sin2Theta_t >= 1.0f)
      r = 1.f;
  float cosTheta_t = safe_sqrt(1.0f - sin2Theta_t);

  float r_parl = (eta * cosTheta_i - cosTheta_t) / (eta * cosTheta_i + cosTheta_t);
  float r_perp = (cosTheta_i - eta * cosTheta_t) / (cosTheta_i + eta * cosTheta_t);

  r = (r_parl * r_parl + r_perp * r_perp) / 2.0f;

  cosTheta_t = cosTheta_i >= 0 ? -cosTheta_t : cosTheta_t;

  return {r, cosTheta_t, eta, 1.f / eta};
}

static inline float4 FrDielectricDetailedV2(float cos_theta_i, float eta) 
{
  cos_theta_i = clamp(cos_theta_i, -1.0f, 1.0f);
  
  float eta_it = eta;
  float eta_ti = 1.f / eta;
  if (cos_theta_i < 0.0f) 
  {
      eta_it = eta_ti;
      eta_ti = eta;
  }

  float cos_theta_t_sqr = -1.f * (-1.f * cos_theta_i * cos_theta_i + 1.f) * eta_ti * eta_ti + 1.f;
  float cos_theta_i_abs = std::abs(cos_theta_i);
  float cos_theta_t_abs = safe_sqrt(cos_theta_t_sqr);


  float r = 0.0f;
  if((eta == 1.f) || (cos_theta_i_abs == 0.f))
  {
    r = (eta == 1.f) ? 0.f : 1.f;
  }
  else
  {
    float a_s = (-1.f * eta_it * cos_theta_t_abs + cos_theta_i_abs) /
                (eta_it * cos_theta_t_abs + cos_theta_i_abs);

    float a_p = (-1.f * eta_it * cos_theta_i_abs + cos_theta_t_abs) /
                (eta_it * cos_theta_i_abs + cos_theta_t_abs);

    r = 0.5f * (a_s * a_s + a_p * a_p);
  }
  
  float cos_theta_t = cos_theta_i >= 0 ? -cos_theta_t_abs : cos_theta_t_abs;

  return float4(r, cos_theta_t, eta_it, eta_ti);

}

static inline float FrComplexConductor(float cosThetaI, complex eta)
{
  float sinThetaI = 1.0f - cosThetaI * cosThetaI;
  complex sinThetaT = sinThetaI / (eta * eta);
  complex cosThetaT = complex_sqrt(1.0f - sinThetaT);

  complex r_parl = (eta * cosThetaI - cosThetaT) / (eta * cosThetaI + cosThetaT);
  complex r_perp = (cosThetaI - eta * cosThetaT) / (cosThetaI + eta * cosThetaT);
  return (complex_norm(r_parl) + complex_norm(r_perp)) / 2.0f;
}

//static inline float fresnelConductor(float cosTheta, float eta, float roughness)
//{
//  float tmp = (eta*eta + roughness*roughness) * (cosTheta * cosTheta);
//  float rParl2 = (tmp - (eta * (2.0f * cosTheta)) + 1.0f) / (tmp + (eta * (2.0f * cosTheta)) + 1.0f);
//  float tmpF = eta*eta + roughness*roughness;
//  float rPerp2 = (tmpF - (eta * (2.0f * cosTheta)) + (cosTheta*cosTheta)) / (tmpF + (eta * (2.0f * cosTheta)) + (cosTheta*cosTheta));
//  return 0.5f*(rParl2 + rPerp2);
//}

static inline float fresnelSlick(const float VdotH)
{
  const float tmp = 1.0f - std::abs(VdotH);
  return (tmp*tmp)*(tmp*tmp)*tmp;
}

static inline float4 hydraFresnelCond(float4 f0, float VdotH, float ior, float roughness) 
{  
  if(ior == 0.0f) // fresnel reflactance is disabled
    return f0;

  return f0 + (float4(1.0f) - f0) * fresnelSlick(VdotH); // return bsdf * (f0 + (1 - f0) * (1 - abs(VdotH))^5)
}

static inline float lerp_gather(const float *data, float x, size_t size) 
{
  x *= float(size - 1);
  uint32_t index = std::min(uint32_t(x), uint32_t(size - 2));

  float v0 = data[index];
  float v1 = data[index + 1];

  return LiteMath::lerp(v0, v1, x - float(index));
}

static inline float lerp_gather_2d(const float *data, float x, float y, size_t size1, size_t size2)
{
  x *= float(size1 - 1);
  y *= float(size2 - 1);
  uint32_t index1 = std::min(uint32_t(x), uint32_t(size1 - 2));
  uint32_t index2 = std::min(uint32_t(y), uint32_t(size2 - 2));

  float alpha = x - float(index1);
  float beta = y - float(index2);
  float v0 = LiteMath::lerp(data[index1 * size2 + index2], data[(index1 + 1) * size2 + index2], alpha);
  float v1 = LiteMath::lerp(data[index1 * size2 + index2 + 1], data[(index1 + 1) * size2 + index2 + 1], alpha);

  return LiteMath::lerp(v0, v1, beta);
}


//////////////////////
// GGX from Mitsuba3

static inline float sin_theta_2(const float3 &v) 
{ 
  return v.x * v.x + v.y * v.y; 
}

/** \brief Give a unit direction, this function returns the sine and cosine
   * of the azimuth in a reference spherical coordinate system 
   */
static inline float2 sincos_phi(const float3 &v) 
{
  float sin_theta2 = sin_theta_2(v);
  float inv_sin_theta = 1.f / safe_sqrt(sin_theta_2(v));

  float2 result = {v.x * inv_sin_theta, v.y * inv_sin_theta};

  result = std::abs(sin_theta2) <= 4.f * EPSILON_32 ? float2(1.f, 0.f) : clamp(result, -1.f, 1.f);

  return { result.y, result.x };
}

/// Low-distortion concentric square to disk mapping by Peter Shirley
static inline float2 square_to_uniform_disk_concentric(const float2 &s) 
{
  float x = 2.f * s.x - 1.f,
        y = 2.f * s.y - 1.f;

  float phi, r;
  if (x == 0 && y == 0) 
  {
    r = phi = 0;
  } 
  else if (x * x > y * y) 
  {
    r = x;
    phi = (M_PI / 4.f) * (y / x);
  } 
  else 
  {
    r = y;
    phi = (M_PI / 2.f) - (x / y) * (M_PI / 4.f);
  }
  return {r * std::cos(phi), r * std::sin(phi)};
}

static inline float3 square_to_cosine_hemisphere(const float2 &s) 
{
    // Low-distortion warping technique based on concentric disk mapping
    float2 p = square_to_uniform_disk_concentric(s);

    // Guard against numerical imprecisions
    float z =  safe_sqrt(1.f - (p.x * p.x + p.y * p.y));
   

    return { p.x, p.y, z };
}

/**
   * \brief Smith's shadowing-masking function for a single direction
   *
   * \param v
   *     An arbitrary direction
   * \param m
   *     The microfacet normal
*/
static inline float smith_g1(const float3 &v, const float3 &m, float2 alpha) 
{
  float xy_alpha_2 = alpha.x * v.x * alpha.x * v.x + alpha.y * v.y * alpha.y * v.y,
        tan_theta_alpha_2 = xy_alpha_2 / (v.z * v.z),
        result;


  result = 2.f / (1.f + safe_sqrt(1.f + tan_theta_alpha_2));

  // Perpendicular incidence -- no shadowing/masking
  if(xy_alpha_2 == 0.f)
    result = 1.f;

  /* Ensure consistent orientation (can't see the back
      of the microfacet from the front and vice versa) */

  if(v.z * dot(v, m) <= 0.f)
    result = 0.f;

  return result;
}

static inline float sqr(float val)
{
  return val * val;
}

static inline float eval_microfacet(const float3 &m, float2 alpha, int type = 1) 
{
  float alpha_uv = alpha.x * alpha.y;
  float cos_theta = m.z;
  float cos_theta_2 = cos_theta * cos_theta;

  float result = 0.0f;
  if (type == 0) // Beckmann distribution function for Gaussian random surfaces 
  {
      result = std::exp(-(sqr(m.x / alpha.x) + sqr(m.y / alpha.y)) / cos_theta_2) / (M_PI * alpha_uv * sqr(cos_theta_2));
  } 
  else // GGX / Trowbridge-Reitz distribution function 
  {
      result = 1.f / (M_PI * alpha_uv * sqr(sqr(m.x / alpha.x) + sqr(m.y / alpha.y) + sqr(m.z)));
  }

  return result * cos_theta > 1e-20f ? result : 0.f;
}

static inline float2 sample_visible_11(float cos_theta_i, float2 samp)
{
  float2 p = square_to_uniform_disk_concentric(samp);

  float s = 0.5f * (1.f + cos_theta_i);
  p.y = LiteMath::lerp(safe_sqrt(1.f - p.x * p.x), p.y, s);

  // Project onto chosen side of the hemisphere
  float x = p.x, y = p.y,
        z = safe_sqrt(1.f - dot(p, p));

  // Convert to slope
  float sin_theta_i = safe_sqrt(1.f - cos_theta_i * cos_theta_i);
  float norm = 1.f / (sin_theta_i * y + cos_theta_i * z);
  return float2(cos_theta_i * y - sin_theta_i * z, x) * norm;
}

static inline float4 sample_visible_normal(float3 wi, float2 rands, float2 alpha)
{
  // Step 1: stretch wi
  float3 wi_p = normalize(float3(alpha.x * wi.x, alpha.y * wi.y, wi.z));

  const float2 sincos = sincos_phi(wi_p);
  const float sin_phi = sincos.x;
  const float cos_phi = sincos.y;

  const float cos_theta = wi_p.z;

  // Step 2: simulate P22_{wi}(slope.x, slope.y, 1, 1)
  float2 slope = sample_visible_11(cos_theta, rands);

  // Step 3: rotate & unstretch
  slope = float2((cos_phi * slope.x - sin_phi * slope.y) * alpha.x,
                  (sin_phi * slope.x + cos_phi * slope.y) * alpha.y);

  // Step 4: compute normal
  float3 m = normalize(float3(-slope.x, -slope.y, 1));

  float pdf = eval_microfacet(m, alpha, 1) * smith_g1(wi, m, alpha) * std::abs(dot(wi, m)) / wi.z;

  return {m.x, m.y, m.z, pdf};
}

// Smith's separable shadowing-masking approximation
static inline float microfacet_G(const float3 &wi, const float3 &wo, const float3 &m, float2 alpha) 
{
  return smith_g1(wi, m, alpha) * smith_g1(wo, m, alpha);
}

static inline float microfacet_pdf(const float3 &wi, const float3 &m, float2 alpha) 
{
  float result = eval_microfacet(m, alpha, 1);

  result *= smith_g1(wi, m, alpha) * std::abs(dot(wi, m)) / wi.z;

  return result;
}

static inline float3 refract(const float3 &wi, float cos_theta_t, float eta_ti) 
{
    return float3(-eta_ti * wi.x, -eta_ti * wi.y, cos_theta_t);
}

////////////////// blends

struct MatIdWeight
{
  uint32_t id;
  float weight;
};

static inline MatIdWeight make_id_weight(uint32_t a, float b)
{
  MatIdWeight res;
  res.id  = a;
  res.weight = b;
  return res;
}

struct MatIdWeightPair
{
  MatIdWeight first;
  MatIdWeight second;
};

static inline MatIdWeightPair make_weight_pair(MatIdWeight a, MatIdWeight b)
{
  MatIdWeightPair res;
  res.first  = a;
  res.second = b;
  return res;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

static inline float sum(float4 v) {return v.x + v.y + v.z + v.w;}

static inline complex conjugate(complex c) {return {c.re, -c.im};}

static constexpr uint PolarizationS = 0;
static constexpr uint PolarizationP = 1;

struct FrReflRefr
{
  float refl, refr;
};

static inline float getRefractionFactor(float cosThetaI, complex cosThetaT, complex iorI, complex iorT)
{
  complex mult = cosThetaT * iorT;
  if (cosThetaI <= 1e-6f || mult.im > 1e-6f)
  {
    return 0;
  }
  return mult.re / (iorI.re * cosThetaI);
}

static inline float getRefractionFactorS(complex cosThetaI, complex cosThetaT, complex iorI, complex iorT)
{
  if (complex_norm(cosThetaI) <= 1e-6f)
  {
    return 0;
  }
  return (iorT * cosThetaT).re / (iorI * cosThetaI).re;
}

static inline float getRefractionFactorP(complex cosThetaI, complex cosThetaT, complex iorI, complex iorT)
{
  if (complex_norm(cosThetaI) <= 1e-6f)
  {
    return 0;
  }
  return (iorT * conjugate(cosThetaT)).re / (iorI * conjugate(cosThetaI)).re;
}

static inline complex FrComplexRefl(complex cosThetaI, complex cosThetaT, complex iorI, complex iorT, uint polarization)
{
  if (complex_norm(cosThetaI) < 1e-6f) //approximated
  {
    return {-1, 0};
  }
  if (polarization == PolarizationS)
  {
    return (iorI * cosThetaI - iorT * cosThetaT) / (iorI * cosThetaI + iorT * cosThetaT);
  }
  else if (polarization == PolarizationP)
  {
    return (iorT * cosThetaI - iorI * cosThetaT) / (iorT * cosThetaI + iorI * cosThetaT);
  }
  return 1.0f;
}

static inline complex FrComplexRefr(complex cosThetaI, complex cosThetaT, complex iorI, complex iorT, uint polarization)
{
  if (complex_norm(cosThetaI) < 1e-6f) //approximated
  {
    if (complex_norm(iorI - iorT) < 1e-6f)
    {
      return {1, 0};
    }
    return {0, 0};
  }
  if (polarization == PolarizationS)
  {
    return (2 * iorI * cosThetaI) / (iorI * cosThetaI + iorT * cosThetaT);
  }
  else if (polarization == PolarizationP)
  {
    return (2 * iorI * cosThetaI) / (iorT * cosThetaI + iorI * cosThetaT);
  }
  return 0.f;
}

static inline complex filmPhaseDiff(complex cosTheta, complex eta, float thickness, float lambda)
{
  return 4 * M_PI * eta * cosTheta * thickness / complex(lambda);
}

#endif
