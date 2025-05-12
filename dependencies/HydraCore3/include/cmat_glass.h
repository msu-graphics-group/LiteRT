#pragma once
#include "cglobals.h"
#include "crandom.h"
#include "cmaterial.h"

typedef struct RefractResultT
{
  float3 rayDir;
  bool   success;
  float  eta;

}RefractResult;


static inline RefractResult myRefract(const float3 a_rayDir, float3 a_normal, 
  const float a_inputIor, const float a_outputIor)
{
  RefractResult res;
  res.eta        = a_inputIor / a_outputIor; 
  float cosTheta = dot(a_normal, a_rayDir) * (-1.0f);

  if (cosTheta < 0.0f)
  {
    cosTheta = cosTheta * (-1.0f);
    a_normal = a_normal * (-1.0f);
    res.eta = 1.0f / res.eta;
  }

  const float dotVN    = cosTheta * (-1.0f);  
  const float radicand = 1.0f + res.eta * res.eta * (cosTheta * cosTheta - 1.0f);

  if (radicand > 0.0f) // refract
  {
    res.rayDir    = normalize(res.eta * a_rayDir + (res.eta * cosTheta - sqrt(radicand)) * a_normal); 
    res.success = true;
  }
  else // internal reflect
  {
    res.rayDir  = normalize((a_normal * dotVN * (-2.0f)) + a_rayDir); 
    res.success = false;
    res.eta     = 1.0f;
  }

  return res;
}


static inline float3 GgxVndf(float3 wo, float roughness, float u1, float u2)
{
  // -- Stretch the view vector so we are sampling as though
  // -- roughnessTransp==1
  const float3 v = normalize(make_float3(wo.x * roughness, wo.y * roughness, wo.z));

  // -- Build an orthonormal basis with v, t1, and t2
  const float3 XAxis = make_float3(1.0f, 0.0f, 0.0f);
  const float3 ZAxis = make_float3(0.0f, 0.0f, 1.0f);
  const float3 t1 = (v.z < 0.999f) ? normalize(cross(v, ZAxis)) : XAxis;
  const float3 t2 = cross(t1, v);

  // -- Choose a point on a disk with each half of the disk weighted
  // -- proportionally to its projection onto direction v
  const float a = 1.0f / (1.0f + v.z);
  const float r = std::sqrt(u1);
  const float phi = (u2 < a) ? (u2 / a) * M_PI : M_PI + (u2 - a) / (1.0f - a) * M_PI;
  const float p1 = r * std::cos(phi);
  const float p2 = r * std::sin(phi) * ((u2 < a) ? 1.0f : v.z);

  // -- Calculate the normal in this stretched tangent space
  const float3 n = p1 * t1 + p2 * t2 + sqrt(std::max(0.0f, 1.0f - p1 * p1 - p2 * p2)) * v;

  // -- unstretch and normalize the normal
  return normalize(make_float3(roughness * n.x, roughness * n.y, std::max(0.0f, n.z)));
}


static inline float SmithGGXMasking(const float dotNV, float roughSqr)
{
  const float denomC = sqrt(roughSqr + (1.0f - roughSqr) * dotNV * dotNV) + dotNV;
  return 2.0f * dotNV / std::max(denomC, 1e-6f);
}


static inline float SmithGGXMaskingShadowing(const float dotNL, const float dotNV, float roughSqr)
{
  const float denomA = dotNV * std::sqrt(roughSqr + (1.0f - roughSqr) * dotNL * dotNL);
  const float denomB = dotNL * std::sqrt(roughSqr + (1.0f - roughSqr) * dotNV * dotNV);
  return 2.0f * dotNL * dotNV / std::max(denomA + denomB, 1e-6f);
}


static inline void refractionGlassSampleAndEval(const float4 a_colorTransp, const float a_inputIor, 
  const float a_outputIor, const float a_roughTransp, const float3 a_normal, const float3 a_fixNormal,
  const float4 a_rands, const float3 a_rayDir, BsdfSample* a_pRes)
{
  const float roughSqr = a_roughTransp * a_roughTransp;

  bool   spec = true;
  float  Pss  = 1.0f;                          // Pass single-scattering.
  float4 Pms = float4(1.0f); // Pass multi-scattering

  RefractResult refrData;

  if (a_roughTransp < 1e-5f)
    refrData = myRefract(a_rayDir, a_fixNormal, a_inputIor, a_outputIor);
  else
  {
    spec                 = false;
    float eta            = a_inputIor / a_outputIor;
    const float cosTheta = dot(a_fixNormal, a_rayDir) * (-1.0f);

    if (cosTheta < 0.0f)
      eta = 1.0f / eta;

    float3 nx, ny, nz = a_normal;
    CoordinateSystemV2(nz, &nx, &ny);

    const float3 wo      = make_float3(-dot(a_rayDir, nx), -dot(a_rayDir, ny), -dot(a_rayDir, nz));
    const float3 wh      = GgxVndf(wo, roughSqr, a_rands.x, a_rands.y);             // New sampling Heitz 2017
    const float  dotWoWh = dot(wo, wh);
    float3       newDir;

    const float radicand = 1.0f + eta * eta * (dotWoWh * dotWoWh - 1.0f);
    if (radicand > 0.0f)
    {
      newDir           = (eta * dotWoWh - std::sqrt(radicand)) * wh - eta * wo;    // refract        
      refrData.success = true;
      refrData.eta     = eta;
    }
    else
    {
      newDir           = 2.0f * dotWoWh * wh - wo;                            // internal reflect 
      refrData.success = false;
      refrData.eta     = 1.0f;
    }

    refrData.rayDir   = normalize(newDir.x * nx + newDir.y * ny + newDir.z * nz);    // back to normal coordinate system

    const float3 v    = (-1.0f) * a_rayDir;
    const float3 l    = refrData.rayDir;
    const float3 n    = a_normal;
    const float dotNV = std::abs(dot(n, v));
    const float dotNL = std::abs(dot(n, l));

    // Fresnel is not needed here, because it is used for the blend.    
    const float G1    = SmithGGXMasking(dotNV, roughSqr);
    const float G2    = SmithGGXMaskingShadowing(dotNL, dotNV, roughSqr);

    // Abbreviated formula without PDF.
    Pss *= G2 / std::max(G1, 1e-6f);

    // Complete formulas with PDF, if we ever make an explicit strategy for glass.
    //const float3 h    = normalize(v + l); // half vector.
    //const float dotNH = fabs(dot(n, h));
    //const float dotHV = fabs(dot(h, v));
    //const float dotHL = fabs(dot(v, l));
    //const float D   = GGX_Distribution(dotNH, roughSqr);
    //float etaI = 1.0f;
    //float etaO = a_ior;
    //if (eta > 1.0f)
    //{      
    //  etaI = a_ior;
    //  etaO = 1.0f;
    //}
    //const float eq1 = (dotHV * dotHL) / fmax(dotNV * dotNL, 1e-6f);
    //const float eq2 = etaO * etaO * G2 * D / fmax(pow(etaI * dotHV + etaO * dotHL, 2), 1e-6f);
    //Pss             = eq1 * eq2 * dotNL; // dotNL is here to cancel cosMult at the end.
    //const float Dv    = D * G1 * dotHV / fmax(dotNV, 1e-6f);
    //const float jacob = etaO * etaO * dotHL / fmax(pow(etaI * dotHV + etaO * dotHL, 2), 1e-6f);
    //a_pRes->pdf        = Dv * jacob;

    // Pass multi-scattering. (not supported yet)
    //Pms = GetMultiscatteringFrom3dTable(a_globals->m_essTranspTable, a_roughTransp, dotNV, 1.0f / eta, 64, 64, 64, color);
  }

  const float cosThetaOut = dot(refrData.rayDir, a_normal);
  const float cosMult     = 1.0f / std::max(std::abs(cosThetaOut), 1e-6f);

  a_pRes->dir             = refrData.rayDir;
  a_pRes->pdf             = 1.0f;

  // only camera paths are multiplied by this factor, and etas are swapped because radiance
  // flows in the opposite direction. See SmallVCM and or Veach adjoint bsdf.
  const bool a_isFwdDir       = true; // It should come from somewhere on top, but it has not yet been implemented, we are making a fake.
  const float adjointBtdfMult = a_isFwdDir ? 1.0f : (refrData.eta * refrData.eta);

  if (refrData.success) a_pRes->val = a_colorTransp * adjointBtdfMult * Pss * Pms * cosMult; 
  else                  a_pRes->val = float4(1.0f) * Pss * Pms * cosMult;

  if (spec)             a_pRes->flags |= (RAY_EVENT_S | RAY_EVENT_T);
  else                  a_pRes->flags |= (RAY_EVENT_G | RAY_EVENT_T);

  if      (refrData.success  && cosThetaOut >= -1e-6f) a_pRes->val = float4(0.0f); // refraction/transparency must be under surface!
  else if (!refrData.success && cosThetaOut < 1e-6f)   a_pRes->val = float4(0.0f); // reflection happened in wrong way
}


static inline float3 reflect2(const float3 dir, const float3 n)
{  
  return normalize(dir - 2.0f * dot(dir, n) * n);  // dir - vector from light
}

static inline float3 refract2(const float3 dir, const float3 n, const float relativeIor)
{  
  const float cosi = dot(dir, n);        // dir - vector from light. The normal should always look at the light vector.
  const float eta  = 1.0f / relativeIor; // Since the incoming vector and the normal are directed in the same direction.
  const float k    = 1.0f - eta * eta * (1.0f - cosi * cosi);
  if (k < 0)       
    return reflect2(dir, n); // full internal reflection 
  else         
    return normalize(eta * dir - (eta * cosi + std::sqrt(k)) * n); // the refracted vector    
}

static inline float fresnel2(float3 v, float3 n, float ior)
{
  // Calculating the angle of incidence of light
  const float cosi = dot(v, n);
  // We calculate the angle of refraction of light according to the Snellius law
  const float sint = std::sqrt(1.0f - cosi * cosi) / ior;
  // Check if there is a complete internal reflection
  if (sint > 1.0f) 
  {
    // If yes, then we return the reflection coefficient equal to 1
    return 1.0f;
  }
  else 
  {
    // Otherwise we calculate the angle of refraction of light
    const float cost = std::sqrt(1.0f - sint * sint);
    // We calculate the reflection coefficients for parallel and perpendicular polarization using Fresnel formulas
    const float Rp   = (ior * cosi - cost) / (ior * cosi + cost);
    const float Rs   = (cosi - ior * cost) / (cosi + ior * cost);
    // We return the average value of these coefficients
    return (Rp * Rp + Rs * Rs) * 0.5f;
  }
}


static inline void glassSampleAndEval(const Material* a_materials, const float4 a_rands, 
  const float3 a_viewDir, const float3 a_normal, const float2 a_tc, BsdfSample* a_pRes, MisData* a_misPrev)
{
  // PLEASE! use 'a_materials[0].' for a while ... , not a_materials-> and not *(a_materials).
  const float4 colorReflect = a_materials[0].colors[GLASS_COLOR_REFLECT];   
  const float4 colorTransp  = a_materials[0].colors[GLASS_COLOR_TRANSP];
  const float  ior                  = a_materials[0].data[GLASS_FLOAT_IOR];

  const float3 rayDir = (-1.0f) * a_viewDir;
  float relativeIor   = ior / a_misPrev->ior;

  if ((a_pRes->flags & RAY_FLAG_HAS_INV_NORMAL) != 0) // hit the reverse side of the polygon from the volume
  {
    if (a_misPrev->ior == ior) // in the previous hit there was material with a equal IOR
      relativeIor = 1.0f / ior;
  }

  const float fresnel = fresnel2(a_viewDir, a_normal, relativeIor);

  float3 dir;

  if (a_rands.w < fresnel) // reflection
  {
    dir            = reflect2(rayDir, a_normal);
    a_pRes->val    =  colorReflect;
    a_pRes->flags |= RAY_EVENT_S;
  }
  else
  {
    dir            = refract2(rayDir, a_normal, relativeIor);
    a_pRes->val    = colorTransp;
    a_misPrev->ior = ior;
    a_pRes->flags |= (RAY_EVENT_S | RAY_EVENT_T);
  }

  const float cosThetaOut = std::abs(dot(dir, a_normal));
  
  a_pRes->val      /= std::max(cosThetaOut, 1e-6f);// BSDF is multiplied (outside) by cosThetaOut. For mirrors this shouldn't be done, so we pre-divide here instead.
  a_pRes->dir       = dir;
  a_pRes->pdf       = 1.0f;
}


// implicit strategy

//static inline void glassSampleAndEval(const Material* a_materials, const float4 a_rands, 
//  float3 a_viewDir, float3 a_normal, const float2 a_tc, BsdfSample* a_pRes, MisData* a_misPrev)
//{
//  // PLEASE! use 'a_materials[0].' for a while ... , not a_materials-> and not *(a_materials).
//  const float3 colorReflect    = to_float3(a_materials[0].colors[GLASS_COLOR_REFLECT]);   
//  const float3 colorTransp     = to_float3(a_materials[0].colors[GLASS_COLOR_TRANSP]);
//  const float  roughReflect    = clamp(1.0f - a_materials[0].data[GLASS_FLOAT_GLOSS_REFLECT], 0.0f, 1.0f);
//  const float  roughTransp     = clamp(1.0f - a_materials[0].data[GLASS_FLOAT_GLOSS_TRANSP], 0.0f, 1.0f);                          
//  float        ior             = a_materials[0].data[GLASS_FLOAT_IOR]; 
//  
//  float3 fixNormal             = a_normal;
//
//  if (a_pRes->flags & RAY_FLAG_HAS_INV_NORMAL) // hit the reverse side of the polygon from the volume
//  {
//    fixNormal      = (-1.0f) * a_normal;
//
//    if (a_misPrev->ior == ior) // in the previous hit there was material with a equal IOR
//      a_misPrev->ior = 1.0f;
//    else
//      ior = 1.0f / ior;
//  }
//
//  const float  dotNV  = dot(a_normal, a_viewDir);
//  const float fresnel = FrDielectricPBRT(dotNV, a_misPrev->ior, ior);
//
//
//  if (a_rands.w < fresnel) // reflection
//  {
//    float3 dir;
//    float  val = 1.0f;
//    float  pdf = 1.0f;
//
//    if (roughReflect < 0.01f)
//    {
//      dir                     = reflect((-1.0f) * a_viewDir, fixNormal);
//      const float cosThetaOut = dot(dir, fixNormal);
//      val                     = (cosThetaOut <= 1e-6f) ? 1.0f : (1.0f / std::max(cosThetaOut, 1e-6f));  // BSDF is multiplied (outside) by cosThetaOut. For mirrors this shouldn't be done, so we pre-divide here instead.
//    }
//    else
//    {
//      dir                     = ggxSample(float2(a_rands.x, a_rands.y), a_viewDir, fixNormal, roughReflect);
//      val                     = ggxEvalBSDF(dir, a_viewDir, fixNormal, roughReflect);
//      pdf                     = ggxEvalPDF(dir, a_viewDir, fixNormal, roughReflect);
//    }
//
//    a_pRes->direction         = dir;
//    a_pRes->color             = val * colorReflect;
//    a_pRes->pdf               = pdf;
//    a_pRes->flags            |= (roughReflect < 0.01f) ? RAY_EVENT_S : RAY_FLAG_HAS_NON_SPEC;
//  }
//  else  // transparency
//  {
//
    //refractionGlassSampleAndEval(colorTransp, a_misPrev->ior, ior, roughTransp, a_normal, fixNormal, a_rands, (-1.0f) * a_viewDir, a_pRes);
//    a_misPrev->ior = ior;
//  }
//}


// explicit strategy
static void glassEval(const Material* a_materials, float3 l, float3 v, float3 n, float2 tc,
  float3 color, BsdfEval* res)
{
  // because we don't want to sample this material with shadow rays
  res->val   = float4(0.0f);
  res->pdf   = 0.0f;
}