#pragma once
#include "cglobals.h"
#include "crandom.h"
#include "cmaterial.h"

static inline void gltfSampleAndEval(const Material* a_materials, float4 rands, float3 v, 
                                     float3 n, float2 tc, float4 baseColor, float4 fourParams, BsdfSample* pRes)
{
  // PLEASE! use 'a_materials[0].' for a while ... , not a_materials-> and not *(a_materials).
  const uint   cflags     = a_materials[0].cflags;
  const float4 metalCol   = a_materials[0].colors[GLTF_COLOR_METAL]*baseColor; 
  const float4 coatCol    = a_materials[0].colors[GLTF_COLOR_COAT];  
  const float  roughness  = clamp(1.0f - a_materials[0].data[GLTF_FLOAT_GLOSINESS]*fourParams.x, 0.0f, 1.0f);   
  float        metalness  = a_materials[0].data[GLTF_FLOAT_ALPHA]*fourParams.y;
  const float  coatValue  = a_materials[0].data[GLTF_FLOAT_REFL_COAT]*fourParams.z;                 
  const float  fresnelIOR = a_materials[0].data[GLTF_FLOAT_IOR];
  
  if(cflags == GLTF_COMPONENT_METAL) // assume only GGX-based metal component set
    metalness = 1.0f;

  float3 ggxDir;
  float  ggxPdf; 
  float  ggxVal;

  if(roughness == 0.0f) // perfect specular reflection in coating or metal layer
  {
    const float3 pefReflDir = reflect((-1.0f) * v, n);
    const float cosThetaOut = dot(pefReflDir, n);
    ggxDir                  = pefReflDir;
    ggxVal                  = (cosThetaOut <= 1e-6f) ? 0.0f : (1.0f/std::max(cosThetaOut, 1e-6f));  // BSDF is multiplied (outside) by cosThetaOut. For mirrors this shouldn't be done, so we pre-divide here instead.
    ggxPdf                  = 1.0f;
  }
  else
  {
    ggxDir                  = ggxSample(float2(rands.x, rands.y), v, n, roughness);
    ggxPdf                  = ggxEvalPDF (ggxDir, v, n, roughness); 
    ggxVal                  = ggxEvalBSDF(ggxDir, v, n, roughness);
  }

  const float3 lambertDir   = lambertSample(float2(rands.x, rands.y), v, n);
  const float  lambertPdf   = lambertEvalPDF(lambertDir, v, n);
  const float  lambertVal   = lambertEvalBSDF(lambertDir, v, n);

  // (1) select between metal and dielectric via rands.z
  //
  float pdfSelect = 1.0f;
  if(rands.z < metalness) // select metall
  {
    pdfSelect         *= metalness;
    const float  VdotH = dot(v,normalize(v + ggxDir));
    pRes->dir          = ggxDir;
    pRes->val          = ggxVal * metalness * hydraFresnelCond(metalCol, VdotH, fresnelIOR, roughness); //TODO: disable fresnel here for mirrors
    pRes->pdf          = ggxPdf;
    pRes->flags        = (roughness == 0.0f) ? RAY_EVENT_S : RAY_FLAG_HAS_NON_SPEC;
  }
  else                // select dielectric
  {
    pdfSelect *= 1.0f - metalness;
    
    // (2) now select between specular and diffise via rands.w
    //
    const float f_i           = FrDielectricPBRT(std::abs(dot(v,n)), 1.0f, fresnelIOR); 
    const float prob_specular = 0.5f*coatValue;
    const float prob_diffuse  = 1.0f-prob_specular;
    
    if(rands.w < prob_specular) // specular
    {
      pdfSelect      *= prob_specular;
      pRes->dir       = ggxDir;
      pRes->val       = ggxVal*coatCol*(1.0f - metalness)*f_i*coatValue;
      pRes->pdf       = ggxPdf;
      pRes->flags     = (roughness == 0.0f) ? RAY_EVENT_S : RAY_FLAG_HAS_NON_SPEC;
    } 
    else
    {
      pdfSelect      *= prob_diffuse; // lambert
      pRes->dir       = lambertDir;
      pRes->val       = lambertVal * baseColor * (1.0f - metalness);
      pRes->pdf       = lambertPdf;
      pRes->flags     = RAY_FLAG_HAS_NON_SPEC;
            
      if(coatValue > 0.0f && fresnelIOR > 0.0f) // Plastic, account for retroreflection between surface and coating layer
      {
        const float m_fdr_int = a_materials[0].data[GLTF_FLOAT_MI_FDR_INT];
        const float f_o       = FrDielectricPBRT(std::abs(dot(lambertDir, n)), 1.0f, fresnelIOR);
        pRes->val            *= lerp(1.0f, (1.0f - f_i) * (1.0f - f_o) / (fresnelIOR * fresnelIOR * (1.0f - m_fdr_int)), coatValue);
      }
    }
  }   
  pRes->pdf *= pdfSelect;
}


static void gltfEval(const Material* a_materials, float3 l, float3 v, float3 n, float2 tc, 
                     float4 baseColor, float4 fourParams, BsdfEval* res)
{
  const uint   cflags     = a_materials[0].cflags;
  const float4 metalCol   = a_materials[0].colors[GLTF_COLOR_METAL]*baseColor;
  const float4 coatCol    = a_materials[0].colors[GLTF_COLOR_COAT];
  const float  roughness  = clamp(1.0f - a_materials[0].data[GLTF_FLOAT_GLOSINESS]*fourParams.x, 0.0f, 1.0f);
        float  metalness  = a_materials[0].data[GLTF_FLOAT_ALPHA]*fourParams.y;
  const float  coatValue  = a_materials[0].data[GLTF_FLOAT_REFL_COAT]*fourParams.z;      
  const float  fresnelIOR = a_materials[0].data[GLTF_FLOAT_IOR];

  if(cflags == GLTF_COMPONENT_METAL) // assume only GGX-based metal
    metalness = 1.0f;
      
  float ggxVal, ggxPdf, VdotH; 
  if(roughness != 0.0f) // perfect specular reflection in coating layer
  {
    ggxVal = ggxEvalBSDF(l, v, n, roughness);
    ggxPdf = ggxEvalPDF (l, v, n, roughness);
    VdotH  = dot(v,normalize(v + l));
  }
  else
  {
    ggxVal = 0.0f;
    ggxPdf = 0.0f;
    VdotH  = dot(v,n);
  }

  float lambertVal       = lambertEvalBSDF(l, v, n);
  const float lambertPdf = lambertEvalPDF (l, v, n);
  float f_i              = 1.0f;
  float coeffLambertPdf  = 1.0f;
      
  if(coatValue > 0.0f && metalness < 1.0f && fresnelIOR > 0.0f) // Plastic, account for retroreflection between surface and coating layer
  {
    f_i                   = FrDielectricPBRT(std::abs(dot(v,n)), 1.0f, fresnelIOR);
    const float f_o       = FrDielectricPBRT(std::abs(dot(l,n)), 1.0f, fresnelIOR);  
    const float m_fdr_int = a_materials[0].data[GLTF_FLOAT_MI_FDR_INT];
    const float coeff     = lerp(1.0f, (1.f - f_i) * (1.f - f_o) / (fresnelIOR*fresnelIOR*(1.f - m_fdr_int)), coatValue);
    lambertVal           *= coeff;
    coeffLambertPdf       = coeff; 
  }
  
  const float4 fConductor    = hydraFresnelCond(metalCol, VdotH, fresnelIOR, roughness); // (1) eval metal component      
  const float4 specularColor = ggxVal*fConductor;                                        // eval metal specular component

  const float prob_specular = 0.5f*coatValue;
  const float prob_diffuse  = 1.0f-prob_specular;

  const float4 dielectricVal = lambertVal * baseColor + ggxVal * coatCol * f_i * coatValue;
  const float  dielectricPdf = lambertPdf * prob_diffuse + ggxPdf*prob_specular; 

  res->val = metalness * specularColor + (1.0f - metalness) * dielectricVal; // (3) accumulate final color and pdf
  res->pdf = metalness * ggxPdf        + (1.0f - metalness) * dielectricPdf; // (3) accumulate final color and pdf
}