#include "integrator_pt.h"
#include "include/crandom.h"

#include "include/cmaterial.h"
#include "include/cmat_gltf.h"
#include "include/cmat_conductor.h"
#include "include/cmat_glass.h"
#include "include/cmat_diffuse.h"
#include "include/cmat_plastic.h"
#include "include/cmat_dielectric.h"

#include <chrono>
#include <string>

#include "Image2d.h"
using LiteImage::Image2D;
using LiteImage::Sampler;
using LiteImage::ICombinedImageSampler;
using namespace LiteMath;

LightSample Integrator::LightSampleRev(int a_lightId, float3 rands, float3 illiminationPoint)
{
  const uint   gtype  = m_lights[a_lightId].geomType;
  const float2 rands2 = float2(rands.x, rands.y);
  switch(gtype)
  {
    case LIGHT_GEOM_DIRECT: return directLightSampleRev(m_lights.data() + a_lightId, rands2, illiminationPoint);
    case LIGHT_GEOM_SPHERE: return sphereLightSampleRev(m_lights.data() + a_lightId, rands2);
    case LIGHT_GEOM_POINT:  return pointLightSampleRev (m_lights.data() + a_lightId);
    case LIGHT_GEOM_ENV: 
    if(KSPEC_LIGHT_ENV != 0)
    {
      const uint32_t offset = m_lights[a_lightId].pdfTableOffset;
      const uint32_t sizeX  = m_lights[a_lightId].pdfTableSizeX;
      const uint32_t sizeY  = m_lights[a_lightId].pdfTableSizeY;
      
      const Map2DPiecewiseSample sam = SampleMap2D(rands, offset, int(sizeX), int(sizeY));

      // apply inverse texcoord transform to get phi and theta (SKY_DOME_INV_MATRIX0 in HydraCore2)
      //
      const float2 texCoordT = mulRows2x4(m_lights[a_lightId].samplerRow0Inv, m_lights[a_lightId].samplerRow1Inv, sam.texCoord);

      float sintheta = 0.0f;
      const float3 sampleDir = texCoord2DToSphereMap(texCoordT, &sintheta);
      const float3 samplePos = illiminationPoint + sampleDir*1000.0f; // TODO: add sceen bounding sphere radius here
      const float  samplePdf = (sam.mapPdf * 1.0f) / (2.f * M_PI * M_PI * std::max(std::abs(sintheta), 1e-20f)); // TODO: pass computed pdf to 'LightEvalPDF'
      
      LightSample res;
      res.hasIES = false;
      res.isOmni = true;
      res.norm   = sampleDir; 
      res.pos    = samplePos;
      res.pdf    = samplePdf; // evaluated here for environment lights 
      return res;
    }
    default:                return areaLightSampleRev  (m_lights.data() + a_lightId, rands2);
  };
}

float Integrator::LightPdfSelectRev(int a_lightId) 
{ 
  return 1.0f/float(m_lights.size()); // uniform select
}

//static inline float DistanceSquared(float3 a, float3 b)
//{
//  const float3 diff = b - a;
//  return dot(diff, diff);
//}

float Integrator::LightEvalPDF(int a_lightId, float3 illuminationPoint, float3 ray_dir, const float3 lpos, const float3 lnorm, float a_envPdf)
{
  const uint gtype = m_lights[a_lightId].geomType;
  if(gtype == LIGHT_GEOM_ENV)
    return a_envPdf;

  const float hitDist   = length(illuminationPoint - lpos);
  const float cosValTmp = dot(ray_dir, -1.0f*lnorm);
  float cosVal = 1.0f;
  switch(gtype)
  {
    case LIGHT_GEOM_SPHERE:
    {
      // const float  lradius = m_lights[a_lightId].size.x;
      // const float3 lcenter = to_float3(m_lights[a_lightId].pos);
      //if (DistanceSquared(illuminationPoint, lcenter) - lradius*lradius <= 0.0f)
      //  return 1.0f;
      const float3 dirToV = normalize(lpos - illuminationPoint);
      cosVal = std::abs(dot(dirToV, lnorm));
    }
    break;

    case LIGHT_GEOM_POINT:
    {
      if(m_lights[a_lightId].distType == LIGHT_DIST_LAMBERT)
        cosVal = std::max(cosValTmp, 0.0f);
    };
    break;

    default: // any type of area light
    //cosVal = std::max(cosValTmp, 0.0f);                                                               ///< Note(!): actual correct way for area lights
    cosVal = (m_lights[a_lightId].iesId == uint(-1)) ? std::max(cosValTmp, 0.0f) : std::abs(cosValTmp); ///< Note(!): this is not physically correct for area lights, see test_206;
    break;                                                                                              ///< Note(!): dark line on top of image for pink light appears because area light don't shine to the side. 
  };
  
  return PdfAtoW(m_lights[a_lightId].pdfA, hitDist, cosVal);
}

float4 Integrator::LightIntensity(uint a_lightId, float4 a_wavelengths, float3 a_rayPos, float3 a_rayDir)
{
  float4 lightColor = m_lights[a_lightId].intensity;  
  
  // get spectral data for light source
  //
  const uint specId = m_lights[a_lightId].specId;
  if(KSPEC_SPECTRAL_RENDERING !=0 && m_spectral_mode != 0 && specId < 0xFFFFFFFF)
  {
    const uint2 data  = m_spec_offset_sz[specId];
    const uint offset = data.x;
    const uint size   = data.y;
    lightColor = SampleUniformSpectrum(m_spec_values.data() + offset, a_wavelengths, size);
  }
  lightColor *= m_lights[a_lightId].mult;
  
  // get ies data for light source
  //
  const uint iesId = m_lights[a_lightId].iesId;
  if(KSPEC_LIGHT_IES != 0 && iesId != uint(-1))
  {
    if((m_lights[a_lightId].flags & LIGHT_FLAG_POINT_AREA) != 0)
      a_rayDir = normalize(to_float3(m_lights[a_lightId].pos) - a_rayPos);
    const float3 dirTrans = to_float3(m_lights[a_lightId].iesMatrix*to_float4(a_rayDir, 0.0f));
    float sintheta        = 0.0f;
    const float2 texCoord = sphereMapTo2DTexCoord((-1.0f)*dirTrans, &sintheta);
    const float4 texColor = m_textures[iesId]->sample(texCoord);
    lightColor *= texColor;
  }

  // get environment color
  //
  const uint texId = m_lights[a_lightId].texId;

  if(m_lights[a_lightId].distType == LIGHT_DIST_SPOT) // areaSpotLightAttenuation
  {
    float cos1      = m_lights[a_lightId].lightCos1;
    float cos2      = m_lights[a_lightId].lightCos2;
    float3 norm     = to_float3(m_lights[a_lightId].norm);
    float cos_theta = std::max(-dot(a_rayDir, norm), 0.0f);
    lightColor *= mylocalsmoothstep(cos2, cos1, cos_theta);

    if(KSPEC_LIGHT_PROJECTIVE != 0 && (m_lights[a_lightId].flags & LIGHT_FLAG_PROJECTIVE) != 0 && texId != uint(-1))
    {
      const float4x4 mat             = m_lights[a_lightId].iesMatrix;
      const float4 posLightClipSpace = mat*to_float4(a_rayPos, 1.0f); // 
      const float3 posLightSpaceNDC  = to_float3(posLightClipSpace)/posLightClipSpace.w;                         // perspective division
      const float2 shadowTexCoord    = float2(posLightSpaceNDC.x, posLightSpaceNDC.y)*0.5f + float2(0.5f, 0.5f); // just shift coords from [-1,1] to [0,1]  
      const float4 texColor          = m_textures[texId]->sample(shadowTexCoord);
      lightColor *= texColor;
    }
  }
  else if(KSPEC_LIGHT_ENV != 0 && texId != uint(-1))
  {
    float sintheta = 0.0f;
    const float2 texCoord  = sphereMapTo2DTexCoord(a_rayDir, &sintheta);
    const float2 texCoordT = mulRows2x4(m_lights[a_lightId].samplerRow0, m_lights[a_lightId].samplerRow1, texCoord);
    const float4 texColor  = m_textures[texId]->sample(texCoordT);
    lightColor *= texColor;
  }

  return lightColor;
}

float4 Integrator::EnvironmentColor(float3 a_dir, float& outPdf)
{
  float4 color = m_envColor;
  
  // apply tex color
  //
  const uint envTexId = m_envTexId;
  if(KSPEC_LIGHT_ENV != 0 && envTexId != uint(-1))
  {
    float sinTheta  = 1.0f;
    const float2 tc = sphereMapTo2DTexCoord(a_dir, &sinTheta);
    const float2 texCoordT = mulRows2x4(m_envSamRow0, m_envSamRow1, tc);
    
    if (sinTheta != 0.f && m_envEnableSam != 0 && m_intergatorType == INTEGRATOR_MIS_PT && m_envLightId != uint(-1))
    {
      const uint32_t offset = m_lights[m_envLightId].pdfTableOffset;
      const uint32_t sizeX  = m_lights[m_envLightId].pdfTableSizeX;
      const uint32_t sizeY  = m_lights[m_envLightId].pdfTableSizeY;

      // apply inverse texcoord transform to get phi and theta and than get correct pdf from table 
      //
      const float mapPdf = evalMap2DPdf(texCoordT, m_pdfLightData.data() + offset, int(sizeX), int(sizeY));
      outPdf = (mapPdf * 1.0f) / (2.f * M_PI * M_PI * std::max(std::abs(sinTheta), 1e-20f));  
    }

    const float4 texColor = m_textures[envTexId]->sample(texCoordT); 
    color *= texColor; 
  }

  return color;
}

Integrator::Map2DPiecewiseSample Integrator::SampleMap2D(float3 rands, uint32_t a_tableOffset, int sizeX, int sizeY)
{
  const float fw = (float)sizeX;
  const float fh = (float)sizeY;
  const float fN = fw*fh;

  float pdf = 1.0f;
  int pixelOffset = SelectIndexPropToOpt(rands.z, m_pdfLightData.data() + a_tableOffset, sizeX*sizeY+1, &pdf);

  if (pixelOffset >= sizeX*sizeY)
    pixelOffset = sizeX*sizeY - 1;

  const int yPos = pixelOffset / sizeX;
  const int xPos = pixelOffset - yPos*sizeX;

  const float texX = (1.0f / fw)*(((float)(xPos) + 0.5f) + (rands.x*2.0f - 1.0f)*0.5f);
  const float texY = (1.0f / fh)*(((float)(yPos) + 0.5f) + (rands.y*2.0f - 1.0f)*0.5f);

  Map2DPiecewiseSample result;
  result.mapPdf   = pdf*fN; 
  result.texCoord = make_float2(texX, texY);
  return result;
}

void Integrator::GetExecutionTime(const char* a_funcName, float a_out[4])
{
  if(std::string(a_funcName) == "NaivePathTrace" || std::string(a_funcName) == "NaivePathTraceBlock")
    a_out[0] = naivePtTime;
  else if(std::string(a_funcName) == "PathTrace" || std::string(a_funcName) == "PathTraceBlock")
    a_out[0] = shadowPtTime;
  else if(std::string(a_funcName) == "RayTrace" || std::string(a_funcName) == "RayTraceBlock")
    a_out[0] = raytraceTime;
  else if(std::string(a_funcName) == "PathTraceFromInputRays" || std::string(a_funcName) == "PathTraceFromInputRaysBlock")
    a_out[0] = fromRaysPtTime;
}

void Integrator::ProgressBarStart()                  { _ProgressBarStart(); }
void Integrator::ProgressBarAccum(float a_progress)  { _ProgressBarAccum(a_progress); }
void Integrator::ProgressBarDone()                   { _ProgressBarDone(); }
