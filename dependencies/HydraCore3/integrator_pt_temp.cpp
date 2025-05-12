////////////////////////////////////////////////////
//// input file: /home/sammael/LiteRTOpen/LiteRT/dependencies/HydraCore3/integrator_pt.cpp
////////////////////////////////////////////////////
#include "integrator_pt.h"
#include "include/crandom.h"

#include <chrono>
#include <string>

#include "Image2d.h"
using LiteImage::Image2D;
using LiteImage::Sampler;
using LiteImage::ICombinedImageSampler;
using namespace LiteMath;

void Integrator::InitRandomGens(int a_maxThreads)
{
  m_randomGens.resize(a_maxThreads);
  #ifndef _DEBUG
  #pragma omp parallel for default(shared)
  #endif
  for(int i=0;i<a_maxThreads;i++)
    m_randomGens[i] = RandomGenInit(i);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float  Integrator::GetRandomNumbersSpec(uint tid, RandomGen* a_gen)               { return rndFloat1_Pseudo(a_gen); }
float  Integrator::GetRandomNumbersTime(uint tid, RandomGen* a_gen)               { return rndFloat1_Pseudo(a_gen); }
float4 Integrator::GetRandomNumbersLens(uint tid, RandomGen* a_gen)               { return rndFloat4_Pseudo(a_gen); }
float4 Integrator::GetRandomNumbersMats(uint tid, RandomGen* a_gen, int a_bounce) { return rndFloat4_Pseudo(a_gen); }
float4 Integrator::GetRandomNumbersLgts(uint tid, RandomGen* a_gen, int a_bounce)
{
  const float  rndId = rndFloat1_Pseudo(a_gen); // don't use single rndFloat4 (!!!)
  const float4 rands = rndFloat4_Pseudo(a_gen); // don't use single rndFloat4 (!!!)
  return float4(rands.x, rands.y, rands.z, rndId);
}

float Integrator::GetRandomNumbersMatB(uint tid, RandomGen* a_gen, int a_bounce, int a_layer) { return rndFloat1_Pseudo(a_gen); }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint Integrator::RandomGenId(uint tid) { return tid; }

Integrator::EyeRayData Integrator::SampleCameraRay(RandomGen* pGen, uint tid)
{
  const uint XY = m_packedXY[tid];
  const uint x  = (XY & 0x0000FFFF);
  const uint y  = (XY & 0xFFFF0000) >> 16;

  if(x == 256 && y == 256)
  {
    int a = 2;
  }

  const float4 pixelOffsets = GetRandomNumbersLens(tid, pGen);

  const float fx = float(x) + pixelOffsets.x;
  const float fy = float(y) + pixelOffsets.y;
  
  const float xCoordNormalized = (fx + float(m_winStartX))/float(m_fbWidth);
  const float yCoordNormalized = (fy + float(m_winStartY))/float(m_fbHeight);

  //const float xCoordNormalized = ( fx*0.5f + float(m_winStartX)*(1.0f + 0.5f))/float(m_fbWidth);
  //const float yCoordNormalized = ( fy*1.0f + float(m_winStartY))/float(m_fbHeight);

  float3 rayDir = EyeRayDirNormalized(xCoordNormalized, yCoordNormalized, m_projInv);
  float3 rayPos = float3(0,0,0);
  
  if (m_camLensRadius > 0.0f)
  {
    const float tFocus         = m_camTargetDist / (-rayDir.z);
    const float3 focusPosition = rayPos + rayDir*tFocus;
    const float2 xy            = m_camLensRadius*2.0f*MapSamplesToDisc(float2(pixelOffsets.z - 0.5f, pixelOffsets.w - 0.5f));
    rayPos.x += xy.x;
    rayPos.y += xy.y;
    rayDir = normalize(focusPosition - rayPos);
  }
  else if(KSPEC_OPTIC_SIM !=0 && m_enableOpticSim != 0)  
  {
    const float2 xy = 0.25f*m_physSize*float2(2.0f*xCoordNormalized - 1.0f, 2.0f*yCoordNormalized - 1.0f);
    
    rayPos = float3(xy.x, xy.y, 0);
    
    const float2 rareSam  = LensRearRadius()*2.0f*MapSamplesToDisc(float2(pixelOffsets.z - 0.5f, pixelOffsets.w - 0.5f));
    const float3 shootTo  = float3(rareSam.x, rareSam.y, LensRearZ());
    const float3 ray_dirF = normalize(shootTo - rayPos);
    
    float cosTheta = std::abs(ray_dirF.z);
    rayDir         = ray_dirF;
    bool success   = TraceLensesFromFilm(rayPos, rayDir);
    
    if (!success) 
    {
      rayPos = float3(0,-10000000.0,0.0); // shoot ray under the floor
      rayDir = float3(0,-1,0);
    }
    else
    {
      rayDir = float3(-1,-1,-1)*normalize(rayDir);
      rayPos = float3(-1,-1,-1)*rayPos;
    }
  }

  EyeRayData res;
  {
    res.rayPos = rayPos;
    res.rayDir = rayDir;
    res.x      = x;
    res.y      = y;
    res.timeSam = 0.0f;
    res.waveSam = 1.0f;
    if(m_normMatrices2.size() != 0)
      res.timeSam = GetRandomNumbersTime(tid, pGen);
    if(KSPEC_SPECTRAL_RENDERING !=0 && m_spectral_mode != 0)
      res.waveSam = GetRandomNumbersSpec(tid, pGen);
    res.cosTheta = 1.0f;
  }
  
  RecordPixelRndIfNeeded(pixelOffsets, float2(res.waveSam,res.timeSam));

  return res;
}


void Integrator::kernel_InitEyeRay2(uint tid, float4* rayPosAndNear, float4* rayDirAndFar, float4* wavelengths, 
                                    float4* accumColor,    float4* accumuThoroughput,
                                    RandomGen* gen, uint* rayFlags, MisData* misData, float* time) // 
{

  if(tid >= m_maxThreadId)
    return;

  *accumColor        = make_float4(0,0,0,0);
  *accumuThoroughput = make_float4(1,1,1,1);
  RandomGen genLocal = m_randomGens[RandomGenId(tid)];
  *rayFlags          = 0;
  *misData           = makeInitialMisData();

  EyeRayData r = SampleCameraRay(&genLocal, tid);
  
  if(KSPEC_SPECTRAL_RENDERING !=0 && m_spectral_mode != 0)
    *wavelengths = SampleWavelengths(r.waveSam, LAMBDA_MIN, LAMBDA_MAX);
  else
    *wavelengths = float4(0.0f);

  *time = r.timeSam;
 
  transform_ray3f(m_worldViewInv, &r.rayPos, &r.rayDir);

  *rayPosAndNear = to_float4(r.rayPos, 0.0f);
  *rayDirAndFar  = to_float4(r.rayDir, FLT_MAX);
  *gen           = genLocal;
}

void Integrator::kernel_InitEyeRayFromInput(uint tid, const RayPosAndW* in_rayPosAndNear, const RayDirAndT* in_rayDirAndFar,
                                            float4* rayPosAndNear, float4* rayDirAndFar, float4* accumColor, float4* accumuThoroughput, 
                                            RandomGen* gen, uint* rayFlags, MisData* misData, float4* wavelengths, float* time)
{
  if(tid >= m_maxThreadId)
    return;

  *accumColor        = make_float4(0,0,0,0);
  *accumuThoroughput = make_float4(1,1,1,1);
  *rayFlags          = 0;
  *misData           = makeInitialMisData();  

  //const int x = int(tid) % m_winWidth;
  //const int y = int(tid) / m_winHeight;

  const RayPosAndW rayPosData = in_rayPosAndNear[tid];
  const RayDirAndT rayDirData = in_rayDirAndFar[tid];

  float3 rayPos = float3(rayPosData.origin[0], rayPosData.origin[1], rayPosData.origin[2]);
  float3 rayDir = float3(rayDirData.direction[0], rayDirData.direction[1], rayDirData.direction[2]);
  transform_ray3f(m_worldViewInv, &rayPos, &rayDir);

  if(KSPEC_SPECTRAL_RENDERING !=0 && m_spectral_mode != 0)
  {
    *wavelengths = float4(rayPosData.wave);
    //const uint2 wavesXY = unpackXY1616(rayPosData.waves01);
    //const uint2 wavesZW = unpackXY1616(rayDirData.waves23);
    //const float scale = (1.0f/65535.0f)*(LAMBDA_MAX - LAMBDA_MIN);
    //*wavelengths = float4(float(wavesXY[0])*scale + LAMBDA_MIN,
    //                      float(wavesXY[1])*scale + LAMBDA_MIN,
    //                      float(wavesZW[0])*scale + LAMBDA_MIN,
    //                      float(wavesZW[1])*scale + LAMBDA_MIN);
  }
  else
    *wavelengths = float4(0,0,0,0);

  *rayPosAndNear = to_float4(rayPos, 0.0f);
  *rayDirAndFar  = to_float4(rayDir, FLT_MAX);
  *time          = rayDirData.time;
  *gen           = m_randomGens[RandomGenId(tid)];
}

float3 decode_normal(float2 e)
{
  float3 v = float3(e.x, e.y, 1.0f - std::abs(e.x) - std::abs(e.y));
  if (v.z < 0) 
  {
    float vx = v.x;
    v.x = (1.0f - std::abs(v.y)) * ((v.x >= 0.0) ? +1.0f : -1.0f);
    v.y = (1.0f - std::abs( vx)) * ((v.y >= 0.0) ? +1.0f : -1.0f);
  }
  return normalize(v);
}


void Integrator::kernel_RayTrace2(uint tid, uint bounce, const float4* rayPosAndNear, const float4* rayDirAndFar, const float* a_time,
                                  float4* out_hit1, float4* out_hit2, float4* out_hit3, uint* out_instId, uint* rayFlags)
{
  if(tid >= m_maxThreadId)
    return;
  uint currRayFlags = *rayFlags;
  if(isDeadRay(currRayFlags))
    return;

  const float4 rayPos = *rayPosAndNear;
  const float4 rayDir = *rayDirAndFar ;
  const float  time   = *a_time;

  const CRT_Hit hit   = m_pAccelStruct->RayQuery_NearestHitMotion(rayPos, rayDir, time);
  RecordRayHitIfNeeded(bounce, hit);

  if(hit.geomId != uint32_t(-1))
  {
const uint32_t type = (hit.geomId >> SH_TYPE);
    const uint32_t geomId = hit.geomId & GEOM_ID_MASK;
    const float3 hitPos = to_float3(rayPos) + hit.t * (1.f - 1e-6f) * to_float3(rayDir);

#ifdef LITERT_RENDERER
    if (type == TYPE_MESH_TRIANGLE)
#endif
    {
      const float2 uv     = float2(hit.coords[0], hit.coords[1]);
      
      // slightly undershoot the intersection to prevent self-intersection and other bugs

      // const float3 overHit  = to_float3(rayPos) + hit.t * (1.f + 1e-6f) * to_float3(rayDir);

      // alternative, you may consider Johannes Hanika solution from  Ray Tracing Gems2  
      /////////////////////////////////////////////////////////////////////////////////
      // // get distance vectors from triangle vertices
      // vec3 tmpu = P - A, tmpv = P - B, tmpw = P - C
      // // project these onto the tangent planes
      // // defined by the shading normals
      // float dotu = min (0.0, dot(tmpu , nA))
      // float dotv = min (0.0, dot(tmpv , nB))
      // float dotw = min (0.0, dot(tmpw , nC))
      // tmpu -= dotu*nA
      // tmpv -= dotv*nB
      // tmpw -= dotw*nC
      // // finally P' is the barycentric mean of these three
      // vec3 Pp = P + u*tmpu + v*tmpv + w*tmpw
      /////////////////////////////////////////////////////////////////////////////////

      const uint triOffset  = m_matIdOffsets[geomId];
      const uint vertOffset = m_vertOffset  [geomId];
    
      const uint A = m_triIndices[(triOffset + hit.primId)*3 + 0];
      const uint B = m_triIndices[(triOffset + hit.primId)*3 + 1];
      const uint C = m_triIndices[(triOffset + hit.primId)*3 + 2];

      const float4 data1 = (1.0f - uv.x - uv.y)*m_vNorm4f[A + vertOffset] + uv.y*m_vNorm4f[B + vertOffset] + uv.x*m_vNorm4f[C + vertOffset];
      const float4 data2 = (1.0f - uv.x - uv.y)*m_vTang4f[A + vertOffset] + uv.y*m_vTang4f[B + vertOffset] + uv.x*m_vTang4f[C + vertOffset];
      float2 hitTexCoord = float2(data1.w, data2.w);

      float3 hitNorm, hitTang;
// #ifdef LITERT_RENDERER
//       if (m_pAccelStruct->().mesh_normal_mode == MESH_NORMAL_MODE_GEOMETRY)
//       {
//         //normals for are calculated along with the hit calculation
//         const float3 n(hit.coords[2], hit.coords[3], sqrt(max(0.0f, 1-hit.coords[2]*hit.coords[2] - hit.coords[3]*hit.coords[3])));
//         const float len = length(n);
//         hitNorm = len > 1e-9f ? n / len : float3(1.0f, 0.0f, 0.0f);

//         //it is not always good, but we do not expect that tangent will be used at all
//         hitTang = float3(0.0f, 1.0f, 0.0f);
//       }
//       else
// #endif
      {
        hitNorm     = to_float3(data1);
        hitTang     = to_float3(data2);

        // transform surface point with matrix and flip normal if needed
        //
        hitNorm = mul3x3(m_normMatrices[hit.instId], hitNorm);
        hitTang = mul3x3(m_normMatrices[hit.instId], hitTang);
      }
    
      if(m_normMatrices2.size() > 0)
      {
        float3 hitNorm2 = mul3x3(m_normMatrices2[hit.instId], hitNorm);
        float3 hitTang2 = mul3x3(m_normMatrices2[hit.instId], hitTang);

        hitNorm = lerp(hitNorm, hitNorm2, time);
        hitTang = lerp(hitTang, hitTang2, time);
      }

      hitNorm = normalize(hitNorm);
      hitTang = normalize(hitTang);
      
      const float flipNorm = dot(to_float3(rayDir), hitNorm) > 0.001f ? -1.0f : 1.0f; // beware of transparent materials which use normal sign to identity "inside/outside" glass for example
      hitNorm              = flipNorm * hitNorm;
      hitTang              = flipNorm * hitTang; // do we need this ??

      if (flipNorm < 0.0f) currRayFlags |=  RAY_FLAG_HAS_INV_NORMAL;
      else                 currRayFlags &= ~RAY_FLAG_HAS_INV_NORMAL;
      
      const uint midOriginal = m_matIdByPrimId[m_matIdOffsets[geomId] + hit.primId];
      const uint midRemaped  = RemapMaterialId(midOriginal, hit.instId);

      *rayFlags              = packMatId(currRayFlags, midRemaped);
      *out_hit1              = to_float4(hitPos,  hitTexCoord.x); 
      *out_hit2              = to_float4(hitNorm, hitTexCoord.y);
      *out_hit3              = to_float4(hitTang, hit.t);
      *out_instId            = hit.instId;
    }
#ifdef LITERT_RENDERER
    else
    {
      //no normal flip/remap on SDFs and other obscure stuff
      currRayFlags &= ~RAY_FLAG_HAS_INV_NORMAL;

      //only one material per geometry without remaps
      const uint matId = m_matIdByPrimId[m_matIdOffsets[geomId]];

      //no texture coordinates, only constant color materials
      const float2 tc = float2(0,0);

      //normals for SDFs are calculated along with the hit calculation
      const float3 n = decode_normal(float2(hit.coords[2], hit.coords[3]));
    
      const float len = length(n);
      const float3 hitNorm = len > 1e-9f ? n / len : float3(1.0f, 0.0f, 0.0f);

      //it is not always good, but we do not expect that tangent will be used at all
      const float3 hitTang = normalize(cross(hitNorm, float3(0.0f, 1.0f, 0.0f)));

      *rayFlags              = packMatId(currRayFlags, matId);
      *out_hit1              = to_float4(hitPos,  tc.x); 
      *out_hit2              = to_float4(hitNorm, tc.y);
      *out_hit3              = to_float4(hitTang, hit.t);
      *out_instId            = hit.instId;
    }
#endif
  }
  else
  {
    const uint flagsToAdd = (bounce == 0) ? (RAY_FLAG_PRIME_RAY_MISS | RAY_FLAG_IS_DEAD | RAY_FLAG_OUT_OF_SCENE) : (RAY_FLAG_IS_DEAD | RAY_FLAG_OUT_OF_SCENE);
    *rayFlags             = currRayFlags | flagsToAdd;
  }
}

void Integrator::kernel_SampleLightSource(uint tid, const float4* rayPosAndNear, const float4* rayDirAndFar, const float4* wavelengths,
                                          const float4* in_hitPart1, const float4* in_hitPart2, const float4* in_hitPart3,
                                          const uint* rayFlags, const float* a_time, uint bounce, RandomGen* a_gen, float4* out_shadeColor)
{
  if(tid >= m_maxThreadId)
    return;
  const uint currRayFlags = *rayFlags;
  if(isDeadRay(currRayFlags))
    return;
    
  const uint32_t matId = extractMatId(currRayFlags);
  const float3 ray_dir = to_float3(*rayDirAndFar);
  
  const float4 data1  = *in_hitPart1;
  const float4 data2  = *in_hitPart2;
  const float4 lambda = *wavelengths;

  SurfaceHit hit;
  hit.pos  = to_float3(data1);
  hit.norm = to_float3(data2);
  hit.tang = to_float3(*in_hitPart3);
  hit.uv   = float2(data1.w, data2.w);
  
  const int bounceTmp = int(bounce); 
  const float4 rands = GetRandomNumbersLgts(tid, a_gen, bounceTmp); 
  const int lightId  = std::min(int(std::floor(rands.w * float(m_lights.size()))), int(m_lights.size() - 1u));
  RecordLightRndIfNeeded(bounce, rands); 

  if(lightId < 0) // no lights or invalid light id
  {
    *out_shadeColor = float4(0.0f, 0.0f, 0.0f, 0.0f);
    return;
  }
  
  const LightSample lSam = LightSampleRev(lightId, to_float3(rands), hit.pos);
  const float  hitDist   = std::sqrt(dot(hit.pos - lSam.pos, hit.pos - lSam.pos));

  const float3 shadowRayDir = normalize(lSam.pos - hit.pos); // explicitSam.direction;
  const float3 shadowRayPos = hit.pos + hit.norm * std::max(maxcomp(hit.pos), 1.0f)*5e-6f; // TODO: see Ray Tracing Gems, also use flatNormal for offset

  float time = *a_time;
  const bool   inIllumArea  = (dot(shadowRayDir, lSam.norm) < 0.0f) || lSam.isOmni || lSam.hasIES;
  const bool   needShade    = inIllumArea && !m_pAccelStruct->RayQuery_AnyHitMotion(to_float4(shadowRayPos, 0.0f), to_float4(shadowRayDir, hitDist*0.9995f), time); /// (!!!) expression-way, RT pipeline bug work around, if change check test_213
  RecordShadowHitIfNeeded(bounce, needShade);

  if(needShade) /// (!!!) expression-way to compute 'needShade', RT pipeline bug work around, if change check test_213
  {
    const BsdfEval bsdfV    = MaterialEval(matId, lambda, shadowRayDir, (-1.0f)*ray_dir, hit.norm, hit.tang, hit.uv);
    float cosThetaOut       = std::max(dot(shadowRayDir, hit.norm), 0.0f);
    
    float      lgtPdfW      = LightPdfSelectRev(lightId) * LightEvalPDF(lightId, shadowRayPos, shadowRayDir, lSam.pos, lSam.norm, lSam.pdf);
    float      misWeight    = (m_intergatorType == INTEGRATOR_MIS_PT) ? misWeightHeuristic(lgtPdfW, bsdfV.pdf) : 1.0f;
    const bool isDirect     = (m_lights[lightId].geomType == LIGHT_GEOM_DIRECT); 
    const bool isPoint      = (m_lights[lightId].geomType == LIGHT_GEOM_POINT); 
    
    if(isDirect)
    {
      misWeight = 1.0f;
      lgtPdfW   = 1.0f;
    }
    else if(isPoint)
      misWeight = 1.0f;

    const bool isDirectLight = !hasNonSpecular(currRayFlags);
    if((m_renderLayer == FB_DIRECT   && !isDirectLight) || 
       (m_renderLayer == FB_INDIRECT && isDirectLight)) // skip some number of bounces if this is set
      misWeight = 0.0f;
      
    
    const float4 lightColor = LightIntensity(lightId, lambda, shadowRayPos, shadowRayDir);
    *out_shadeColor = (lightColor * bsdfV.val / lgtPdfW) * cosThetaOut * misWeight;
  }
  else
    *out_shadeColor = float4(0.0f, 0.0f, 0.0f, 0.0f);
}

void Integrator::kernel_NextBounce(uint tid, uint bounce, const float4* in_hitPart1, const float4* in_hitPart2, const float4* in_hitPart3,
                                   const uint* in_instId, const float4* in_shadeColor, float4* rayPosAndNear, float4* rayDirAndFar,
                                   const float4* wavelengths, float4* accumColor, float4* accumThoroughput,
                                   RandomGen* a_gen, MisData* misPrev, uint* rayFlags)
{
  if(tid >= m_maxThreadId)
    return;
  const uint currRayFlags = *rayFlags;
  if(isDeadRay(currRayFlags))
    return;
    
  const uint32_t matId = extractMatId(currRayFlags);

  // process surface hit case
  //
  const float3 ray_dir = to_float3(*rayDirAndFar);
  const float3 ray_pos = to_float3(*rayPosAndNear);
  const float4 lambda  = *wavelengths;
  
  const float4 data1 = *in_hitPart1;
  const float4 data2 = *in_hitPart2;
  
  SurfaceHit hit;
  hit.pos  = to_float3(data1);
  hit.norm = to_float3(data2);
  hit.tang = to_float3(*in_hitPart3);
  hit.uv   = float2(data1.w, data2.w);

  const float hitDist = in_hitPart3->w;
  
  const MisData prevBounce = *misPrev;
  const float   prevPdfW   = prevBounce.matSamplePdf;

  // process light hit case
  //
  if(m_materials[matId].mtype == MAT_TYPE_LIGHT_SOURCE)
  {
    const uint   texId     = m_materials[matId].texid[0];
    const float2 texCoordT = mulRows2x4(m_materials[matId].row0[0], m_materials[matId].row1[0], hit.uv);
    const float4 texColor  = m_textures[texId]->sample(texCoordT);
    const uint   lightId   = m_instIdToLightInstId[*in_instId]; 
    
    const float4 emissColor = m_materials[matId].colors[EMISSION_COLOR];
    float4 lightIntensity   = emissColor * texColor;

    if(lightId != 0xFFFFFFFF)
    {
      const float lightCos = dot(to_float3(*rayDirAndFar), to_float3(m_lights[lightId].norm));
      const float lightDirectionAtten = (lightCos < 0.0f || m_lights[lightId].geomType == LIGHT_GEOM_SPHERE) ? 1.0f : 0.0f;
      lightIntensity = LightIntensity(lightId, lambda, ray_pos, to_float3(*rayDirAndFar))*lightDirectionAtten;
    }

    float misWeight = 1.0f;
    if(m_intergatorType == INTEGRATOR_MIS_PT) 
    {
      if(bounce > 0 && lightId != 0xFFFFFFFF)
      {
        const float lgtPdf  = LightPdfSelectRev(lightId) * LightEvalPDF(lightId, ray_pos, ray_dir, hit.pos, hit.norm, 1.0f);
        misWeight           = misWeightHeuristic(prevPdfW, lgtPdf);
        if (prevPdfW <= 0.0f) // specular bounce
          misWeight = 1.0f;
      }
    }
    else if(m_intergatorType == INTEGRATOR_SHADOW_PT && hasNonSpecular(currRayFlags))
      misWeight = 0.0f;
    
    const bool isDirectLight  = !hasNonSpecular(currRayFlags);
    const bool isFirstNonSpec = (currRayFlags & RAY_FLAG_FIRST_NON_SPEC) != 0;
    if(m_renderLayer == FB_INDIRECT && (isDirectLight || isFirstNonSpec))
      misWeight = 0.0f;

    float4 currAccumColor      = *accumColor;
    float4 currAccumThroughput = *accumThoroughput;
    
    currAccumColor += currAccumThroughput * lightIntensity * misWeight;
   
    *accumColor = currAccumColor;
    *rayFlags   = currRayFlags | (RAY_FLAG_IS_DEAD | RAY_FLAG_HIT_LIGHT);
    return;
  }
  
  const uint bounceTmp    = bounce;
  const BsdfSample matSam = MaterialSampleAndEval(matId, tid, bounceTmp, lambda, a_gen, (-1.0f)*ray_dir, hit.norm, hit.tang, hit.uv, misPrev, currRayFlags);
  const float4 bxdfVal    = matSam.val * (1.0f / std::max(matSam.pdf, 1e-20f));
  const float  cosTheta   = std::abs(dot(matSam.dir, hit.norm)); 

  MisData nextBounceData      = *misPrev;        // remember current pdfW for next bounce
  nextBounceData.matSamplePdf = (matSam.flags & RAY_EVENT_S) != 0 ? -1.0f : matSam.pdf; 
  nextBounceData.cosTheta     = cosTheta;   
  *misPrev                    = nextBounceData;

  if(m_intergatorType == INTEGRATOR_STUPID_PT)
  {
    *accumThoroughput *= cosTheta * bxdfVal; 
  }
  else if(m_intergatorType == INTEGRATOR_SHADOW_PT || m_intergatorType == INTEGRATOR_MIS_PT)
  {
    const float4 currThoroughput = *accumThoroughput;
    const float4 shadeColor      = *in_shadeColor;
    float4 currAccumColor        = *accumColor;

    currAccumColor += currThoroughput * shadeColor;
    *accumColor       = currAccumColor;
    *accumThoroughput = currThoroughput*cosTheta*bxdfVal; 
  }

  // compute point on the other side of the surface in case of transmission
  if((matSam.flags & RAY_EVENT_T) != 0)
  {
    hit.pos = hit.pos + hitDist * ray_dir * 2 * 1e-6f;
  }  

  *rayPosAndNear = to_float4(OffsRayPos(hit.pos, hit.norm, matSam.dir), 0.0f); // todo: use flatNormal for offset
  *rayDirAndFar  = to_float4(matSam.dir, FLT_MAX);
  
  uint nextFlags = ((currRayFlags & ~RAY_FLAG_FIRST_NON_SPEC) | matSam.flags); // always force reset RAY_FLAG_FIRST_NON_SPEC;
  if(m_renderLayer == FB_DIRECT && hasNonSpecular(currRayFlags))   // NOTE: use currRayFlags for check, not nextFlags because of MIS: a ray may hit light source in next bounce
    nextFlags |= RAY_FLAG_IS_DEAD;                                 //       but if we already have non specular bounce previously, definitely can stop  
  else if(!hasNonSpecular(currRayFlags) && hasNonSpecular(nextFlags))
    nextFlags |= RAY_FLAG_FIRST_NON_SPEC;
  *rayFlags      = nextFlags;                                   
}

void Integrator::kernel_HitEnvironment(uint tid, const uint* rayFlags, const float4* rayDirAndFar, const MisData* a_prevMisData, const float4* accumThoroughput,
                                       float4* accumColor)
{
  if(tid >= m_maxThreadId)
    return;
  const uint currRayFlags = *rayFlags;
  if(!isOutOfScene(currRayFlags))
    return;
  
  float envPdf = 1.0f;
  float4 envColor = EnvironmentColor(to_float3(*rayDirAndFar), envPdf);

  const auto misPrev  = *a_prevMisData;
  const bool isSpec   = isSpecular(&misPrev);
  const bool exitZero = (currRayFlags & RAY_FLAG_PRIME_RAY_MISS) != 0;

  if(m_intergatorType == INTEGRATOR_MIS_PT && m_envEnableSam != 0 && !isSpec && !exitZero)
  {
    float lgtPdf    = LightPdfSelectRev(m_envLightId)*envPdf;
    float bsdfPdf   = misPrev.matSamplePdf;
    float misWeight = misWeightHeuristic(bsdfPdf, lgtPdf); // (bsdfPdf*bsdfPdf) / (lgtPdf*lgtPdf + bsdfPdf*bsdfPdf);
    envColor *= misWeight;    
  }
  else if(m_intergatorType == INTEGRATOR_SHADOW_PT && m_envEnableSam != 0)
  {
    envColor = float4(0.0f);
  }
  
  const uint camBackId = m_envCamBackId;
  if(exitZero && camBackId != uint(-1)) // apply camera back color to ray
  {
    const uint XY = m_packedXY[tid];
    const uint x  = (XY & 0x0000FFFF);
    const uint y  = (XY & 0xFFFF0000) >> 16;

    const float2 texCoord = float2((float(x) + 0.5f)/float(m_winWidth), 
                                   (float(y) + 0.5f)/float(m_winHeight));

    envColor = m_textures[camBackId]->sample(texCoord);
  }
 
  if(m_intergatorType == INTEGRATOR_STUPID_PT)     // todo: when explicit sampling will be added, disable contribution here for 'INTEGRATOR_SHADOW_PT'
    *accumColor = (*accumThoroughput) * envColor;
  else
    *accumColor += (*accumThoroughput) * envColor;
}


void Integrator::kernel_ContributeToImage(uint tid, const uint* rayFlags, uint channels, const float4* a_accumColor, const RandomGen* gen,
                                          const uint* in_pakedXY, const float4* wavelengths, float* out_color)
{
  
  if(tid >= m_maxThreadId) // don't contrubute to image in any "record" mode
    return;
  
  m_randomGens[RandomGenId(tid)] = *gen;
  if(m_disableImageContrib !=0)
    return;

  const uint XY = in_pakedXY[tid];
  const uint x  = (XY & 0x0000FFFF);
  const uint y  = (XY & 0xFFFF0000) >> 16;
  
  float4 specSamples = *a_accumColor; 
  float4 tmpVal      = specSamples*m_camRespoceRGB;
  float3 rgb         = to_float3(tmpVal);

  if(KSPEC_SPECTRAL_RENDERING!=0 && m_spectral_mode != 0) 
  {
    const float4 waves   = *wavelengths;
    const uint rayFlags2 = *rayFlags; 
    rgb = SpectralCamRespoceToRGB(specSamples, waves, rayFlags2);
  }

  float4 colorRes = m_exposureMult * to_float4(rgb, 1.0f);
  
  if(channels == 1) // monochromatic spectral
  {
    // const float mono = 0.2126f*colorRes.x + 0.7152f*colorRes.y + 0.0722f*colorRes.z;
    out_color[y * m_winWidth + x] += specSamples.x * m_exposureMult;
  } 
  else if(channels <= 4) 
  {
    out_color[(y*m_winWidth+x)*channels + 0] += colorRes.x;
    out_color[(y*m_winWidth+x)*channels + 1] += colorRes.y;
    out_color[(y*m_winWidth+x)*channels + 2] += colorRes.z;
  }
  else // always spectral rendering
  {
    auto waves = (*wavelengths);
    auto color = (*a_accumColor)*m_exposureMult;
    for(int i=0;i<4;i++) {
      const float t         = (waves[i] - LAMBDA_MIN)/(LAMBDA_MAX-LAMBDA_MIN);
      const int channelId   = std::min(int(float(channels)*t), int(channels)-1);
      const int offsetPixel = int(y)*m_winWidth + int(x);
      const int offsetLayer = channelId*m_winWidth*m_winHeight;
      out_color[offsetLayer + offsetPixel] += color[i];
    }
  }

}

void Integrator::kernel_CopyColorToOutput(uint tid, uint channels, const float4* a_accumColor, const RandomGen* gen, float* out_color)
{
  if(tid >= m_maxThreadId)
    return;
  
  const float4 color = *a_accumColor;

  if(channels == 4)
  {
    out_color[tid*4+0] += color[0];
    out_color[tid*4+1] += color[1];
    out_color[tid*4+2] += color[2];
    out_color[tid*4+3] += color[3];
  }
  else if(channels == 1)
    out_color[tid] += color[0];

  m_randomGens[RandomGenId(tid)] = *gen;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Integrator::NaivePathTrace(uint tid, uint channels, float* out_color)
{
  float4 accumColor, accumThroughput;
  float4 rayPosAndNear, rayDirAndFar;
  float4 wavelengths;
  RandomGen gen; 
  MisData   mis;
  uint      rayFlags;
  float     time;
  kernel_InitEyeRay2(tid, &rayPosAndNear, &rayDirAndFar, &wavelengths, &accumColor, &accumThroughput, &gen, &rayFlags, &mis, &time);

  for(uint depth = 0; depth < m_traceDepth + 1; ++depth) // + 1 due to NaivePT uses additional bounce to hit light 
  {
    float4 shadeColor, hitPart1, hitPart2, hitPart3;
    uint instId = 0;
    kernel_RayTrace2(tid, depth, &rayPosAndNear, &rayDirAndFar, &time, 
                     &hitPart1, &hitPart2, &hitPart3, &instId, &rayFlags);
    if(isDeadRay(rayFlags))
      break;
    
    kernel_NextBounce(tid, depth, &hitPart1, &hitPart2, &hitPart3, &instId, &shadeColor,
                      &rayPosAndNear, &rayDirAndFar, &wavelengths, &accumColor, &accumThroughput, &gen, &mis, &rayFlags);
    if(isDeadRay(rayFlags))
      break;
  }

  kernel_HitEnvironment(tid, &rayFlags, &rayDirAndFar, &mis, &accumThroughput,
                        &accumColor);

  kernel_ContributeToImage(tid, &rayFlags, channels, &accumColor, &gen, m_packedXY.data(), &wavelengths, 
                           out_color);
}

void Integrator::PathTrace(uint tid, uint channels, float* out_color)
{
  float4 accumColor, accumThroughput;
  float4 rayPosAndNear, rayDirAndFar;
  float4 wavelengths;
  RandomGen gen; 
  MisData   mis;
  uint      rayFlags;
  float     time;
  kernel_InitEyeRay2(tid, &rayPosAndNear, &rayDirAndFar, &wavelengths, &accumColor, &accumThroughput, &gen, &rayFlags, &mis, &time);

  for(uint depth = 0; depth < m_traceDepth; depth++) 
  {
    float4   shadeColor, hitPart1, hitPart2, hitPart3;
    uint instId;
    kernel_RayTrace2(tid, depth, &rayPosAndNear, &rayDirAndFar, &time, 
                     &hitPart1, &hitPart2, &hitPart3, &instId, &rayFlags);
    if(isDeadRay(rayFlags))
      break;
    
    kernel_SampleLightSource(tid, &rayPosAndNear, &rayDirAndFar, &wavelengths, &hitPart1, &hitPart2, &hitPart3, &rayFlags, &time,
                             depth, &gen, &shadeColor);

    kernel_NextBounce(tid, depth, &hitPart1, &hitPart2, &hitPart3, &instId, &shadeColor,
                      &rayPosAndNear, &rayDirAndFar, &wavelengths, &accumColor, &accumThroughput, &gen, &mis, &rayFlags);

    if(isDeadRay(rayFlags))
      break;
  }

  kernel_HitEnvironment(tid, &rayFlags, &rayDirAndFar, &mis, &accumThroughput,
                        &accumColor);

  kernel_ContributeToImage(tid, &rayFlags, channels, &accumColor, &gen, m_packedXY.data(), &wavelengths, out_color);
}

void Integrator::PathTraceFromInputRays(uint tid, uint channels, const RayPosAndW* in_rayPosAndNear, const RayDirAndT* in_rayDirAndFar, float* out_color)
{
  float4 accumColor, accumThroughput;
  float4 rayPosAndNear, rayDirAndFar;
  float4 wavelengths;
  RandomGen gen; 
  MisData   mis;
  uint      rayFlags;
  float     time;
  kernel_InitEyeRayFromInput(tid, in_rayPosAndNear, in_rayDirAndFar, 
                             &rayPosAndNear, &rayDirAndFar, &accumColor, &accumThroughput, &gen, &rayFlags, &mis, &wavelengths, &time);
  
  for(uint depth = 0; depth < m_traceDepth; depth++) 
  {
    float4 shadeColor, hitPart1, hitPart2, hitPart3;
    uint instId;
    kernel_RayTrace2(tid, depth, &rayPosAndNear, &rayDirAndFar, &time, 
                     &hitPart1, &hitPart2, &hitPart3, &instId, &rayFlags);
    if(isDeadRay(rayFlags))
      break;
    
    kernel_SampleLightSource(tid, &rayPosAndNear, &rayDirAndFar, &wavelengths, &hitPart1, &hitPart2, &hitPart3, &rayFlags, &time,
                             depth, &gen, &shadeColor);

    kernel_NextBounce(tid, depth, &hitPart1, &hitPart2, &hitPart3, &instId, &shadeColor, &rayPosAndNear, &rayDirAndFar,
                      &wavelengths, &accumColor, &accumThroughput, &gen, &mis, &rayFlags);

    if(isDeadRay(rayFlags))
      break;
  }

  kernel_HitEnvironment(tid, &rayFlags, &rayDirAndFar, &mis, &accumThroughput,
                        &accumColor);
  
  //////////////////////////////////////////////////// same as for PathTrace

  kernel_CopyColorToOutput(tid, channels, &accumColor, &gen, out_color);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static inline bool Quadratic(float A, float B, float C, float *t0, float *t1) {
  // Find quadratic discriminant
  double discrim = (double)B * (double)B - 4. * (double)A * (double)C;
  if (discrim < 0.) 
    return false;
  double rootDiscrim = std::sqrt(discrim);
  float floatRootDiscrim   = float(rootDiscrim);
  // Compute quadratic _t_ values
  float q;
  if ((float)B < 0)
      q = -.5 * (B - floatRootDiscrim);
  else
      q = -.5 * (B + floatRootDiscrim);
  *t0 = q / A;
  *t1 = C / q;
  if ((float)*t0 > (float)*t1) 
  {
    // std::swap(*t0, *t1);
    float temp = *t0;
    *t0 = *t1;
    *t1 = temp;
  }
  return true;
}

static inline bool Refract(const float3 wi, const float3 n, float eta, float3 *wt) {
  // Compute $\cos \theta_\roman{t}$ using Snell's law
  float cosThetaI  = dot(n, wi);
  float sin2ThetaI = std::max(float(0), float(1.0f - cosThetaI * cosThetaI));
  float sin2ThetaT = eta * eta * sin2ThetaI;
  // Handle total internal reflection for transmission
  if (sin2ThetaT >= 1) return false;
  float cosThetaT = std::sqrt(1 - sin2ThetaT);
  *wt = eta * (-1.0f)*wi + (eta * cosThetaI - cosThetaT) * n;
  return true;
}

static inline float3 faceforward(const float3 n, const float3 v) { return (dot(n, v) < 0.f) ? (-1.0f)*n : n; }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Integrator::IntersectSphericalElement(float radius, float zCenter, float3 rayPos, float3 rayDir, 
                                              float *t, float3 *n) const
{
  // Compute _t0_ and _t1_ for ray--element intersection
  const float3 o = rayPos - float3(0, 0, zCenter);
  const float  A = rayDir.x * rayDir.x + rayDir.y * rayDir.y + rayDir.z * rayDir.z;
  const float  B = 2 * (rayDir.x * o.x + rayDir.y * o.y + rayDir.z * o.z);
  const float  C = o.x * o.x + o.y * o.y + o.z * o.z - radius * radius;
  float  t0, t1;
  if (!Quadratic(A, B, C, &t0, &t1)) 
    return false;
  
  // Select intersection $t$ based on ray direction and element curvature
  bool useCloserT = (rayDir.z > 0.0f) != (radius < 0.0);
  *t = useCloserT ? std::min(t0, t1) : std::max(t0, t1);
  if (*t < 0.0f) 
    return false;
  
  // Compute surface normal of element at ray intersection point
  *n = normalize(o + (*t)*rayDir);
  *n = faceforward(*n, -1.0f*rayDir);
  return true;
}

bool Integrator::TraceLensesFromFilm(float3& inoutRayPos, float3& inoutRayDir) const
{
  float elementZ = 0;
  // Transform _rCamera_ from camera to lens system space
  // 
  float3 rayPosLens = float3(inoutRayPos.x, inoutRayPos.y, -inoutRayPos.z);
  float3 rayDirLens = float3(inoutRayDir.x, inoutRayDir.y, -inoutRayDir.z);

  for(int i=0; i<m_lines.size(); i++)
  {
    const auto element = m_lines[i];                                  
    // Update ray from film accounting for interaction with _element_
    elementZ -= element.thickness;
    
    // Compute intersection of ray with lens element
    float t;
    float3 n;
    bool isStop = (element.curvatureRadius == 0.0f);
    if (isStop) 
    {
      // The refracted ray computed in the previous lens element
      // interface may be pointed towards film plane(+z) in some
      // extreme situations; in such cases, 't' becomes negative.
      if (rayDirLens.z >= 0.0f) 
        return false;
      t = (elementZ - rayPosLens.z) / rayDirLens.z;
    } 
    else 
    {
      const float radius  = element.curvatureRadius;
      const float zCenter = elementZ + element.curvatureRadius;
      if (!IntersectSphericalElement(radius, zCenter, rayPosLens, rayDirLens, &t, &n))
        return false;
    }

    // Test intersection point against element aperture
    const float3 pHit = rayPosLens + t*rayDirLens;
    const float r2    = pHit.x * pHit.x + pHit.y * pHit.y;
    if (r2 > element.apertureRadius * element.apertureRadius) 
      return false;
    
    rayPosLens = pHit;
    // Update ray path for from-scene element interface interaction
    if (!isStop) 
    {
      float3 wt;
      float etaI = m_lines[i+0].eta;                                                      
      float etaT = (i == m_lines.size()-1) ? 1.0f : m_lines[i+1].eta;
      if(etaT == 0.0f)
        etaT = 1.0f;                                                          
      if (!Refract(normalize((-1.0f)*rayDirLens), n, etaI / etaT, &wt))
        return false;
      rayDirLens = wt;
    }

  }

  // Transform _rLens_ from lens system space back to camera space
  //
  inoutRayPos = float3(rayPosLens.x, rayPosLens.y, -rayPosLens.z);
  inoutRayDir = float3(rayDirLens.x, rayDirLens.y, -rayDirLens.z);
  return true;  
}
////////////////////////////////////////////////////
//// input file: /home/sammael/LiteRTOpen/LiteRT/dependencies/HydraCore3/integrator_pt_lgt.cpp
////////////////////////////////////////////////////
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
////////////////////////////////////////////////////
//// input file: /home/sammael/LiteRTOpen/LiteRT/dependencies/HydraCore3/integrator_pt_mat.cpp
////////////////////////////////////////////////////
#include "integrator_pt.h"
#include "include/crandom.h"

#include "include/cmaterial.h"
#include "include/cmat_gltf.h"
#include "include/cmat_conductor.h"
#include "include/cmat_glass.h"
#include "include/cmat_film.h"
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


uint32_t Integrator::BlendSampleAndEval(uint a_materialId, uint tid, uint bounce, uint layer, float4 wavelengths, RandomGen* a_gen, float3 v, float3 n, float2 tc, 
                                        MisData* a_misPrev, BsdfSample* a_pRes)
{
  const float2 texCoordT = mulRows2x4(m_materials[a_materialId].row0[0], m_materials[a_materialId].row1[0], tc);
  const uint   texId     = m_materials[a_materialId].texid[0];
  const float4 weightDat = m_textures[texId]->sample(texCoordT);
  const float  weightTex = weightDat.x;
  const float  weight    = m_materials[a_materialId].data[BLEND_WEIGHT] * weightTex;

  const uint matId1 = m_materials[a_materialId].datai[0];
  const uint matId2 = m_materials[a_materialId].datai[1];

  uint32_t selectedMatId = matId1;
  const float select = GetRandomNumbersMatB(tid, a_gen, int(bounce), int(layer));
  RecordBlendRndNeeded(bounce, layer, select);

  if(select < weight)
  {
    a_pRes->pdf *= weight;
    a_pRes->val *= weight;
    selectedMatId = matId2;
  }
  else
  {
    a_pRes->pdf *= 1.0f - weight;
    a_pRes->val *= 1.0f - weight;
    selectedMatId = matId1;
  }

  return selectedMatId;
}

MatIdWeightPair Integrator::BlendEval(MatIdWeight a_mat, float4 wavelengths, float3 l, float3 v, float3 n, float2 tc)
{
  const float2 texCoordT = mulRows2x4(m_materials[a_mat.id].row0[0], m_materials[a_mat.id].row1[0], tc);
  const uint   texId     = m_materials[a_mat.id].texid[0];
  const float4 weightDat = m_textures[texId]->sample(texCoordT);
  
  const float  weightTex = weightDat.x;
  const float  weight    = m_materials[a_mat.id].data[BLEND_WEIGHT] * weightTex;

  const uint matId1      = m_materials[a_mat.id].datai[0];
  const uint matId2      = m_materials[a_mat.id].datai[1];

  MatIdWeight p1, p2;
  p1.id     = matId1;
  p1.weight = a_mat.weight * (1.0f - weight);
  p2.id     = matId2;
  p2.weight = a_mat.weight * weight;

  return make_weight_pair(p1, p2);
}

static inline float3 NormalMapTransform(const uint materialFlags, float3 normalFromTex)
{
  float3 normalTS = make_float3(2.0f * normalFromTex.x - 1.0f, 2.0f * normalFromTex.y - 1.0f, normalFromTex.z);

  if((materialFlags & FLAG_NMAP_INVERT_X) != 0)
    normalTS.x *= (-1.0f);

  if((materialFlags & FLAG_NMAP_INVERT_Y) != 0)
    normalTS.y *= (-1.0f);

  if((materialFlags & FLAG_NMAP_SWAP_XY) != 0)
  {
    float temp = normalTS.x;
    normalTS.x = normalTS.y;
    normalTS.y = temp;
  }

  return normalTS; // normalize(normalTS); // do we nedd this normalize here?
}

float3 Integrator::BumpMapping(uint normalMapId, uint currMatId, float3 n, float3 tan, float2 tc)
{
  const uint   mflags    = m_materials[currMatId].cflags;
  const float2 texCoordT = mulRows2x4(m_materials[currMatId].row0[1], m_materials[currMatId].row1[1], tc);
  const float4 normalTex = m_textures[normalMapId]->sample(texCoordT);
  const float3 normalTS  = NormalMapTransform(mflags, to_float3(normalTex));
  
  const float3   bitan = cross(n, tan);
  const float3x3 tangentTransform = make_float3x3(tan, bitan, n);

  return normalize(inverse3x3(tangentTransform)*normalTS);
}

BsdfSample Integrator::MaterialSampleAndEval(uint a_materialId, uint tid, uint bounce, float4 wavelengths, RandomGen* a_gen, float3 v, float3 n, float3 tan, float2 tc, 
                                             MisData* a_misPrev, const uint a_currRayFlags)
{
  BsdfSample res;
  {
    res.val   = float4(0, 0, 0, 0);
    res.pdf   = 1.0f;
    res.dir   = float3(0,1,0);
    res.ior   = 1.0f;
    res.flags = a_currRayFlags;
    res.ior   = 1.0f;
  }

  uint32_t currMatId = a_materialId;
  uint     mtype     = m_materials[currMatId].mtype;
  uint     layer     = 0;
  while(KSPEC_MAT_TYPE_BLEND != 0 && mtype == MAT_TYPE_BLEND)
  {
    currMatId = BlendSampleAndEval(currMatId, tid, bounce, layer, wavelengths, a_gen, v, n, tc, a_misPrev, &res);
    mtype     = m_materials[currMatId].mtype;
    layer++;
  }
  
  // BSDF is multiplied (outside) by cosThetaOut1.
  // When normal map is enables this becames wrong because normal is changed;
  // First : return cosThetaOut in sam;
  // Second: apply cos(theta2)/cos(theta1) to cos(theta1) to get cos(theta2)
  //
  const uint normalMapId   = m_materials[currMatId].texid[1];
  const float3 geomNormal  = n;
        float3 shadeNormal = n;

  if(KSPEC_BUMP_MAPPING != 0 && normalMapId != 0xFFFFFFFF)
    shadeNormal = BumpMapping(normalMapId, currMatId, geomNormal, tan, tc);

  const float2 texCoordT = mulRows2x4(m_materials[currMatId].row0[0], m_materials[currMatId].row1[0], tc);
  const uint   texId     = m_materials[currMatId].texid[0];
  const float4 texColor  = m_textures[texId]->sample(texCoordT);
  const float4 rands     = GetRandomNumbersMats(tid, a_gen, int(bounce));
  const uint cflags      = m_materials[currMatId].cflags;
  RecordMatRndNeeded(bounce, rands);

  float4 fourScalarMatParams = float4(1,1,1,1);
  if(KSPEC_MAT_FOUR_TEXTURES != 0 && (cflags & FLAG_FOUR_TEXTURES) != 0)
  {
    const uint texId2  = m_materials[currMatId].texid[2];
    const uint texId3  = m_materials[currMatId].texid[3];
    
    const float2 texCoord2T = mulRows2x4(m_materials[currMatId].row0[2], m_materials[currMatId].row1[2], tc);
    const float2 texCoord3T = mulRows2x4(m_materials[currMatId].row0[3], m_materials[currMatId].row1[3], tc);

    const float4 color2 = m_textures[texId2]->sample(texCoord2T);
    const float4 color3 = m_textures[texId3]->sample(texCoord3T);
    
    if((cflags & FLAG_PACK_FOUR_PARAMS_IN_TEXTURE) != 0)
      fourScalarMatParams = color2;
    else
      fourScalarMatParams = float4(color2.x, color3.x, 1, 1);
  }

  switch(mtype)
  {
    case MAT_TYPE_GLTF:
    if(KSPEC_MAT_TYPE_GLTF != 0)
    {
      const float4 color = m_materials[currMatId].colors[GLTF_COLOR_BASE]*texColor;
      gltfSampleAndEval(m_materials.data() + currMatId, rands, v, shadeNormal, tc, color, fourScalarMatParams, &res);
    }
    break;
    case MAT_TYPE_GLASS: 
    if(KSPEC_MAT_TYPE_GLASS != 0)
    {
      glassSampleAndEval(m_materials.data() + currMatId, rands, v, geomNormal, tc, &res, a_misPrev);
    }
    break;
    case MAT_TYPE_CONDUCTOR:
    if(KSPEC_MAT_TYPE_CONDUCTOR != 0)
    {
      const float3 alphaTex = to_float3(texColor);    
      const float2 alpha    = float2(m_materials[currMatId].data[CONDUCTOR_ROUGH_V], m_materials[currMatId].data[CONDUCTOR_ROUGH_U]);
      const float4 etaSpec  = SampleMatParamSpectrum(currMatId, wavelengths, CONDUCTOR_ETA, 0);
      const float4 kSpec    = SampleMatParamSpectrum(currMatId, wavelengths, CONDUCTOR_K,   1);
      if(trEffectivelySmooth(alpha))
        conductorSmoothSampleAndEval(m_materials.data() + currMatId, etaSpec, kSpec, rands, v, shadeNormal, tc, &res);
      else
        conductorRoughSampleAndEval(m_materials.data() + currMatId, etaSpec, kSpec, rands, v, shadeNormal, tc, alphaTex, &res);
    }
    break;
    case MAT_TYPE_THIN_FILM:
    if(KSPEC_MAT_TYPE_THIN_FILM != 0)
    {
      const float3 alphaTex = to_float3(texColor);  
      const float2 alpha = float2(m_materials[currMatId].data[FILM_ROUGH_V], m_materials[currMatId].data[FILM_ROUGH_U]);

      uint t_offset = as_uint(m_materials[currMatId].data[FILM_THICKNESS_OFFSET]);
      uint layers = as_uint(m_materials[currMatId].data[FILM_LAYERS_COUNT]);
      const bool spectral_mode = wavelengths[0] > 0.0f;
      // sampling 3 wavelengths for naive RGB method
      float4 wavelengths_spec = spectral_mode? float4(wavelengths[0], 0.0f, 0.0f, 0.0f) : float4(645.f, 525.f, 445.f, 0.0f);
      float4 wavelengths_sample = spectral_mode? float4(wavelengths[0], 0.0f, 0.0f, 0.0f) : float4(525.f, 0.f, 0.f, 0.0f);
      float extIOR = m_materials[currMatId].data[FILM_ETA_EXT];
      complex intIOR = complex(
        SampleFilmsSpectrum(currMatId, wavelengths_sample, FILM_ETA_OFFSET, FILM_ETA_SPECID_OFFSET, layers - 1)[0],
        SampleFilmsSpectrum(currMatId, wavelengths_sample, FILM_K_OFFSET, FILM_K_SPECID_OFFSET, layers - 1)[0]
      );
      complex filmIOR = complex(
        SampleFilmsSpectrum(currMatId, wavelengths, FILM_ETA_OFFSET, FILM_ETA_SPECID_OFFSET, 0)[0],
        SampleFilmsSpectrum(currMatId, wavelengths, FILM_K_OFFSET, FILM_K_SPECID_OFFSET, 0)[0]
      );

      float thickness;
      if (as_uint(m_materials[currMatId].data[FILM_THICKNESS_MAP]) > 0u)
      {
        const uint texId  = m_materials[currMatId].texid[2];
        const float2 texCoord = mulRows2x4(m_materials[currMatId].row0[2], m_materials[currMatId].row1[2], tc);
        const float4 thickness_val = m_textures[texId]->sample(texCoord);
        float thickness_max = m_materials[currMatId].data[FILM_THICKNESS_MAX];
        float thickness_min = m_materials[currMatId].data[FILM_THICKNESS_MIN];
        thickness = (thickness_max - thickness_min) * thickness_val.x + thickness_min;
        //std::cout << fourScalarMatParams.x << " " << fourScalarMatParams.y << " " << fourScalarMatParams.z << " " << fourScalarMatParams.w << std::endl;
      }
      else
      {
        thickness = m_materials[currMatId].data[FILM_THICKNESS];
      }

      bool precomp_flag = as_uint(m_materials[currMatId].data[FILM_PRECOMP_FLAG]) > 0u;

      uint precomp_offset = precomp_flag ? as_uint(m_materials[currMatId].data[FILM_PRECOMP_OFFSET]) : 0;
      if(trEffectivelySmooth(alpha))
        filmSmoothSampleAndEval(m_materials.data() + currMatId, extIOR, filmIOR, intIOR, thickness, wavelengths_spec, a_misPrev->ior, rands, v, n, tc, &res,
                          m_precomp_thin_films.data() + precomp_offset, spectral_mode, precomp_flag);
      else
        filmRoughSampleAndEval(m_materials.data() + currMatId, extIOR, filmIOR, intIOR, thickness, wavelengths_spec, a_misPrev->ior, rands, v, n, tc, alphaTex, &res,
                          m_precomp_thin_films.data() + precomp_offset, spectral_mode, precomp_flag);

      //res.flags |= (specId < 0xFFFFFFFF) ? RAY_FLAG_WAVES_DIVERGED : 0;
      res.flags |= RAY_FLAG_WAVES_DIVERGED;

      a_misPrev->ior = res.ior;
    }
    break;
    case MAT_TYPE_DIFFUSE:
    if(KSPEC_MAT_TYPE_DIFFUSE != 0)
    {
      const float4 color = texColor;
      float4 reflSpec    = SampleMatColorSpectrumTexture(currMatId, wavelengths, DIFFUSE_COLOR, 0, tc);
      if(m_spectral_mode == 0)
        reflSpec *= color;
      
      diffuseSampleAndEval(m_materials.data() + currMatId, reflSpec, rands, v, shadeNormal, tc, &res);
    }
    break;
    case MAT_TYPE_PLASTIC:
    if(KSPEC_MAT_TYPE_PLASTIC != 0)
    {
      const float4 color = texColor;
      float4 reflSpec    = SampleMatColorSpectrumTexture(currMatId, wavelengths, PLASTIC_COLOR, 0, tc);
      // float4 reflSpec    = SampleMatColorParamSpectrum(currMatId, wavelengths, PLASTIC_COLOR, 0);
      if(m_spectral_mode == 0)
        reflSpec *= color;

      const uint precomp_id = m_materials[currMatId].datai[0];

      plasticSampleAndEval(m_materials.data() + currMatId, reflSpec, rands, v, shadeNormal, tc, &res,
                           m_precomp_coat_transmittance.data() + precomp_id * MI_ROUGH_TRANSMITTANCE_RES);
    }
    break;
    case MAT_TYPE_DIELECTRIC:
    if(KSPEC_MAT_TYPE_DIELECTRIC != 0)
    {
      const float4 intIORSpec = SampleMatParamSpectrum(currMatId, wavelengths, DIELECTRIC_ETA_INT, 0);
      const uint specId = m_materials[currMatId].spdid[0];
      dielectricSmoothSampleAndEval(m_materials.data() + currMatId, intIORSpec, a_misPrev->ior, rands, v, shadeNormal, tc, &res);

      res.flags |= (specId < 0xFFFFFFFF) ? RAY_FLAG_WAVES_DIVERGED : 0;

      a_misPrev->ior = res.ior;
    }
    break;
    default:
    break;
  }
  
  // BSDF is multiplied (outside) by cosThetaOut1.
  // When normal map is enables this becames wrong because normal is changed;
  // First : return cosThetaOut in sam;
  // Second: apply cos(theta2)/cos(theta1) to cos(theta1) to get cos(theta2)
  //
  if(KSPEC_BUMP_MAPPING != 0 && normalMapId != 0xFFFFFFFF)
  {
    const float cosThetaOut1 = std::abs(dot(res.dir, geomNormal));
    const float cosThetaOut2 = std::abs(dot(res.dir, shadeNormal));
    res.val *= cosThetaOut2 / std::max(cosThetaOut1, 1e-10f);
  }

  return res;
}

BsdfEval Integrator::MaterialEval(uint a_materialId, float4 wavelengths, float3 l, float3 v, float3 n, float3 tan, float2 tc)
{
  BsdfEval res;
  {
    res.val = float4(0,0,0,0);
    res.pdf   = 0.0f;
  }

  MatIdWeight currMat = make_id_weight(a_materialId, 1.0f);
  MatIdWeight material_stack[KSPEC_BLEND_STACK_SIZE];
  if(KSPEC_MAT_TYPE_BLEND != 0)
    material_stack[0] = currMat;
  int top = 0;
  bool needPop = false;

  do
  {
    if(KSPEC_MAT_TYPE_BLEND != 0)
    {
      if(needPop)
      {
        top--;
        currMat = material_stack[std::max(top, 0)];
      }
      else
        needPop = true; // if not blend, pop on next iter
    } 
    
    // BSDF is multiplied (outside) by old cosThetaOut.
    // When normal map is enables this becames wrong because normal is changed;
    // First : return cosThetaOut in sam;
    // Second: apply cos(theta2)/cos(theta1) to cos(theta1) to get cos(theta2)
    //
    const float3 geomNormal = n;
          float3 shadeNormal = n;
    float bumpCosMult = 1.0f; 
    const uint normalMapId = m_materials[currMat.id].texid[1];
    if(KSPEC_BUMP_MAPPING != 0 && normalMapId != 0xFFFFFFFF) 
    {
      shadeNormal = BumpMapping(normalMapId, currMat.id, geomNormal, tan, tc);
      const float3 lDir     = l;     
      const float  clampVal = 1e-6f;  
      const float cosThetaOut1 = std::max(dot(lDir, geomNormal),  0.0f);
      const float cosThetaOut2 = std::max(dot(lDir, shadeNormal), 0.0f);
      bumpCosMult              = cosThetaOut2 / std::max(cosThetaOut1, clampVal);
      if (cosThetaOut1 <= 0.0f)
        bumpCosMult = 0.0f;
    }

    const float2 texCoordT = mulRows2x4(m_materials[currMat.id].row0[0], m_materials[currMat.id].row1[0], tc);
    const uint   texId     = m_materials[currMat.id].texid[0];
    const float4 texColor  = m_textures[texId]->sample(texCoordT);
    const uint   mtype     = m_materials[currMat.id].mtype;
    const uint   cflags    = m_materials[currMat.id].cflags;

    float4 fourScalarMatParams = float4(1,1,1,1);
    if(KSPEC_MAT_FOUR_TEXTURES != 0 && (cflags & FLAG_FOUR_TEXTURES) != 0)
    {
      const uint texId2  = m_materials[currMat.id].texid[2];
      const uint texId3  = m_materials[currMat.id].texid[3];

      const float2 texCoord2T = mulRows2x4(m_materials[currMat.id].row0[2], m_materials[currMat.id].row1[2], tc);
      const float2 texCoord3T = mulRows2x4(m_materials[currMat.id].row0[3], m_materials[currMat.id].row1[3], tc);

      const float4 color2 = m_textures[texId2]->sample(texCoord2T);
      const float4 color3 = m_textures[texId3]->sample(texCoord3T);
    
      if((cflags & FLAG_PACK_FOUR_PARAMS_IN_TEXTURE) != 0)
        fourScalarMatParams = color2;
      else
        fourScalarMatParams = float4(color2.x, color3.x, 1, 1);
    }

    BsdfEval currVal;
    {
      currVal.val = float4(0,0,0,0);
      currVal.pdf   = 0.0f;
    }
    switch(mtype)
    {
      case MAT_TYPE_GLTF:
      if(KSPEC_MAT_TYPE_GLTF != 0)
      {
        const float4 color     = (m_materials[currMat.id].colors[GLTF_COLOR_BASE]) * texColor;
        gltfEval(m_materials.data() + currMat.id, l, v, shadeNormal, tc, color, fourScalarMatParams, &currVal);
        res.val += currVal.val * currMat.weight * bumpCosMult;
        res.pdf += currVal.pdf * currMat.weight;
      }
      break;
      case MAT_TYPE_GLASS:
      if(KSPEC_MAT_TYPE_GLASS != 0)
      {
        glassEval(m_materials.data() + currMat.id, l, v, geomNormal, tc, float3(0,0,0), &currVal);
        res.val += currVal.val * currMat.weight * bumpCosMult;
        res.pdf += currVal.pdf * currMat.weight;
      }
      break;
      case MAT_TYPE_CONDUCTOR: 
      if(KSPEC_MAT_TYPE_CONDUCTOR != 0)
      {
        const float3 alphaTex  = to_float3(texColor);
        const float2 alpha     = float2(m_materials[currMat.id].data[CONDUCTOR_ROUGH_V], m_materials[currMat.id].data[CONDUCTOR_ROUGH_U]);

        if(!trEffectivelySmooth(alpha))
        {
          const float4 etaSpec = SampleMatParamSpectrum(currMat.id, wavelengths, CONDUCTOR_ETA, 0);
          const float4 kSpec   = SampleMatParamSpectrum(currMat.id, wavelengths, CONDUCTOR_K,   1);
          conductorRoughEval(m_materials.data() + currMat.id, etaSpec, kSpec, l, v, shadeNormal, tc, alphaTex, &currVal);
        }

        res.val += currVal.val * currMat.weight * bumpCosMult;
        res.pdf += currVal.pdf * currMat.weight;
      }
      break;
      case MAT_TYPE_THIN_FILM: 
      if(KSPEC_MAT_TYPE_THIN_FILM != 0)
      {
        const float3 alphaTex = to_float3(texColor);  
        const float2 alpha = float2(m_materials[currMat.id].data[FILM_ROUGH_V], m_materials[currMat.id].data[FILM_ROUGH_U]);

        if(!trEffectivelySmooth(alpha))
        {
          uint t_offset = as_uint(m_materials[currMat.id].data[FILM_THICKNESS_OFFSET]);
          uint layers = as_uint(m_materials[currMat.id].data[FILM_LAYERS_COUNT]);
          const bool spectral_mode = wavelengths[0] > 0.0f;
          // sampling 3 wavelengths for naive RGB method
          float4 wavelengths_spec = spectral_mode? float4(wavelengths[0], 0.0f, 0.0f, 0.0f) : float4(700.f, 525.f, 450.f, 0.0f);
          float4 wavelengths_sample = spectral_mode? float4(wavelengths[0], 0.0f, 0.0f, 0.0f) : float4(525.f, 0.f, 0.f, 0.0f);
          float extIOR = m_materials[currMat.id].data[FILM_ETA_EXT];
          complex intIOR = complex(
            SampleFilmsSpectrum(currMat.id, wavelengths_sample, FILM_ETA_OFFSET, FILM_ETA_SPECID_OFFSET, layers - 1)[0],
            SampleFilmsSpectrum(currMat.id, wavelengths_sample, FILM_K_OFFSET, FILM_K_SPECID_OFFSET, layers - 1)[0]
          );      
          complex filmIOR = complex(
            SampleFilmsSpectrum(currMat.id, wavelengths, FILM_ETA_OFFSET, FILM_ETA_SPECID_OFFSET, 0)[0],
            SampleFilmsSpectrum(currMat.id, wavelengths, FILM_K_OFFSET, FILM_K_SPECID_OFFSET, 0)[0]
          );

          float thickness;
          if (as_uint(m_materials[currMat.id].data[FILM_THICKNESS_MAP]) > 0u)
          {
            const uint texId  = m_materials[currMat.id].texid[2];
            const float2 texCoord = mulRows2x4(m_materials[currMat.id].row0[2], m_materials[currMat.id].row1[2], tc);
            const float4 thickness_val = m_textures[texId]->sample(texCoord);
            float thickness_max = m_materials[currMat.id].data[FILM_THICKNESS_MAX];
            float thickness_min = m_materials[currMat.id].data[FILM_THICKNESS_MIN];
            thickness = (thickness_max - thickness_min) * thickness_val.x + thickness_min;
          }
          else
          {
            thickness = m_materials[currMat.id].data[FILM_THICKNESS];
          }

          bool precomp_flag = as_uint(m_materials[currMat.id].data[FILM_PRECOMP_FLAG]) > 0u;
          uint precomp_offset = precomp_flag ? as_uint(m_materials[currMat.id].data[FILM_PRECOMP_OFFSET]) : 0;
          filmRoughEval(m_materials.data() + currMat.id, extIOR, filmIOR, intIOR, thickness, wavelengths_spec, l, v, n, tc, alphaTex, &currVal,
                          m_precomp_thin_films.data() + precomp_offset, spectral_mode, precomp_flag);
        }

        res.val += currVal.val * currMat.weight * bumpCosMult;
        res.pdf += currVal.pdf * currMat.weight;
      }
      break;
      case MAT_TYPE_DIFFUSE:
      if(KSPEC_MAT_TYPE_DIFFUSE != 0)
      {
        const float4 color = texColor;
        float4 reflSpec    = SampleMatColorSpectrumTexture(currMat.id, wavelengths, DIFFUSE_COLOR, 0, tc);
        if(m_spectral_mode == 0)
          reflSpec *= color;        
        diffuseEval(m_materials.data() + currMat.id, reflSpec, l, v, shadeNormal, tc,  &currVal);

        res.val += currVal.val * currMat.weight * bumpCosMult;
        res.pdf += currVal.pdf * currMat.weight;
      }
      break;
      case MAT_TYPE_PLASTIC:
      if(KSPEC_MAT_TYPE_PLASTIC != 0)
      {
        const float4 color = texColor;
        float4 reflSpec    = SampleMatColorSpectrumTexture(currMat.id, wavelengths, PLASTIC_COLOR, 0, tc);
        // float4 reflSpec    = SampleMatColorParamSpectrum(currMat.id, wavelengths, PLASTIC_COLOR, 0);
        if(m_spectral_mode == 0)
          reflSpec *= color;
        const uint precomp_id = m_materials[currMat.id].datai[0];
        plasticEval(m_materials.data() + currMat.id, reflSpec, l, v, shadeNormal, tc, &currVal, 
                    m_precomp_coat_transmittance.data() + precomp_id * MI_ROUGH_TRANSMITTANCE_RES);

        res.val += currVal.val * currMat.weight * bumpCosMult;
        res.pdf += currVal.pdf * currMat.weight;
      }
      break;
      case MAT_TYPE_DIELECTRIC:
      if(KSPEC_MAT_TYPE_DIELECTRIC != 0)
      {
        dielectricSmoothEval(&res); // val and pdf are always zero
      }
      break;
      case MAT_TYPE_BLEND:
      if(KSPEC_MAT_TYPE_BLEND != 0)
      {
        auto childMats = BlendEval(currMat, wavelengths, l, v, geomNormal, tc);
        currMat = childMats.first;
        needPop = false;                        // we already put 'childMats.first' in 'currMat'
        if(top + 1 <= KSPEC_BLEND_STACK_SIZE)
        {
          material_stack[top] = childMats.second; // remember second mat in stack
          top++;
        }
      }
      break;
      default:
        break;
    }

  } while(KSPEC_MAT_TYPE_BLEND != 0 && top > 0);

  return res;
}

uint Integrator::RemapMaterialId(uint a_mId, int a_instId)
{
  const int remapListId  = m_remapInst[a_instId];
  if(remapListId == -1)
    return a_mId;

  const int r_offset     = m_allRemapListsOffsets[remapListId];
  const int r_size       = m_allRemapListsOffsets[remapListId+1] - r_offset;
  const int2 offsAndSize = int2(r_offset, r_size);
  
  uint res = a_mId;
  
  // for (int i = 0; i < offsAndSize.y; i++) // linear search version
  // {
  //   int idRemapFrom = m_allRemapLists[offsAndSize.x + i * 2 + 0];
  //   int idRemapTo   = m_allRemapLists[offsAndSize.x + i * 2 + 1];
  //   if (idRemapFrom == a_mId) {
  //     res = idRemapTo;
  //     break;
  //   }
  // }

  int low  = 0;
  int high = offsAndSize.y - 1;              // binary search version
  
  while (low <= high)
  {
    const int mid         = low + ((high - low) / 2);
    const int idRemapFrom = m_allRemapLists[offsAndSize.x + mid * 2 + 0];
    if (uint(idRemapFrom) >= a_mId)
      high = mid - 1;
    else //if(a[mid]<i)
      low = mid + 1;
  }

  if (high+1 < offsAndSize.y)
  {
    const int idRemapFrom = m_allRemapLists[offsAndSize.x + (high + 1) * 2 + 0];
    const int idRemapTo   = m_allRemapLists[offsAndSize.x + (high + 1) * 2 + 1];
    res                   = (uint(idRemapFrom) == a_mId) ? uint(idRemapTo) : a_mId;
  }

  return res;
} 
////////////////////////////////////////////////////
//// input file: /home/sammael/LiteRTOpen/LiteRT/dependencies/HydraCore3/integrator_rt.cpp
////////////////////////////////////////////////////
#include "integrator_pt.h"
#include "include/crandom.h"

#include <chrono>
#include <string>

#include "Image2d.h"
using LiteImage::Image2D;
using LiteImage::Sampler;
using LiteImage::ICombinedImageSampler;
using namespace LiteMath;

void Integrator::kernel_PackXY(uint tidX, uint tidY, uint* out_pakedXY)
{
  if(int(tidX) >= m_winWidth || int(tidY) >= m_winHeight)
    return;
  uint offset = tidY*m_winWidth + tidX;
  if(m_tileSize != 1)
  {
    const uint inBlockIdX = tidX % m_tileSize; // 8x8 blocks
    const uint inBlockIdY = tidY % m_tileSize; // 8x8 blocks
   
    const uint localIndex = inBlockIdY*m_tileSize + inBlockIdX;
    const uint wBlocks    = m_winWidth/m_tileSize;
  
    const uint blockX     = tidX/m_tileSize;
    const uint blockY     = tidY/m_tileSize;
    offset                = (blockX + blockY*wBlocks)*m_tileSize*m_tileSize + localIndex;
  }
  out_pakedXY[offset] = ((tidY << 16) & 0xFFFF0000) | (tidX & 0x0000FFFF);
}

void Integrator::kernel_InitEyeRay(uint tid, const uint* packedXY, float4* rayPosAndNear, float4* rayDirAndFar) // (tid,tidX,tidY,tidZ) are SPECIAL PREDEFINED NAMES!!!
{
  if(tid >= m_maxThreadId)
    return;
  const uint XY = packedXY[tid];

  const uint x = (XY & 0x0000FFFF);
  const uint y = (XY & 0xFFFF0000) >> 16;
  
  const float xCoordNormalized = (float(x + m_winStartX) + 0.5f)/float(m_fbWidth);
  const float yCoordNormalized = (float(y + m_winStartY) + 0.5f)/float(m_fbHeight);

  float3 rayDir = EyeRayDirNormalized(xCoordNormalized, yCoordNormalized, m_projInv);
  float3 rayPos = float3(0,0,0);

  transform_ray3f(m_worldViewInv, 
                  &rayPos, &rayDir);
  
  *rayPosAndNear = to_float4(rayPos, 0.0f);
  *rayDirAndFar  = to_float4(rayDir, FLT_MAX);
}

void Integrator::kernel_InitEyeRay3(uint tid, const uint* packedXY, 
                                   float4* rayPosAndNear, float4* rayDirAndFar,
                                   float4* accumColor,    float4* accumuThoroughput,
                                   uint* rayFlags) // 
{
  if(tid >= m_maxThreadId)
    return;
  *accumColor        = make_float4(0,0,0,1);
  *accumuThoroughput = make_float4(1,1,1,1);
  //RandomGen genLocal = m_randomGens[tid];
  *rayFlags          = 0;

  const uint XY = packedXY[tid];

  const uint x = (XY & 0x0000FFFF);
  const uint y = (XY & 0xFFFF0000) >> 16;

  const float xCoordNormalized = (float(x + m_winStartX) + 0.5f)/float(m_fbWidth);
  const float yCoordNormalized = (float(y + m_winStartY) + 0.5f)/float(m_fbHeight);

  float3 rayDir = EyeRayDirNormalized(xCoordNormalized, yCoordNormalized, m_projInv);
  float3 rayPos = float3(0,0,0);

  transform_ray3f(m_worldViewInv, &rayPos, &rayDir);
  
  *rayPosAndNear = to_float4(rayPos, 0.0f);
  *rayDirAndFar  = to_float4(rayDir, FLT_MAX);
}


bool Integrator::kernel_RayTrace(uint tid, const float4* rayPosAndNear, float4* rayDirAndFar,
                                 Lite_Hit* out_hit, float2* out_bars)
{
  if(tid >= m_maxThreadId)
    return false;
  const float4 rayPos = *rayPosAndNear;
  const float4 rayDir = *rayDirAndFar ;

  CRT_Hit hit = m_pAccelStruct->RayQuery_NearestHit(rayPos, rayDir);
  
  Lite_Hit res;
  res.primId = hit.primId;
  res.instId = hit.instId;
  res.geomId = hit.geomId;
  res.t      = hit.t;

  float2 baricentrics = float2(hit.coords[0], hit.coords[1]);
 
  *out_hit  = res;
  *out_bars = baricentrics;
  return (res.primId != -1);
}


void Integrator::kernel_RealColorToUint32(uint tid, float4* a_accumColor, uint* out_color)
{
  if(tid >= m_maxThreadId)
    return;
  out_color[tid] = RealColorToUint32(*a_accumColor);
}

void Integrator::kernel_GetRayColor(uint tid, const Lite_Hit* in_hit, const float2* bars, const uint* in_pakedXY, float* out_color)
{ 
  if(tid >= m_maxThreadId)
    return;

  const Lite_Hit hit = *in_hit;
  if(hit.geomId == -1)
  {
    out_color[tid] = 0;
    return;
  }

  const uint32_t matId  = m_matIdByPrimId[m_matIdOffsets[hit.geomId] + hit.primId];
  const float4 mdata    = m_materials[matId].colors[GLTF_COLOR_BASE];
  const float2 uv       = *bars;

  const uint triOffset  = m_matIdOffsets[hit.geomId];
  const uint vertOffset = m_vertOffset  [hit.geomId];

  const uint A = m_triIndices[(triOffset + hit.primId)*3 + 0];
  const uint B = m_triIndices[(triOffset + hit.primId)*3 + 1];
  const uint C = m_triIndices[(triOffset + hit.primId)*3 + 2];
  const float4 data1 = (1.0f - uv.x - uv.y)*m_vNorm4f[A + vertOffset] + uv.y*m_vNorm4f[B + vertOffset] + uv.x*m_vNorm4f[C + vertOffset];
  const float4 data2 = (1.0f - uv.x - uv.y)*m_vTang4f[A + vertOffset] + uv.y*m_vTang4f[B + vertOffset] + uv.x*m_vTang4f[C + vertOffset];
  //float3 hitNorm     = to_float3(data1);
  //float3 hitTang     = to_float3(data2);
  float2 hitTexCoord = float2(data1.w, data2.w);

  const uint   texId     = m_materials[matId].texid[0];
  const float2 texCoordT = mulRows2x4(m_materials[matId].row0[0], m_materials[matId].row1[0], hitTexCoord);
  const float4 texColor  = m_textures[texId]->sample(texCoordT);

  const float3 color     = mdata.w > 0.0f ? clamp(float3(mdata.w,mdata.w,mdata.w), 0.0f, 1.0f) : to_float3(mdata*texColor);

  const uint XY = in_pakedXY[tid];
  const uint x  = (XY & 0x0000FFFF);
  const uint y  = (XY & 0xFFFF0000) >> 16;

  out_color[(y*m_winWidth+x)*4 + 0] = color.x;
  out_color[(y*m_winWidth+x)*4 + 1] = color.y;
  out_color[(y*m_winWidth+x)*4 + 2] = color.z;
  out_color[(y*m_winWidth+x)*4 + 3] = 0.0f;
}


float3 Integrator::MaterialEvalWhitted(uint a_materialId, float3 l, float3 v, float3 n, float2 tc)
{
  const uint   texId     = m_materials[a_materialId].texid[0];
  const float2 texCoordT = mulRows2x4(m_materials[a_materialId].row0[0], m_materials[a_materialId].row1[0], tc);
  const float3 texColor  = to_float3(m_textures[texId]->sample(texCoordT));
  const float3 color     = to_float3(m_materials[a_materialId].colors[GLTF_COLOR_BASE])*texColor;
  return lambertEvalBSDF(l, v, n)*color;
}

BsdfSample Integrator::MaterialSampleWhitted(uint a_materialId, float3 v, float3 n, float2 tc)
{ 
  const float4 specular  = m_materials[a_materialId].colors[GLTF_COLOR_METAL];
  const float4 coat      = m_materials[a_materialId].colors[GLTF_COLOR_COAT];
  float alpha            = m_materials[a_materialId].data[GLTF_FLOAT_ALPHA];
  
  const float3 pefReflDir = reflect((-1.0f)*v, n);
  const float4 reflColor  = alpha*specular + (1.0f - alpha)*coat;

  //if(a_materialId == 4)
  //{
  //  int a = 2;
  //}

  BsdfSample res;
  res.dir   = pefReflDir;
  res.val   = reflColor;
  res.pdf   = 1.0f;
  res.flags = RAY_EVENT_S;
  return res;
}


void Integrator::kernel_RayBounce(uint tid, uint bounce, const float4* in_hitPart1, const float4* in_hitPart2,
                                  float4* rayPosAndNear, float4* rayDirAndFar, float4* accumColor, float4* accumThoroughput,
                                  uint* rayFlags)
{
  if(tid >= m_maxThreadId)
    return;
  const uint currRayFlags = *rayFlags;
  if(isDeadRay(currRayFlags))
    return;

  const uint32_t matId = extractMatId(currRayFlags);

  // process surface hit case
  //
  const float3 ray_dir = to_float3(*rayDirAndFar);
  //const float3 ray_pos = to_float3(*rayPosAndNear);

  const float4 data1 = *in_hitPart1;
  const float4 data2 = *in_hitPart2;

  SurfaceHit hit;
  hit.pos  = to_float3(data1);
  hit.norm = to_float3(data2);
  hit.uv   = float2(data1.w, data2.w);

  // process light hit case
  //
  if(m_materials[matId].mtype == MAT_TYPE_LIGHT_SOURCE)
  {
    const uint   texId          = m_materials[matId].texid[0];
    const float2 texCoordT      = mulRows2x4(m_materials[matId].row0[0], m_materials[matId].row1[0], hit.uv);
    const float3 texColor       = to_float3(m_textures[texId]->sample(texCoordT));

    const float3 lightIntensity = to_float3(m_materials[matId].colors[GLTF_COLOR_BASE])*texColor;
    const uint lightId          = m_materials[matId].lightId;
    float lightDirectionAtten   = (lightId == 0xFFFFFFFF) ? 1.0f : dot(to_float3(*rayDirAndFar), float3(0,-1,0)) < 0.0f ? 1.0f : 0.0f; // TODO: read light info, gety light direction and e.t.c;

    float4 currAccumColor      = *accumColor;
    float4 currAccumThroughput = *accumThoroughput;

    currAccumColor.x += currAccumThroughput.x * lightIntensity.x * lightDirectionAtten;
    currAccumColor.y += currAccumThroughput.y * lightIntensity.y * lightDirectionAtten;
    currAccumColor.z += currAccumThroughput.z * lightIntensity.z * lightDirectionAtten;

    *accumColor = currAccumColor;
    *rayFlags   = currRayFlags | (RAY_FLAG_IS_DEAD | RAY_FLAG_HIT_LIGHT);
    return;
  }

  float4 shadeColor = float4(0.0f, 0.0f, 0.0f, 1.0f);
  for(uint lightId = 0; lightId < m_lights.size(); ++lightId)
  {
    const float3 lightPos = to_float3(m_lights[lightId].pos);
    const float hitDist   = sqrt(dot(hit.pos - lightPos, hit.pos - lightPos));

    const float3 shadowRayDir = normalize(lightPos - hit.pos);
    const float3 shadowRayPos = hit.pos + hit.norm * std::max(maxcomp(hit.pos), 1.0f) * 5e-6f; // TODO: see Ray Tracing Gems, also use flatNormal for offset

    const bool inShadow = m_pAccelStruct->RayQuery_AnyHit(to_float4(shadowRayPos, 0.0f), to_float4(shadowRayDir, hitDist * 0.9995f));

    if(!inShadow && dot(shadowRayDir, to_float3(m_lights[lightId].norm)) < 0.0f)
    {
      const float3 matSamColor = MaterialEvalWhitted(matId, shadowRayDir, (-1.0f)*ray_dir, hit.norm, hit.uv);
      const float cosThetaOut  = std::max(dot(shadowRayDir, hit.norm), 0.0f);
      shadeColor += to_float4(to_float3(m_lights[lightId].intensity) * matSamColor*cosThetaOut / (hitDist * hitDist), 0.0f);
    }
  }

  const BsdfSample matSam = MaterialSampleWhitted(matId, (-1.0f)*ray_dir, hit.norm, hit.uv);
  const float4 bxdfVal    = matSam.val;
  const float  cosTheta   = dot(matSam.dir, hit.norm);

  const float4 currThoroughput = *accumThoroughput;
  float4 currAccumColor        = *accumColor;

  currAccumColor.x += currThoroughput.x * shadeColor.x;
  currAccumColor.y += currThoroughput.y * shadeColor.y;
  currAccumColor.z += currThoroughput.z * shadeColor.z;

  *accumColor       = currAccumColor;
  *accumThoroughput = currThoroughput * cosTheta * bxdfVal;

  *rayPosAndNear = to_float4(OffsRayPos(hit.pos, hit.norm, matSam.dir), 0.0f);
  *rayDirAndFar  = to_float4(matSam.dir, FLT_MAX);
  *rayFlags      = currRayFlags | matSam.flags;
}

void Integrator::kernel_ContributeToImage3(uint tid, uint channels, const float4* a_accumColor, const uint* in_pakedXY, float* out_color)
{
  if(tid >= m_maxThreadId)
    return;
  const uint XY = in_pakedXY[tid];
  const uint x  = (XY & 0x0000FFFF);
  const uint y  = (XY & 0xFFFF0000) >> 16;

  float4 color = *a_accumColor;
  //out_color[y*m_winWidth+x] += color;
  if(channels <= 4)
  {
    out_color[(y*m_winWidth+x)*channels + 0] += color.x;
    out_color[(y*m_winWidth+x)*channels + 1] += color.y;
    out_color[(y*m_winWidth+x)*channels + 2] += color.z;
  }
}

static inline float2 clipSpaceToScreenSpace(float4 a_pos, const float fw, const float fh)
{
  const float x = a_pos.x * 0.5f + 0.5f;
  const float y = a_pos.y * 0.5f + 0.5f;
  return make_float2(x * fw, y * fh);
}

//static inline float4x4 make_float4x4(const float* a_data)
//{
//  float4x4 matrix;
//  matrix.m_col[0] = make_float4(a_data[0], a_data[1], a_data[2], a_data[3]);
//  matrix.m_col[1] = make_float4(a_data[4], a_data[5], a_data[6], a_data[7]);
//  matrix.m_col[2] = make_float4(a_data[8], a_data[9], a_data[10], a_data[11]);
//  matrix.m_col[3] = make_float4(a_data[12], a_data[13], a_data[14], a_data[15]);
//  return matrix;
//}

static inline float2 worldPosToScreenSpace(float3 a_wpos, const int width, const int height, 
                                           float4x4 worldView, float4x4 proj)
{
  const float4 posWorldSpace  = to_float4(a_wpos, 1.0f);
  const float4 posCamSpace    = mul4x4x4(worldView, posWorldSpace);
  const float4 posNDC         = mul4x4x4(proj, posCamSpace);
  const float4 posClipSpace   = posNDC * (1.0f / std::max(posNDC.w, DEPSILON));
  const float2 posScreenSpace = clipSpaceToScreenSpace(posClipSpace, float(width), float(height));
  return posScreenSpace;
}

void drawLine(const float3 a_pos1, const float3 a_pos2, float4 * a_outColor, const int a_winWidth,
              const int a_winHeight, const float4 a_rayColor1, const float4 a_rayColor2, const bool a_blendColor,
              const bool a_multDepth, const int a_spp)
{
  const int dx   = int(std::abs(a_pos2.x - a_pos1.x));
  const int dy   = int(std::abs(a_pos2.y - a_pos1.y));

  const int step = dx > dy ? dx : dy;

  float x_inc    = float(dx) / (float)step;
  float y_inc    = float(dy) / (float)step;

  if (a_pos1.x > a_pos2.x) x_inc *= -1.0f;
  if (a_pos1.y > a_pos2.y) y_inc *= -1.0f;

  float x = a_pos1.x;
  float y = a_pos1.y;

  const float depthMult1 = std::tanh(a_pos1.z * 0.25f) * 0.5f + 0.5f; // rescale for 0 - 1
  const float depthMult2 = std::tanh(a_pos2.z * 0.25f) * 0.5f + 0.5f; // rescale for 0 - 1

  for (int i = 0; i <= step; ++i) 
  {
    if (x >= float(0) && x <= float(a_winWidth - 1) && y >= 0 && y <= float(a_winHeight - 1))
    {
      float4 color;
      float weight    = (float)(i) / (float)(step);
      
      float depthMult = 1.0f; 
      
      if (a_multDepth) 
        depthMult = lerp(depthMult1, depthMult2, weight);
      
      if (!a_blendColor)
        weight = 0.0f;

      color = lerp(a_rayColor1, a_rayColor2, weight) * depthMult;
         
      a_outColor[(int)(y)*a_winWidth + (int)(x)] += color * float(a_spp);
    }
 
    x += x_inc;
    y += y_inc;
  }
  if (a_pos1.x >= 0 && a_pos1.x <= float(a_winWidth - 1) && a_pos1.y >= 0 && a_pos1.y <= float(a_winHeight - 1))
    a_outColor[(int)(a_pos1.y)*a_winWidth + (int)(a_pos1.x)] = float4(0, float(a_spp), 0, 0);
}

void Integrator::kernel_ContributePathRayToImage3(float4* out_color, 
  const std::vector<float4>& a_rayColor, std::vector<float3>& a_rayPos)
{  
  for (uint32_t i = 1; i < a_rayPos.size(); ++i)
  {
    const float2 posScreen1 = worldPosToScreenSpace(a_rayPos[i - 1], m_winWidth, m_winHeight, m_worldView, m_proj);
    const float2 posScreen2 = worldPosToScreenSpace(a_rayPos[i - 0], m_winWidth, m_winHeight, m_worldView, m_proj);
    
    const float3 pos1 = float3(posScreen1.x, posScreen1.y, a_rayPos[i - 1].z);
    const float3 pos2 = float3(posScreen2.x, posScreen2.y, a_rayPos[i    ].z);

    // fix color
    //const float4 rayColor = float4(1, 1, 1, 1); 

    // shade color
    //const float4 rayColor1 = a_rayColor[i - 1]; 
    //const float4 rayColor2 = a_rayColor[i - 0];

    // direction color
    //const float4 rayColor1 = (a_rayColor[i - 1]) * 0.5f + 0.5f;
    //const float4 rayColor2 = (a_rayColor[i - 0]) * 0.5f + 0.5f;

    // position color with rescale to 0-1
    const float scaleSize  = 0.5f;
    const float4 rayColor1 = float4(std::tanh(a_rayPos[i - 1].x * scaleSize), std::tanh(a_rayPos[i - 1].y * scaleSize),
                                    std::tanh(a_rayPos[i - 1].z * scaleSize), 0) * 0.5f + 0.5f;
    const float4 rayColor2 = float4(std::tanh(a_rayPos[i - 0].x * scaleSize), std::tanh(a_rayPos[i - 0].y * scaleSize),
                                    std::tanh(a_rayPos[i - 0].z * scaleSize), 0) * 0.5f + 0.5f;
        
    drawLine(pos1, pos2, out_color, m_winWidth, m_winHeight, rayColor1, rayColor2, true, true, m_spp);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Integrator::PackXY(uint tidX, uint tidY)
{
  kernel_PackXY(tidX, tidY, m_packedXY.data());
}

void Integrator::CastSingleRay(uint tid, float* out_color)
{
  float4 rayPosAndNear, rayDirAndFar;
  kernel_InitEyeRay(tid, m_packedXY.data(), &rayPosAndNear, &rayDirAndFar);

  Lite_Hit hit; 
  float2   baricentrics; 
  if(!kernel_RayTrace(tid, &rayPosAndNear, &rayDirAndFar, &hit, &baricentrics))
    return;
  
  kernel_GetRayColor(tid, &hit, &baricentrics, m_packedXY.data(), out_color);
}

void Integrator::RayTrace(uint tid, uint channels, float* out_color)
{
  float4 accumColor, accumThroughput;
  float4 rayPosAndNear, rayDirAndFar;
  uint   rayFlags = 0;
  kernel_InitEyeRay3(tid, m_packedXY.data(), 
                     &rayPosAndNear, &rayDirAndFar, &accumColor, &accumThroughput, &rayFlags);

  for(uint depth = 0; depth < m_traceDepth; depth++)
  {
    float4 hitPart1, hitPart2, hitPart3;
    uint instId;
    float time = 0.0f;
    kernel_RayTrace2(tid, depth, &rayPosAndNear, &rayDirAndFar, &time, 
                     &hitPart1, &hitPart2, &hitPart3, &instId, &rayFlags);
    if(isDeadRay(rayFlags))
      break;

    kernel_RayBounce(tid, depth, &hitPart1, &hitPart2,
                     &rayPosAndNear, &rayDirAndFar, &accumColor, &accumThroughput, &rayFlags);

    if(isDeadRay(rayFlags))
      break;
  }

//  kernel_HitEnvironment(tid, &rayFlags, &rayDirAndFar, &mis, &accumThroughput,
//                        &accumColor);

  kernel_ContributeToImage3(tid, channels, &accumColor, m_packedXY.data(), out_color);
}
////////////////////////////////////////////////////
//// input file: /home/sammael/LiteRTOpen/LiteRT/dependencies/HydraCore3/integrator_spectrum.cpp
////////////////////////////////////////////////////
#include "integrator_pt.h"


float4 Integrator::SampleMatColorParamSpectrum(uint32_t matId, float4 a_wavelengths, uint32_t paramId, uint32_t paramSpecId)
{  
  float4 res = m_materials[matId].colors[paramId];
  if(a_wavelengths[0] == 0.0f)
    return res;

  const uint specId = m_materials[matId].spdid[paramSpecId];
  if(specId < 0xFFFFFFFF)
  {
    const uint2 data  = m_spec_offset_sz[specId];
    const uint offset = data.x;
    const uint size   = data.y;
    //res = SampleSpectrum(m_wavelengths.data() + offset, m_spec_values.data() + offset, a_wavelengths, size);
    res = SampleUniformSpectrum(m_spec_values.data() + offset, a_wavelengths, size);
  }

  return res;
}

float4 Integrator::SampleMatParamSpectrum(uint32_t matId, float4 a_wavelengths, uint32_t paramId, uint32_t paramSpecId)
{  
  float4 res = float4(m_materials[matId].data[paramId]);
  if(a_wavelengths[0] == 0.0f)
    return res;

  const uint specId = m_materials[matId].spdid[paramSpecId];
  if(specId < 0xFFFFFFFF)
  {
    const uint2 data  = m_spec_offset_sz[specId];
    const uint offset = data.x;
    const uint size   = data.y;
    //res = SampleSpectrum(m_wavelengths.data() + offset, m_spec_values.data() + offset, a_wavelengths, size);
    res = SampleUniformSpectrum(m_spec_values.data() + offset, a_wavelengths, size);
  }

  return res;
}

float4 Integrator::SampleFilmsSpectrum(uint32_t matId, float4 a_wavelengths, uint32_t paramId, uint32_t paramSpecId, uint32_t layer)
{  
  float4 res = float4(m_films_eta_k_vec[as_uint(m_materials[matId].data[paramId]) + layer]);
  if(a_wavelengths[0] == 0.0f && false)
    return res;

  const uint specId = m_films_spec_id_vec[as_uint(m_materials[matId].data[paramSpecId]) + layer];
  if(specId < 0xFFFFFFFF)
  {
    const uint2 data  = m_spec_offset_sz[specId];
    const uint offset = data.x;
    const uint size   = data.y;
    res = SampleUniformSpectrum(m_spec_values.data() + offset, a_wavelengths, size);
  }

  return res;
}

float3 Integrator::SpectralCamRespoceToRGB(float4 specSamples, float4 waves, uint32_t rayFlags)
{
  float3 rgb = to_float3(specSamples);

  if(m_camResponseSpectrumId[0] < 0)
  {
    const float3 xyz = SpectrumToXYZ(specSamples, waves, LAMBDA_MIN, LAMBDA_MAX, m_cie_x.data(), m_cie_y.data(), m_cie_z.data(), terminateWavelngths(rayFlags));
    rgb = XYZToRGB(xyz);
  }
  else
  {
    float4 responceX, responceY, responceZ;
    {
      int specId = m_camResponseSpectrumId[0];
      if(specId >= 0)
      {
        const uint2 data  = m_spec_offset_sz[specId];
        const uint offset = data.x;
        const uint size   = data.y;
        responceX = SampleUniformSpectrum(m_spec_values.data() + offset, waves, size);
      }
      else
        responceX = float4(1,1,1,1);
      specId = m_camResponseSpectrumId[1];
      if(specId >= 0)
      {
        const uint2 data  = m_spec_offset_sz[specId];
        const uint offset = data.x;
        const uint size   = data.y;
        responceY = SampleUniformSpectrum(m_spec_values.data() + offset, waves, size);
      }
      else
        responceY = responceX;
      specId = m_camResponseSpectrumId[2];
      if(specId >= 0)
      {
        const uint2 data  = m_spec_offset_sz[specId];
        const uint offset = data.x;
        const uint size   = data.y;
        responceZ = SampleUniformSpectrum(m_spec_values.data() + offset, waves, size);
      }
      else
        responceZ = responceY;
    }
    float3 xyz = float3(0,0,0);
    for (uint32_t i = 0; i < SPECTRUM_SAMPLE_SZ; ++i) {
      xyz.x += specSamples[i]*responceX[i];
      xyz.y += specSamples[i]*responceY[i];
      xyz.z += specSamples[i]*responceZ[i]; 
    } 
    if(m_camResponseType == CAM_RESPONCE_XYZ)
      rgb = XYZToRGB(xyz);
    else
      rgb = xyz;
  }
  
  return rgb;
}

float4 Integrator::SampleMatColorSpectrumTexture(uint32_t matId, float4 a_wavelengths, uint32_t paramId, uint32_t paramSpecId, float2 texCoords)
{  
  float4 res = m_materials[matId].colors[paramId];
  if(a_wavelengths[0] == 0.0f)
    return res;

  const uint specId = m_materials[matId].spdid[paramSpecId];
  if(KSPEC_SPECTRAL_RENDERING !=0 && specId < 0xFFFFFFFF)
  {
    const uint2 data   = m_spec_offset_sz[specId];
    const uint  offset = data.x;
    const uint  size   = data.y;

    res = SampleUniformSpectrum(m_spec_values.data() + offset, a_wavelengths, size);
    
    if(KSPEC_SPD_TEX != 0) // check if spectrum is represented as textures
    {
      const uint2 tex_data  = m_spec_tex_offset_sz[specId];
      const uint tex_offset = tex_data.x;
      const uint tex_size   = tex_data.y;
      if(tex_size > 0)
      {
        for(int i = 0; i < 4; ++i)
        {
          if (a_wavelengths[i] < float(m_spec_tex_ids_wavelengths[tex_offset].y) ||
              a_wavelengths[i] > float(m_spec_tex_ids_wavelengths[tex_offset + tex_size - 1].y) )
          {
            res[i] = 0.0f;
            continue;
          }

          uint32_t o = BinarySearchU2(m_spec_tex_ids_wavelengths.data() + tex_offset, tex_size, a_wavelengths[i]);

          uint32_t texID1 = m_spec_tex_ids_wavelengths[tex_offset + o + 0].x;
          uint32_t texID2 = m_spec_tex_ids_wavelengths[tex_offset + o + 1].x;

          const float2 texCoordT = mulRows2x4(m_materials[matId].row0[0], m_materials[matId].row1[0], texCoords);
          const float4 texColor1 = m_textures[texID1]->sample(texCoordT);
          const float4 texColor2 = m_textures[texID2]->sample(texCoordT);
  
          float t = (a_wavelengths[i] - m_spec_tex_ids_wavelengths[tex_offset + o].y) / 
                    float(m_spec_tex_ids_wavelengths[tex_offset + o + 1].y - m_spec_tex_ids_wavelengths[tex_offset + o].y );
                    
          float4 outColor = lerp(texColor1, texColor2, t);

          res[i] = outColor.x;
        }
      }
    }
  }

  return res;
}
////////////////////////////////////////////////////
//// input file: /home/sammael/LiteRTOpen/LiteRT/BVH/BVH2Common.cpp
////////////////////////////////////////////////////
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <cassert>
#include <cfloat>
#include <chrono>

#include "BVH2Common.h"

using uvec3 = uint3;
using LiteMath::M_PI;
using LiteMath::clamp;

#ifndef KERNEL_SLICER
#define bitCount(x) __builtin_popcount(x)
#endif

bool BVHRT::need_normal()
{
  return m_preset.render_mode == MULTI_RENDER_MODE_LAMBERT_NO_TEX || 
         m_preset.render_mode == MULTI_RENDER_MODE_NORMAL  ||
         m_preset.render_mode == MULTI_RENDER_MODE_PHONG_NO_TEX ||
         m_preset.render_mode == MULTI_RENDER_MODE_LAMBERT ||
         m_preset.render_mode == MULTI_RENDER_MODE_PHONG;
}

//Octahedral Normal Vectors (ONV) encoding https://jcgt.org/published/0003/02/01/
float2 BVHRT::encode_normal(float3 v)
{
  float2 p = float2(v.x, v.y) * (1.0f / (std::abs(v.x) + std::abs(v.y) + std::abs(v.z)));
  float2 signNotZero = float2((p.x >= 0.0f) ? +1.0f : -1.0f, (p.y >= 0.0f) ? +1.0f : -1.0f);
  return (v.z <= 0.0f) ? ((1.0f - abs(float2(p.y, p.x))) * signNotZero) : p;
}

float2 BVHRT::box_intersects(const float3 &min_pos, const float3 &max_pos, const float3 &origin, const float3 &dir)
{
  float3 safe_dir = sign(dir) * max(float3(1e-9f), abs(dir));
  float3 tMin = (min_pos - origin) / safe_dir;
  float3 tMax = (max_pos - origin) / safe_dir;
  float3 t1 = min(tMin, tMax);
  float3 t2 = max(tMin, tMax);
  float tNear = std::max(t1.x, std::max(t1.y, t1.z));
  float tFar = std::min(t2.x, std::min(t2.y, t2.z));

  return float2(tNear, tFar);
}

void BVHRT::IntersectAllPrimitivesInLeaf(const float3 ray_pos, const float3 ray_dir,
                                             float tNear, uint32_t instId, uint32_t geomId,
                                             uint32_t a_start, uint32_t a_count,
                                             CRT_Hit *pHit)
{
  uint32_t type = m_geomData[geomId].type;
  const float SDF_BIAS = 0.1f;
  const float tNearSdf = std::max(tNear, SDF_BIAS);
  
  //m_abstractObjectPtrs[geomId]->Intersect(type, ray_pos, ray_dir, tNearSdf, instId, geomId, a_start, a_count, pHit, this);
}

float BVHRT::eval_dist_trilinear(const float values[8], float3 dp)
{
  return (1-dp.x)*(1-dp.y)*(1-dp.z)*values[0] + 
         (1-dp.x)*(1-dp.y)*(  dp.z)*values[1] + 
         (1-dp.x)*(  dp.y)*(1-dp.z)*values[2] + 
         (1-dp.x)*(  dp.y)*(  dp.z)*values[3] + 
         (  dp.x)*(1-dp.y)*(1-dp.z)*values[4] + 
         (  dp.x)*(1-dp.y)*(  dp.z)*values[5] + 
         (  dp.x)*(  dp.y)*(1-dp.z)*values[6] + 
         (  dp.x)*(  dp.y)*(  dp.z)*values[7];
}

//intersects ray with the particular leaf node of SDF strcuture
//it reqires a 1-to-1 mapping between leaf nodes of SDF and BVH
//like in frame octree or SVS
void BVHRT::RayNodeIntersection(uint32_t type, const float3 ray_pos, const float3 ray_dir, 
                                float tNear, uint32_t geomId, uint32_t bvhNodeId, 
                                float values[8], uint32_t &primId, uint32_t &nodeId, float &d, 
                                float &qNear, float &qFar, float2 &fNearFar, float3 &start_q)
{
  float3 min_pos, max_pos;

  if (type == TYPE_SDF_FRAME_OCTREE)
  {
#ifndef DISABLE_SDF_FRAME_OCTREE
    uint32_t sdfId =  m_geomData[geomId].offset.x;
    primId = m_origNodes[bvhNodeId].leftOffset;
    nodeId = primId + m_SdfFrameOctreeRoots[sdfId];
    min_pos = m_origNodes[bvhNodeId].boxMin;
    max_pos = m_origNodes[bvhNodeId].boxMax;
    float3 size = max_pos - min_pos;

    for (int i=0;i<8;i++)
      values[i] = m_SdfFrameOctreeNodes[nodeId].values[i];

    fNearFar = RayBoxIntersection2(ray_pos, SafeInverse(ray_dir), min_pos, max_pos);
    float3 start_pos = ray_pos + fNearFar.x*ray_dir;
    d = std::max(size.x, std::max(size.y, size.z));
    start_q = (start_pos - min_pos) / d;
    qFar = (fNearFar.y - fNearFar.x) / d;
    qNear = tNear > fNearFar.x ? (tNear - fNearFar.x) / d : 0.0f;
#endif
  }
  else if (type == TYPE_SDF_FRAME_OCTREE_TEX)
  {
#ifndef DISABLE_SDF_FRAME_OCTREE_TEX
    uint32_t sdfId =  m_geomData[geomId].offset.x;
    primId = m_origNodes[bvhNodeId].leftOffset;
    nodeId = primId + m_SdfFrameOctreeTexRoots[sdfId];
    min_pos = m_origNodes[bvhNodeId].boxMin;
    max_pos = m_origNodes[bvhNodeId].boxMax;
    float3 size = max_pos - min_pos;

    for (int i=0;i<8;i++)
      values[i] = m_SdfFrameOctreeTexNodes[nodeId].values[i];

    fNearFar = RayBoxIntersection2(ray_pos, SafeInverse(ray_dir), min_pos, max_pos);
    float3 start_pos = ray_pos + fNearFar.x*ray_dir;
    d = std::max(size.x, std::max(size.y, size.z));
    start_q = (start_pos - min_pos) / d;
    qFar = (fNearFar.y - fNearFar.x) / d;
    qNear = tNear > fNearFar.x ? (tNear - fNearFar.x) / d : 0.0f;
#endif
  }
  else if (type == TYPE_SDF_SVS)
  {
#ifndef DISABLE_SDF_SVS
    uint32_t sdfId =  m_geomData[geomId].offset.x;
    primId = bvhNodeId;
    nodeId = primId + m_SdfSVSRoots[sdfId];

    float px = m_SdfSVSNodes[nodeId].pos_xy >> 16;
    float py = m_SdfSVSNodes[nodeId].pos_xy & 0x0000FFFF;
    float pz = m_SdfSVSNodes[nodeId].pos_z_lod_size >> 16;
    float sz = m_SdfSVSNodes[nodeId].pos_z_lod_size & 0x0000FFFF;
    float d_max = 2*1.73205081f/sz;

    min_pos = float3(-1,-1,-1) + 2.0f*float3(px,py,pz)/sz;
    max_pos = min_pos + 2.0f*float3(1,1,1)/sz;
    float3 size = max_pos - min_pos;

    for (int i=0;i<8;i++)
      values[i] = -d_max + 2*d_max*(1.0/255.0f)*((m_SdfSVSNodes[nodeId].values[i/4] >> (8*(i%4))) & 0xFF);

    fNearFar = RayBoxIntersection2(ray_pos, SafeInverse(ray_dir), min_pos, max_pos);
    float3 start_pos = ray_pos + fNearFar.x*ray_dir;
    d = std::max(size.x, std::max(size.y, size.z));
    start_q = (start_pos - min_pos) / d;
    qFar = (fNearFar.y - fNearFar.x) / d;
    qNear = tNear > fNearFar.x ? (tNear - fNearFar.x) / d : 0.0f;
#endif
  }
}

void BVHRT::TricubicLocalSurfaceIntersection(uint32_t type, const float3 ray_dir, uint32_t instId, uint32_t geomId, 
                                float values[64], uint32_t nodeId, uint32_t primId, float d, float qNear, 
                                float qFar, float2 fNearFar, float3 start_q,
                                CRT_Hit *pHit)
{
  const float EPS = 1e-6f;
  float d_inv = 1.0f / d;
  float t = qNear;
  bool hit = false;
  unsigned iter = 0;

  float start_dist = 10000;
  float start_sign = 1;

  float point[3] = {(start_q + t * ray_dir).x, (start_q + t * ray_dir).y, (start_q + t * ray_dir).z};
  start_dist = tricubicInterpolation(values, point);

  if (m_preset.representation_mode == REPRESENTATION_MODE_SURFACE)
    start_sign = sign(start_dist);
  start_dist *= start_sign;

  if (start_dist <= EPS || m_preset.sdf_node_intersect == SDF_OCTREE_NODE_INTERSECT_BBOX)
  {
    hit = true;
  }
  else if (m_preset.sdf_node_intersect == SDF_OCTREE_NODE_INTERSECT_ST)
  {
    const unsigned ST_max_iters = 256;
    float dist = start_dist;
    float3 pp0 = start_q + t * ray_dir;

    while (t < qFar && dist > EPS && iter < ST_max_iters)
    {
      t += dist * d_inv;

      float point[3] = {(start_q + t * ray_dir).x, (start_q + t * ray_dir).y, (start_q + t * ray_dir).z};
      dist = tricubicInterpolation(values, point);
      
      float3 pp = start_q + t * ray_dir;
      iter++;
    }
    hit = (dist <= EPS);
  }

  float tReal = fNearFar.x + d * t;

  if (t <= qFar && hit && tReal < pHit->t)
  {
    float3 norm = float3(0, 0, 1);
    if (need_normal())
    {
      float3 p0 = start_q + t * ray_dir;
      const float h = 0.001;

      float p1[3] = {(p0 + float3(h, 0, 0)).x, (p0).y, (p0).z};
      float p2[3] = {(p0 + float3(-h, 0, 0)).x, p0.y, p0.z};
      
      float ddx = (tricubicInterpolation(values, p1) -
                   tricubicInterpolation(values, p2)) /
                  (2 * h);

      float p3[3] = {p0.x, (p0 + float3(0, h, 0)).y, p0.z};
      float p4[3] = {p0.x, (p0 + float3(0, -h, 0)).y, p0.z};
      float ddy = (tricubicInterpolation(values, p3) -
                   tricubicInterpolation(values, p4)) /
                  (2 * h);

      float p5[3] = {p0.x, p0.y, (p0 + float3(0, 0, h)).z};
      float p6[3] = {p0.x, p0.y, (p0 + float3(0, 0, -h)).z};

      float ddz = (tricubicInterpolation(values, p5) -
                   tricubicInterpolation(values, p6)) /
                  (2 * h);

      norm = start_sign * normalize(matmul4x3(m_instanceData[instId].transformInvTransposed, float3(ddx, ddy, ddz)));
    }
    
    float2 encoded_norm = encode_normal(norm);

    pHit->t = tReal;
    pHit->primId = primId;
    pHit->instId = instId;
    pHit->geomId = geomId | (type << SH_TYPE);
    pHit->coords[0] = 0;
    pHit->coords[1] = 0;
    pHit->coords[2] = encoded_norm.x;
    pHit->coords[3] = encoded_norm.y;

    if (m_preset.render_mode == MULTI_RENDER_MODE_ST_ITERATIONS)
      pHit->primId = iter;
    
  #ifndef DISABLE_SDF_FRAME_OCTREE_TEX
    if (type == TYPE_SDF_FRAME_OCTREE_TEX)
    {
      float3 dp = start_q + t * ray_dir;
      
      pHit->coords[0] = (1-dp.x)*(1-dp.y)*(1-dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[0] + 
                        (1-dp.x)*(1-dp.y)*(  dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[2] + 
                        (1-dp.x)*(  dp.y)*(1-dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[4] + 
                        (1-dp.x)*(  dp.y)*(  dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[6] + 
                        (  dp.x)*(1-dp.y)*(1-dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[8] + 
                        (  dp.x)*(1-dp.y)*(  dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[10] + 
                        (  dp.x)*(  dp.y)*(1-dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[12] + 
                        (  dp.x)*(  dp.y)*(  dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[14];

      pHit->coords[1] = (1-dp.x)*(1-dp.y)*(1-dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[1] + 
                        (1-dp.x)*(1-dp.y)*(  dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[3] + 
                        (1-dp.x)*(  dp.y)*(1-dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[5] + 
                        (1-dp.x)*(  dp.y)*(  dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[7] + 
                        (  dp.x)*(1-dp.y)*(1-dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[9] + 
                        (  dp.x)*(1-dp.y)*(  dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[11] + 
                        (  dp.x)*(  dp.y)*(1-dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[13] + 
                        (  dp.x)*(  dp.y)*(  dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[15];
    }
  #endif
  }
}

void BVHRT::LocalSurfaceIntersection(uint32_t type, const float3 ray_dir, uint32_t instId, uint32_t geomId,
                                     float values[8], uint32_t nodeId, uint32_t primId, float d, float qNear, 
                                     float qFar, float2 fNearFar, float3 start_q,
                                     CRT_Hit *pHit)
{
  const float EPS = 1e-6f;
  float d_inv = 1.0f / d;
  float t = qNear;
  bool hit = false;
  unsigned iter = 0;

  float start_dist = 10000;
  float start_sign = 1;

  start_dist = eval_dist_trilinear(values, start_q + t * ray_dir);
  /*If we want to represent thin surfaces, we should find rays that reach 
  //zero distance regardless of what thww sign of initial distance is.
  //However, it can lead to visual artifacts and disabled by default
  */
  if (m_preset.representation_mode == REPRESENTATION_MODE_SURFACE)
    start_sign = sign(start_dist);
  start_dist *= start_sign;

  if (start_dist <= EPS || m_preset.sdf_node_intersect == SDF_OCTREE_NODE_INTERSECT_BBOX)
  {
    hit = true;
  }
  else if (m_preset.sdf_node_intersect == SDF_OCTREE_NODE_INTERSECT_ST)
  {
    const unsigned ST_max_iters = 256;
    float dist = start_dist;
    float3 pp0 = start_q + t * ray_dir;

    while (t < qFar && dist > EPS && iter < ST_max_iters)
    {
      t += dist * d_inv;

      dist = start_sign*eval_dist_trilinear(values, start_q + t * ray_dir);
      
      float3 pp = start_q + t * ray_dir;
      iter++;
    }
    hit = (dist <= EPS);
  }
  else //if (m_preset.sdf_node_intersect == SDF_OCTREE_NODE_INTERSECT_ANALYTIC ||
       //    m_preset.sdf_node_intersect == SDF_OCTREE_NODE_INTERSECT_NEWTON ||
       //    m_preset.sdf_node_intersect == SDF_OCTREE_NODE_INTERSECT_IT)
  {
    //finding exact intersection between surface sdf(x,y,z) = 0 and ray
    // based on paper "Ray Tracing of Signed Distance Function Grids, 
    // Journal of Computer Graphics Techniques (JCGT), vol. 11, no. 3, 94-113, 2022"
    // http://jcgt.org/published/0011/03/06/

    // define values and constants as proposed in paper
    float s000 = values[0]*d_inv;
    float s001 = values[1]*d_inv;
    float s010 = values[2]*d_inv;
    float s011 = values[3]*d_inv;
    float s100 = values[4]*d_inv;
    float s101 = values[5]*d_inv;
    float s110 = values[6]*d_inv;
    float s111 = values[7]*d_inv;

    float a = s101-s001;

    float k0 = s000;
    float k1 = s100-s000;
    float k2 = s010-s000;
    float k3 = s110-s010-k1;
    float k4 = k0-s001;
    float k5 = k1-a;
    float k6 = k2-(s011-s001);
    float k7 = k3-(s111-s011-a);

    float3 o = start_q;
    float3 d3 = ray_dir;

    float m0 = o.x*o.y;
    float m1 = d3.x*d3.y;
    float m2 = o.x*d3.y + o.y*d3.x;
    float m3 = k5*o.z - k1;
    float m4 = k6*o.z - k2;
    float m5 = k7*o.z - k3;

    float c0 = (k4*o.z - k0) + o.x*m3 + o.y*m4 + m0*m5;
    float c1 = d3.x*m3 + d3.y*m4 + m2*m5 + d3.z*(k4 + k5*o.x + k6*o.y + k7*m0);
    float c2 = m1*m5 + d3.z*(k5*d3.x + k6*d3.y + k7*m2);
    float c3 = k7*m1*d3.z;

    // the surface is defined by equation c3*t^3 + c2*t^2 + c1*t + c0 = 0;
    // solve this equation analytically or numerically using the Newton's method
    // see "Numerical Recipes - The Art of Scientific Computing - 3rd Edition" for details

    if (m_preset.sdf_node_intersect == SDF_OCTREE_NODE_INTERSECT_ANALYTIC)
    {
      float x1 = 1000;
      float x2 = 1000;
      float x3 = 1000;
      unsigned type = 0;
      if (std::abs(c3) > 1e-2)
      {
        type = 3;
        //it is a cubic equation, transform it to x^3 + a*x^2 + b*x + c = 0
        //use Vieta method to obtain 3 or 1 real roots
        float a = c2/c3;
        float b = c1/c3;
        float c = c0/c3;   

        float Q = (a*a - 3*b)/9;
        float R = (2*a*a - 9*a*b + 27*c)/54;
        float Q3 = Q*Q*Q;

        if (R*R < Q3) //equation has three real roots
        {
          float theta = std::acos(R/sqrt(Q3));
          x1 = -2*sqrt(Q)*std::cos(theta/3) - a/3;
          x2 = -2*sqrt(Q)*std::cos((theta+2*M_PI)/3) - a/3;
          x3 = -2*sqrt(Q)*std::cos((theta-2*M_PI)/3) - a/3;
        }
        else //equation has only one real roots
        {
          float A = -sign(R)*std::pow(std::abs(R) + sqrt(R*R - Q3), 1.0f/3.0f);
          float B = std::abs(A) > EPS ? Q/A : 0;
          x1 = A+B - a/3;
        }
      }
      else if (std::abs(c2) > 1e-4)
      {
        type = 2;
        //it is a quadratic equation a*x^2 + b*x + c = 0
        float a = c2;
        float b = c1;
        float c = c0;

        float D = b*b - 4*a*c;
        if (D > 0)
        {
          float q = -0.5f*(b + sign(b)*std::sqrt(D));
          x1 = q/a;
          if (std::abs(q) > EPS)
            x2 = c/q; 
        }
      }
      else if (std::abs(c1) > EPS)
      {
        type = 1;
        //it is a linear equation c1*x + c0 = 0
        x1 = -c0/c1;
      }
      //else
      //no roots or inf roots, something's fucked up so just drop it

      x1 = x1 < 0 ? 1000 : x1;
      x2 = x2 < 0 ? 1000 : x2;
      x3 = x3 < 0 ? 1000 : x3;

      //bool prev_hit = hit;
      //float nt = std::min(x1, std::min(x2,x3));
      //if (prev_hit && std::abs(t - nt) > 0.1)
      //  printf("%f-%f -- %f %f %f %f -- %f %f %f, type %u\n",t, nt, c3,c2,c1,c0, x1,x2,x3, type);
      t = std::min(x1, std::min(x2,x3));
      hit = (t >= 0 && t <= qFar);
    }
    else if (m_preset.sdf_node_intersect == SDF_OCTREE_NODE_INTERSECT_NEWTON)
    {
      // our polynom is c3*t^3 + c2*t^2 + c1*t + c0 = 0;
      // it's derivative is  3*c3*t^2 + 2*c2*t + c1 = 0; 
      // find where it equals 0 to determine interval where the root is located
      // by solving a quadratic equation a*x^2 + b*x + c = 0
      float a = 3*c3;
      float b = 2*c2;
      float c = c1;

      float t0 = 0;
      float t1 = qFar;
      float t2 = qFar;
      float t3 = qFar;

      float D = b*b - 4*a*c;
      if (D >= 0)
      {
        float q = -0.5f*(b + sign(b)*std::sqrt(D));
        t1 = std::abs(a) > EPS ? q/a : t0;
        t2 = std::abs(q) > EPS ? c/q : qFar;

        float tmp = std::min(t1,t2);
        t2 = std::max(t1, t2);
        t1 = tmp;

        t1 = clamp(t1, t0, t3);
        t2 = clamp(t2, t0, t3);
      }
      
      //calculate sign of initial polynom at each critical point
      bool s0 = c0 > 0;
      bool s1 = (c0 + t1*(c1 + t1*(c2 + t1*c3))) > 0;
      bool s2 = (c0 + t2*(c1 + t2*(c2 + t2*c3))) > 0;
      bool s3 = (c0 + t3*(c1 + t3*(c2 + t3*c3))) > 0;

      //determine the range to apply Newton's method
      float nwt_min = t0;
      float nwt_max = t0;
      if (s0 != s1)
      {
        nwt_min = t0;
        nwt_max = t1;
      }
      else if (s1 != s2)
      {
        nwt_min = t1;
        nwt_max = t2;
      }
      else if (s2 != s3)
      {
        nwt_min = t2;
        nwt_max = t3;
      }

      float rtn = -100;

      if (nwt_min < nwt_max)
      {
        //perform Newton's method
        const unsigned max_iters = 10;
        unsigned iter = 0;
        rtn = 0.5f*(nwt_min + nwt_max);
        float f = 1000;
        while (iter < max_iters && std::abs(f) >= EPS)
        {
          f = c0 + rtn*(c1 + rtn*(c2 + rtn*c3));
          float df = c1 + rtn*(2*c2 + rtn*3*c3);
          float dx = f/(df + sign(df)*1e-9f);
          rtn -= dx;
          iter++;
        }
        t = rtn;
        hit = (t >= 0 && t <= qFar && std::abs(f) < EPS);
      }
      else
      {
        //no hit
        hit = false;
      }

      //bool prev_hit = hit;
      //float nt = rtn;
      //if (prev_hit && std::abs(t - nt) > 0.1)
      //  printf("%f-%f -- %f %f %f %f -- %f -- %f %f %f %f %d %d %d %d\n",t, nt, c3,c2,c1,c0, rtn, t0, t1, t2, t3, s0, s1, s2, s3);
    }
    else //if (m_preset.sdf_node_intersect == SDF_OCTREE_NODE_INTERSECT_IT)
    {
      const unsigned IT_max_iters = 256;
      const float k = 2;

      float e = 0.1f*qFar;
      float t_max = std::abs(c3) < EPS ? 1e6 : -c2/(3*c3);
      float df_max = 3*c3*t_max*t_max + 2*c2*t_max + c1;

      float dist = start_dist;
      float3 pp = start_q + t * ray_dir;

      while (t < qFar && dist > EPS && iter < IT_max_iters)
      {
        float df_1 = 3*c3*t*t + 2*c2*t + c1;
        float df_2 = 3*c3*(t+e)*(t+e) + 2*c2*(t+e) + c1;
        float L = (t_max > t && t_max < t + e) ? std::max(df_max, std::max(df_1, df_2)) : std::max(df_1, df_2);
        L = std::max(L, EPS);
        float s = std::min((dist*d_inv)/L, e);
        t += s;
        e = k*s;
        dist = start_sign*eval_dist_trilinear(values, start_q + t * ray_dir);
        pp = start_q + t * ray_dir;
        iter++;
      }
      hit = (dist <= EPS);
     
    }
  }

  float tReal = fNearFar.x + d * t;


#if ON_CPU==1
  if (debug_cur_pixel)
  {
    printf("\n");
    printf("sdf type = %u\n", type);
    //printf("node bbox [(%f %f %f)-(%f %f %f)]\n", min_pos.x, min_pos.y, min_pos.z, max_pos.x, max_pos.y, max_pos.z);
    printf("sdf values %f %f %f %f %f %f %f %f\n", 
           values[0], values[1], values[2], values[3],
           values[4], values[5], values[6], values[7]);
    printf("t = %f in [0, %f], tReal = %f in [%f %f]\n",t,qFar,tReal,fNearFar.x,fNearFar.y);
    printf("ray_dir = (%f %f %f)\n", ray_dir.x, ray_dir.y, ray_dir.z);
    //printf("ray_pos = (%f %f %f)\n", ray_pos.x, ray_pos.y, ray_pos.z);
    printf("\n");
  }
#endif

  if (t <= qFar && hit && tReal < pHit->t)
  {
    float3 norm = float3(0, 0, 1);
    if (need_normal())
    {
      float3 p0 = start_q + t * ray_dir;
      const float h = 0.001;
      float ddx = (eval_dist_trilinear(values, p0 + float3(h, 0, 0)) -
                   eval_dist_trilinear(values, p0 + float3(-h, 0, 0))) /
                  (2 * h);
      float ddy = (eval_dist_trilinear(values, p0 + float3(0, h, 0)) -
                   eval_dist_trilinear(values, p0 + float3(0, -h, 0))) /
                  (2 * h);
      float ddz = (eval_dist_trilinear(values, p0 + float3(0, 0, h)) -
                   eval_dist_trilinear(values, p0 + float3(0, 0, -h))) /
                  (2 * h);

      norm = start_sign*normalize(matmul4x3(m_instanceData[instId].transformInvTransposed, float3(ddx, ddy, ddz)));
    }
    
    float2 encoded_norm = encode_normal(norm);

    pHit->t = tReal;
    pHit->primId = primId;
    pHit->instId = instId;
    pHit->geomId = geomId | (type << SH_TYPE);
    pHit->coords[0] = 0;
    pHit->coords[1] = 0;
    pHit->coords[2] = encoded_norm.x;
    pHit->coords[3] = encoded_norm.y;

    if (m_preset.render_mode == MULTI_RENDER_MODE_ST_ITERATIONS)
      pHit->primId = iter;
    
  #ifndef DISABLE_SDF_FRAME_OCTREE_TEX
    if (type == TYPE_SDF_FRAME_OCTREE_TEX)
    {
      float3 dp = start_q + t * ray_dir;
      
      pHit->coords[0] = (1-dp.x)*(1-dp.y)*(1-dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[0] + 
                        (1-dp.x)*(1-dp.y)*(  dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[2] + 
                        (1-dp.x)*(  dp.y)*(1-dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[4] + 
                        (1-dp.x)*(  dp.y)*(  dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[6] + 
                        (  dp.x)*(1-dp.y)*(1-dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[8] + 
                        (  dp.x)*(1-dp.y)*(  dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[10] + 
                        (  dp.x)*(  dp.y)*(1-dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[12] + 
                        (  dp.x)*(  dp.y)*(  dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[14];

      pHit->coords[1] = (1-dp.x)*(1-dp.y)*(1-dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[1] + 
                        (1-dp.x)*(1-dp.y)*(  dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[3] + 
                        (1-dp.x)*(  dp.y)*(1-dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[5] + 
                        (1-dp.x)*(  dp.y)*(  dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[7] + 
                        (  dp.x)*(1-dp.y)*(1-dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[9] + 
                        (  dp.x)*(1-dp.y)*(  dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[11] + 
                        (  dp.x)*(  dp.y)*(1-dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[13] + 
                        (  dp.x)*(  dp.y)*(  dp.z)*m_SdfFrameOctreeTexNodes[nodeId].tex_coords[15];
    }
  #endif
  }
}

void BVHRT::OctreeNodeIntersect(uint32_t type, const float3 ray_pos, const float3 ray_dir,
                                float tNear, uint32_t instId, uint32_t geomId,
                                uint32_t a_start, uint32_t a_count,
                                CRT_Hit *pHit)
{
  float values[8];
  uint32_t nodeId, primId;
  float d, qNear, qFar;
  float2 fNearFar;
  float3 start_q;

  qNear = 1.0f;

  RayNodeIntersection(type, ray_pos, ray_dir, tNear, geomId, a_start, /*in */
                      values, primId, nodeId, d, qNear, qFar, fNearFar, start_q); /*out*/

  //fast return if starting point in this exact node or type is not supported
  if (qNear > 0.0f) 
    return;
  
  LocalSurfaceIntersection(type, ray_dir, instId, geomId, values, nodeId, primId, d, qNear, qFar, fNearFar, start_q, /*in */
                           pHit); /*out*/
}

  static float3 eval_dist_trilinear_diff(const float values[8], float3 dp)
  {
    float ddist_dx = -(1-dp.y)*(1-dp.z)*values[0] + 
                     -(1-dp.y)*(  dp.z)*values[1] + 
                     -(  dp.y)*(1-dp.z)*values[2] + 
                     -(  dp.y)*(  dp.z)*values[3] + 
                      (1-dp.y)*(1-dp.z)*values[4] + 
                      (1-dp.y)*(  dp.z)*values[5] + 
                      (  dp.y)*(1-dp.z)*values[6] + 
                      (  dp.y)*(  dp.z)*values[7];
    
    float ddist_dy = -(1-dp.x)*(1-dp.z)*values[0] + 
                     -(1-dp.x)*(  dp.z)*values[1] + 
                      (1-dp.x)*(1-dp.z)*values[2] + 
                      (1-dp.x)*(  dp.z)*values[3] + 
                     -(  dp.x)*(1-dp.z)*values[4] + 
                     -(  dp.x)*(  dp.z)*values[5] + 
                      (  dp.x)*(1-dp.z)*values[6] + 
                      (  dp.x)*(  dp.z)*values[7];

    float ddist_dz = -(1-dp.x)*(1-dp.y)*values[0] + 
                      (1-dp.x)*(1-dp.y)*values[1] + 
                     -(1-dp.x)*(  dp.y)*values[2] + 
                      (1-dp.x)*(  dp.y)*values[3] + 
                     -(  dp.x)*(1-dp.y)*values[4] + 
                      (  dp.x)*(1-dp.y)*values[5] + 
                     -(  dp.x)*(  dp.y)*values[6] + 
                      (  dp.x)*(  dp.y)*values[7];
  
    return float3(ddist_dx, ddist_dy, ddist_dz);
  }

//transition from 0 (x=edge0) to 1 (x=edge1)
static float step(float edge0, float edge1, float x)
{
  float t = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
  return t;
}

float BVHRT::load_tricubic_distance_values(uint32_t nodeId, float3 voxelPos, uint32_t v_size, float sz_inv, const SdfSBSHeader &header, float values[64])
{
  float vmin = 1e6f;
#ifndef DISABLE_SDF_SBS
  if (header.aux_data == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F ||
      header.aux_data == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F_IN)
  {
    uint32_t v_off = m_SdfSBSNodes[nodeId].data_offset;
    
    float3 p0 = float3(int3(voxelPos)) - 1;

    for (int x = 0; x < 4; x++)
    {
      for (int y = 0; y < 4; y++)
      {
        for (int z = 0; z < 4; z++)
        {
          float3 vPos = p0 + float3(x, y, z);
          uint32_t vId = SBS_v_to_i(vPos.x, vPos.y, vPos.z, v_size, header.brick_pad); 
          values[16 * z + 4 * y + x] = m_SdfSBSDataF[m_SdfSBSData[v_off + vId]];
          vmin = std::min(vmin, values[16 * z + 4 * y + x]);
        }
      }
    }
  }
#endif
  return vmin;
}

float BVHRT::load_distance_values(uint32_t nodeId, float3 voxelPos, uint32_t v_size, float sz_inv, const SdfSBSHeader &header, float values[8])
{
  float vmin = 1e6f;
#ifndef DISABLE_SDF_SBS
  if (header.aux_data == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F ||
      header.aux_data == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F_IN)
  {
    uint32_t v_off = m_SdfSBSNodes[nodeId].data_offset;
    //  (000) (001) (010) (011) (100) (101) (110) (111)

    for (int i = 0; i < 8; i++)
    {
      uint3 vPos = uint3(voxelPos) + uint3((i & 4) >> 2, (i & 2) >> 1, i & 1);
      uint32_t vId = SBS_v_to_i(vPos.x, vPos.y, vPos.z, v_size, header.brick_pad);
      values[i] = m_SdfSBSDataF[m_SdfSBSData[v_off + vId]];
      // printf("%f\n", values[i]);
      vmin = std::min(vmin, values[i]);
    }
  }
  else
  {
    uint32_t v_off = m_SdfSBSNodes[nodeId].data_offset;
    uint32_t vals_per_int = 4 / header.bytes_per_value;
    uint32_t bits = 8 * header.bytes_per_value;
    uint32_t max_val = header.bytes_per_value == 4 ? 0xFFFFFFFF : ((1 << bits) - 1);
    float d_max = 1.73205081f * sz_inv;
    float mult = 2 * d_max / max_val;
    for (int i = 0; i < 8; i++)
    {
      uint3 vPos = uint3(voxelPos) + uint3((i & 4) >> 2, (i & 2) >> 1, i & 1);
      uint32_t vId = SBS_v_to_i(vPos.x, vPos.y, vPos.z, v_size, header.brick_pad);;
      values[i] = -d_max + mult * ((m_SdfSBSData[v_off + vId / vals_per_int] >> (bits * (vId % vals_per_int))) & max_val);
      vmin = std::min(vmin, values[i]);
    }
  }
#endif
  return vmin;
}

void BVHRT::OctreeBrickIntersect(uint32_t type, const float3 ray_pos, const float3 ray_dir,
                                 float tNear, uint32_t instId, uint32_t geomId,
                                 uint32_t bvhNodeId, uint32_t a_count,
                                 CRT_Hit *pHit)
{
  #ifndef DISABLE_SDF_SBS
  float tricub_values[64], trilinear_values[8]; // values

  uint32_t nodeId, primId;
  float d, qNear, qFar;
  float2 fNearFar;
  float3 start_q;

  qNear = 1.0f;

  uint32_t sdfId =  m_geomData[geomId].offset.x;
  primId = bvhNodeId; //id of bbox in BLAS
  nodeId = primId + m_SdfSBSRoots[sdfId];
  SdfSBSHeader header = m_SdfSBSHeaders[sdfId];
  uint32_t v_size = header.brick_size + 2*header.brick_pad + 1;

  float px = m_SdfSBSNodes[nodeId].pos_xy >> 16;
  float py = m_SdfSBSNodes[nodeId].pos_xy & 0x0000FFFF;
  float pz = m_SdfSBSNodes[nodeId].pos_z_lod_size >> 16;
  float sz = m_SdfSBSNodes[nodeId].pos_z_lod_size & 0x0000FFFF;
  float sz_inv = 2.0f/sz;
  
  d = 2.0f/(sz*header.brick_size);

  float3 brick_min_pos = float3(-1,-1,-1) + sz_inv*float3(px,py,pz);
  float3 brick_max_pos = brick_min_pos + sz_inv*float3(1,1,1);
  float3 brick_size = brick_max_pos - brick_min_pos;

  float2 brick_fNearFar = RayBoxIntersection2(ray_pos, SafeInverse(ray_dir), brick_min_pos, brick_max_pos);
  float old_t = pHit->t;
  while (brick_fNearFar.x < brick_fNearFar.y && pHit->t == old_t)
  {
    float3 hit_pos = ray_pos + brick_fNearFar.x*ray_dir;
    float3 local_pos = (hit_pos - brick_min_pos) * (0.5f*sz*header.brick_size);
    float3 voxelPos = floor(clamp(local_pos, 1e-6f, header.brick_size-1e-6f));

    float3 min_pos = brick_min_pos + d*voxelPos;
    float3 max_pos = min_pos + d*float3(1,1,1);
    float3 size = max_pos - min_pos;

    float vmin = 0;

    if (m_preset.interpolation_mode == INTERPOLATION_MODE_TRILINEAR)
    {
      vmin = load_distance_values(nodeId, voxelPos, v_size, sz_inv, header, trilinear_values);
    }
    else
    {
      vmin = load_tricubic_distance_values(nodeId, voxelPos, v_size, sz_inv, header, tricub_values);
    }

    fNearFar = RayBoxIntersection2(ray_pos, SafeInverse(ray_dir), min_pos, max_pos);
    if (tNear < fNearFar.x && vmin <= 0.0f)    
    {
      float3 start_pos = ray_pos + fNearFar.x*ray_dir;
      start_q = (start_pos - min_pos) * (0.5f*sz*header.brick_size);
      qFar = (fNearFar.y - fNearFar.x) * (0.5f*sz*header.brick_size);
    
      if (m_preset.interpolation_mode == INTERPOLATION_MODE_TRILINEAR)
      {
        LocalSurfaceIntersection(type, ray_dir, instId, geomId, trilinear_values, nodeId, primId, d, 0.0f, qFar, fNearFar, start_q, /*in */
                               pHit); /*out*/
      }
      else
      {
        TricubicLocalSurfaceIntersection(type, ray_dir, instId, geomId, tricub_values, nodeId, primId, d, 0.0f, qFar, fNearFar, start_q, /*in */
                               pHit);
      }
    
      if (m_preset.interpolation_mode == INTERPOLATION_MODE_TRILINEAR && header.aux_data == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F_IN &&
          m_preset.normal_mode == NORMAL_MODE_SDF_SMOOTHED && 
          need_normal() && pHit->t != old_t)
      {
        const float beta = 0.5f;
        float3 dp = start_q + ((pHit->t - fNearFar.x)/d)*ray_dir; //linear interpolation coefficients in voxels

        float3 normals[8];
        float values_n[8];
        for (int i=0;i<8;i++)
          normals[i] = float3(0,0,0);
        
        float3 nmq = float3(0.5f,0.5f,0.5f);
        nmq.x = dp.x < 0.5f ? step(-beta, beta, dp.x) : step(-beta, beta, dp.x - 1.0f);
        nmq.y = dp.y < 0.5f ? step(-beta, beta, dp.y) : step(-beta, beta, dp.y - 1.0f);
        nmq.z = dp.z < 0.5f ? step(-beta, beta, dp.z) : step(-beta, beta, dp.z - 1.0f);

        int3 voxelPos0 = int3(dp.x < 0.5f ? voxelPos.x - 1 : voxelPos.x, dp.y < 0.5f ? voxelPos.y - 1 : voxelPos.y, dp.z < 0.5f ? voxelPos.z - 1 : voxelPos.z);
        for (int i=0;i<8;i++)
        {
          int3 VoxelPosI = voxelPos0 + int3((i & 4) >> 2, (i & 2) >> 1, i & 1);
          int3 BrickPosI = int3((VoxelPosI.x >= 0 ? (VoxelPosI.x < header.brick_size ? 0 : 1) : -1),
                                (VoxelPosI.y >= 0 ? (VoxelPosI.y < header.brick_size ? 0 : 1) : -1),
                                (VoxelPosI.z >= 0 ? (VoxelPosI.z < header.brick_size ? 0 : 1) : -1));

          uint32_t neighborId = 0;
          neighborId += 3*3 * (BrickPosI.x + 1);
          neighborId +=   3 * (BrickPosI.y + 1);
          neighborId +=       (BrickPosI.z + 1);
          
          float3 dVoxelPos = float3(VoxelPosI) - voxelPos;
          uint32_t neighbor_nodeId;
          float3   neighbor_voxelPos;

          if (neighborId == 9+3+1)//we have our neighbor voxel in the same brick
          {
            neighbor_nodeId = nodeId;
            neighbor_voxelPos = float3(VoxelPosI);
          }
          else //we have our neighbor voxel in a different brick, read it from neigbors data
          {
            uint32_t v_off = m_SdfSBSNodes[nodeId].data_offset;
            uint32_t neighbors_data_offset = v_size*v_size*v_size + 8;

            neighbor_nodeId = m_SdfSBSData[v_off + neighbors_data_offset + neighborId];
            neighbor_voxelPos = float3(VoxelPosI) - header.brick_size*float3(BrickPosI);
          }

          if (neighbor_nodeId != INVALID_IDX)
          {
            load_distance_values(neighbor_nodeId, neighbor_voxelPos, v_size, sz_inv, header, values_n);
            normals[i] = normalize(eval_dist_trilinear_diff(values_n, dp - dVoxelPos));
          }
        }

        float3 smoothed_norm = (1-nmq.x)*(1-nmq.y)*(1-nmq.z)*normals[0] + 
                               (1-nmq.x)*(1-nmq.y)*(  nmq.z)*normals[1] + 
                               (1-nmq.x)*(  nmq.y)*(1-nmq.z)*normals[2] + 
                               (1-nmq.x)*(  nmq.y)*(  nmq.z)*normals[3] + 
                               (  nmq.x)*(1-nmq.y)*(1-nmq.z)*normals[4] + 
                               (  nmq.x)*(1-nmq.y)*(  nmq.z)*normals[5] + 
                               (  nmq.x)*(  nmq.y)*(1-nmq.z)*normals[6] + 
                               (  nmq.x)*(  nmq.y)*(  nmq.z)*normals[7];
        smoothed_norm = normalize(smoothed_norm);
        float2 encoded_norm = encode_normal(smoothed_norm);

        pHit->coords[2] = encoded_norm.x;
        pHit->coords[3] = encoded_norm.y;
      }
    }
    
    brick_fNearFar.x += std::max(0.0f, fNearFar.y-brick_fNearFar.x) + 1e-6f;
  }
  
  //ray hit a brick
  if (pHit->t < old_t)
  {
    float3 pos = ray_pos + pHit->t*ray_dir;
    float3 dp = (pos - brick_min_pos)*(0.5f*sz);

    if (header.aux_data == SDF_SBS_NODE_LAYOUT_DX_UV16)
    {   
      uint32_t vals_per_int = 4/header.bytes_per_value;
      uint32_t t_off = m_SdfSBSNodes[nodeId].data_offset + (v_size*v_size*v_size+vals_per_int-1)/vals_per_int;

      pHit->coords[0] = (1-dp.x)*(1-dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfSBSData[t_off+0] >> 16)) + 
                        (1-dp.x)*(1-dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfSBSData[t_off+1] >> 16)) + 
                        (1-dp.x)*(  dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfSBSData[t_off+2] >> 16)) + 
                        (1-dp.x)*(  dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfSBSData[t_off+3] >> 16)) + 
                        (  dp.x)*(1-dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfSBSData[t_off+4] >> 16)) + 
                        (  dp.x)*(1-dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfSBSData[t_off+5] >> 16)) + 
                        (  dp.x)*(  dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfSBSData[t_off+6] >> 16)) + 
                        (  dp.x)*(  dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfSBSData[t_off+7] >> 16));

      pHit->coords[1] = (1-dp.x)*(1-dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfSBSData[t_off+0] & 0xFFFF)) + 
                        (1-dp.x)*(1-dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfSBSData[t_off+1] & 0xFFFF)) + 
                        (1-dp.x)*(  dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfSBSData[t_off+2] & 0xFFFF)) + 
                        (1-dp.x)*(  dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfSBSData[t_off+3] & 0xFFFF)) + 
                        (  dp.x)*(1-dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfSBSData[t_off+4] & 0xFFFF)) + 
                        (  dp.x)*(1-dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfSBSData[t_off+5] & 0xFFFF)) + 
                        (  dp.x)*(  dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfSBSData[t_off+6] & 0xFFFF)) + 
                        (  dp.x)*(  dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfSBSData[t_off+7] & 0xFFFF));
    }
    else if (header.aux_data == SDF_SBS_NODE_LAYOUT_DX_RGB8)
    {
      uint32_t vals_per_int = 4/header.bytes_per_value;
      uint32_t t_off = m_SdfSBSNodes[nodeId].data_offset + (v_size*v_size*v_size+vals_per_int-1)/vals_per_int;

      float3 color = (1-dp.x)*(1-dp.y)*(1-dp.z)*float3((m_SdfSBSData[t_off+0] >> 0) & 0xFF, (m_SdfSBSData[t_off+0] >> 8) & 0xFF, (m_SdfSBSData[t_off+0] >> 16) & 0xFF) + 
                     (1-dp.x)*(1-dp.y)*(  dp.z)*float3((m_SdfSBSData[t_off+1] >> 0) & 0xFF, (m_SdfSBSData[t_off+1] >> 8) & 0xFF, (m_SdfSBSData[t_off+1] >> 16) & 0xFF) + 
                     (1-dp.x)*(  dp.y)*(1-dp.z)*float3((m_SdfSBSData[t_off+2] >> 0) & 0xFF, (m_SdfSBSData[t_off+2] >> 8) & 0xFF, (m_SdfSBSData[t_off+2] >> 16) & 0xFF) + 
                     (1-dp.x)*(  dp.y)*(  dp.z)*float3((m_SdfSBSData[t_off+3] >> 0) & 0xFF, (m_SdfSBSData[t_off+3] >> 8) & 0xFF, (m_SdfSBSData[t_off+3] >> 16) & 0xFF) + 
                     (  dp.x)*(1-dp.y)*(1-dp.z)*float3((m_SdfSBSData[t_off+4] >> 0) & 0xFF, (m_SdfSBSData[t_off+4] >> 8) & 0xFF, (m_SdfSBSData[t_off+4] >> 16) & 0xFF) + 
                     (  dp.x)*(1-dp.y)*(  dp.z)*float3((m_SdfSBSData[t_off+5] >> 0) & 0xFF, (m_SdfSBSData[t_off+5] >> 8) & 0xFF, (m_SdfSBSData[t_off+5] >> 16) & 0xFF) + 
                     (  dp.x)*(  dp.y)*(1-dp.z)*float3((m_SdfSBSData[t_off+6] >> 0) & 0xFF, (m_SdfSBSData[t_off+6] >> 8) & 0xFF, (m_SdfSBSData[t_off+6] >> 16) & 0xFF) + 
                     (  dp.x)*(  dp.y)*(  dp.z)*float3((m_SdfSBSData[t_off+7] >> 0) & 0xFF, (m_SdfSBSData[t_off+7] >> 8) & 0xFF, (m_SdfSBSData[t_off+7] >> 16) & 0xFF);

      color = clamp(floor(color + 0.5f), 0.0f, 255.0f);

      pHit->coords[0] = color.x + color.y/256.0f;
      pHit->coords[1] = color.z;

      //printf("color = %f %f %f coords = %f %f\n", color.x, color.y, color.z, pHit->coords[0], pHit->coords[1]);
    }
    else if (header.aux_data == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F ||
             header.aux_data == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F_IN)
    {
      uint32_t t_off = m_SdfSBSNodes[nodeId].data_offset + v_size*v_size*v_size;

      float3 color = (1-dp.x)*(1-dp.y)*(1-dp.z)*float3(m_SdfSBSDataF[m_SdfSBSData[t_off+0]+0], m_SdfSBSDataF[m_SdfSBSData[t_off+0]+1], m_SdfSBSDataF[m_SdfSBSData[t_off+0]+2]) + 
                     (1-dp.x)*(1-dp.y)*(  dp.z)*float3(m_SdfSBSDataF[m_SdfSBSData[t_off+1]+0], m_SdfSBSDataF[m_SdfSBSData[t_off+1]+1], m_SdfSBSDataF[m_SdfSBSData[t_off+1]+2]) + 
                     (1-dp.x)*(  dp.y)*(1-dp.z)*float3(m_SdfSBSDataF[m_SdfSBSData[t_off+2]+0], m_SdfSBSDataF[m_SdfSBSData[t_off+2]+1], m_SdfSBSDataF[m_SdfSBSData[t_off+2]+2]) + 
                     (1-dp.x)*(  dp.y)*(  dp.z)*float3(m_SdfSBSDataF[m_SdfSBSData[t_off+3]+0], m_SdfSBSDataF[m_SdfSBSData[t_off+3]+1], m_SdfSBSDataF[m_SdfSBSData[t_off+3]+2]) + 
                     (  dp.x)*(1-dp.y)*(1-dp.z)*float3(m_SdfSBSDataF[m_SdfSBSData[t_off+4]+0], m_SdfSBSDataF[m_SdfSBSData[t_off+4]+1], m_SdfSBSDataF[m_SdfSBSData[t_off+4]+2]) + 
                     (  dp.x)*(1-dp.y)*(  dp.z)*float3(m_SdfSBSDataF[m_SdfSBSData[t_off+5]+0], m_SdfSBSDataF[m_SdfSBSData[t_off+5]+1], m_SdfSBSDataF[m_SdfSBSData[t_off+5]+2]) + 
                     (  dp.x)*(  dp.y)*(1-dp.z)*float3(m_SdfSBSDataF[m_SdfSBSData[t_off+6]+0], m_SdfSBSDataF[m_SdfSBSData[t_off+6]+1], m_SdfSBSDataF[m_SdfSBSData[t_off+6]+2]) + 
                     (  dp.x)*(  dp.y)*(  dp.z)*float3(m_SdfSBSDataF[m_SdfSBSData[t_off+7]+0], m_SdfSBSDataF[m_SdfSBSData[t_off+7]+1], m_SdfSBSDataF[m_SdfSBSData[t_off+7]+2]);

      color = clamp(floor(255.0f*color + 0.5f), 0.0f, 255.0f);

      pHit->coords[0] = color.x + color.y/256.0f;
      pHit->coords[1] = color.z;
      //printf("color = %f %f %f coords = %f %f\n", color.x, color.y, color.z, pHit->coords[0], pHit->coords[1]);
    }
  }
#endif
}

void BVHRT::OctreeAdaptBrickIntersect(uint32_t type, const float3 ray_pos, const float3 ray_dir,
                                      float tNear, uint32_t instId, uint32_t geomId,
                                      uint32_t bvhNodeId, uint32_t a_count,
                                      CRT_Hit *pHit)
{
#ifndef DISABLE_SDF_SBS_ADAPT
  float values[8];
  uint32_t nodeId, primId;
  float qNear, qFar;
  float2 fNearFar;
  float3 start_q;

  qNear = 1.0f;

  uint32_t sdfId =  m_geomData[geomId].offset.x;
  primId = bvhNodeId; //id of bbox in BLAS
  nodeId = primId + m_SdfSBSAdaptRoots[sdfId];
  SdfSBSAdaptHeader header = m_SdfSBSAdaptHeaders[sdfId];


  float px = m_SdfSBSAdaptNodes[nodeId].pos_xy >> 16;
  float py = m_SdfSBSAdaptNodes[nodeId].pos_xy & 0x0000FFFF;
  float pz = m_SdfSBSAdaptNodes[nodeId].pos_z_vox_size >> 16;
  float vs = m_SdfSBSAdaptNodes[nodeId].pos_z_vox_size & 0x0000FFFF;

  // voxel count
  uint3 brick_size{(m_SdfSBSAdaptNodes[nodeId].vox_count_xyz_pad >> 16) & 0x000000FF,
                   (m_SdfSBSAdaptNodes[nodeId].vox_count_xyz_pad >>  8) & 0x000000FF,
                   (m_SdfSBSAdaptNodes[nodeId].vox_count_xyz_pad      ) & 0x000000FF};
  uint3 v_size = brick_size + 2*header.brick_pad + 1;


  float d = (2.0f/SDF_SBS_ADAPT_MAX_UNITS) * vs;
  float voxel_abs_size_inv = 1.f / d;
  float3 brick_abs_size = d * float3(brick_size);
  float3 brick_min_pos = float3(-1,-1,-1) + (2.0f/SDF_SBS_ADAPT_MAX_UNITS)*float3(px,py,pz);
  float3 brick_max_pos = brick_min_pos + brick_abs_size;


  float2 brick_fNearFar = RayBoxIntersection2(ray_pos, SafeInverse(ray_dir), brick_min_pos, brick_max_pos);
  float old_t = pHit->t;
  while (brick_fNearFar.x < brick_fNearFar.y && pHit->t == old_t)
  {
    float3 hit_pos = ray_pos + brick_fNearFar.x*ray_dir;
    float3 local_pos = (hit_pos - brick_min_pos) * voxel_abs_size_inv;
    uint3  voxel_pos{(uint32_t)clamp(local_pos.x, 1e-6f, brick_size.x-1e-6f),
                     (uint32_t)clamp(local_pos.y, 1e-6f, brick_size.y-1e-6f),
                     (uint32_t)clamp(local_pos.z, 1e-6f, brick_size.z-1e-6f)};

    float3 min_pos = brick_min_pos + d*float3(voxel_pos);
    float3 max_pos = min_pos + d;
    float3 size = max_pos - min_pos;

    float vmin = 1.0f;

    if (header.aux_data == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F)
    {
      uint32_t v_off = m_SdfSBSAdaptNodes[nodeId].data_offset;
      for (int i=0;i<8;i++)
      {
        uint3 vPos = voxel_pos + uint3((i & 4) >> 2, (i & 2) >> 1, i & 1);
        uint32_t vId = vPos.x*v_size.y*v_size.z + vPos.y*v_size.z + vPos.z;
        values[i] = m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[v_off + vId]];
        vmin = std::min(vmin, values[i]);
      }
    }
    else
    {
      uint32_t v_off = m_SdfSBSAdaptNodes[nodeId].data_offset;
      uint32_t vals_per_int = 4/header.bytes_per_value; 
      uint32_t bits = 8*header.bytes_per_value;
      uint32_t max_val = header.bytes_per_value == 4 ? 0xFFFFFFFF : ((1 << bits) - 1);
      float d_max = 1.73205081f*std::max(std::max(brick_abs_size.x, brick_abs_size.y), brick_abs_size.z);
      float mult = 2*d_max/max_val;
      for (int i=0;i<8;i++)
      {
        uint3 vPos = voxel_pos + uint3((i & 4) >> 2, (i & 2) >> 1, i & 1);
        uint32_t vId = vPos.x*v_size.y*v_size.z + vPos.y*v_size.z + vPos.z;
        values[i] = -d_max + mult*((m_SdfSBSAdaptData[v_off + vId/vals_per_int] >> (bits*(vId%vals_per_int))) & max_val);
        vmin = std::min(vmin, values[i]);
      }
    }

    fNearFar = RayBoxIntersection2(ray_pos, SafeInverse(ray_dir), min_pos, max_pos);
    if (tNear < fNearFar.x && vmin <= 0.0f)
    {
      float3 start_pos = ray_pos + fNearFar.x*ray_dir;
      start_q = (start_pos - min_pos) * voxel_abs_size_inv;
      qFar = (fNearFar.y - fNearFar.x) * voxel_abs_size_inv;

      LocalSurfaceIntersection(type, ray_dir, instId, geomId, values, nodeId, primId, d, 0.0f, qFar, fNearFar, start_q, /*in */
                               pHit); /*out*/
    }

    brick_fNearFar.x += std::max(0.0f, fNearFar.y-brick_fNearFar.x) + 1e-6f;
  }

  //ray hit a brick
  if (pHit->t < old_t)
  {
    float3 pos = ray_pos + pHit->t*ray_dir;
    float3 dp = (pos - brick_min_pos)*voxel_abs_size_inv;

    if (header.aux_data == SDF_SBS_NODE_LAYOUT_DX_UV16)
    {
      uint32_t vals_per_int = 4/header.bytes_per_value;
      uint32_t t_off = m_SdfSBSAdaptNodes[nodeId].data_offset + (v_size.x*v_size.y*v_size.z+vals_per_int-1)/vals_per_int;

      pHit->coords[0] = (1-dp.x)*(1-dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfSBSAdaptData[t_off+0] >> 16)) + 
                        (1-dp.x)*(1-dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfSBSAdaptData[t_off+1] >> 16)) + 
                        (1-dp.x)*(  dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfSBSAdaptData[t_off+2] >> 16)) + 
                        (1-dp.x)*(  dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfSBSAdaptData[t_off+3] >> 16)) + 
                        (  dp.x)*(1-dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfSBSAdaptData[t_off+4] >> 16)) + 
                        (  dp.x)*(1-dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfSBSAdaptData[t_off+5] >> 16)) + 
                        (  dp.x)*(  dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfSBSAdaptData[t_off+6] >> 16)) + 
                        (  dp.x)*(  dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfSBSAdaptData[t_off+7] >> 16));

      pHit->coords[1] = (1-dp.x)*(1-dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfSBSAdaptData[t_off+0] & 0xFFFF)) + 
                        (1-dp.x)*(1-dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfSBSAdaptData[t_off+1] & 0xFFFF)) + 
                        (1-dp.x)*(  dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfSBSAdaptData[t_off+2] & 0xFFFF)) + 
                        (1-dp.x)*(  dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfSBSAdaptData[t_off+3] & 0xFFFF)) + 
                        (  dp.x)*(1-dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfSBSAdaptData[t_off+4] & 0xFFFF)) + 
                        (  dp.x)*(1-dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfSBSAdaptData[t_off+5] & 0xFFFF)) + 
                        (  dp.x)*(  dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfSBSAdaptData[t_off+6] & 0xFFFF)) + 
                        (  dp.x)*(  dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfSBSAdaptData[t_off+7] & 0xFFFF));
    }
    else if (header.aux_data == SDF_SBS_NODE_LAYOUT_DX_RGB8)
    {
      uint32_t vals_per_int = 4/header.bytes_per_value;
      uint32_t t_off = m_SdfSBSAdaptNodes[nodeId].data_offset + (v_size.x*v_size.y*v_size.z+vals_per_int-1)/vals_per_int;

      float3 color = (1-dp.x)*(1-dp.y)*(1-dp.z)*float3((m_SdfSBSAdaptData[t_off+0] >> 0) & 0xFF, (m_SdfSBSAdaptData[t_off+0] >> 8) & 0xFF, (m_SdfSBSAdaptData[t_off+0] >> 16) & 0xFF) + 
                     (1-dp.x)*(1-dp.y)*(  dp.z)*float3((m_SdfSBSAdaptData[t_off+1] >> 0) & 0xFF, (m_SdfSBSAdaptData[t_off+1] >> 8) & 0xFF, (m_SdfSBSAdaptData[t_off+1] >> 16) & 0xFF) + 
                     (1-dp.x)*(  dp.y)*(1-dp.z)*float3((m_SdfSBSAdaptData[t_off+2] >> 0) & 0xFF, (m_SdfSBSAdaptData[t_off+2] >> 8) & 0xFF, (m_SdfSBSAdaptData[t_off+2] >> 16) & 0xFF) + 
                     (1-dp.x)*(  dp.y)*(  dp.z)*float3((m_SdfSBSAdaptData[t_off+3] >> 0) & 0xFF, (m_SdfSBSAdaptData[t_off+3] >> 8) & 0xFF, (m_SdfSBSAdaptData[t_off+3] >> 16) & 0xFF) + 
                     (  dp.x)*(1-dp.y)*(1-dp.z)*float3((m_SdfSBSAdaptData[t_off+4] >> 0) & 0xFF, (m_SdfSBSAdaptData[t_off+4] >> 8) & 0xFF, (m_SdfSBSAdaptData[t_off+4] >> 16) & 0xFF) + 
                     (  dp.x)*(1-dp.y)*(  dp.z)*float3((m_SdfSBSAdaptData[t_off+5] >> 0) & 0xFF, (m_SdfSBSAdaptData[t_off+5] >> 8) & 0xFF, (m_SdfSBSAdaptData[t_off+5] >> 16) & 0xFF) + 
                     (  dp.x)*(  dp.y)*(1-dp.z)*float3((m_SdfSBSAdaptData[t_off+6] >> 0) & 0xFF, (m_SdfSBSAdaptData[t_off+6] >> 8) & 0xFF, (m_SdfSBSAdaptData[t_off+6] >> 16) & 0xFF) + 
                     (  dp.x)*(  dp.y)*(  dp.z)*float3((m_SdfSBSAdaptData[t_off+7] >> 0) & 0xFF, (m_SdfSBSAdaptData[t_off+7] >> 8) & 0xFF, (m_SdfSBSAdaptData[t_off+7] >> 16) & 0xFF);

      color = clamp(floor(color + 0.5f), 0.0f, 255.0f);

      pHit->coords[0] = color.x + color.y/256.0f;
      pHit->coords[1] = color.z;
    }
    else if (header.aux_data == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F)
    {
      uint32_t t_off = m_SdfSBSAdaptNodes[nodeId].data_offset + v_size.x*v_size.y*v_size.z;

      float3 color = (1-dp.x)*(1-dp.y)*(1-dp.z)*float3(m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+0]+0], m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+0]+1], m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+0]+2]) + 
                     (1-dp.x)*(1-dp.y)*(  dp.z)*float3(m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+1]+0], m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+1]+1], m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+1]+2]) + 
                     (1-dp.x)*(  dp.y)*(1-dp.z)*float3(m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+2]+0], m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+2]+1], m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+2]+2]) + 
                     (1-dp.x)*(  dp.y)*(  dp.z)*float3(m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+3]+0], m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+3]+1], m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+3]+2]) + 
                     (  dp.x)*(1-dp.y)*(1-dp.z)*float3(m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+4]+0], m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+4]+1], m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+4]+2]) + 
                     (  dp.x)*(1-dp.y)*(  dp.z)*float3(m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+5]+0], m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+5]+1], m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+5]+2]) + 
                     (  dp.x)*(  dp.y)*(1-dp.z)*float3(m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+6]+0], m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+6]+1], m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+6]+2]) + 
                     (  dp.x)*(  dp.y)*(  dp.z)*float3(m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+7]+0], m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+7]+1], m_SdfSBSAdaptDataF[m_SdfSBSAdaptData[t_off+7]+2]);

      color = clamp(floor(255.0f*color + 0.5f), 0.0f, 255.0f);

      pHit->coords[0] = color.x + color.y/256.0f;
      pHit->coords[1] = color.z;
    }
  }
#endif
}

void BVHRT::IntersectAllSdfsInLeaf(const float3 ray_pos, const float3 ray_dir,
                                   float tNear, uint32_t instId, uint32_t geomId,
                                   uint32_t a_start, uint32_t a_count,
                                   CRT_Hit *pHit)
{

  uint32_t type = m_geomData[geomId].type;
  uint32_t sdfId = 0;
  uint32_t primId = 0;

  float3 min_pos = float3(0,0,0), max_pos = float3(0,0,0);

  switch (type)
  {
#ifndef DISABLE_SDF_GRID
  case TYPE_SDF_GRID:
    sdfId = m_geomData[geomId].offset.x;
    primId = 0;
    min_pos = float3(-1,-1,-1);
    max_pos = float3( 1, 1, 1);
    break;
#endif
  default:
    break;
  }

  float l = length(ray_dir);
  float3 dir = ray_dir/l;
  SdfHit hit = sdf_sphere_tracing(type, sdfId, min_pos, max_pos, tNear, ray_pos, dir, need_normal());
  if (hit.hit_pos.w > 0)
  {
    float t = length(to_float3(hit.hit_pos)-ray_pos)/l;
    if (t > tNear && t < pHit->t)
    {
      float3 n = normalize(matmul4x3(m_instanceData[instId].transformInvTransposed, to_float3(hit.hit_norm)));
      float2 encoded_norm = encode_normal(n);

      pHit->t         = t;
      pHit->primId    = primId;
      pHit->instId    = instId;
      pHit->geomId    = geomId | (type << SH_TYPE);  
      pHit->coords[0] = 0;
      pHit->coords[1] = 0;
      pHit->coords[2] = encoded_norm.x;
      pHit->coords[3] = encoded_norm.y;

      if (m_preset.render_mode == MULTI_RENDER_MODE_ST_ITERATIONS)
        pHit->primId = uint32_t(hit.hit_norm.w);
    }
  }
}

#ifndef DISABLE_RF_GRID
int indexGrid(int x, int y, int z, int gridSize) {
    return (x + y * gridSize + z * gridSize * gridSize) * 28;
}

void lerpCellf(const float v0[28], const float v1[28], const float t, float memory[28])
{
  for (int i = 0; i < 28; i++)
    memory[i] = LiteMath::lerp(v0[i], v1[i], t);
}

void BVHRT::lerpCell(const uint32_t idx0, const uint32_t idx1, const float t, float memory[28]) {
  for (int i = 0; i < 28; i++)
    memory[i] = LiteMath::lerp(m_RFGridData[28 * idx0 + i], m_RFGridData[28 * idx1 + i], t);
}

// From Mitsuba 3
void sh_eval_2(const float3 &d, float fout[9])
{
  float x = d.x, y = d.y, z = d.z, z2 = z * z;
  float c0, c1, s0, s1, tmp_a, tmp_b, tmp_c;

  fout[0] = 0.28209479177387814;
  fout[2] = z * 0.488602511902919923;
  fout[6] = z2 * 0.94617469575756008 + -0.315391565252520045;
  c0 = x;
  s0 = y;

  tmp_a = -0.488602511902919978;
  fout[3] = tmp_a * c0;
  fout[1] = tmp_a * s0;
  tmp_b = z * -1.09254843059207896;
  fout[7] = tmp_b * c0;
  fout[5] = tmp_b * s0;
  c1 = x * c0 - y * s0;
  s1 = x * s0 + y * c0;

  tmp_c = 0.546274215296039478;
  fout[8] = tmp_c * c1;
  fout[4] = tmp_c * s1;
}

float eval_sh(float sh[28], float3 rayDir, const int offset)
{
  float sh_coeffs[9];
  sh_eval_2(rayDir, sh_coeffs);

  float sum = 0.0f;
  for (int i = 0; i < 9; i++)
    sum += sh[offset + i] * sh_coeffs[i];

  return sum;
}

static float sigmoid(float x) {
  return 1 / (1 + exp(-x));
}

void BVHRT::RayGridIntersection(float3 ray_dir, uint32_t gridSize, float3 p, float3 lastP, uint4 ptrs, uint4 ptrs2, float &throughput, float3 &colour)
{
  float3 coords = p * (float)(gridSize);

  int3 nearCoords = clamp((int3)coords, int3(0), int3(gridSize - 1));
  int3 farCoords = clamp((int3)coords + int3(1), int3(0), int3(gridSize - 1));

  float3 lerpFactors = coords - (float3)nearCoords;

  float xy00[28];
  float xy10[28];
  float xy01[28];
  float xy11[28];
  
  lerpCell(ptrs[0], ptrs[1], lerpFactors.x, xy00);
  lerpCell(ptrs[2], ptrs2[0], lerpFactors.x, xy10);
  lerpCell(ptrs[3], ptrs2[2], lerpFactors.x, xy01);
  lerpCell(ptrs2[1], ptrs2[3], lerpFactors.x, xy11);


  float xyz0[28];
  float xyz1[28];
  lerpCellf(xy00, xy10, lerpFactors.y, xyz0);
  lerpCellf(xy01, xy11, lerpFactors.y, xyz1);

  float gridVal[28];
  lerpCellf(xyz0, xyz1, lerpFactors.z, gridVal);

  // relu
  /* if (gridVal[0] < 0.0) */
  /*   gridVal[0] = 0.0; */

  // for (size_t i = 0; i < 28; i++)
    // std::cout << (&grid[indexGrid(nearCoords[0], nearCoords[1], nearCoords[2], gridSize)])[i] << ' ';
  // std::cout << std::endl;

  float dist = 1.0f / (float) gridSize * 4.0f;
  /* float dist = length(p - lastP); */
  /* if (dist > sqrt(3) / (float)gridSize) */
  /*     dist -= ((int)(dist * (float)gridSize) - 1) / (float)gridSize; */

  float tr = clamp(exp(-gridVal[0] * m_RFGridScales[0] * dist), 0.0f, 1.0f);

  // std::cout << tr << ' ' << gridVal[0] << ' ' << length(p - lastP) << ' ' << gridSize << std::endl;

  // float3 RGB = float3(1.0f);
  float3 RGB = float3(sigmoid(eval_sh(gridVal, ray_dir, 1)), sigmoid(eval_sh(gridVal, ray_dir, 10)), sigmoid(eval_sh(gridVal, ray_dir, 19)));
  colour = colour + throughput * (1 - tr) * RGB;
  throughput *= tr;
}

void BVHRT::IntersectRFInLeaf(const float3 ray_pos, const float3 ray_dir,
                                   float tNear, uint32_t instId, uint32_t geomId,
                                   uint32_t a_start, uint32_t a_count,
                                   CRT_Hit *pHit)
{
  uint32_t type = m_geomData[geomId].type;
  uint32_t sdfId = 0;
  uint32_t primId = 0;

  auto bbox = m_origNodes[a_start];
  float2 smallBox = RayBoxIntersection2(ray_pos, SafeInverse(ray_dir), bbox.boxMin, bbox.boxMax);

  float depth = (smallBox.x + (smallBox.y - smallBox.x) * 0.5);
  float3 p = ray_pos + ray_dir * depth;

  float throughput = pHit->coords[0];
  float3 colour = float3(pHit->coords[1], pHit->coords[2], pHit->coords[3]);

  if (m_RFGridFlags[1] == 1) {
    uint4 ptrs;
    ptrs[0] = 8 * a_start;
    ptrs[1] = 8 * a_start + 1;
    ptrs[2] = 8 * a_start + 2;
    ptrs[3] = 8 * a_start + 3;

    uint4 ptrs2;
    ptrs2[0] = 8 * a_start + 4;
    ptrs2[1] = 8 * a_start + 5;
    ptrs2[2] = 8 * a_start + 6;
    ptrs2[3] = 8 * a_start + 7;

    RayGridIntersection(ray_dir, m_RFGridSizes[0], p, float3(0.0f), ptrs, ptrs2, throughput, colour);
  } else
    RayGridIntersection(ray_dir, m_RFGridSizes[0], p, float3(0.0f), m_RFGridPtrs[2 * a_start], m_RFGridPtrs[2 * a_start + 1], throughput, colour);
  
  // std::cout << throughput << std::endl;

  pHit->primId = a_start;
  pHit->geomId = geomId | (type << SH_TYPE);

  pHit->coords[0] = throughput;
  pHit->coords[1] = colour[0];
  pHit->coords[2] = colour[1];
  pHit->coords[3] = colour[2];

  pHit->t = depth;

  /* float3 edge = ray_pos + ray_dir * zNearAndFar.y; */
  /* pHit->adds[0] = length(edge - p); */
  /* pHit->adds[1] = 0.0f; */
  /* pHit->adds[2] = 0.0f; */
  /* pHit->adds[3] = 0.0f; */

  // std::cout << "Mew" << std::endl;
}
#endif

#ifndef DISABLE_GS_PRIMITIVE
float4 QuaternionMultiply(const float4& a, const float4& b) {
    float4 c;

    c.x = a.x * b.x - a.y * b.y - a.z * b.z - a.w * b.w;
    c.y = a.x * b.y + a.y * b.x + a.z * b.w - a.w * b.z;
    c.z = a.x * b.z - a.y * b.w + a.z * b.x + a.w * b.y;
    c.w = a.x * b.w + a.y * b.z - a.z * b.y + a.w * b.x;

    return c;
}

float4 QuaternionConjugate(const float4& q) {
    return float4(q.x, -q.y, -q.z, -q.w);
}

float3 RotatePoint(const float3& p, const float4& q) {
    float4 result = QuaternionMultiply(QuaternionMultiply(QuaternionConjugate(q), float4(0.0f, p.x, p.y, p.z)), q);

    return float3(result.y, result.z, result.w);
}

void BVHRT::IntersectGSInLeaf(const float3& ray_pos, const float3& ray_dir,
                              float tNear, uint32_t instId,
                              uint32_t geomId, uint32_t a_start,
                              uint32_t a_count, CRT_Hit* pHit) {
    float sh_c0 = 0.28209479177387814f;

    for (uint32_t i = a_start; i < a_start + a_count; ++i) {
        float3 mean = float3(m_gs_data_0[i][0][0], m_gs_data_0[i][0][1], m_gs_data_0[i][0][2]);
        float3 scale = float3(exp(m_gs_data_0[i][1][3]), exp(m_gs_data_0[i][2][0]), exp(m_gs_data_0[i][2][1])) * 3.0f;
        float4 rotation = QuaternionConjugate(normalize(float4(m_gs_data_0[i][2][2], -m_gs_data_0[i][2][3], m_gs_data_0[i][3][0], m_gs_data_0[i][3][1])));
        float3 diffuse_color = float3(m_gs_data_0[i][0][3], m_gs_data_0[i][1][0], m_gs_data_0[i][1][1]) * sh_c0;
        float opacity = sigmoid(m_gs_data_0[i][1][2]);

        float3 origin = ray_pos - mean;
        float3 direction = ray_dir;

        origin = RotatePoint(origin, rotation);
        direction = RotatePoint(direction, rotation);

        origin    = origin / scale;
        direction = direction / scale;

        float a = dot(direction, direction);
        float b = 2.0f * dot(origin, direction);
        float c = dot(origin, origin) - 1.0f;

        float discriminant = b * b - 4.0f * a * c;

        if (discriminant < 1e-9f) {
            continue;
        }

        float t1 = (-b + sqrt(discriminant)) / (2.0f * a);
        float t2 = (-b - sqrt(discriminant)) / (2.0f * a);

        if (t1 > tNear && t2 > tNear) {
            float t = (t1 + t2) / 2.0f;
            float3 intersection = ray_pos + t * ray_dir;
            float3 distance = intersection - mean;

            float power  = -0.5f * (
                m_gs_conic[i][0][0] * distance.x * distance.x +
                m_gs_conic[i][1][1] * distance.y * distance.y +
                m_gs_conic[i][2][2] * distance.z * distance.z) -
                m_gs_conic[i][0][1] * distance.x * distance.y -
                m_gs_conic[i][0][2] * distance.x * distance.z -
                m_gs_conic[i][1][2] * distance.y * distance.z;

            if (power > 0.0f) {
                continue;
            }

            float alpha = min(0.99f, opacity * float(exp(power)));

            if (alpha < 1.0f / 255.0f) {
                continue;
            }

            float transparency = pHit->coords[0] * (1.0f - alpha);

            if (transparency < 0.0001f) { 
                break;
            }

            float weight = alpha * pHit->coords[0];

            pHit->coords[1] += (0.5f + diffuse_color.x) * weight;
            pHit->coords[2] += (0.5f + diffuse_color.y) * weight;
            pHit->coords[3] += (0.5f + diffuse_color.z) * weight;
            pHit->coords[0] = transparency;

            pHit->primId = i;
        }
    }
}
#endif

#ifndef DISABLE_NURBS

//////////////////// NURBS SECTION /////////////////////////////////////////////////////
float4 BVHRT::control_point(uint i, int offset) {
  return float4 {
    m_NURBSData[offset+i*4+0],
    m_NURBSData[offset+i*4+1],
    m_NURBSData[offset+i*4+2],
    m_NURBSData[offset+i*4+3],
  };
}

float BVHRT::knot(uint i, int knots_offset) {
  return m_NURBSData[knots_offset+i];
}

int BVHRT::find_span(float t, int knots_offset, int knots_count, NURBSHeader h) {
  if (t == knot(knots_count-1, knots_offset))
    return knots_count-2;

  int l = 0;
  int r = knots_count-1;
  while(r-l > 1) {
    int m = (l+r)/2;
    if (t < knot(m, knots_offset))
      r = m;
    else 
      l = m;
  }

  //assert(knot(l, knots_offset) <= t && t < knot(l+1, knots_offset));
  return l;
}

float4 BVHRT::rbezier_curve_point(float u, int p, int offset) {
  float u_n = 1.0f;
  float _1_u = 1.0f - u;
  int bc = 1;
  float4 res = control_point(0, offset) * _1_u;
  for (int i = 1; i <= p-1; ++i) {
    u_n *= u;
    bc = bc * (p-i+1)/i;
    res = (res + u_n * bc * control_point(i, offset)) * _1_u;
  }
  res += (u_n * u) * control_point(p, offset);
  return res;
}

float4 BVHRT::rbezier_surface_point(float u, float v, int points_offset, NURBSHeader h) {
  int p = h.p;
  int q = h.q;
  float u_n = 1.0f;
  float _1_u = 1.0f - u;
  int bc = 1;
  float4 res = rbezier_curve_point(v, q, points_offset+(q+1)*4*0) * _1_u;
  for (int i = 1; i <= p-1; ++i) {
    u_n *= u;
    bc = bc * (p-i+1)/i;
    res = (res + u_n * bc * rbezier_curve_point(v, q, points_offset+(q+1)*4*i)) * _1_u;
  }
  res += (u_n * u) * rbezier_curve_point(v, q, points_offset+(q+1)*4*p);
  return res;
}

float4 BVHRT::rbezier_grid_point(float u, float v, NURBSHeader h) {
  int uoffset = uknots_offset(h);
  int voffset = vknots_offset(h);
  int uspan = find_span(u, uoffset, h.uknots_cnt, h);
  int vspan = find_span(v, voffset, h.vknots_cnt, h);
  float umin = knot(uspan, uoffset), umax = knot(uspan+1, uoffset);
  float vmin = knot(vspan, voffset), vmax = knot(vspan+1, voffset);
  u = (u-umin)/(umax-umin);
  v = (v-vmin)/(vmax-vmin);
  return rbezier_surface_point(u, v, pts_offset(h, uspan, vspan), h);
}

float4 BVHRT::rbezier_curve_der(float u, int p, int offset) {
  if (p == 1) {
    float4 cur_pnt = control_point(0, offset);
    float4 next_pnt = control_point(1, offset);
    float4 res = (next_pnt-cur_pnt) * p;
    return res;
  }

  float u_n = 1.0f;
  float _1_u = 1.0f - u;
  int bc = 1;

  float4 cur_pnt = control_point(0, offset);
  float4 next_pnt = control_point(1, offset);
  float4 res = (next_pnt-cur_pnt) * _1_u;
  cur_pnt = next_pnt;

  for (int i = 1; i <= p-2; ++i) {
    u_n *= u;
    bc = bc * (p-i)/i;
    next_pnt = control_point(i+1, offset);
    res = (res + u_n * bc * (next_pnt-cur_pnt)) * _1_u;
    cur_pnt = next_pnt;
  }

  next_pnt = control_point(p, offset);
  res += (u_n * u) * (next_pnt-cur_pnt);

  res *= p;

  return res;
}

float4 BVHRT::rbezier_surface_vder(float u, float v, const float4 &Sw, int points_offset, NURBSHeader h) {
  int p = h.p;
  int q = h.q;
  float u_n = 1.0f;
  float _1_u = 1.0f - u;
  int bc = 1;
  float4 Sw_der = rbezier_curve_der(v, q, points_offset+(q+1)*4*0) * _1_u;
  for (int i = 1; i <= p-1; ++i) {
    u_n *= u;
    bc = bc * (p-i+1)/i;
    Sw_der = (Sw_der + u_n * bc * rbezier_curve_der(v, q, points_offset+(q+1)*4*i)) * _1_u;
  }
  Sw_der += (u_n * u) * rbezier_curve_der(v, q, points_offset+(q+1)*4*p);

  float4 S_der = (Sw_der * Sw.w - Sw * Sw_der.w) / (Sw.w * Sw.w);
  return S_der;
}

float4 BVHRT::rbezier_surface_uder(float u, float v, const float4 &Sw, int points_offset, NURBSHeader h) {
  int p = h.p;
  int q = h.q;
  if (p == 1) {
    float4 cur_point = rbezier_curve_point(v, q, points_offset+(q+1)*4*0);
    float4 next_point = rbezier_curve_point(v, q, points_offset+(q+1)*4*1);
    float4 Sw_der = (next_point - cur_point) * p;
    float4 S_der = (Sw_der * Sw.w - Sw * Sw_der.w)/(Sw.w * Sw.w);
    return S_der;
  }

  float u_n = 1.0f;
  float _1_u = 1.0f - u;
  int bc = 1;
  float4 Sw_der = { 0.0f, 0.0f, 0.0f, 0.0f };

  float4 cur_point = rbezier_curve_point(v, q, points_offset+(q+1)*4*0);
  float4 next_point = rbezier_curve_point(v, q, points_offset+(q+1)*4*1);
  Sw_der = (next_point - cur_point) * _1_u;
  cur_point = next_point;

  for (int i = 1; i <= p-2; ++i) {
    u_n *= u;
    bc = bc * (p-i)/i;
    next_point = rbezier_curve_point(v, q, points_offset+(q+1)*4*(i+1));
    Sw_der = (Sw_der + u_n * bc * (next_point-cur_point)) * _1_u;
    cur_point = next_point;
  }

  next_point = rbezier_curve_point(v, q, points_offset+(q+1)*4*p);
  Sw_der += (u_n * u) * (next_point-cur_point);

  Sw_der *= p;
  
  float4 S_der = (Sw_der * Sw.w - Sw * Sw_der.w)/(Sw.w * Sw.w);
  return S_der;
}

float4 BVHRT::rbezier_grid_uder(float u, float v, const float4 &Sw, NURBSHeader h) {
  int uoffset = uknots_offset(h);
  int voffset = vknots_offset(h);
  int uspan = find_span(u, uoffset, h.uknots_cnt, h);
  int vspan = find_span(v, voffset, h.vknots_cnt, h);
  float umin = knot(uspan, uoffset), umax = knot(uspan+1, uoffset);
  float vmin = knot(vspan, voffset), vmax = knot(vspan+1, voffset);
  u = (u-umin)/(umax-umin);
  v = (v-vmin)/(vmax-vmin);
  float4 surf_der = rbezier_surface_uder(u, v, Sw, pts_offset(h, uspan, vspan), h);
  return surf_der * (1.0f/(umax-umin));
}

float4 BVHRT::rbezier_grid_vder(float u, float v, const float4 &Sw, NURBSHeader h) {
  int uoffset = uknots_offset(h);
  int voffset = vknots_offset(h);
  int uspan = find_span(u, uoffset, h.uknots_cnt, h);
  int vspan = find_span(v, voffset, h.vknots_cnt, h);
  float umin = knot(uspan, uoffset), umax = knot(uspan+1, uoffset);
  float vmin = knot(vspan, voffset), vmax = knot(vspan+1, voffset);
  u = (u-umin)/(umax-umin);
  v = (v-vmin)/(vmax-vmin);
  float4 surf_der = rbezier_surface_vder(u, v, Sw, pts_offset(h, uspan, vspan), h);
  return surf_der * (1.0f/(vmax-vmin));
}

inline
float2 project2planes(
    const float4 &P1, 
    const float4 &P2,
    const float4 &point) {
  return float2{ dot(P1, point), dot(P2, point) };
}

NURBS_HitInfo BVHRT::ray_nurbs_newton_intersection(
    const LiteMath::float3 &pos,
    const LiteMath::float3 &ray,
    float2 uv,
    NURBSHeader h) {
  const int max_steps = 16;
  const float EPS = 1e-3f;

  float3 ortho_dir1 = (abs(ray.x) > abs(ray.y) && abs(ray.x) > abs(ray.z)) 
                    ? float3{ -ray.y, ray.x, 0 } 
                    : float3{ 0, -ray.z, ray.y };
  float3 ortho_dir2 = normalize(cross(ortho_dir1, ray));
  ortho_dir1 = normalize(cross(ray, ortho_dir2));
  // assert(dot(ortho_dir1, ortho_dir2) < 1e-2f);
  // assert(dot(ortho_dir1, ray) < 1e-2f);
  // assert(dot(ortho_dir2, ray) < 1e-2f);

  float4 P1 = to_float4(ortho_dir1, -dot(ortho_dir1, pos));
  float4 P2 = to_float4(ortho_dir2, -dot(ortho_dir2, pos));
  // assert(dot(P1, to_float4(pos, 1.0f)) < 1e-2);
  // assert(dot(P2, to_float4(pos, 1.0f)) < 1e-2);

  float4 Sw = rbezier_grid_point(uv.x, uv.y, h);
  float4 surf_point = Sw;
  surf_point /= surf_point.w;
  float2 D = project2planes(P1, P2, surf_point); 

  NURBS_HitInfo hit_info;
  hit_info.hitten = false;

  int steps_left = max_steps-1;
  while(length(D) > EPS && steps_left > 0) {
    --steps_left;

    float2 J[2] = 
    { 
      project2planes(P1, P2, rbezier_grid_uder(uv.x, uv.y, Sw, h)), //col1
      project2planes(P1, P2, rbezier_grid_vder(uv.x, uv.y, Sw, h)) //col2
    };

    float det = J[0][0]*J[1][1] - J[0][1] * J[1][0];

    float2 J_inversed[2] = 
    {
      { J[1][1]/det, -J[0][1]/det },
      { -J[1][0]/det, J[0][0]/det }
    };

    uv = uv - (J_inversed[0]*D[0]+J_inversed[1]*D[1]);
    uv.x = clamp(uv.x, 0.0f, 1.0f);
    uv.y = clamp(uv.y, 0.0f, 1.0f);
    // assert(0 <= uv.x && uv.x <= 1);
    // assert(0 <= uv.y && uv.y <= 1);

    Sw = rbezier_grid_point(uv.x, uv.y, h);
    surf_point = Sw;
    surf_point /= surf_point.w;
    float2 new_D = project2planes(P1, P2, surf_point);
    
    if (length(new_D) > length(D))
      return hit_info; // hitten = false
    
    D = new_D;
  }

  if (length(D) > EPS)
    return hit_info; // hitten = false;
  
  float3 uder = to_float3(rbezier_grid_uder(uv.x, uv.y, Sw, h));
  float3 vder = to_float3(rbezier_grid_vder(uv.x, uv.y, Sw, h));
  float3 normal = normalize(cross(uder, vder));

  hit_info.hitten = true;
  hit_info.point = to_float3(surf_point);
  hit_info.normal = normal;
  hit_info.uv = uv;
  return hit_info;
}
#endif

void BVHRT::IntersectNURBS(const float3& ray_pos, const float3& ray_dir,
                           float tNear, uint32_t approx_offset, uint32_t instId,
                           uint32_t geomId, CRT_Hit* pHit) {
#ifndef DISABLE_NURBS
  uint nurbsId = m_geomData[geomId].offset.x;
  NURBSHeader header  = m_NURBSHeaders[nurbsId];
  uint type = m_geomData[geomId].type;

  float3 min_pos = to_float3(m_geomData[geomId].boxMin);
  float3 max_pos = to_float3(m_geomData[geomId].boxMax);
  float2 tNear_tFar = box_intersects(min_pos, max_pos, ray_pos, ray_dir);

  float u0 = m_NURBS_approxes[approx_offset];
  float v0 = m_NURBS_approxes[approx_offset+1];

  NURBS_HitInfo hit = ray_nurbs_newton_intersection(ray_pos, ray_dir, float2{u0, v0}, header);
  if (hit.hitten) {
    float2 uv = hit.uv;
    float3 point = hit.point;
    float3 normal = hit.normal;
    float t = dot(normalize(ray_dir), point-ray_pos);
    if (t < tNear_tFar.x || t > tNear_tFar.y)
      return;
    pHit->geomId = geomId | (type << SH_TYPE);
    pHit->t = t;
    pHit->primId = 0;
    pHit->instId = instId;

    pHit->coords[0] = uv.x;
    pHit->coords[1] = uv.y;
  } 
  // pHit->geomId = geomId | (type << SH_TYPE);
  // pHit->t = tNear;
  // pHit->primId = 0;
  // pHit->instId = instId;

  // pHit->coords[0] = u0;
  // pHit->coords[1] = v0;
#endif
}
//////////////////////// END NURBS SECTION ///////////////////////////////////////////////



//////////////////// CATMUL_CLARK SECTION /////////////////////////////////////////////////////
void BVHRT::IntersectCatmulClark(const float3& ray_pos, const float3& ray_dir,
                      float tNear, uint32_t instId,
                      uint32_t geomId, CRT_Hit* pHit) {
#ifndef DISABLE_CATMUL_CLARK
  // offset of catmul clark object in headers array
  uint32_t offset = m_geomData[geomId].offset.x;
  // object header
  CatmulClarkHeader header = m_CatmulClarkHeaders[offset];
  // object type <=> catmul_clark
  uint32_t type = m_geomData[geomId].type;

  // you can access your box of entire object (this is not box of bvh leave)
  float3 min_pos = to_float3(m_geomData[geomId].boxMin);
  float3 max_pos = to_float3(m_geomData[geomId].boxMax);
  float2 tNear_tFar = box_intersects(min_pos, max_pos, ray_pos, ray_dir);

  // you can pass bvh leave information to this function in GeomDataCatmulClark::Intersect 
  // ...

  float3 norm = normalize(ray_pos + tNear_tFar.x * ray_dir);
  float2 encoded_norm = encode_normal(norm); // compress 3dim normal vector to 2dim vector
  
  pHit->t = tNear_tFar.x;
  pHit->primId = 0;
  pHit->geomId = geomId | (type << SH_TYPE);
  pHit->instId = instId;
  pHit->coords[0] = 0; // u texture coordinate
  pHit->coords[1] = 0; // v texture coordinate
  pHit->coords[2] = encoded_norm.x;
  pHit->coords[3] = encoded_norm.y;
#endif
}
//////////////////////// END CATMUL CLARK SECTION ///////////////////////////////////////////////

//////////////////// RIBBON SECTION /////////////////////////////////////////////////////
void BVHRT::IntersectRibbon(const float3& ray_pos, const float3& ray_dir,
                      float tNear, uint32_t instId,
                      uint32_t geomId, CRT_Hit* pHit) {
#ifndef DISABLE_RIBBON
  // offset of ribbon object in headers array
  uint32_t offset = m_geomData[geomId].offset.x;
  // object header
  RibbonHeader header = m_RibbonHeaders[offset];
  // object type <=> catmul_clark
  uint32_t type = m_geomData[geomId].type;

  // you can access your box of entire object (this is not box of bvh leave)
  float3 min_pos = to_float3(m_geomData[geomId].boxMin);
  float3 max_pos = to_float3(m_geomData[geomId].boxMax);
  float2 tNear_tFar = box_intersects(min_pos, max_pos, ray_pos, ray_dir);

  // you can pass bvh leave information to this function in GeomDataRibbon::Intersect 
  // ...

  float3 norm = normalize(ray_pos + tNear_tFar.x * ray_dir);
  float2 encoded_norm = encode_normal(norm); // compress 3dim normal vector to 2dim vector
  
  pHit->t = tNear_tFar.x;
  pHit->primId = 0;
  pHit->geomId = geomId | (type << SH_TYPE);
  pHit->instId = instId;
  pHit->coords[0] = 0; // u texture coordinate
  pHit->coords[1] = 0; // v texture coordinate
  pHit->coords[2] = encoded_norm.x;
  pHit->coords[3] = encoded_norm.y;
#endif
}
//////////////////////// END RIBBON SECTION ///////////////////////////////////////////////

#ifndef KERNEL_SLICER
#ifndef DISABLE_OPENVDB
void BVHRT::IntersectOpenVDB_Grid(const float3& ray_pos, const float3& ray_dir,
                           float tNear, uint32_t instId,
                           uint32_t geomId, CRT_Hit* pHit)
{
  uint32_t openvdbId = m_geomData[geomId].offset.x;
  uint32_t type = m_geomData[geomId].type;
  OpenVDBHeader header = m_VDBHeaders[openvdbId];
  OpenVDB_Grid grid = m_VDBData[header.offset];

  float3 min_pos = to_float3(m_geomData[geomId].boxMin);
  float3 max_pos = to_float3(m_geomData[geomId].boxMax);
  float2 tNear_tFar = box_intersects(min_pos, max_pos, ray_pos, ray_dir);
  
  float dist = 1000;
  float t = std::max(tNear, tNear_tFar.x), tFar = tNear_tFar.y;
  float start_sign = 1;
  float EPS = 1e-6f;
  bool hit = 0;
  int iter = 0;
  const uint32_t ST_max_iters = 256;

  float3 point = ray_pos + t * ray_dir;
  
  dist = grid.get_distance(point);
  start_sign = sign(dist);
  dist *= start_sign;

  while (t < tFar && dist > EPS && iter < ST_max_iters)
  {
    t += dist + EPS;
    point = ray_pos + t * ray_dir;
    dist = start_sign * grid.get_distance(point);
    iter++;  
  }
  
  hit = (dist <= EPS);

  if (!hit || t < tNear || t > tFar)
  {
    return;
  }

  pHit->geomId = geomId | (type << SH_TYPE);
  pHit->t = t;
  pHit->primId = 0;
  pHit->instId = instId;

  pHit->coords[0] = 0;
  pHit->coords[1] = 0;

  float3 norm = float3(0, 0, 1);
  if (need_normal())
  {
    point = ray_pos + t * ray_dir;

    const float h = 0.001;
    float ddx = (start_sign * grid.get_distance(point + float3(h, 0, 0)) -
                  grid.get_distance(point + float3(-h, 0, 0))) /
                (2 * h);
    float ddy = (grid.get_distance(point + float3(0, h, 0)) -
                  grid.get_distance(point + float3(0, -h, 0))) /
                (2 * h);
    float ddz = (grid.get_distance(point + float3(0, 0, h)) -
                  grid.get_distance(point + float3(0, 0, -h))) /
                (2 * h);

    norm = start_sign * normalize(matmul4x3(m_instanceData[instId].transformInvTransposed, float3(ddx, ddy, ddz)));
    float2 encoded_norm = encode_normal(norm);
    pHit->coords[2] = encoded_norm.x;
    pHit->coords[3] = encoded_norm.y;
  }
}

#endif
#endif

float4 rayCapsuleIntersect(const float3& ray_pos, const float3& ray_dir,
                           const float3& pa, const float3& pb, float ra)
{
  float3 ba = pb - pa;
  float3 oa = ray_pos - pa;
  float3 ob = ray_pos - pb;
  float baba = dot(ba,ba);
  float bard = dot(ba,ray_dir);
  float baoa = dot(ba,oa);
  float rdoa = dot(ray_dir,oa);
  float oaoa = dot(oa,oa);
  float a = baba      - bard*bard;
  float b = baba*rdoa - baoa*bard;
  float c = baba*oaoa - baoa*baoa - ra*ra*baba;
  float h = b*b - a*c;

  if(h >= 0.f)
  {
    float dist = (-b-sqrt(h))/a;
    float y = baoa + dist*bard;

    // body
    if(y > 0.f && y < baba)
      return to_float4(normalize(oa+dist*ray_dir - ba*y/baba), dist);

    // cap
    float3 oc = (y <= 0.0) ? oa : ob;
    b = dot(ray_dir,oc);
    c = dot(oc,oc) - ra*ra;
    h = b*b - c;
    if(h > 0.f)
    {
      dist = -b - sqrt(h);
      float3 norm = normalize(ray_pos+dist*ray_dir + (oc - ray_pos));
      return to_float4(norm, dist);
    }
  }
  return float4(2e15f);
}


void BVHRT::IntersectGraphicPrims(const float3& ray_pos, const float3& ray_dir,
                                  float tNear, uint32_t instId,
                                  uint32_t geomId, uint32_t a_start,
                                  uint32_t a_count, CRT_Hit* pHit)
{
#ifndef DISABLE_GRAPHICS_PRIM

  const float EPS = 1e-5f, T_MAX = 1e15f;
  uint32_t primId     = m_geomData[geomId].offset.x;
  uint32_t nextPrimId = m_geomData[geomId].offset.y;
  GraphicsPrimHeader header = m_GraphicsPrimHeaders[primId];
  float3 color = header.color;

  uint32_t prim_len = 2u;
  if (header.prim_type == GRAPH_PRIM_POINT)
    prim_len = 1u;
  if (header.prim_type >= GRAPH_PRIM_LINE_COLOR)
    prim_len = 3u;
  
  uint32_t start_index = primId + prim_len * a_start;
  uint32_t end_index = start_index + prim_len; // 1 per bbox
  // start_index = primId;
  // end_index = nextPrimId;

  bool has_custom_color = header.prim_type >= GRAPH_PRIM_POINT_COLOR;

  if (header.prim_type == GRAPH_PRIM_POINT ||
      header.prim_type == GRAPH_PRIM_POINT_COLOR)
  {
    for (uint32_t i = start_index; i < end_index; i += prim_len)
    {
      float4 point = m_GraphicsPrimPoints[i];
      if (has_custom_color)
        color = to_float3(m_GraphicsPrimPoints[i+1]);

      float3 point3 = float3(point.x, point.y, point.z);
      float pt_radius = point.w;
      float3 pos_minus_point3 = ray_pos - point3;

      float b = 2.f * dot(ray_dir, pos_minus_point3);
      float c = dot(pos_minus_point3, pos_minus_point3) - pt_radius * pt_radius;
      float D = b*b - 4*c;
      if (D >= EPS)
      {
        D = std::sqrt(D);
        float t = -(b + D) * 0.5f;
        if (t < EPS)
          t = (-b + D) * 0.5f;

        if (t > tNear && t < pHit->t)
        {
          float3 norm = normalize((ray_pos + t*ray_dir - point3) * 100.f);
          float2 encoded_norm = encode_normal(norm);

          pHit->t         = t;
          pHit->primId    = primId;
          pHit->instId    = instId;
          pHit->geomId    = geomId | (TYPE_GRAPHICS_PRIM << SH_TYPE);
          pHit->coords[0] = color.x + color.y/256.0f;
          pHit->coords[1] = color.z;
          pHit->coords[2] = encoded_norm.x;
          pHit->coords[3] = encoded_norm.y;
        }
      }
    }
  }
  else
  {
    for (uint32_t i = start_index; i < end_index; i += prim_len)
    {
      float4 point1 = m_GraphicsPrimPoints[i  ];
      float4 point2 = m_GraphicsPrimPoints[i+1];
      if (has_custom_color)
        color = to_float3(m_GraphicsPrimPoints[i+2]);

      float3 point1_3 = float3(point1.x, point1.y, point1.z);
      float3 point2_3 = float3(point2.x, point2.y, point2.z);
      if (header.prim_type == GRAPH_PRIM_LINE_SEGMENT_DIR ||
          header.prim_type == GRAPH_PRIM_LINE_SEGMENT_DIR_COLOR)
        point2_3 = point2_3 * 0.71f + point1_3 * 0.29f; // so that the line segment doesn't clip through the cone
      float ra = point1.w; // line (cylinder) radius

      //assert(ra > EPS);

      float t = T_MAX;
      float3 norm = float3(0.f, 0.f, 0.f);

      if (header.prim_type == GRAPH_PRIM_LINE ||
          header.prim_type == GRAPH_PRIM_LINE_SEGMENT ||
          header.prim_type == GRAPH_PRIM_LINE_SEGMENT_DIR ||
          header.prim_type == GRAPH_PRIM_LINE_COLOR ||
          header.prim_type == GRAPH_PRIM_LINE_SEGMENT_COLOR ||
          header.prim_type == GRAPH_PRIM_LINE_SEGMENT_DIR_COLOR)
      {
        float3 oc =  ray_pos - point1_3;
        float3 ba = point2_3 - point1_3;

        float baba = dot(ba, ba);
        float bard = dot(ba, ray_dir);
        float baoc = dot(ba, oc);

        float k2 = baba - (bard * bard);
        float k1 = baba*dot(oc, ray_dir) - baoc*bard;
        float k0 = baba*dot(oc, oc) - baoc*baoc - ra*ra*baba;

        float D = k1*k1 - k2*k0;

        if (D > 0.f)
        {
          float dist = -(k1 + std::sqrt(D)) / k2;
          float y = baoc + dist*bard;

          if (header.prim_type == GRAPH_PRIM_LINE || (y > 0.f && y < baba))
          {
            t = dist;
            norm = normalize(oc+t*ray_dir - ba*y/baba);
          }
          else
          {
            dist = ((y < 0.f ? 0.f : baba) - baoc)/bard;
            if (std::abs(k1+k2*dist)<std::sqrt(D))
            {
              t = dist;
              norm = normalize(ba*sign(y));
            }
          }
        }
      }
      if (header.prim_type == GRAPH_PRIM_LINE_SEGMENT_DIR ||
          header.prim_type == GRAPH_PRIM_LINE_SEGMENT_DIR_COLOR)
      {
        ra = length(point2_3 - point1_3) * 0.15f;
        point2_3 = float3(point2.x, point2.y, point2.z);
        float3 pa = point2_3 * 0.7f + point1_3 * 0.3f; // cone length along line segment is 0.3*length(b - a) for now
        float3 ba = point2_3 - pa;
        float3 oa = ray_pos - pa;
        float3 ob = ray_pos - point2_3;
        float  m0 = dot(ba,ba);
        float  m1 = dot(oa,ba);
        float  m2 = dot(ray_dir,ba);
        float  m3 = dot(ray_dir,oa);
        float  m5 = dot(oa,oa);
        float  m9 = dot(ob,ba);

        // cap - only one
        if(m1 < 0.f)
        {
          float3 tmp1 = oa * m2 - ray_dir * m1;
          if(dot(tmp1, tmp1) < (ra*ra*m2*m2))
          {
            t = -m1/m2;
            norm = normalize(-1.0f*ba);
          }
        }
        
        // body
        float rr = ra;
        float hy = m0 + rr*rr;
        float k2 = m0*m0    - m2*m2*hy;
        float k1 = m0*m0*m3 - m1*m2*hy + m0*ra*(rr*m2);
        float k0 = m0*m0*m5 - m1*m1*hy + m0*ra*(rr*m1*2.0 - m0*ra);

        float D = k1*k1 - k2*k0;
        if(D > 0.f)
        {
          float dist = -(k1 + std::sqrt(D)) / k2;
          float y = m1 + dist * m2;
          if(y >= 0.f && y <= m0)
          {
            t = dist;
            norm = normalize(m0*(m0*(oa+t*ray_dir)+rr*ba*ra)-ba*hy*y);
          }
        }
      }
      if (header.prim_type == GRAPH_PRIM_BOX ||
          header.prim_type == GRAPH_PRIM_BOX_COLOR)
      {
        float3 pa = min(point2_3, point1_3);
        float3 pb = max(point2_3, point1_3);
        float minmax[3][2] = {{pa.x, pb.x}, {pa.y, pb.y}, {pa.z, pb.z}};

        const int4 vert_indices{0,3,5,6};
        for (int vert_ind = 0; vert_ind < 4; ++vert_ind)
        {
          int axis_indices[3] = { int((vert_indices[vert_ind] & 4u) != 0),
                                  int((vert_indices[vert_ind] & 2u) != 0),
                                  int((vert_indices[vert_ind] & 1u) != 0) };
          pa = float3(minmax[0][axis_indices[0]],
                      minmax[1][axis_indices[1]],
                      minmax[2][axis_indices[2]]);
          for (int axis = 0; axis < 3; ++axis)
          {
            pb = pa;
            pb[axis] = minmax[axis][1-axis_indices[axis]];
            float4 norm_dist = rayCapsuleIntersect(ray_pos, ray_dir, pa, pb, ra);
            if (norm_dist.w < t)
            {
              t = norm_dist.w;
              norm = to_float3(norm_dist);
            }
          }
        }
      }

      if (t > tNear && t < pHit->t)
      {
        float2 encoded_norm = encode_normal(norm);

        pHit->t         = t;
        pHit->primId    = primId;
        pHit->instId    = instId;
        pHit->geomId    = geomId | (TYPE_GRAPHICS_PRIM << SH_TYPE);
        pHit->coords[0] = color.x + color.y/256.0f;
        pHit->coords[1] = color.z;
        pHit->coords[2] = encoded_norm.x;
        pHit->coords[3] = encoded_norm.y;
      }
    }
  }
#endif // DISABLE_GRAPHICS_PRIM
}

SdfHit BVHRT::sdf_sphere_tracing(uint32_t type, uint32_t sdf_id, const float3 &min_pos, const float3 &max_pos,
                                 float tNear, const float3 &pos, const float3 &dir, bool need_norm)
{
  const float EPS = 1e-5;

  SdfHit hit;
  hit.hit_pos = float4(0,0,0,-1);
  hit.hit_norm = float4(1,0,0,0);
  float2 tNear_tFar = box_intersects(min_pos, max_pos, pos, dir);
  float t = std::max(tNear, tNear_tFar.x);
  float tFar = tNear_tFar.y;
  if (t > tFar)
    return hit;
  
  int iter = 0;
  float d = eval_distance_sdf(type, sdf_id, pos + t * dir);
  while (iter < 1000 && d > EPS && t < tFar)
  {
    t += d + EPS;
    d = eval_distance_sdf(type, sdf_id, pos + t * dir);
    iter++;
  }

  if (d > EPS)
    return hit;

  float3 p0 = pos + t * dir;
  float3 norm = float3(0,0,1);
  if (need_norm)
  {
    const float h = 0.001;
    float ddx = (eval_distance_sdf(type, sdf_id, p0 + float3(h, 0, 0)) -
                 eval_distance_sdf(type, sdf_id, p0 + float3(-h, 0, 0))) /
                (2 * h);
    float ddy = (eval_distance_sdf(type, sdf_id, p0 + float3(0, h, 0)) -
                 eval_distance_sdf(type, sdf_id, p0 + float3(0, -h, 0))) /
                (2 * h);
    float ddz = (eval_distance_sdf(type, sdf_id, p0 + float3(0, 0, h)) -
                 eval_distance_sdf(type, sdf_id, p0 + float3(0, 0, -h))) /
                (2 * h);

    norm = normalize(float3(ddx, ddy, ddz));
    // fprintf(stderr, "st %d (%f %f %f)\n", iter, surface_normal->x, surface_normal->y, surface_normal->z);
  }
  // fprintf(stderr, "st %d (%f %f %f)", iter, p0.x, p0.y, p0.z);
  hit.hit_pos = to_float4(p0, 1);
  hit.hit_norm = to_float4(norm, float(iter));
  return hit;
}

float BVHRT::eval_distance_sdf(uint32_t type, uint32_t sdf_id, float3 pos)
{
  float val = 1000;
  switch (type)
  {
#ifndef DISABLE_SDF_GRID
  case TYPE_SDF_GRID:
    val = eval_distance_sdf_grid(sdf_id, pos);
    break;
#endif
#ifndef DISABLE_SDF_SBS
  case TYPE_SDF_SBS:
    val = eval_distance_sdf_sbs(sdf_id, pos);
    break;
#endif
  default:
    break;
  }
  return val;
}

#ifndef DISABLE_SDF_GRID
float BVHRT::eval_distance_sdf_grid(uint32_t grid_id, float3 pos)
{
  uint32_t off = m_SdfGridOffsets[grid_id];
  uint3 size = m_SdfGridSizes[grid_id];

  //bbox for grid is a unit cube
  float3 grid_size_f = float3(size);
  float3 vox_f = grid_size_f*((pos-float3(-1,-1,-1))/float3(2,2,2)) - float3(0.5, 0.5, 0.5);
  vox_f = min(max(vox_f, float3(0.0f)), grid_size_f - float3(1e-5f));
  uint3 vox_u = uint3(vox_f);
  float3 dp = vox_f - float3(vox_u);

  //trilinear sampling
  float res = 0.0;
  if (m_preset.interpolation_mode == INTERPOLATION_MODE_TRILINEAR)
  {
    if (vox_u.x < size.x-1 && vox_u.y < size.y-1 && vox_u.z < size.z-1)
    {
      for (uint32_t i=0;i<2;i++)
      {
        for (uint32_t j=0;j<2;j++)
        {
          for (uint32_t k=0;k<2;k++)
          {
            float qx = (1 - dp.x + i*(2*dp.x-1));
            float qy = (1 - dp.y + j*(2*dp.y-1));
            float qz = (1 - dp.z + k*(2*dp.z-1));   
            res += qx*qy*qz*m_SdfGridData[off + (vox_u.z + k)*size.x*size.y + (vox_u.y + j)*size.x + (vox_u.x + i)];   
          }      
        }
      }
    }
    else
    {
      res += m_SdfGridData[off + (vox_u.z)*size.x*size.y + (vox_u.y)*size.x + (vox_u.x)]; 
    }
  } // tricubic interpolation
  else if (m_preset.interpolation_mode == INTERPOLATION_MODE_TRICUBIC)
  {
    if (vox_u.x < size.x-2 && vox_u.y < size.y-2 && vox_u.z < size.z-2 && vox_u.x > 0 && vox_u.y > 0 && vox_u.z > 0)
    {
      uint32_t x0 = vox_u.x - 1, y0 = vox_u.y - 1, z0 = vox_u.z - 1;
      float f_dp[3] = {dp.x, dp.y, dp.z}, grid_part[64];

      for (int x = 0; x < 4; ++x)
      {
        for (int y = 0; y < 4; ++y)
        {
          for (int z = 0; z < 4; ++z)
          {
            grid_part[z * 4 * 4 + y * 4 + x] = m_SdfGridData[off + (z0 + z)*size.x*size.y + (y0 + y)*size.x + (x0 + x)];
          }
        }
      }
      res = tricubicInterpolation(grid_part, f_dp);
    }
    else if (vox_u.x < size.x-1 && vox_u.y < size.y-1 && vox_u.z < size.z-1)
    {
      for (uint32_t i=0;i<2;i++)
      {
        for (uint32_t j=0;j<2;j++)
        {
          for (uint32_t k=0;k<2;k++)
          {
            float qx = (1 - dp.x + i*(2*dp.x-1));
            float qy = (1 - dp.y + j*(2*dp.y-1));
            float qz = (1 - dp.z + k*(2*dp.z-1));   
            res += qx*qy*qz*m_SdfGridData[off + (vox_u.z + k)*size.x*size.y + (vox_u.y + j)*size.x + (vox_u.x + i)];   
          }      
        }
      }
    }
    else
    {
      res += m_SdfGridData[off + (vox_u.z)*size.x*size.y + (vox_u.y)*size.x + (vox_u.x)]; 
    }
  }
  
  return res;
}
#endif

#ifndef DISABLE_SDF_SVS
float BVHRT::eval_distance_sdf_svs(uint32_t svs_id, float3 pos)
{
  uint32_t type = m_geomData[svs_id].type;
  // assert (type == TYPE_SDF_SBS); // || type == TYPE_SDF_SBS_COL || type == TYPE_SDF_SBS_TEX
  uint32_t leftNodeOffset = eval_distance_traverse_bvh(svs_id, pos);

  if (leftNodeOffset == 0xFFFFFFFF)
    return 11.f; // cannot be used for ST
  // printf("NodeOffset: %d\n", leftNodeOffset);

  uint32_t globalAABBId = startEnd[svs_id].x + EXTRACT_START(leftNodeOffset); // + aabbId
  uint32_t start_count_packed = m_primIdCount[globalAABBId];
  uint32_t a_start = EXTRACT_START(start_count_packed);
  uint32_t a_count = EXTRACT_COUNT(start_count_packed);

  float values[8];

  uint32_t nodeId, primId;
  float d, qNear, qFar;
  float2 fNearFar;
  float3 start_q;

  qNear = 1.0f;

  uint32_t sdfId =  m_geomData[svs_id].offset.x;
  primId = a_start;
  nodeId = primId + m_SdfSVSRoots[sdfId];

  float px = m_SdfSVSNodes[nodeId].pos_xy >> 16;
  float py = m_SdfSVSNodes[nodeId].pos_xy & 0x0000FFFF;
  float pz = m_SdfSVSNodes[nodeId].pos_z_lod_size >> 16;
  float sz = m_SdfSVSNodes[nodeId].pos_z_lod_size & 0x0000FFFF;
  float d_max = 2*1.73205081f/sz;

  float3 min_pos = float3(-1,-1,-1) + 2.0f*float3(px,py,pz)/sz;
  float3 max_pos = min_pos + 2.0f*float3(1,1,1)/sz;
  float3 size = max_pos - min_pos;

  float res_dist = 10.0f;
  if (pos.x >= min_pos.x && pos.x <= max_pos.x &&
      pos.y >= min_pos.y && pos.y <= max_pos.y &&
      pos.z >= min_pos.z && pos.z <= max_pos.z)
  {
    // hit

    for (int i=0;i<8;i++)
      values[i] = -d_max + 2*d_max*(1.0/255.0f)*((m_SdfSVSNodes[nodeId].values[i/4] >> (8*(i%4))) & 0xFF);

    d = std::max(size.x, std::max(size.y, size.z));
    start_q = (pos - min_pos) / d;

    res_dist = eval_dist_trilinear(values, start_q);
  }
  return res_dist;
}
#endif

#ifndef DISABLE_SDF_SBS
float BVHRT::eval_distance_sdf_sbs(uint32_t sbs_id, float3 pos)
{
  uint32_t type = m_geomData[sbs_id].type;
  // assert (type == TYPE_SDF_SBS); // || type == TYPE_SDF_SBS_COL || type == TYPE_SDF_SBS_TEX
  uint32_t leftNodeOffset = eval_distance_traverse_bvh(sbs_id, pos);

  if (leftNodeOffset == 0xFFFFFFFF)
    return 11.f; // cannot be used for ST
  // printf("NodeOffset: %d\n", leftNodeOffset);

  uint32_t globalAABBId = startEnd[sbs_id].x + EXTRACT_START(leftNodeOffset); // + aabbId
  uint32_t start_count_packed = m_primIdCount[globalAABBId];
  uint32_t a_start = EXTRACT_START(start_count_packed);
  uint32_t a_count = EXTRACT_COUNT(start_count_packed);

  #ifdef USE_TRICUBIC
  float values[64];
  #else
  float values[8];
  #endif

  uint32_t nodeId, primId;
  float d, qNear, qFar;
  float2 fNearFar;
  float3 start_q;

  qNear = 1.0f;

  uint32_t sdfId =  m_geomData[sbs_id].offset.x;
  primId = a_start; //id of bbox in BLAS
  nodeId = primId + m_SdfSBSRoots[sdfId];
  SdfSBSHeader header = m_SdfSBSHeaders[sdfId];
  uint32_t v_size = header.brick_size + 2*header.brick_pad + 1;

  float px = m_SdfSBSNodes[nodeId].pos_xy >> 16;
  float py = m_SdfSBSNodes[nodeId].pos_xy & 0x0000FFFF;
  float pz = m_SdfSBSNodes[nodeId].pos_z_lod_size >> 16;
  float sz = m_SdfSBSNodes[nodeId].pos_z_lod_size & 0x0000FFFF;
  float sz_inv = 2.0f/sz;
  
  d = 2.0f/(sz*header.brick_size);

  float3 brick_min_pos = float3(-1,-1,-1) + sz_inv*float3(px,py,pz);
  float3 brick_max_pos = brick_min_pos + sz_inv*float3(1,1,1);

  float res_dist = 10.0f;
  if (pos.x >= brick_min_pos.x && pos.x <= brick_max_pos.x &&
      pos.y >= brick_min_pos.y && pos.y <= brick_max_pos.y &&
      pos.z >= brick_min_pos.z && pos.z <= brick_max_pos.z)
  {
    // hit

    float3 local_pos = (pos - brick_min_pos) * (0.5f*sz*header.brick_size);
    float3 voxelPos = floor(clamp(local_pos, 1e-6f, header.brick_size-1e-6f));

    float3 min_pos = brick_min_pos + d*voxelPos;
    float3 max_pos = min_pos + d*float3(1,1,1);
    start_q = (pos - min_pos) * (0.5f*sz*header.brick_size);

    load_distance_values(nodeId, voxelPos, v_size, sz_inv, header, values);

#ifdef USE_TRICUBIC
    float point[3] = {start_q.x, start_q.y, start_q.z};
    res_dist = tricubicInterpolation(values, point);
#else
    res_dist = eval_dist_trilinear(values, start_q);
#endif
  }
  return res_dist;
}
#endif

#ifndef DISABLE_SDF_COCTREE_V3
float BVHRT::eval_distance_sdf_coctree_v3(uint32_t octree_id, float3 pos)
{
#if ON_CPU==1
  assert(COctreeV3::VERSION == 5); //if version is changed, this function should be revisited, as some changes may be needed
#endif
  uint32_t type = m_geomData[octree_id].type;
  // assert (type == TYPE_COCTREE_V3);
  uint32_t leftNodeOffset = eval_distance_traverse_bvh(octree_id, pos);

  if (leftNodeOffset == 0xFFFFFFFF)
    return 11.f; // cannot be used for ST

  uint32_t geometryId = octree_id;
  uint32_t globalAABBId = startEnd[geometryId].x + EXTRACT_START(leftNodeOffset);
  uint32_t start_count_packed = m_primIdCount[globalAABBId];
  uint32_t a_start = EXTRACT_START(start_count_packed);
  uint32_t a_count = EXTRACT_COUNT(start_count_packed);

  OTStackElement curr_node{};

  float3 min_pos = m_origNodes[a_start].boxMin;
  float3 max_pos = m_origNodes[a_start].boxMax;

  uint32_t start_sz = uint32_t(2.0f / (max_pos.x - min_pos.x));
  uint3 start_p_orig = uint3(float(start_sz)*0.5f*(min_pos - float3(-1,-1,-1)));

  uint3 p;
  float3 p_f;
  uint32_t level_sz;
  float d;
  float3 start_q;
  float values[8];

  curr_node.nodeId = m_origNodes[a_start].leftOffset;
  curr_node.info = 0;
  curr_node.p_size = uint2((start_p_orig.x << 16) | start_p_orig.y, (start_p_orig.z << 16) | start_sz);

  while (true)
  {
    level_sz = curr_node.p_size.y & 0xFFFF;
    p = uint3(curr_node.p_size.x >> 16, curr_node.p_size.x & 0xFFFF, curr_node.p_size.y >> 16);
    d = 1.0f/float(level_sz);
    p_f = float3(p);

    if((curr_node.info & COCTREE_LEAF_TYPE_MASK) != COCTREE_LEAF_TYPE_NOT_A_LEAF) //leaf node
    {
      float sz_inv = 2.0f/level_sz;

      float3 brick_min_pos = float3(-1,-1,-1) + sz_inv*p_f;
      float3 brick_max_pos = brick_min_pos + sz_inv*float3(1,1,1);

      float res_dist = 10.0f;
      if (pos.x >= brick_min_pos.x && pos.x <= brick_max_pos.x &&
          pos.y >= brick_min_pos.y && pos.y <= brick_max_pos.y &&
          pos.z >= brick_min_pos.z && pos.z <= brick_max_pos.z)
      {
        // hit
        COctreeV3Header header = coctree_v3_header;
        uint32_t v_size = header.brick_size + 2*header.brick_pad + 1;
        d = 2.0f/(level_sz*header.brick_size);

        float3 local_pos = (pos - brick_min_pos) * (0.5f*level_sz*header.brick_size);
        float3 voxelPos = floor(clamp(local_pos, 1e-6f, header.brick_size-1e-6f));

        float3 min_pos = brick_min_pos + d*voxelPos;
        float3 max_pos = min_pos + d*float3(1,1,1);
        start_q = (pos - min_pos) * (0.5f*level_sz*header.brick_size);

        uint32_t leafOffset = (curr_node.nodeId & coctree_v3_header.idx_mask) >> coctree_v3_header.idx_sh;
        uint32_t leafType = curr_node.info & COCTREE_LEAF_TYPE_MASK;
        uint32_t transformCode = curr_node.info >> COCTREE_LEAF_TYPE_BITS;
        float vmin = COctreeV3_LoadDistanceValues(leafType, leafOffset, voxelPos, v_size, sz_inv, header, transformCode, values);

        if (vmin <= 0.f)
          res_dist = eval_dist_trilinear(values, start_q);
        else
          res_dist = 13.f;
      }
      return res_dist;
    }
    else
    {
      float sz_inv = 2.0f/level_sz;

      float3 c0 = 0.5f * level_sz * (pos + 1.f) - p_f;
      int currNode = (uint32_t(c0.x >= 0.5f) << 2u) | (uint32_t(c0.y >= 0.5f) << 1u) | uint32_t(c0.z >= 0.5f);
      uint32_t childrenInfo = m_SdfCompactOctreeV3Data[curr_node.nodeId + 0];

      // if child is active
      if ((childrenInfo & (1u << currNode)) > 0)
      {
        uint32_t childPos = 1 + coctree_v3_header.uints_per_child_info*bitCount(childrenInfo & ((1u << currNode) - 1));
        uint32_t childOffset = m_SdfCompactOctreeV3Data[curr_node.nodeId + childPos];
        uint32_t childType = (childrenInfo >> (8 + COCTREE_LEAF_TYPE_BITS*currNode)) & COCTREE_LEAF_TYPE_MASK;
        uint32_t childInfoOffset = curr_node.nodeId + childPos + coctree_v3_header.trans_off;
        uint32_t childInfo = (m_SdfCompactOctreeV3Data[childInfoOffset] << COCTREE_LEAF_TYPE_BITS) | childType;
        curr_node.nodeId = childOffset;
        curr_node.info = childInfo;
        curr_node.p_size = (curr_node.p_size << 1) | uint2(((currNode & 4) << (16-2)) | ((currNode & 2) >> 1), (currNode & 1) << 16);
      }
      else
      {
        // missed coctree
        return 12.f;
      }
    }
  }
}
#endif

uint32_t BVHRT::eval_distance_traverse_bvh(uint32_t geomId, float3 pos)
{
  const uint32_t bvhOffset = m_geomData[geomId].bvhOffset;

  uint32_t stack[STACK_SIZE];
  int top = 0;
  uint32_t leftNodeOffset = 0;

  while (top >= 0)
  {
#ifndef DISABLE_RF_GRID
    // if (m_RFGridFlags.size() > 0 && pHit->coords[0] <= 0.01f)
    //   break;
#endif

    while (top >= 0 && ((leftNodeOffset & LEAF_BIT) == 0))
    {
      const BVHNodePair fatNode = m_allNodePairs[bvhOffset + leftNodeOffset];
      const uint32_t node0_leftOffset = fatNode.left.leftOffset;
      const uint32_t node1_leftOffset = fatNode.right.leftOffset;

      const bool hitChild0 = (pos.x >= fatNode.left.boxMin.x) && (pos.x <= fatNode.left.boxMax.x) &&
                             (pos.y >= fatNode.left.boxMin.y) && (pos.y <= fatNode.left.boxMax.y) &&
                             (pos.z >= fatNode.left.boxMin.z) && (pos.z <= fatNode.left.boxMax.z);
      const bool hitChild1 = (pos.x >= fatNode.right.boxMin.x) && (pos.x <= fatNode.right.boxMax.x) &&
                             (pos.y >= fatNode.right.boxMin.y) && (pos.y <= fatNode.right.boxMax.y) &&
                             (pos.z >= fatNode.right.boxMin.z) && (pos.z <= fatNode.right.boxMax.z);

      // traversal decision
      if (hitChild0 && hitChild1)
      {
        leftNodeOffset = node0_leftOffset;
        stack[top]     = node1_leftOffset;
        top++;
      }
      else if (!hitChild0 && !hitChild1) // both miss, stack.pop()
      {
        leftNodeOffset = -1;
        if (--top >= 0)
          leftNodeOffset = stack[top];
      }
      else
      {
        leftNodeOffset = hitChild0 ? node0_leftOffset : node1_leftOffset;
      }

    } // end while (searchingForLeaf)

    // leaf node, intersect triangles
    if (top >= 0 && leftNodeOffset != 0xFFFFFFFF)
    {
      return leftNodeOffset;
    }

    // continue BVH traversal
    leftNodeOffset = -1;
    if (--top >= 0)
      leftNodeOffset = stack[top];
  } // end while (top >= 0)
  return leftNodeOffset;
}

float tricubic_spline(float p0, float p1, float p2, float p3, float x)
{
  return p1 + 0.5 * x * (p2 - p0 + x * (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3 + x * (3.0 * (p1 - p2) + p3 - p0)));
}

float
BVHRT::tricubicInterpolation(const float grid[64], const float dp[3])
{
  float res = 0;
  float values_yz[16];
  float values_z[4];

  for (uint32_t j = 0; j < 4; ++j)
  {
    for (uint32_t k = 0; k < 4; ++k)
    {
      //m_SdfGridData[off + (vox_u.z)*size.x*size.y + (vox_u.y)*size.x + (vox_u.x)]
      values_yz[4*j + k] = 
        tricubic_spline(
            grid[k*4*4 + (j)*4 + (0)],
            grid[k*4*4 + (j)*4 + (1)],
            grid[k*4*4 + (j)*4 + (2)],
            grid[k*4*4 + (j)*4 + (3)], 
            dp[0]
      );
    }
  }

  for (uint32_t k = 0; k < 4; ++k)
  {
    values_z[k] = 
      tricubic_spline(
          values_yz[4*0 + k], 
          values_yz[4*1 + k], 
          values_yz[4*2 + k], 
          values_yz[4*3 + k], 
          dp[1]
    );
  }

  res = tricubic_spline(values_z[0], values_z[1], values_z[2], values_z[3], dp[2]);

  return res;
}

#ifdef USE_ENZYME
void __enzyme_autodiff(void*, ...);
int enzyme_dup;
int enzyme_dupnoneed;
int enzyme_out;
int enzyme_const;
#endif

void 
BVHRT::tricubicInterpolationDerrivative(const float grid[64], const float dp[3], float d_pos[3], float d_grid[64])
{
  #ifdef USE_ENZYME
  __enzyme_autodiff((void*)(tricubicInterpolation), 
                    enzyme_dup, grid, d_grid, 
                    enzyme_dup, dp, d_pos);

  #endif
}

#ifndef DISABLE_SDF_FRAME_OCTREE
float BVHRT::eval_distance_sdf_frame_octree(uint32_t octree_id, float3 position)
{
  float3 pos = clamp(0.5f*(position + 1.0f), 0.0f, 1.0f);
  uint32_t idx = m_SdfFrameOctreeRoots[octree_id];
  float d = 1;
  float3 p = float3(0,0,0);
  float3 dp = pos;
  while (m_SdfFrameOctreeNodes[idx].offset != 0)
  {
    uint32_t ch_index = 4*uint32_t(dp.x >= 0.5) + 2*uint32_t(dp.y >= 0.5) + uint32_t(dp.z >= 0.5);
    //printf("%u dp %f %f %f %u\n",idx, dp.x, dp.y, dp.z, ch_index);
    idx = m_SdfFrameOctreeNodes[idx].offset + ch_index;
    d = d/2;
    p = 2*p + float3((ch_index & 4) >> 2, (ch_index & 2) >> 1, ch_index & 1);
    dp = pos/d - p;
  }
  //printf("\n");
  //printf("%u last dp \n",idx);

  //bilinear sampling
  return (1-dp.x)*(1-dp.y)*(1-dp.z)*m_SdfFrameOctreeNodes[idx].values[0] + 
         (1-dp.x)*(1-dp.y)*(  dp.z)*m_SdfFrameOctreeNodes[idx].values[1] + 
         (1-dp.x)*(  dp.y)*(1-dp.z)*m_SdfFrameOctreeNodes[idx].values[2] + 
         (1-dp.x)*(  dp.y)*(  dp.z)*m_SdfFrameOctreeNodes[idx].values[3] + 
         (  dp.x)*(1-dp.y)*(1-dp.z)*m_SdfFrameOctreeNodes[idx].values[4] + 
         (  dp.x)*(1-dp.y)*(  dp.z)*m_SdfFrameOctreeNodes[idx].values[5] + 
         (  dp.x)*(  dp.y)*(1-dp.z)*m_SdfFrameOctreeNodes[idx].values[6] + 
         (  dp.x)*(  dp.y)*(  dp.z)*m_SdfFrameOctreeNodes[idx].values[7];
}
#endif

void BVHRT::IntersectAllTrianglesInLeaf(const float3 ray_pos, const float3 ray_dir,
                                        float tNear, uint32_t instId, uint32_t geomId,
                                        uint32_t a_start, uint32_t a_count,
                                        CRT_Hit *pHit)
{
#ifndef DISABLE_MESH
  const uint2 a_geomOffsets = m_geomData[geomId].offset;

  for (uint32_t triId = a_start; triId < a_start + a_count; triId++)
  {
    const uint32_t A = m_indices[a_geomOffsets.x + triId*3 + 0];
    const uint32_t B = m_indices[a_geomOffsets.x + triId*3 + 1];
    const uint32_t C = m_indices[a_geomOffsets.x + triId*3 + 2];

    const float3 A_pos = to_float3(m_vertPos[a_geomOffsets.y + A]);
    const float3 B_pos = to_float3(m_vertPos[a_geomOffsets.y + B]);
    const float3 C_pos = to_float3(m_vertPos[a_geomOffsets.y + C]);

    const float3 edge1 = B_pos - A_pos;
    const float3 edge2 = C_pos - A_pos;
    const float3 pvec = cross(ray_dir, edge2);
    const float3 tvec = ray_pos - A_pos;
    const float3 qvec = cross(tvec, edge1);

    const float invDet = 1.0f / dot(edge1, pvec);
    const float v = dot(tvec, pvec) * invDet;
    const float u = dot(qvec, ray_dir) * invDet;
    const float t = dot(edge2, qvec) * invDet;

    if (v >= -1e-6f && u >= -1e-6f && (u + v <= 1.0f + 1e-6f) && t > tNear && t < pHit->t) 
    {
      pHit->t = t;
      pHit->primId = triId;
      pHit->instId = instId;
      pHit->geomId = geomId | (TYPE_MESH_TRIANGLE << SH_TYPE);
      pHit->coords[0] = u;
      pHit->coords[1] = v;

      if (need_normal())
      {
        float3 n = float3(1,0,0);
        if (m_preset.normal_mode == NORMAL_MODE_GEOMETRY)
        {
          n = cross(edge1, edge2);
        }
        else if (m_preset.normal_mode == NORMAL_MODE_VERTEX)
        {
          n = to_float3(m_vertNorm[a_geomOffsets.y + A] * (1.0f - u - v) + m_vertNorm[a_geomOffsets.y + B] * v + u * m_vertNorm[a_geomOffsets.y + C]);
        }

        n = normalize(matmul4x3(m_instanceData[instId].transformInvTransposed, n));
        float2 encoded_norm = encode_normal(n);

        pHit->coords[2] = encoded_norm.x;
        pHit->coords[3] = encoded_norm.y;
      }
      else
      {
        pHit->coords[2] = 0;
        pHit->coords[3] = 0;
      }
    }
  }
#endif
}

static inline int first_node(float3 t0, float3 tm)
{
uint32_t answer = 0;   // initialize to 00000000
// select the entry plane and set bits
if(t0.x > t0.y){
    if(t0.x > t0.z){ // PLANE YZ
        if(t0.x > tm.y) answer|=2;    // set bit at position 1
        if(t0.x > tm.z) answer|=1;    // set bit at position 0
        return (int) answer;
    }
}
else {
    if(t0.y > t0.z){ // PLANE XZ
        if(tm.x < t0.y) answer|=4;    // set bit at position 2
        if(t0.y > tm.z) answer|=1;    // set bit at position 0
        return (int) answer;
    }
}
// PLANE XY
if(tm.x < t0.z) answer|=4;    // set bit at position 2
if(tm.y < t0.z) answer|=2;    // set bit at position 1
return (int) answer;
}

static inline int new_node(float txm, int x, float tym, int y, float tzm, int z)
{
  return (txm < tym) ? (txm < tzm ? x : z) : (tym < tzm ? y : z);
}

//Octree intersect for Compact Octrees V1 and V2
//We don't use BVh with theese octrees, so some of the parameters
//are not needed, but were included to match other *Intersect functions
void BVHRT::OctreeIntersect(uint32_t type, const float3 ray_pos, const float3 ray_dir,
                            float tNear, uint32_t instId, uint32_t geomId,
                            uint32_t a_start, uint32_t a_count,
                            CRT_Hit *pHit)
{
#ifndef DISABLE_SDF_COCTREE_V2
  float3 pos_ray_pos = ray_pos;
  float3 pos_ray_dir = ray_dir;
  //assume octree is box [-1,1]^3
  uint32_t a = 0;
  if (ray_dir.x < 0)
  {
    pos_ray_pos.x *= -1;
    pos_ray_dir.x *= -1;
    a |= 4;
  }

  if (ray_dir.y < 0)
  {
    pos_ray_pos.y *= -1;
    pos_ray_dir.y *= -1;
    a |= 2;
  }

  if (ray_dir.z < 0)
  {
    pos_ray_pos.z *= -1;
    pos_ray_dir.z *= -1;
    a |= 1;
  }

  pos_ray_pos *= -1;
  const float3 pos_ray_dir_inv = SafeInverse(pos_ray_dir);
  const float3 _t0 = pos_ray_pos * pos_ray_dir_inv - pos_ray_dir_inv;
  const float3 _t1 = pos_ray_pos * pos_ray_dir_inv + pos_ray_dir_inv;
  const float3 _l = _t1 - _t0;
  //printf("_t0 %f %f %f\n", _t0.x, _t0.y, _t0.z);
  //printf("_t1 %f %f %f\n", _t1.x, _t1.y, _t1.z);

  const uint3 nn_indices[8] = {uint3(4, 2, 1), uint3(5, 3, 8), uint3(6, 8, 3), uint3(7, 8, 8),
                               uint3(8, 6, 5), uint3(8, 7, 8), uint3(8, 8, 7), uint3(8, 8, 8)};

  OTStackElement stack[32];
  OTStackElement tmp_buf[4];

  int top = 0;
  int buf_top = 0;
  uint3 p;
  float3 p_f, t0, t1, tm;
  int currNode;
  uint32_t nodeId;
  uint32_t level_sz;
  float d, qFar;
  float2 fNearFar;
  float3 start_q;
  float values[8];
  float old_t = pHit->t;

  stack[top].nodeId = 0;
  stack[top].info = 0;
  stack[top].p_size = uint2(0,1);

    while (top >= 0)
    {
      level_sz = stack[top].p_size.y & 0xFFFF;
      p = uint3(stack[top].p_size.x >> 16, stack[top].p_size.x & 0xFFFF, stack[top].p_size.y >> 16);
      d = 1.0f/float(level_sz);
      p_f = float3(p);
      t0 = _t0 + d*p_f * _l;
      t1 = _t0 + d*(p_f + 1) * _l;

      //static int counter = 0;
      //if (counter < 100)
      //{
      //  printf("node %u, p = (%u %u %u) d = %f\n", stack[top].nodeId, p.x, p.y, p.z, d);
      //  counter++;
      //}

      if(stack[top].info > 0) //leaf node
      {
        float tmin = std::max(t0.x, std::max(t0.y, t0.z));
        float tmax = std::min(t1.x, std::min(t1.y, t1.z));

        {
          uint32_t p_mask = level_sz - 1;
          uint3 real_p = uint3(((a & 4) > 0) ? (~p.x & p_mask) : p.x, 
                               ((a & 2) > 0) ? (~p.y & p_mask) : p.y, 
                               ((a & 1) > 0) ? (~p.z & p_mask) : p.z);

          nodeId = stack[top].nodeId;
          float sz = 0.5f*float(level_sz);
          d = 1.0f/sz;
          float3 min_pos = float3(-1,-1,-1) + d*float3(real_p.x,real_p.y,real_p.z);

          fNearFar = float2(tmin, tmax);
          float3 start_pos = ray_pos + fNearFar.x*ray_dir;
          start_q = sz*(start_pos - min_pos);
          qFar = sz*(fNearFar.y - fNearFar.x);
        }

        float d_max = 2*1.73205081f/float(level_sz);
        for (int i=0;i<8;i++)
        {
          values[i] = -d_max + 2*d_max*(1.0/255.0f)*((m_SdfCompactOctreeV2Data[stack[top].nodeId + i/4] >> (8*(i%4))) & 0xFF);
        }
        //if (counter < 100)
        //{
        //  printf("values = %f %f %f %f %f %f %f %f\n", values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7]);
        //  counter++;
        //}
        
        LocalSurfaceIntersection(TYPE_SDF_FRAME_OCTREE, ray_dir, 0, 0, values, nodeId, nodeId, d, 0.0f, qFar, fNearFar, start_q, /*in */
                                 pHit); /*out*/

        if (pHit->t < old_t)
          top = -1;
        else
          top--;
      }
      else
      { 
        buf_top = 0;
        tm = 0.5f*(t0 + t1);

        currNode = first_node(t0, tm);
        do
        {
          //0-7 bits are child_is_active flags, next 8-15 bits are child_is_leaf flags
          uint32_t childrenInfo = m_SdfCompactOctreeV2Data[stack[top].nodeId + 1];
          uint32_t childNode = currNode ^ a; //child node number, from 0 to 8

          // if child is active
          if ((childrenInfo & (1u << childNode)) > 0)
          {
            uint32_t currChildOffset = m_bitcount[childrenInfo & ((1u << childNode) - 1)]; //number of child in the list of active children
            uint32_t baseChildrenOffset = m_SdfCompactOctreeV2Data[stack[top].nodeId];
            tmp_buf[buf_top].nodeId = baseChildrenOffset + 2u*currChildOffset;
            tmp_buf[buf_top].info = childrenInfo & (1u << (childNode + 8)); // > 0 is child is leaf
            tmp_buf[buf_top].p_size = (stack[top].p_size << 1) | uint2(((currNode & 4) << (16-2)) | ((currNode & 2) >> 1), (currNode & 1) << 16);
            buf_top++;
          }
          //return (txm < tym) ? (txm < tzm ? x : z) : (tym < tzm ? y : z);
          currNode = new_node(((currNode & 4) > 0) ? t1.x : tm.x, nn_indices[currNode].x,
                              ((currNode & 2) > 0) ? t1.y : tm.y, nn_indices[currNode].y,
                              ((currNode & 1) > 0) ? t1.z : tm.z, nn_indices[currNode].z);
        } while (currNode<8);

        for (int i = 0; i < buf_top; i++)
        {
          stack[top+i] = tmp_buf[buf_top-i-1];
        }
        top += buf_top - 1;
      }
    }
#endif
}

float BVHRT::COctreeV3_LoadDistanceValues(uint32_t leafType, uint32_t brickOffset, float3 voxelPos, uint32_t v_size, float sz_inv, 
                                          const COctreeV3Header &header, uint32_t transform_code,
                                          float values[8] /*out*/)
{
  switch (leafType)
  {
  case COCTREE_LEAF_TYPE_GRID:
    return COctreeV3_LoadDistanceValuesLeafGrid(brickOffset, voxelPos, v_size, sz_inv, header, transform_code, values);
  case COCTREE_LEAF_TYPE_BIT_PACK:
    return COctreeV3_LoadDistanceValuesLeafBitPack(brickOffset, voxelPos, v_size, sz_inv, header, transform_code, values);
  case COCTREE_LEAF_TYPE_SLICES:
    return COctreeV3_LoadDistanceValuesLeafSlices(brickOffset, voxelPos, v_size, sz_inv, header, transform_code, values);
  }
  return 1e6f;
}

float BVHRT::COctreeV3_LoadDistanceValuesLeafGrid(uint32_t brickOffset, float3 voxelPos, uint32_t v_size, float sz_inv, 
                                                  const COctreeV3Header &header, uint32_t transform_code,
                                                  float values[8] /*out*/)
{
#if ON_CPU==1
  assert(COctreeV3::VERSION == 5); //if version is changed, this function should be revisited, as some changes may be needed
#endif

  float vmin = 1e6f;
#ifndef DISABLE_SDF_COCTREE_V3
  uint32_t rotIdx = transform_code & header.rot_mask;
  const unsigned min_range_size_uints = 2;

  //<><><><min value and range><tex_coords><distances>
  unsigned off_3 = 0;                                   // min value and range
  unsigned off_4 = off_3 + min_range_size_uints;        // texture coordinates
  unsigned off_5 = off_4 + 8 * header.uv_size;          // distances

  uint32_t vals_per_int = 32 / header.bits_per_value;
  uint32_t bits = header.bits_per_value;
  uint32_t max_val = header.bits_per_value == 32 ? 0xFFFFFFFF : ((1 << bits) - 1);

  float add_transform = header.sim_compression > 0 ? 2*1.73205081f*sz_inv*(2*(float(transform_code & header.add_mask) / float(header.add_mask)) - 1) : 0.0f;

  float min_val = -float(m_SdfCompactOctreeV3Data[brickOffset + off_3 + 0]) / float(0xFFFFFFFFu) + add_transform;
  float range   =  (float(m_SdfCompactOctreeV3Data[brickOffset + off_3 + 1]) / float(0xFFFFFFFFu)) / max_val;

  uint32_t vId0 = uint32_t(dot(m_SdfCompactOctreeRotModifiers[rotIdx], 
                           int4(voxelPos.x + header.brick_pad, voxelPos.y + header.brick_pad, voxelPos.z + header.brick_pad, 1)));
  uint32_t vId, dist;

  vId = vId0;
  dist = ((m_SdfCompactOctreeV3Data[brickOffset + off_5 + vId / vals_per_int] >> (bits * (vId % vals_per_int))) & max_val);
  values[0] = min_val + range * dist;
  vmin = std::min(values[0], vmin);

  vId = vId0 + m_SdfCompactOctreeRotModifiers[rotIdx].z;
  dist = ((m_SdfCompactOctreeV3Data[brickOffset + off_5 + vId / vals_per_int] >> (bits * (vId % vals_per_int))) & max_val);
  values[1] = min_val + range * dist;
  vmin = std::min(values[1], vmin);

  vId = vId0 + m_SdfCompactOctreeRotModifiers[rotIdx].y;
  dist = ((m_SdfCompactOctreeV3Data[brickOffset + off_5 + vId / vals_per_int] >> (bits * (vId % vals_per_int))) & max_val);
  values[2] = min_val + range * dist;
  vmin = std::min(values[2], vmin);

  vId = vId0 + m_SdfCompactOctreeRotModifiers[rotIdx].y + m_SdfCompactOctreeRotModifiers[rotIdx].z;
  dist = ((m_SdfCompactOctreeV3Data[brickOffset + off_5 + vId / vals_per_int] >> (bits * (vId % vals_per_int))) & max_val);
  values[3] = min_val + range * dist;
  vmin = std::min(values[3], vmin);

  vId = vId0 + m_SdfCompactOctreeRotModifiers[rotIdx].x;
  dist = ((m_SdfCompactOctreeV3Data[brickOffset + off_5 + vId / vals_per_int] >> (bits * (vId % vals_per_int))) & max_val);
  values[4] = min_val + range * dist;
  vmin = std::min(values[4], vmin);

  vId = vId0 + m_SdfCompactOctreeRotModifiers[rotIdx].x + m_SdfCompactOctreeRotModifiers[rotIdx].z;
  dist = ((m_SdfCompactOctreeV3Data[brickOffset + off_5 + vId / vals_per_int] >> (bits * (vId % vals_per_int))) & max_val);
  values[5] = min_val + range * dist;
  vmin = std::min(values[5], vmin);

  vId = vId0 + m_SdfCompactOctreeRotModifiers[rotIdx].x + m_SdfCompactOctreeRotModifiers[rotIdx].y; 
  dist = ((m_SdfCompactOctreeV3Data[brickOffset + off_5 + vId / vals_per_int] >> (bits * (vId % vals_per_int))) & max_val);
  values[6] = min_val + range * dist;
  vmin = std::min(values[6], vmin);

  vId = vId0 + m_SdfCompactOctreeRotModifiers[rotIdx].x + m_SdfCompactOctreeRotModifiers[rotIdx].y + m_SdfCompactOctreeRotModifiers[rotIdx].z;
  dist = ((m_SdfCompactOctreeV3Data[brickOffset + off_5 + vId / vals_per_int] >> (bits * (vId % vals_per_int))) & max_val);
  values[7] = min_val + range * dist;
  vmin = std::min(values[7], vmin);

  if (std::abs(add_transform) > 0.001f)
  {
  //printf("%u] loaded values %f %f %f %f %f %f %f %f, add = %f\n", 
  //       brickOffset + off_5, values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7], add_transform);
  }

#endif
  return vmin;
}

float BVHRT::COctreeV3_LoadDistanceValuesLeafBitPack(uint32_t brickOffset, float3 voxelPos, uint32_t v_size, float sz_inv, 
                                                     const COctreeV3Header &header, uint32_t transform_code,
                                                     float values[8] /*out*/)
{
#if ON_CPU==1
  assert(COctreeV3::VERSION == 5); //if version is changed, this function should be revisited, as some changes may be needed
#endif

  float vmin = 1e6f;
#ifndef DISABLE_SDF_COCTREE_V3
  uint32_t rotIdx = transform_code & header.rot_mask;
  uint32_t p_size = header.brick_size + 2 * header.brick_pad;
  uint32_t PFlagPos = uint32_t(dot(m_SdfCompactOctreeRotModifiers[ROT_COUNT + rotIdx], 
                               int4(voxelPos.x + header.brick_pad, voxelPos.y + header.brick_pad, voxelPos.z + header.brick_pad, 1)));
  
  //early exit if voxel is not present
  if ((m_SdfCompactOctreeV3Data[brickOffset + PFlagPos/32] & (1u << (PFlagPos%32))) == 0)
    return 1e6f;
  
  //this voxel is guaranteed to have surface
  uint32_t distance_flags_size_uints = (v_size * v_size * v_size + 32 - 1) / 32;
  uint32_t presence_flags_size_uints = (p_size * p_size * p_size + 32 - 1) / 32;
  uint32_t      min_range_size_uints = 2;

  //<presence_flags><distance_flags><><min value and range><tex_coords><distances>
  unsigned off_0 = 0;                                   // presence flags
  unsigned off_1 = off_0 + presence_flags_size_uints;   // distance flags
  unsigned off_3 = off_1 + distance_flags_size_uints;   // min value and range
  unsigned off_4 = off_3 + min_range_size_uints;        // texture coordinates
  unsigned off_5 = off_4 + 8 * header.uv_size;          // distances

  uint32_t vals_per_int = 32 / header.bits_per_value;
  uint32_t bits = header.bits_per_value;
  uint32_t max_val = header.bits_per_value == 32 ? 0xFFFFFFFF : ((1 << bits) - 1);

  float add_transform = header.sim_compression > 0 ? 2*1.73205081f*sz_inv*(2*(float(transform_code & header.add_mask) / float(header.add_mask)) - 1) : 0.0f;

  float min_val = -float(m_SdfCompactOctreeV3Data[brickOffset + off_3 + 0]) / float(0xFFFFFFFFu) + add_transform;
  float range   =  (float(m_SdfCompactOctreeV3Data[brickOffset + off_3 + 1]) / float(0xFFFFFFFFu)) / max_val;

  for (int i = 0; i < 8; i++)
  {
    float3 vPosOrig = voxelPos + float3(header.brick_pad) + float3((i & 4) >> 2, (i & 2) >> 1, i & 1);
    uint32_t localId = uint32_t(dot(m_SdfCompactOctreeRotModifiers[rotIdx], int4(vPosOrig.x, vPosOrig.y, vPosOrig.z, 1)));

    uint32_t b0 = m_SdfCompactOctreeV3Data[brickOffset + off_1];
    b0 = localId > 31 ? b0 : b0 & ((1u << localId) - 1);
    b0 = bitCount(b0);

    uint32_t b1 = localId > 32 ? m_SdfCompactOctreeV3Data[brickOffset + off_1 + 1] & ((1u << (localId - 32)) - 1) : 0;
    b1 = bitCount(b1);

    uint32_t vId0 = b0 + b1;
    uint32_t dist0 = ((m_SdfCompactOctreeV3Data[brickOffset + off_5 + vId0 / vals_per_int] >> (bits * (vId0 % vals_per_int))) & max_val);
    values[i] = min_val + range * dist0;
  }
  return -1.0f;
#endif
  return vmin;
}

float BVHRT::COctreeV3_LoadDistanceValuesLeafSlices(uint32_t brickOffset, float3 voxelPos, uint32_t v_size, float sz_inv, 
                                                    const COctreeV3Header &header, uint32_t transform_code,
                                                    float values[8] /*out*/)
{
#if ON_CPU==1
  assert(COctreeV3::VERSION == 5); //if version is changed, this function should be revisited, as some changes may be needed
#endif

  float vmin = 1e6f;
#ifndef DISABLE_SDF_COCTREE_V3
  uint32_t rotIdx = transform_code & header.rot_mask;
  uint32_t p_size = header.brick_size + 2 * header.brick_pad;
  uint32_t PFlagPos = uint32_t(dot(m_SdfCompactOctreeRotModifiers[ROT_COUNT + rotIdx], 
                               int4(voxelPos.x + header.brick_pad, voxelPos.y + header.brick_pad, voxelPos.z + header.brick_pad, 1)));
  
  //early exit if voxel is not present
  if ((m_SdfCompactOctreeV3Data[brickOffset + PFlagPos/32] & (1u << (PFlagPos%32))) == 0)
    return 1e6f;
  
  //this voxel is guaranteed to have surface
  const uint32_t line_distances_offset_bits = 16;
  const uint32_t line_distances_offsets_per_uint = 32 / line_distances_offset_bits;

  uint32_t slice_distance_flags_bits = v_size * v_size;
  uint32_t slice_distance_flags_uints = (slice_distance_flags_bits + 32 - 1) / 32;
  uint32_t distance_flags_size_uints = v_size * slice_distance_flags_uints;
  uint32_t presence_flags_size_uints = (p_size * p_size * p_size + 32 - 1) / 32;
  uint32_t distance_offsets_size_uints = (v_size + line_distances_offsets_per_uint - 1) / line_distances_offsets_per_uint; // 16 bits for offset, should be enough for all cases

  const unsigned min_range_size_uints = 2;

  //<presence_flags><distance_flags><distance_offsets><min value and range><tex_coords><distances>
  unsigned off_0 = 0;                                   // presence flags
  unsigned off_1 = off_0 + presence_flags_size_uints;   // distance flags
  unsigned off_2 = off_1 + distance_flags_size_uints;   // distance offsets
  unsigned off_3 = off_2 + distance_offsets_size_uints; // min value and range
  unsigned off_4 = off_3 + min_range_size_uints;        // texture coordinates
  unsigned off_5 = off_4 + 8 * header.uv_size;          // distances

  uint32_t vals_per_int = 32 / header.bits_per_value;
  uint32_t bits = header.bits_per_value;
  uint32_t max_val = header.bits_per_value == 32 ? 0xFFFFFFFF : ((1 << bits) - 1);

  float add_transform = header.sim_compression > 0 ? 2*1.73205081f*sz_inv*(2*(float(transform_code & header.add_mask) / float(header.add_mask)) - 1) : 0.0f;

  float min_val = -float(m_SdfCompactOctreeV3Data[brickOffset + off_3 + 0]) / float(0xFFFFFFFFu) + add_transform;
  float range   =  (float(m_SdfCompactOctreeV3Data[brickOffset + off_3 + 1]) / float(0xFFFFFFFFu)) / max_val;

  for (int i = 0; i < 8; i++)
  {
    float3 vPosOrig = voxelPos + float3(header.brick_pad) + float3((i & 4) >> 2, (i & 2) >> 1, i & 1);
    uint32_t voxelId = uint32_t(dot(m_SdfCompactOctreeRotModifiers[rotIdx], int4(vPosOrig.x, vPosOrig.y, vPosOrig.z, 1)));
    uint32_t sliceId = voxelId / (v_size * v_size);
    uint32_t localId = voxelId % (v_size * v_size);

    uint32_t b0 = m_SdfCompactOctreeV3Data[brickOffset + off_1 + slice_distance_flags_uints * sliceId];
    b0 = localId > 31 ? b0 : b0 & ((1u << localId) - 1);
    b0 = bitCount(b0);

    uint32_t b1 = localId > 32 ? m_SdfCompactOctreeV3Data[brickOffset + off_1 + slice_distance_flags_uints * sliceId + 1] & ((1u << (localId - 32)) - 1)
                               : 0;
    b1 = bitCount(b1);

    uint32_t sliceOffset = (m_SdfCompactOctreeV3Data[brickOffset + off_2 + sliceId / line_distances_offsets_per_uint] >>
                            (line_distances_offset_bits * (sliceId % line_distances_offsets_per_uint))) &
                           (((1u << line_distances_offset_bits) - 1));
    uint32_t localOffset = b0 + b1;

    uint32_t vId0 = sliceOffset + localOffset;
    uint32_t dist0 = ((m_SdfCompactOctreeV3Data[brickOffset + off_5 + vId0 / vals_per_int] >> (bits * (vId0 % vals_per_int))) & max_val);
    values[i] = min_val + range * dist0;
  }
  return -1.0f;
#endif
  return vmin;
}

void BVHRT::COctreeV3_BrickIntersect(uint32_t type, const float3 ray_pos, const float3 ray_dir,
                                     float tNear, uint32_t instId, uint32_t geomId, const COctreeV3Header &header,
                                     uint32_t brickOffset, float3 p, float sz, uint32_t transform_code_leaf_type,
                                     CRT_Hit *pHit)
{

#if ON_CPU==1
  assert(COctreeV3::VERSION == 5); //if version is changed, this function should be revisited, as some changes may be needed
#endif

#ifndef DISABLE_SDF_COCTREE_V3
  float values[8];

  float d, qNear, qFar;
  float2 fNearFar;
  float3 start_q;

  qNear = 1.0f;

  //uint32_t sdfId =  m_geomData[geomId].offset.x;

  uint32_t v_size = header.brick_size + 2*header.brick_pad + 1;
  uint32_t leaf_type = transform_code_leaf_type & COCTREE_LEAF_TYPE_MASK;
  uint32_t transform_code = transform_code_leaf_type >> COCTREE_LEAF_TYPE_BITS;

  float sz_inv = 2.0f/sz;
  
  d = 2.0f/(sz*header.brick_size);

  float3 brick_min_pos = float3(-1,-1,-1) + sz_inv*p;
  float3 brick_max_pos = brick_min_pos + sz_inv*float3(1,1,1);
  float3 brick_size = brick_max_pos - brick_min_pos;

  float2 brick_fNearFar = RayBoxIntersection2(ray_pos, SafeInverse(ray_dir), brick_min_pos, brick_max_pos);
  brick_fNearFar.x = std::max(tNear, brick_fNearFar.x);
  float old_t = pHit->t;
  // if (ray_pos.x != 0.f || ray_pos.y != 0.f)
  //   printf("Pos: [%f, %f, %f]: brick_t: [%f, %f], t: %f\n", ray_pos.x, ray_pos.y, ray_pos.z, brick_fNearFar.x, brick_fNearFar.y, old_t);
  while (brick_fNearFar.x < brick_fNearFar.y && pHit->t == old_t)
  {
    float3 hit_pos = ray_pos + brick_fNearFar.x*ray_dir;
    float3 local_pos = (hit_pos - brick_min_pos) * (0.5f*sz*header.brick_size);
    float3 voxelPos = floor(clamp(local_pos, 1e-6f, header.brick_size-1e-6f));

    float3 min_pos = brick_min_pos + d*voxelPos;
    float3 max_pos = min_pos + d*float3(1,1,1);
    float3 size = max_pos - min_pos;

    float vmin = COctreeV3_LoadDistanceValues(leaf_type, brickOffset, voxelPos, v_size, sz_inv, header, transform_code, values);

    fNearFar = RayBoxIntersection2(ray_pos, SafeInverse(ray_dir), min_pos, max_pos);
    if (tNear < fNearFar.x && vmin <= 0.0f)    
    {
      float3 start_pos = ray_pos + fNearFar.x*ray_dir;
      start_q = (start_pos - min_pos) * (0.5f*sz*header.brick_size);
      qFar = (fNearFar.y - fNearFar.x) * (0.5f*sz*header.brick_size);
    
      LocalSurfaceIntersection(type, ray_dir, instId, geomId, values, brickOffset, brickOffset, d, 0.0f, qFar, fNearFar, start_q, /*in */
                               pHit); /*out*/
    
      if (m_preset.normal_mode == NORMAL_MODE_SDF_SMOOTHED && need_normal() && pHit->t < old_t)
      {
        const float beta = 0.5f;
        float3 dp = start_q + ((pHit->t - fNearFar.x)/d)*ray_dir; //linear interpolation coefficients in voxels

        float3 normals[8];
        float values_n[8];
        for (int i=0;i<8;i++)
          normals[i] = float3(0,0,0);
        
        float3 nmq = float3(0.5f,0.5f,0.5f);
        nmq.x = dp.x < 0.5f ? step(-beta, beta, dp.x) : step(-beta, beta, dp.x - 1.0f);
        nmq.y = dp.y < 0.5f ? step(-beta, beta, dp.y) : step(-beta, beta, dp.y - 1.0f);
        nmq.z = dp.z < 0.5f ? step(-beta, beta, dp.z) : step(-beta, beta, dp.z - 1.0f);

        int3 voxelPos0 = int3(dp.x < 0.5f ? voxelPos.x - 1 : voxelPos.x, dp.y < 0.5f ? voxelPos.y - 1 : voxelPos.y, dp.z < 0.5f ? voxelPos.z - 1 : voxelPos.z);
        for (int i=0;i<8;i++)
        {
          int3 VoxelPosI = voxelPos0 + int3((i & 4) >> 2, (i & 2) >> 1, i & 1);
          float3 dVoxelPos = float3(VoxelPosI) - voxelPos;

          float vmin_n = COctreeV3_LoadDistanceValues(leaf_type, brickOffset, float3(VoxelPosI), v_size, sz_inv, header, transform_code, values_n);
          if (vmin_n <= 0.0f)
            normals[i] = normalize(eval_dist_trilinear_diff(values_n, dp - dVoxelPos));
        }

        float3 smoothed_norm = (1-nmq.x)*(1-nmq.y)*(1-nmq.z)*normals[0] + 
                               (1-nmq.x)*(1-nmq.y)*(  nmq.z)*normals[1] + 
                               (1-nmq.x)*(  nmq.y)*(1-nmq.z)*normals[2] + 
                               (1-nmq.x)*(  nmq.y)*(  nmq.z)*normals[3] + 
                               (  nmq.x)*(1-nmq.y)*(1-nmq.z)*normals[4] + 
                               (  nmq.x)*(1-nmq.y)*(  nmq.z)*normals[5] + 
                               (  nmq.x)*(  nmq.y)*(1-nmq.z)*normals[6] + 
                               (  nmq.x)*(  nmq.y)*(  nmq.z)*normals[7];
        smoothed_norm = normalize(matmul4x3(m_instanceData[instId].transformInvTransposed, smoothed_norm));
        float2 encoded_norm = encode_normal(smoothed_norm);

        pHit->coords[2] = encoded_norm.x;
        pHit->coords[3] = encoded_norm.y;
      }
    }
    
    brick_fNearFar.x += std::max(0.0f, fNearFar.y-brick_fNearFar.x) + 1e-6f;
  }
  
  //ray hit a brick
  if (pHit->t < old_t && header.uv_size > 0)
  {
    uint32_t p_size = header.brick_size + 2 * header.brick_pad;
  
    //this voxel is guaranteed to have surface
    const uint32_t line_distances_offset_bits = 16;
    const uint32_t line_distances_offsets_per_uint = 32 / line_distances_offset_bits;

    uint32_t slice_distance_flags_bits = v_size * v_size;
    uint32_t slice_distance_flags_uints = (slice_distance_flags_bits + 32 - 1) / 32;
    uint32_t distance_flags_size_uints = v_size * slice_distance_flags_uints;
    uint32_t presence_flags_size_uints = (p_size * p_size * p_size + 32 - 1) / 32;
    uint32_t distance_offsets_size_uints = (v_size + line_distances_offsets_per_uint - 1) / line_distances_offsets_per_uint; // 16 bits for offset, should be enough for all cases

    const unsigned min_range_size_uints = 2;

    //<presence_flags><distance_flags><distance_offsets><min value and range><distances>
    unsigned off_0 = 0;                                   // presence flags
    unsigned off_1 = off_0 + presence_flags_size_uints;   // distance flags
    unsigned off_2 = off_1 + distance_flags_size_uints;   // distance offsets
    unsigned off_3 = off_2 + distance_offsets_size_uints; // min value and range
    unsigned off_4 = off_3 + min_range_size_uints;        // texture coordinates
    unsigned off_5 = off_4 + 8 * header.uv_size;          // distances

    float3 pos = ray_pos + pHit->t*ray_dir;
    float3 dp = (pos - brick_min_pos)*(0.5f*sz);

      pHit->coords[0] = (1-dp.x)*(1-dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfCompactOctreeV3Data[brickOffset + off_4 + 0] >> 16)) + 
                        (1-dp.x)*(1-dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfCompactOctreeV3Data[brickOffset + off_4 + 1] >> 16)) + 
                        (1-dp.x)*(  dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfCompactOctreeV3Data[brickOffset + off_4 + 2] >> 16)) + 
                        (1-dp.x)*(  dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfCompactOctreeV3Data[brickOffset + off_4 + 3] >> 16)) + 
                        (  dp.x)*(1-dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfCompactOctreeV3Data[brickOffset + off_4 + 4] >> 16)) + 
                        (  dp.x)*(1-dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfCompactOctreeV3Data[brickOffset + off_4 + 5] >> 16)) + 
                        (  dp.x)*(  dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfCompactOctreeV3Data[brickOffset + off_4 + 6] >> 16)) + 
                        (  dp.x)*(  dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfCompactOctreeV3Data[brickOffset + off_4 + 7] >> 16));

      pHit->coords[1] = (1-dp.x)*(1-dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfCompactOctreeV3Data[brickOffset + off_4 + 0] & 0xFFFF)) + 
                        (1-dp.x)*(1-dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfCompactOctreeV3Data[brickOffset + off_4 + 1] & 0xFFFF)) + 
                        (1-dp.x)*(  dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfCompactOctreeV3Data[brickOffset + off_4 + 2] & 0xFFFF)) + 
                        (1-dp.x)*(  dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfCompactOctreeV3Data[brickOffset + off_4 + 3] & 0xFFFF)) + 
                        (  dp.x)*(1-dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfCompactOctreeV3Data[brickOffset + off_4 + 4] & 0xFFFF)) + 
                        (  dp.x)*(1-dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfCompactOctreeV3Data[brickOffset + off_4 + 5] & 0xFFFF)) + 
                        (  dp.x)*(  dp.y)*(1-dp.z)*(1.5259022e-5f*float(m_SdfCompactOctreeV3Data[brickOffset + off_4 + 6] & 0xFFFF)) + 
                        (  dp.x)*(  dp.y)*(  dp.z)*(1.5259022e-5f*float(m_SdfCompactOctreeV3Data[brickOffset + off_4 + 7] & 0xFFFF));

  }
#endif
}

//Octree intersect for Compact Octrees V3
//This is used with BVH (so it is a two-layer hierarchy)
void BVHRT::OctreeIntersectV3(uint32_t type, const float3 ray_pos, const float3 ray_dir,
                              float tNear, uint32_t instId, uint32_t geomId,
                              uint32_t bvhNodeId, uint32_t a_count,
                              CRT_Hit *pHit)
{
#if ON_CPU==1
  assert(COctreeV3::VERSION == 5); //if version is changed, this function should be revisited, as some changes may be needed
#endif

#ifndef DISABLE_SDF_COCTREE_V3
  float3 pos_ray_pos = ray_pos;
  float3 pos_ray_dir = ray_dir;

  //assume octree is box [-1,1]^3
  uint32_t a = 0;
  if (ray_dir.x < 0)
  {
    pos_ray_pos.x *= -1;
    pos_ray_dir.x *= -1;
    a |= 4;
  }

  if (ray_dir.y < 0)
  {
    pos_ray_pos.y *= -1;
    pos_ray_dir.y *= -1;
    a |= 2;
  }

  if (ray_dir.z < 0)
  {
    pos_ray_pos.z *= -1;
    pos_ray_dir.z *= -1;
    a |= 1;
  }

  float3 min_pos = m_origNodes[bvhNodeId].boxMin;
  float3 max_pos = m_origNodes[bvhNodeId].boxMax;
  float3 center = 0.5f * (min_pos + max_pos);

  uint32_t start_sz = uint32_t(2.0f / (max_pos.x - min_pos.x));
  uint3 start_p_orig = uint3(float(start_sz)*0.5f*(min_pos - float3(-1,-1,-1)));
  uint32_t p_mask = start_sz - 1;
  uint3 start_p = uint3(((a & 4) > 0) ? (~start_p_orig.x & p_mask) : start_p_orig.x,
                        ((a & 2) > 0) ? (~start_p_orig.y & p_mask) : start_p_orig.y,
                        ((a & 1) > 0) ? (~start_p_orig.z & p_mask) : start_p_orig.z);

  uint32_t startNodeOffset = m_origNodes[bvhNodeId].leftOffset;

  pos_ray_pos *= -1;
  const float3 pos_ray_dir_inv = SafeInverse(pos_ray_dir);
  const float3 _t0 = pos_ray_pos * pos_ray_dir_inv - pos_ray_dir_inv;
  const float3 _t1 = pos_ray_pos * pos_ray_dir_inv + pos_ray_dir_inv;
  const float3 _l = _t1 - _t0;

  const uint3 nn_indices[8] = {uint3(4, 2, 1), uint3(5, 3, 8), uint3(6, 8, 3), uint3(7, 8, 8),
                               uint3(8, 6, 5), uint3(8, 7, 8), uint3(8, 8, 7), uint3(8, 8, 8)};

  OTStackElement stack[32];
  OTStackElement tmp_buf[4];

  int top = 0;
  int buf_top = 0;
  uint3 p;
  float3 p_f, t0, t1, tm;
  int currNode;
  uint32_t level_sz;
  float d;
  const float old_t = pHit->t;

  stack[top].nodeId = startNodeOffset;
  stack[top].info = 0;
  stack[top].p_size = uint2((start_p.x << 16) | start_p.y, (start_p.z << 16) | start_sz);

    while (top >= 0)
    {
      level_sz = stack[top].p_size.y & 0xFFFF;
      p = uint3(stack[top].p_size.x >> 16, stack[top].p_size.x & 0xFFFF, stack[top].p_size.y >> 16);
      d = 1.0f/float(level_sz);
      p_f = float3(p);
      t0 = _t0 + d*p_f * _l;
      t1 = _t0 + d*(p_f + 1) * _l;

      float t_out = std::min(std::min(t1.x, t1.y), t1.z);
      if (t_out < tNear)
      {
        top--;
        continue;
      }

      bool is_leaf = (stack[top].info & COCTREE_LEAF_TYPE_MASK) != COCTREE_LEAF_TYPE_NOT_A_LEAF;
      if (coctree_v3_header.lods > 0 && !is_leaf)
      {
        float t = m_preset.fixed_lod > 0 ? 1.0f : std::max(t0.x, std::max(t0.y, t0.z));
        float target_lod_size = (std::pow(2.0f, m_preset.level_of_detail) - 0.01f)/t;
        if (float(level_sz*coctree_v3_header.brick_size) > target_lod_size)
        {
          uint32_t lodLeafId = m_SdfCompactOctreeV3Data[stack[top].nodeId + 1];
          uint32_t type = (m_SdfCompactOctreeV3Data[stack[top].nodeId + 0] >> COCTREE_LOD_LEAF_TYPE_SHIFT) & COCTREE_LEAF_TYPE_MASK;
          uint32_t infoOffset = stack[top].nodeId + 1 + coctree_v3_header.trans_off;
          stack[top].nodeId = lodLeafId;
          stack[top].info = (m_SdfCompactOctreeV3Data[infoOffset] << COCTREE_LEAF_TYPE_BITS) | type;
          is_leaf = true;

          //we should stop on this LOD level, but this node is empty, so just skip it
          if (type == COCTREE_LEAF_TYPE_NOT_A_LEAF)
          {
            top--;
            continue; //
          }
        }
      }

      if (is_leaf)
      {
        uint32_t p_mask = level_sz - 1;
        uint3 real_p = uint3(((a & 4) > 0) ? (~p.x & p_mask) : p.x,
                             ((a & 2) > 0) ? (~p.y & p_mask) : p.y,
                             ((a & 1) > 0) ? (~p.z & p_mask) : p.z);

        uint32_t leafOffset = (stack[top].nodeId & coctree_v3_header.idx_mask) >> coctree_v3_header.idx_sh;
        COctreeV3_BrickIntersect(TYPE_SDF_FRAME_OCTREE, ray_pos, ray_dir, tNear, instId, geomId, coctree_v3_header, leafOffset, 
                                 float3(real_p), float(level_sz), stack[top].info, pHit);

        if (pHit->t < old_t)
        {
          top = -1;
          if (m_preset.render_mode == MULTI_RENDER_MODE_LOD)
            pHit->primId = uint32_t(std::log2(float(level_sz*coctree_v3_header.brick_size)));
        }
        else
          top--;
      }
      else
      { 
        buf_top = 0;
        tm = 0.5f*(t0 + t1);

        currNode = first_node(t0, tm);
        do
        {
          //0-7 bits are child_is_active flags, next 8-15 bits are child_is_leaf flags
          uint32_t childrenInfo = m_SdfCompactOctreeV3Data[stack[top].nodeId + 0];
          uint32_t childNode = currNode ^ a; //child node number, from 0 to 8

          // if child is active
          if ((childrenInfo & (1u << childNode)) > 0)
          {
            uint32_t localChildPos = coctree_v3_header.lods + bitCount(childrenInfo & ((1u << childNode) - 1));
            uint32_t childPos = 1 + coctree_v3_header.uints_per_child_info*localChildPos;
            uint32_t childOffset = m_SdfCompactOctreeV3Data[stack[top].nodeId + childPos];
            uint32_t childType = (childrenInfo >> (8 + COCTREE_LEAF_TYPE_BITS*childNode)) & COCTREE_LEAF_TYPE_MASK;
            uint32_t childInfoOffset = stack[top].nodeId + childPos + coctree_v3_header.trans_off;
            uint32_t childInfo = (m_SdfCompactOctreeV3Data[childInfoOffset] << COCTREE_LEAF_TYPE_BITS) | childType;

            tmp_buf[buf_top].nodeId = childOffset;
            tmp_buf[buf_top].info = childInfo;
            tmp_buf[buf_top].p_size = (stack[top].p_size << 1) | uint2(((currNode & 4) << (16-2)) | ((currNode & 2) >> 1), (currNode & 1) << 16);
            buf_top++;
          }
          currNode = new_node(((currNode & 4) > 0) ? t1.x : tm.x, nn_indices[currNode].x,
                              ((currNode & 2) > 0) ? t1.y : tm.y, nn_indices[currNode].y,
                              ((currNode & 1) > 0) ? t1.z : tm.z, nn_indices[currNode].z);
        } while (currNode<8);

        for (int i = 0; i < buf_top; i++)
        {
          stack[top+i] = tmp_buf[buf_top-i-1];
        }
        top += buf_top - 1;
      }
    }
#endif
}

static bool first_hit_is_closest(uint32_t tag)
{
  return tag != 1; /*TAG_TRIANGLE*/
}

void BVHRT::BVH2TraverseF32(const float3 ray_pos, const float3 ray_dir, float tNear,
                                uint32_t instId, uint32_t geomId, bool stopOnFirstHit,
                                CRT_Hit* pHit)
{
  const uint32_t bvhOffset = m_geomData[geomId].bvhOffset;

  uint32_t stack[STACK_SIZE];
  int top = 0;
  uint32_t leftNodeOffset = 0;
  bool hitFound = false; //set to true if we found a hit with an opaque object
                         //1) if stopOnFirstHit = true, it is some hit, not closest
                         //2) if we hit and object of some specific type (i.e. octree)
                         //   we can guarantee that the first hit will be the closest

  const float3 rayDirInv = SafeInverse(ray_dir);
  while (top >= 0 && !hitFound)
  {
#ifndef DISABLE_RF_GRID
    if (m_RFGridFlags.size() > 0 && pHit->coords[0] <= 0.01f)
      break;
#endif

    while (top >= 0 && ((leftNodeOffset & LEAF_BIT) == 0))
    {
      const BVHNodePair fatNode = m_allNodePairs[bvhOffset + leftNodeOffset];

      const uint32_t node0_leftOffset = fatNode.left.leftOffset;
      const uint32_t node1_leftOffset = fatNode.right.leftOffset;

      const float2 tm0 = RayBoxIntersection2(ray_pos, rayDirInv, fatNode.left.boxMin, fatNode.left.boxMax);
      const float2 tm1 = RayBoxIntersection2(ray_pos, rayDirInv, fatNode.right.boxMin, fatNode.right.boxMax);

#ifndef DISABLE_RF_GRID
      const bool hitChild0 = (tm0.x <= tm0.y) && (tm0.y >= tNear) && (tm0.x <= pHit->t || !stopOnFirstHit);
      const bool hitChild1 = (tm1.x <= tm1.y) && (tm1.y >= tNear) && (tm1.x <= pHit->t || !stopOnFirstHit);
#else
      const bool hitChild0 = (tm0.x <= tm0.y) && (tm0.y >= tNear) && (tm0.x <= pHit->t);
      const bool hitChild1 = (tm1.x <= tm1.y) && (tm1.y >= tNear) && (tm1.x <= pHit->t);
#endif
      // traversal decision
      leftNodeOffset = hitChild0 ? node0_leftOffset : node1_leftOffset;

      if (hitChild0 && hitChild1)
      {
        leftNodeOffset = (tm0.x <= tm1.x) ? node0_leftOffset : node1_leftOffset; // GPU style branch
        stack[top]     = (tm0.x <= tm1.x) ? node1_leftOffset : node0_leftOffset; // GPU style branch
        top++;
      }

      if (!hitChild0 && !hitChild1) // both miss, stack.pop()
      {
        top--;
        leftNodeOffset = stack[std::max(top,0)];
      }

    } // end while (searchingForLeaf)

    // leaf node, intersect triangles
    //
    if (top >= 0 && leftNodeOffset != 0xFFFFFFFF)
    {
      CRT_LeafInfo leafInfo;
      leafInfo.aabbId = EXTRACT_START(leftNodeOffset);
      leafInfo.instId = instId;

      const float SDF_BIAS = 0.01f;
      const float tNearSdf = std::max(tNear, SDF_BIAS);
  
      // if (debug_cur_pixel)
      //  printf("intersecting with leaf %u, inst %u\n", leafInfo.aabbId, leafInfo.instId);
      uint32_t hitTag = m_abstractObjectPtrs[geomId]->Intersect(to_float4(ray_pos, tNearSdf), to_float4(ray_dir, 1e9f), leafInfo, pHit, this);
      hitFound = (hitTag != 0 /*TAG_NONE*/) && (first_hit_is_closest(hitTag) || stopOnFirstHit);
    }

    // continue BVH traversal
    //
    top--;
    leftNodeOffset = stack[std::max(top,0)];

  } // end while (top >= 0)

}

CRT_Hit BVHRT::RayQuery_NearestHit(float4 posAndNear, float4 dirAndFar)
{
  bool stopOnFirstHit = (dirAndFar.w <= 0.0f);
  if(stopOnFirstHit)
    dirAndFar.w *= -1.0f;
  const float3 rayDirInv = SafeInverse(to_float3(dirAndFar));

  CRT_Hit hit;
  hit.t      = dirAndFar.w;
  hit.primId = uint32_t(-1);
  hit.instId = uint32_t(-1);
  hit.geomId = uint32_t(-1);
  hit.coords[0] = 1.0f;
  hit.coords[1] = 0.0f;
  hit.coords[2] = 0.0f;
  hit.coords[3] = 0.0f;

  {
    uint32_t nodeIdx = 0;
    do
    {
#ifndef DISABLE_RF_GRID
      if (m_RFGridFlags.size() > 0 && hit.coords[0] <= 0.01f)
        break;
#endif

      uint32_t travFlags  = 0;
      uint32_t leftOffset = 0;
      do
      {
        const BVHNode currNode = m_nodesTLAS[nodeIdx];
        const float2 boxHit    = RayBoxIntersection2(to_float3(posAndNear), rayDirInv, currNode.boxMin, currNode.boxMax);
        const bool intersects  = (boxHit.x <= boxHit.y) && (boxHit.y > posAndNear.w) && (boxHit.x < hit.t); // (tmin <= tmax) && (tmax > 0.f) && (tmin < curr_t)

        travFlags  = (currNode.leftOffset & LEAF_BIT) | uint32_t(intersects); // travFlags  = (((currNode.leftOffset & LEAF_BIT) == 0) ? 0 : LEAF_BIT) | (intersects ? 1 : 0);
        leftOffset = currNode.leftOffset;
        nodeIdx    = isLeafOrNotIntersect(travFlags) ? currNode.escapeIndex : leftOffset;

      } while (notLeafAndIntersect(travFlags) && nodeIdx != 0 && nodeIdx < 0xFFFFFFFE); 
      
      if(isLeafAndIntersect(travFlags)) 
      {
        const uint32_t instId = EXTRACT_START(leftOffset);
        const uint32_t geomId = m_instanceData[instId].geomId;
    
        // transform ray with matrix to local space
        //
        const float3 ray_pos = matmul4x3(m_instanceData[instId].transformInv, to_float3(posAndNear));
        const float3 ray_dir = matmul3x3(m_instanceData[instId].transformInv, to_float3(dirAndFar)); // DON'float NORMALIZE IT !!!! When we transform to local space of node, ray_dir must be unnormalized!!!
    
        // if (debug_cur_pixel)
        // {
        //   printf("intersect %u %u (stopOnFirstHit = %u)\n", instId, geomId, (unsigned)stopOnFirstHit);
        //   printf("ray_pos before = %f %f %f, after = %f %f %f\n", posAndNear.x, posAndNear.y, posAndNear.z, ray_pos.x, ray_pos.y, ray_pos.z);
        // }
        BVH2TraverseF32(ray_pos, ray_dir, posAndNear.w, instId, geomId, stopOnFirstHit, &hit);
      }
    } while (nodeIdx < 0xFFFFFFFE && !(stopOnFirstHit && hit.primId != uint32_t(-1))); //
  }

#ifndef DISABLE_MESH
  if(hit.geomId < uint32_t(-1) && ((hit.geomId >> SH_TYPE) == TYPE_MESH_TRIANGLE)) 
  {
    const uint2 geomOffsets = m_geomData[hit.geomId & GEOM_ID_MASK].offset;
    hit.primId = m_primIndices[geomOffsets.x/3 + hit.primId];
  }
#endif
  
  return hit;
}

bool BVHRT::RayQuery_AnyHit(float4 posAndNear, float4 dirAndFar)
{
  dirAndFar.w *= -1.0f;
  CRT_Hit hit = RayQuery_NearestHit(posAndNear, dirAndFar);
  return (hit.geomId != uint32_t(-1));
}
