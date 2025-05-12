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
