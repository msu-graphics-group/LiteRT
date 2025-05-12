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