#include "integrator_pt.h"
#include "include/crandom.h"

#include <chrono>
#include <string>

void PlaneHammersley(float *result, int n)
{
  for (int k = 0; k<n; k++)
  {
    float u = 0;
    int kk = k;

    for (float p = 0.5f; kk; p *= 0.5f, kk >>= 1)
      if (kk & 1)                           // kk mod 2 == 1
        u += p;

    float v = (k + 0.5f) / n;

    result[2 * k + 0] = u;
    result[2 * k + 1] = v;
  }
}

void Integrator::InitDataForGbuffer()
{
  m_qmcHammersley.resize(GBUFFER_SAMPLES);                 // TODO: move to class constructor sms like that
  PlaneHammersley(&m_qmcHammersley[0].x, GBUFFER_SAMPLES); //
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static inline float projectedPixelSize(float dist, float FOV, float w, float h)
{
  float ppx = (FOV / w)*dist;
  float ppy = (FOV / h)*dist;

  if (dist > 0.0f)
    return 2.0f*std::max(ppx, ppy);
  else
    return 1000.0f;
}

static inline float surfaceSimilarity(float4 data1, float4 data2, const float MADXDIFF)
{
  const float MANXDIFF = 0.15f;

  float3 n1 = to_float3(data1);
  float3 n2 = to_float3(data2);

  float dist = length(n1 - n2);
  if (dist >= MANXDIFF)
    return 0.0f;

  float d1 = data1.w;
  float d2 = data2.w;

  if (std::abs(d1 - d2) >= MADXDIFF)
    return 0.0f;

  float normalSimilar = std::sqrt(1.0f - (dist / MANXDIFF));
  float depthSimilar  = std::sqrt(1.0f - std::abs(d1 - d2) / MADXDIFF);

  return normalSimilar * depthSimilar;
}

static inline float gbuffDiff(const Integrator::GBufferPixel& s1, const Integrator::GBufferPixel& s2, const float a_fov, float w, float h)
{
  const float ppSize         = projectedPixelSize(s1.depth, a_fov, w, h);
  const float surfaceSimilar = surfaceSimilarity(float4(s1.norm[0], s1.norm[1], s1.norm[2], s1.depth),
                                                 float4(s2.norm[0], s2.norm[1], s2.norm[2], s2.depth), ppSize*2.0f);

  const float surfaceDiff    = 1.0f - surfaceSimilar;
  const float objDiff        = (s1.instId == s2.instId && s1.objId == s2.objId) ? 0.0f : 1.0f;
  const float matDiff        = (s1.matId  == s2.matId) ? 0.0f : 1.0f;
  const float alphaDiff      = std::abs(s1.rgba[3] - s2.rgba[3]);

  return surfaceDiff + objDiff + matDiff + alphaDiff;
}

static inline float gbuffDiffObj(const Integrator::GBufferPixel& s1, const Integrator::GBufferPixel& s2, const float a_fov, int w, int h)
{
  const float objDiff = (s1.instId == s2.instId && s1.objId == s2.objId) ? 0.0f : 1.0f;
  const float matDiff = (s1.matId  == s2.matId) ? 0.0f : 1.0f;
  return objDiff + matDiff;
}

void Integrator::kernel_InitEyeRayGB(uint tidX, uint tidY, const uint* packedXY, float4* rayPosAndNear, float4* rayDirAndFar) // TODO: refactor and insert here more sophisticated cam sampling
{
  const uint XY = m_packedXY[tidX];
  const uint x  = (XY & 0x0000FFFF);
  const uint y  = (XY & 0xFFFF0000) >> 16;

  float2 pixelOffsets = m_qmcHammersley[tidY];

  const float xCoordNormalized = (float(x + m_winStartX) + pixelOffsets.x)/float(m_fbWidth);
  const float yCoordNormalized = (float(y + m_winStartY) + pixelOffsets.y)/float(m_fbHeight);

  float3 rayDir = EyeRayDirNormalized(xCoordNormalized, yCoordNormalized, m_projInv);
  float3 rayPos = float3(0,0,0);

  transform_ray3f(m_worldViewInv, &rayPos, &rayDir);

  *rayPosAndNear = to_float4(rayPos, 0.0f);
  *rayDirAndFar  = to_float4(rayDir, FLT_MAX);
}

void Integrator::kernel_GetRayGBuff(uint tidX, uint tidY, const Lite_Hit* pHit, const float2* bars, GBufferPixel* out_gbuffer)
{
  if(tidX >= m_maxThreadId)
    return;

  const Lite_Hit hit = *pHit;
  if(hit.geomId == -1)
  {
    out_gbuffer[tidY].depth   = 0.0f;
    out_gbuffer[tidY].norm[0] = 0.0f;
    out_gbuffer[tidY].norm[1] = 0.0f;
    out_gbuffer[tidY].norm[2] = 1.0f;
  
    out_gbuffer[tidY].texc[0] = 0.0f;
    out_gbuffer[tidY].texc[1] = 0.0f;
  
    out_gbuffer[tidY].rgba[0] = 0.0f;
    out_gbuffer[tidY].rgba[1] = 0.0f;
    out_gbuffer[tidY].rgba[2] = 0.0f;
    out_gbuffer[tidY].rgba[3] = 0.0f;
    
    out_gbuffer[tidY].objId  = uint(-1);
    out_gbuffer[tidY].instId = uint(-1);
    out_gbuffer[tidY].matId  = uint(-1);
  
    out_gbuffer[tidY].coverage = 0.0f;
    out_gbuffer[tidY].shadow   = 0.0f; // not implemented
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
  float2 hitTexCoord = float2(data1.w, data2.w);
  float3 hitNorm     = to_float3(data1);

  // transform surface point with matrix 
  //
  {
    hitNorm = mul3x3(m_normMatrices[hit.instId], hitNorm);
    hitNorm = normalize(hitNorm);
  }

  const uint   texId     = m_materials[matId].texid[0];
  const float2 texCoordT = mulRows2x4(m_materials[matId].row0[0], m_materials[matId].row1[0], hitTexCoord);
  const float4 texColor  = m_textures[texId]->sample(texCoordT);

  float3 color = to_float3(mdata*texColor);
  if(m_materials[matId].mtype == MAT_TYPE_LIGHT_SOURCE)
    color = float3(0,0,0);

  out_gbuffer[tidY].depth   = hit.t;
  out_gbuffer[tidY].norm[0] = hitNorm[0];
  out_gbuffer[tidY].norm[1] = hitNorm[1];
  out_gbuffer[tidY].norm[2] = hitNorm[2];

  out_gbuffer[tidY].texc[0] = texCoordT[0];
  out_gbuffer[tidY].texc[1] = texCoordT[1];

  out_gbuffer[tidY].rgba[0] = color[0];
  out_gbuffer[tidY].rgba[1] = color[1];
  out_gbuffer[tidY].rgba[2] = color[2];
  out_gbuffer[tidY].rgba[3] = color[3];
  
  out_gbuffer[tidY].objId  = hit.geomId;
  out_gbuffer[tidY].instId = hit.instId;
  out_gbuffer[tidY].matId  = matId;

  out_gbuffer[tidY].coverage = 1.0f;
  out_gbuffer[tidY].shadow   = 0.0f; // not implemented
}

void Integrator::EvalGBuffer(uint blockId, uint localId, GBufferPixel* out_gbuffer)
{
  float4 rayPosAndNear, rayDirAndFar;
  kernel_InitEyeRayGB(blockId, localId, m_packedXY.data(), &rayPosAndNear, &rayDirAndFar);

  Lite_Hit hit; 
  float2   baricentrics; 
  kernel_RayTrace(blockId, &rayPosAndNear, &rayDirAndFar, &hit, &baricentrics); // blockId or blockId*256 + localId ?
  
  kernel_GetRayGBuff(blockId, localId, &hit, &baricentrics, out_gbuffer);
}

void Integrator::GBufferReduction(uint blockId, uint blockSize, GBufferPixel* samples, GBufferPixel* out_gbuffer)
{
  float minDiff   = 100000000.0f; 
  int   minDiffId = 0;

  const float fw = float(m_winWidth);
  const float fh = float(m_winHeight);
  float4 summColor = float4(0.0f);

  for (int i = 0; i < blockSize; i++)
  {
    float diff     = 0.0f;
    float coverage = 0.0f;
    for (int j = 0; j < blockSize; j++)
    {
      const float thisDiff = gbuffDiff(samples[i], samples[j], DEG_TO_RAD*90.0f, fw, fh);
      diff += thisDiff;
      if (thisDiff < 1.0f)
        coverage += 1.0f;
    }
  
    coverage *= (1.0f / (float)blockSize);
    samples[i].coverage = coverage;
    summColor  += float4(samples[i].rgba[0], samples[i].rgba[1], samples[i].rgba[2],  samples[i].rgba[3]);
  
    if (diff < minDiff)
    {
      minDiff   = diff;
      minDiffId = i;
    }
  }
  
  const float4 avgColor = summColor*(1.0f / (float)blockSize);

  const uint XY = m_packedXY[blockId];
  const uint x  = (XY & 0x0000FFFF);
  const uint y  = (XY & 0xFFFF0000) >> 16;
  
  const uint offset   = y*m_winWidth + x;
  out_gbuffer[offset] = samples[minDiffId];
  out_gbuffer[offset].rgba[0] = avgColor[0];
  out_gbuffer[offset].rgba[1] = avgColor[1];
  out_gbuffer[offset].rgba[2] = avgColor[2];
  out_gbuffer[offset].rgba[3] = avgColor[3];
}

void Integrator::kernelBE1D_EvalGBuffer(uint blockNum, GBufferPixel* out_gbuffer)
{
  #ifndef _DEBUG
  #pragma omp parallel for default(shared)
  #endif
  for(int blockId = 0; blockId < int(blockNum); blockId++) 
  {
    GBufferPixel blockData[GBUFFER_SAMPLES]; 
    
    for(int localId = 0; localId < GBUFFER_SAMPLES; localId++)
      EvalGBuffer(blockId, localId, blockData);

    GBufferReduction(blockId, GBUFFER_SAMPLES, blockData, out_gbuffer);
  }
}

void Integrator::EvalGBuffer(uint blockNum, GBufferPixel* out_gbuffer)
{
  kernelBE1D_EvalGBuffer(blockNum, out_gbuffer);
}
