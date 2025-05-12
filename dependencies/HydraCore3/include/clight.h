#pragma once

#include "cglobals.h"

static constexpr uint LIGHT_GEOM_RECT   = 1; 
static constexpr uint LIGHT_GEOM_DISC   = 2;
static constexpr uint LIGHT_GEOM_SPHERE = 3;
static constexpr uint LIGHT_GEOM_DIRECT = 4;
static constexpr uint LIGHT_GEOM_POINT  = 5;
static constexpr uint LIGHT_GEOM_ENV    = 6;

static constexpr uint LIGHT_DIST_LAMBERT = 0;
static constexpr uint LIGHT_DIST_OMNI    = 1;
static constexpr uint LIGHT_DIST_SPOT    = 2;

static constexpr uint LIGHT_FLAG_POINT_AREA = 1;
static constexpr uint LIGHT_FLAG_PROJECTIVE = 2;

struct LightSource
{
  float4x4 matrix;         ///<! translation in matrix is always (0,0,0,1)
  float4x4 iesMatrix;      ///<! translation in matrix is always (0,0,0,1), except projective light when it is used for light matrix ('mWorldLightProj')

  float4   samplerRow0;    ///<! texture sampler, row0
  float4   samplerRow1;    ///<! texture sampler, row1
  float4   samplerRow0Inv; ///<! texture sampler, inverse matrix, row0
  float4   samplerRow1Inv; ///<! texture sampler, inverse matrix, row1
  
  float4   pos;            ///<! translation aclually stored here
  float4   intensity;      ///<! brightress, i.e. screen value if light is visable directly
  float4   norm;           ///<! light direction

  float2   size;
  float    pdfA;
  uint     geomType;  ///<! LIGHT_GEOM_RECT, LIGHT_GEOM_DISC, LIGHT_GEOM_SPHERE, ...
  
  uint     distType;  ///<! LIGHT_DIST_LAMBERT, LIGHT_DIST_OMNI, ...
  uint     flags;     ///<! 
  uint     pdfTableOffset;
  uint     pdfTableSize;

  uint     specId;
  uint     texId;
  uint     iesId;
  float    mult;

  uint     pdfTableSizeX;
  uint     pdfTableSizeY;
  uint     camBackTexId;
  float    lightCos1;
  
  float    lightCos2;
  uint     matId;
  float    dummy2;
  float    dummy3;
};

struct LightSample
{
  float3 pos;
  float3 norm;
  float  pdf;
  bool   isOmni;
  bool   hasIES;
};

static inline LightSample areaLightSampleRev(const LightSource* a_pLight, float2 rands)
{
  float2 sampleOff = 2.0f * (float2(-0.5f,-0.5f) + rands) * a_pLight[0].size;  // PLEASE! use 'a_pLight[0].' for a while ... , not a_pLight-> and not *(a_pLight[0])
  if(a_pLight[0].geomType == LIGHT_GEOM_DISC)
  {
    const float offsetX = rands.x * 2.0f - 1.0f;
    const float offsetY = rands.y * 2.0f - 1.0f;
    sampleOff = MapSamplesToDisc(float2(offsetX, offsetY))*a_pLight[0].size.x; 
  }
  const float3 samplePos = mul3x3(a_pLight[0].matrix, float3(sampleOff.x, 0.0f, sampleOff.y)) + to_float3(a_pLight[0].pos) + epsilonOfPos(to_float3(a_pLight[0].pos)) * to_float3(a_pLight[0].norm);
  LightSample res;
  res.pos    = samplePos;
  res.norm   = to_float3(a_pLight[0].norm);
  res.isOmni = false;
  res.hasIES = (a_pLight[0].iesId != uint(-1));
  res.pdf    = 1.0f; // evaluated later 
  return res;
}

static inline LightSample sphereLightSampleRev(const LightSource* a_pLight, float2 rands)
{
  const float theta = 2.0f * M_PI * rands.x;
  const float phi   = std::acos(1.0f - 2.0f * rands.y);
  const float x     = std::sin(phi) * std::cos(theta);
  const float y     = std::sin(phi) * std::sin(theta);
  const float z     = std::cos(phi);
  const float3 lcenter   = to_float3(a_pLight[0].pos);
  const float  lradius   = a_pLight[0].size.x;
  const float3 samplePos = lcenter + (lradius*1.000001f)*make_float3(x, y, z);
  LightSample res;
  res.pos  = samplePos;
  res.norm = normalize(samplePos - lcenter);
  res.isOmni = false;
  res.hasIES = (a_pLight[0].iesId != uint(-1));
  res.pdf    = 1.0f; // evaluated later 
  return res;
}

static inline LightSample directLightSampleRev(const LightSource* a_pLight, float2 rands, float3 illuminationPoint)
{
  const float3 norm = to_float3(a_pLight[0].norm);
  LightSample res;
  res.pos    = illuminationPoint - norm*100000.0f;
  res.norm   = norm;
  res.isOmni = false;
  res.hasIES = false;
  res.pdf    = 1.0f; // evaluated later 
  return res;
}

static inline LightSample pointLightSampleRev(const LightSource* a_pLight)
{
  LightSample res;
  res.pos    = to_float3(a_pLight[0].pos);
  res.norm   = to_float3(a_pLight[0].norm);
  res.isOmni = (a_pLight[0].distType == LIGHT_DIST_OMNI);
  res.hasIES = (a_pLight[0].iesId != uint(-1));
  res.pdf    = 1.0f; // evaluated later 
  return res;
}

/**
\brief  Select index proportional to piecewise constant function that is stored in a_accum[0 .. N-2]; Binary search version.
\param  a_r     - input random variable in rage [0, 1]
\param  a_accum - input float array. it must be a result of prefix summ - i.e. it must be sorted.
\param  N       - size of extended array - i.e. a_accum[N-1] == summ(a_accum[0 .. N-2]).
\param  pPDF    - out parameter. probability of picking up found value.
\return found index

*/
static int SelectIndexPropToOpt(const float a_r, __global const float* a_accum, const int N, 
                                float* pPDF) 
{
  int leftBound  = 0;
  int rightBound = N - 2; // because a_accum[N-1] == summ(a_accum[0 .. N-2]).
  int counter    = 0;
  int currPos    = -1;

  const int maxStep = 50;
  const float x = a_r*a_accum[N - 1];

  while (rightBound - leftBound > 1 && counter < maxStep)
  {
    const int currSize = rightBound + leftBound;
    const int currPos1 = (currSize % 2 == 0) ? (currSize + 1) / 2 : (currSize + 0) / 2;

    const float a = a_accum[currPos1 + 0];
    const float b = a_accum[currPos1 + 1];

    if (a < x && x <= b)
    {
      currPos = currPos1;
      break;
    }
    else if (x <= a)
      rightBound = currPos1;
    else if (x > b)
      leftBound = currPos1;

    counter++;
  }

  if (currPos < 0) // check the rest intervals
  {
    const float a1 = a_accum[leftBound + 0];
    const float b1 = a_accum[leftBound + 1];
    const float a2 = a_accum[rightBound + 0];
    const float b2 = a_accum[rightBound + 1];
    if (a1 < x && x <= b1)
      currPos = leftBound;
    if (a2 < x && x <= b2)
      currPos = rightBound;
  }

  if (x == 0.0f)
    currPos = 0;
  else if (currPos < 0)
    currPos = (rightBound + leftBound + 1) / 2;

  (*pPDF) = (a_accum[currPos + 1] - a_accum[currPos]) / a_accum[N - 1];
  return currPos;
}

static inline float evalMap2DPdf(float2 texCoordT, const float* intervals, const int sizeX, const int sizeY)
{  
  const float fw = (float)sizeX;
  const float fh = (float)sizeY;
  
  //texCoordT.x = WrapVal(texCoordT.x);
  //texCoordT.y = WrapVal(texCoordT.y);

  if (texCoordT.x < 0.0f || texCoordT.x > 1.0f) texCoordT.x -= (float)((int)(texCoordT.x));
  if (texCoordT.y < 0.0f || texCoordT.x > 1.0f) texCoordT.y -= (float)((int)(texCoordT.y));

  int pixelX = (int)(fw*texCoordT.x - 0.5f);
  int pixelY = (int)(fh*texCoordT.y - 0.5f);

  if (pixelX >= sizeX) pixelX = sizeX - 1;
  if (pixelY >= sizeY) pixelY = sizeY - 1;

  if (pixelX < 0) pixelX += sizeX;
  if (pixelY < 0) pixelY += sizeY;

  const int pixelOffset = pixelY*sizeX + pixelX;
  const int maxSize     = sizeX*sizeY;
  const int offset0     = (pixelOffset + 0 < maxSize+0) ? pixelOffset + 0 : maxSize - 1;
  const int offset1     = (pixelOffset + 1 < maxSize+1) ? pixelOffset + 1 : maxSize;

  const float2 interval = make_float2(intervals[offset0], intervals[offset1]);
  
  return (interval.y - interval.x)*(fw*fh)/intervals[sizeX*sizeY];
}

static inline float mylocalsmoothstep(float edge0, float edge1, float x)
{
  float  tVal = (x - edge0) / (edge1 - edge0);
  float  t    = std::min(std::max(tVal, 0.0f), 1.0f); 
  return t * t * (3.0f - 2.0f * t);
}