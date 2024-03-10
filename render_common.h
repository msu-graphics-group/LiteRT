#pragma once
#include "LiteMath.h"

using namespace LiteMath;

static inline float3 EyeRayDirNormalized(float x, float y, float4x4 a_mViewProjInv)
{
  float4 pos = float4(2.0f*x - 1.0f, -2.0f*y + 1.0f, 0.0f, 1.0f );
  pos = a_mViewProjInv * pos;
  pos /= pos.w;
  return normalize(to_float3(pos));
}

static inline float3 mymul3x3(float4x4 m, float3 v)
{ 
  return to_float3(m*to_float4(v, 0.0f));
}

static inline float maxcomp(float3 v) { return std::max(v.x, std::max(v.y, v.z)); }

static inline float3 mymul4x3(float4x4 m, float3 v)
{
  return to_float3(m*to_float4(v, 1.0f));
}

static inline void transform_ray3f(float4x4 a_mWorldViewInv, 
                                   float3* ray_pos, float3* ray_dir) 
{
  float3 pos  = mymul4x3(a_mWorldViewInv, (*ray_pos));
  float3 pos2 = mymul4x3(a_mWorldViewInv, ((*ray_pos) + 100.0f*(*ray_dir)));

  float3 diff = pos2 - pos;

  (*ray_pos)  = pos;
  (*ray_dir)  = normalize(diff);
}

static inline void CoordinateSystem(float3 v1, float3* v2, float3* v3)
{
  float invLen = 1.0f;

  if (std::abs(v1.x) > std::abs(v1.y))
  {
    invLen = 1.0f / std::sqrt(v1.x*v1.x + v1.z*v1.z);
    (*v2)  = float3((-1.0f) * v1.z * invLen, 0.0f, v1.x * invLen);
  }
  else
  {
    invLen = 1.0f / sqrt(v1.y * v1.y + v1.z * v1.z);
    (*v2)  = float3(0.0f, v1.z * invLen, (-1.0f) * v1.y * invLen);
  }

  (*v3) = cross(v1, (*v2));
}

static inline float3 MapSampleToCosineDistribution(float r1, float r2, float3 direction, float3 hit_norm, float power)
{
  if(power >= 1e6f)
    return direction;

  const float sin_phi = sin(M_TWOPI * r1);
  const float cos_phi = cos(M_TWOPI * r1);

  //sincos(2.0f*r1*3.141592654f, &sin_phi, &cos_phi);

  const float cos_theta = pow(1.0f - r2, 1.0f / (power + 1.0f));
  const float sin_theta = sqrt(1.0f - cos_theta*cos_theta);

  float3 deviation;
  deviation.x = sin_theta*cos_phi;
  deviation.y = sin_theta*sin_phi;
  deviation.z = cos_theta;

  float3 ny = direction, nx, nz;
  CoordinateSystem(ny, &nx, &nz);

  {
    float3 temp = ny;
    ny = nz;
    nz = temp;
  }

  float3 res = nx*deviation.x + ny*deviation.y + nz*deviation.z;

  float invSign = dot(direction, hit_norm) > 0.0f ? 1.0f : -1.0f;

  if (invSign*dot(res, hit_norm) < 0.0f) // reflected ray is below surface #CHECK_THIS
  {
    res = (-1.0f)*nx*deviation.x + ny*deviation.y - nz*deviation.z;
    //belowSurface = true;
  }

  return res;
}

constexpr float GEPSILON = 2e-5f ;
constexpr float DEPSILON = 1e-20f;
static inline float epsilonOfPos(float3 hitPos) { return std::max(std::max(std::abs(hitPos.x), std::max(std::abs(hitPos.y), std::abs(hitPos.z))), 2.0f*GEPSILON)*GEPSILON; }

/**
\brief offset reflected ray position by epsilon;
\param  a_hitPos      - world space position on surface
\param  a_surfaceNorm - surface normal at a_hitPos
\param  a_sampleDir   - ray direction in which we are going to trace reflected ray
\return offseted ray position
*/
static inline float3 OffsRayPos(const float3 a_hitPos, const float3 a_surfaceNorm, const float3 a_sampleDir)
{
  const float signOfNormal2 = dot(a_sampleDir, a_surfaceNorm) < 0.0f ? -1.0f : 1.0f;
  const float offsetEps     = epsilonOfPos(a_hitPos);
  return a_hitPos + signOfNormal2 * offsetEps * a_surfaceNorm;
}

static inline unsigned int encodeNormal(float3 n)
{
  const int x = (int)(n.x*32767.0f);
  const int y = (int)(n.y*32767.0f);

  const unsigned int sign = (n.z >= 0) ? 0 : 1;
  const unsigned int sx   = ((unsigned int)(x & 0xfffe) | sign);
  const unsigned int sy   = ((unsigned int)(y & 0xffff) << 16);

  return (sx | sy);
}

static inline float3 decodeNormal(unsigned int a_data)
{  
  const unsigned int a_enc_x = (a_data  & 0x0000FFFF);
  const unsigned int a_enc_y = ((a_data & 0xFFFF0000) >> 16);
  const float sign           = ((a_enc_x & 0x0001) != 0) ? -1.0f : 1.0f;

  const float x = ((short)(a_enc_x & 0xfffe))*(1.0f / 32767.0f);
  const float y = ((short)(a_enc_y & 0xffff))*(1.0f / 32767.0f);
  const float z = sign*std::sqrt(std::max(1.0f - x*x - y*y, 0.0f));

  return float3(x, y, z);
}


static inline uint RealColorToUint32(float4 real_color)
{
  float  r = real_color[0]*255.0f;
  float  g = real_color[1]*255.0f;
  float  b = real_color[2]*255.0f;
  float  a = real_color[3]*255.0f;

  uint32_t red   = uint32_t(r);
  uint32_t green = uint32_t(g);
  uint32_t blue  = uint32_t(b);
  uint32_t alpha = uint32_t(a);

  return red | (green << 8) | (blue << 16) | (alpha << 24);
}

static inline float2 mulRows2x4(const float4 row0, const float4 row1, float2 v)
{
  float2 res;
  res.x = row0.x*v.x + row0.y*v.y + row0.w;
  res.y = row1.x*v.x + row1.y*v.y + row1.w;
  return res;
}

static inline float clamp1f(float u, float a, float b) { return std::min(std::max(a, u), b); }

