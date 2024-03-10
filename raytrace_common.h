#pragma once
#include <cmath>
#include <cstdint>
#include "LiteMath.h"

using LiteMath::cross;
using LiteMath::dot;
using LiteMath::float2;
using LiteMath::float3;
using LiteMath::float4;
using LiteMath::float4x4;
using LiteMath::int2;
using LiteMath::inverse4x4;
using LiteMath::normalize;
using LiteMath::sign;
using LiteMath::to_float3;
using LiteMath::uint2;

struct BoxHit
{
  uint32_t id;
  float tHit;
};

static inline BoxHit make_BoxHit(uint32_t a_id, float a_t)
{
  BoxHit res;
  res.id   = a_id;
  res.tHit = a_t;
  return res;
}

static constexpr int LBVH_MAXHITS = 32; 
static constexpr int STACK_SIZE   = 80; 

// NOINTERVALS format
//
static constexpr uint32_t START_MASK = 0x00FFFFFF;
static constexpr uint32_t END_MASK   = 0xFF000000;
static constexpr uint32_t SIZE_MASK  = 0x7F000000;
static constexpr uint32_t LEAF_BIT   = 0x80000000;
static constexpr uint32_t EMPTY_NODE = 0x7fffffff;

static inline bool isLeafAndIntersect  (uint32_t flags) { return (flags == (LEAF_BIT | 0x1 )); }
static inline bool notLeafAndIntersect (uint32_t flags) { return (flags != (LEAF_BIT | 0x1)); }
static inline bool isLeafOrNotIntersect(uint32_t flags) { return (flags & LEAF_BIT) !=0 || (flags & 0x1) == 0; }

static inline uint32_t PackOffsetAndSize(uint32_t start, uint32_t size)
{
  return LEAF_BIT | ((size << 24) & SIZE_MASK) | (start & START_MASK);
}

static inline uint32_t EXTRACT_OFFSET(uint32_t a_leftOffset) { return  a_leftOffset & 0x7fffffff; }
static inline uint32_t EXTRACT_START(uint32_t a_leftOffset)  { return  a_leftOffset & START_MASK; }
static inline uint32_t EXTRACT_COUNT(uint32_t a_leftOffset)  { return (a_leftOffset & SIZE_MASK) >> 24; }

static inline float3 SafeInverse(float3 d)
{
  const float ooeps = 1.0e-36f; // Avoid div by zero.
  float3 res;
  res.x = 1.0f / (std::abs(d.x) > ooeps ? d.x : std::copysign(ooeps, d.x));
  res.y = 1.0f / (std::abs(d.y) > ooeps ? d.y : std::copysign(ooeps, d.y));
  res.z = 1.0f / (std::abs(d.z) > ooeps ? d.z : std::copysign(ooeps, d.z));
  return res;
}

static inline float2 RayBoxIntersection2(float3 rayOrigin, float3 rayDirInv, float3 boxMin, float3 boxMax)
{
  const float lo  = rayDirInv.x * (boxMin.x - rayOrigin.x);
  const float hi  = rayDirInv.x * (boxMax.x - rayOrigin.x);
  const float lo1 = rayDirInv.y * (boxMin.y - rayOrigin.y);
  const float hi1 = rayDirInv.y * (boxMax.y - rayOrigin.y);
  const float lo2 = rayDirInv.z * (boxMin.z - rayOrigin.z);
  const float hi2 = rayDirInv.z * (boxMax.z - rayOrigin.z);

  const float tmin = std::max(std::min(lo, hi), std::min(lo1, hi1));
  const float tmax = std::min(std::max(lo, hi), std::max(lo1, hi1));

  return float2(std::max(tmin, std::min(lo2, hi2)), std::min(tmax, std::max(lo2, hi2)));
}

static inline float3 matmul3x3(float4x4 m, float3 v)
{ 
  return to_float3(m*to_float4(v, 0.0f));
}

static inline float3 matmul4x3(float4x4 m, float3 v)
{
  return to_float3(m*to_float4(v, 1.0f));
}

