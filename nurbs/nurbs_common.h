#pragma once

#include <LiteMath.h>
using LiteMath::float2, LiteMath::float3;

struct NURBSHeader
{
  int offset;
  int p, q;
  int uknots_cnt, vknots_cnt;
};

inline
int pts_offset(NURBSHeader h, int uspan, int vspan) { 
  return h.offset+(h.p + 1)*(h.q + 1) * 4 * ((h.vknots_cnt-1)*uspan + vspan); 
}
inline
int uknots_offset(NURBSHeader h) { 
  return h.offset+(h.p + 1)*(h.q + 1) * 4 * (h.uknots_cnt-1) * (h.vknots_cnt-1);
}
inline
int vknots_offset(NURBSHeader h) {
  return uknots_offset(h)+h.uknots_cnt;
}

struct NURBS_HitInfo
{
  bool hitten;
  float3 point;
  float3 normal;
  float2 uv;
};