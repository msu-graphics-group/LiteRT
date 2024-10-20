#include "utils.hpp"

using namespace LiteMath;

LiteMath::float3 get_center_of_mass(const RBezierGrid &surf)
{
  if (surf.grid.get_n() == 0)
    return {};
  float3 res = {};
  constexpr int COUNT = 255;
  for (int ui = 0; ui < COUNT; ++ui)
  for (int vi = 0; vi < COUNT; ++vi)
  {
    float u = ui * 1.0f/COUNT;
    float v = vi * 1.0f/COUNT;
    float4 point = surf.get_point(u, v);
    point /= point.w;
    res += to_float3(point);
  }
  return res / COUNT / COUNT;
}

float get_sphere_bound(const RBezierGrid &surface, const LiteMath::float3 &center) {
  if (surface.grid.get_n() == 0)
    return 0;
  float res = 0;
  constexpr int COUNT = 255;
  for (int ui = 0; ui < COUNT; ++ui)
  for (int vi = 0; vi < COUNT; ++vi)
  {
    float u = ui * 1.0f/COUNT;
    float v = vi * 1.0f/COUNT;
    float4 point = surface.get_point(u, v);
    point /= point.w;
    res = std::max(res, length(to_float3(point)-center));
  }
  return res;
}