#include "utils.hpp"

using namespace LiteMath;

LiteMath::float3 get_center_of_mass(const Surface &surface)
{
  if (surface.u_knots.size() < 2 || surface.v_knots.size() < 2)
    return {};

  constexpr int COUNT = 255;
  float3 res = {};
  for (int ui = 0; ui < COUNT; ++ui)
  for (int vi = 0; vi < COUNT; ++vi)
  {
    float u = surface.u_knots.back()*ui * 1.0f/COUNT;
    float v = surface.v_knots.back()*vi * 1.0f/COUNT;
    float4 point = surface.get_point(u, v);
    res += to_float3(point);
  }
  return res / COUNT / COUNT;
}

float get_sphere_bound(const Surface &surface, const LiteMath::float3 &center) {
  if (surface.u_knots.size() < 2 || surface.v_knots.size() < 2)
    return 0;
    
  constexpr int COUNT = 255;
  float res = 0;
  for (int ui = 0; ui < COUNT; ++ui)
  for (int vi = 0; vi < COUNT; ++vi)
  {
    float u = surface.u_knots.back()*ui * 1.0f/COUNT;
    float v = surface.v_knots.back()*vi * 1.0f/COUNT;
    float4 point = surface.get_point(u, v);
    res = std::max(res, length(to_float3(point)-center));
  }
  return res;
}