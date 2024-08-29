#include <random>
#include <cmath>
#include <iostream>
#include <cassert>

#include "raytracer.hpp"


constexpr float EPS = 1e-2f;
constexpr int max_steps = 15;

using namespace LiteMath;
using namespace LiteImage;

static float2
mul2x2x2(float m[2][2], const float2 &v)
{
  return { v[0] * m[0][0] + v[1] * m[0][1], v[0] * m[1][0] + v[1] * m[1][1] };
}

float clamp_parameter(float val) {
  if (val < 0)
    return -val;
  if (val > 1)
    return 1-val;
  return val;
}

std::optional<float3> trace_surface_newton(
    const float3 &pos,
    const float3 &ray,
    const Surface &surf) {
  float3 ortho_dir1 = (ray.x || ray.z) ? float3{ 0, 1, 0 } : float3{ 1, 0, 0 };
  float3 ortho_dir2 = normalize(cross(ortho_dir1, ray));
  ortho_dir1 = normalize(cross(ray, ortho_dir2));

  float4 P1 = to_float4(ortho_dir1, -dot(ortho_dir1, pos));
  float4 P2 = to_float4(ortho_dir2, -dot(ortho_dir2, pos));
  
  float2 x = { static_cast<float>(rand())/RAND_MAX, static_cast<float>(rand())/RAND_MAX };
  float2 D = { dot(P1, surf.get_point(x.x, x.y)), dot(P2, surf.get_point(x.x, x.y)) };
  
  int steps_left = max_steps-1;
  while(length(D) > EPS && steps_left--) {
    float J[2][2] = 
    { 
      { dot(P1, surf.uderivative(x.x, x.y)), dot(P1, surf.vderivative(x.x, x.y)) },
      { dot(P2, surf.uderivative(x.x, x.y)), dot(P2, surf.vderivative(x.x, x.y)) }
    };


    float det = J[0][0]*J[1][1] - J[0][1] * J[1][0];

    float J_inversed[2][2] = 
    {
      { J[1][1]/det, -J[0][1]/det },
      { -J[1][0]/det, J[0][0]/det }
    };

    x = x - mul2x2x2(J_inversed, D);
    x.x = clamp_parameter(x.x);
    x.y = clamp_parameter(x.y);

    float2 new_D = { dot(P1, surf.get_point(x.x, x.y)), dot(P2, surf.get_point(x.x, x.y)) };
    
    if (length(new_D) > length(D))
      return {};
  }

  if (length(D) > EPS)
    return {};
  
  return to_float3(surf.get_point(x.x, x.y));
}

void draw(
    const Surface &surface,
    const Camera &camera,
    Image2D<uint32_t> &image, 
    float col[4]) {
  image.clear(LiteMath::uchar4{ 153, 153, 153, 255 }.u32);
  // float3 forward = normalize(camera.target - camera.position);
  // for (uint32_t y = 0; y < image.height(); ++y)
  // for (uint32_t x = 0; x < image.width();  ++x)
  // {
  //   float2 norm_crds = { 2*(x+0.5f)/image.width() - 1.0f, 2*(image.height()-y+0.5f)/image.height() - 1.0f };
  //   float3 ray = normalize(forward 
  //              + std::tan(camera.fov/2)*camera.up*norm_crds.y
  //              + std::tan(camera.fov/2)*camera.right*norm_crds.x*camera.aspect);
  //   float3 pos = camera.position;
  //   auto intersect_point = trace_surface_newton(pos, ray, surface);
  //   if (intersect_point.has_value()) {
  //     image[uint2{ x, y }] = uchar4{ 
  //       static_cast<u_char>(col[0]*255.0f),
  //       static_cast<u_char>(col[1]*255.0f),
  //       static_cast<u_char>(col[2]*255.0f),
  //       static_cast<u_char>(col[3]*255.0f) }.u32;
  //   }
  // }
  if (surface.u_knots.size() < 2 || surface.v_knots.size() < 2)
    return;
    
  float4x4 view = lookAt(camera.position, camera.target, camera.up);
  float4x4 proj = perspectiveMatrix(45, camera.aspect, 0.01f, 150.0f);
  int count = 250;
  for (int ui = 0; ui < count; ++ui)
  for (int vi = 0; vi < count; ++vi)
  {
    float u = surface.u_knots.back()*ui * 1.0f/count;
    float v = surface.v_knots.back()*vi * 1.0f/count;
    float4 point = surface.get_point(u, v);
    point = proj * view * point;
    float w = point.w;
    if (!(-w <= point.x && point.x <= w) ||
        !(-w <= point.y && point.y <= w) ||
        !(-w <= point.z && point.z <= w)) {
      continue;
    }

    point /= point.w;
    uint32_t x = clamp(static_cast<uint32_t>((point.x+1.0f)/2 * image.width()), 0u, image.width()-1);
    uint32_t y = static_cast<uint32_t>((point.y+1.0f)/2 * image.height());
    y = clamp(image.height() - y, 0u, image.height()-1);

    image[uint2{x, y}] = uchar4{ 
        static_cast<u_char>(col[0]*255.0f),
        static_cast<u_char>(col[1]*255.0f),
        static_cast<u_char>(col[2]*255.0f),
        static_cast<u_char>(col[3]*255.0f) }.u32;
  }
}