#ifndef NURBS_SRC_RAYTRACER
#define NURBS_SRC_RAYTRACER

#include <optional>

#include <LiteMath.h>
#include <Image2d.h>
#include <functional>

#include "Surface.hpp"

struct Camera
{
public:
  Camera(
      float aspect, float fov,
      const LiteMath::float3 &position,
      const LiteMath::float3 &target,
      const LiteMath::float3 &up = { 0.0f, 1.0f, 0.0f })
      : position(position), target(target), up(up), 
          aspect(aspect), fov(fov) {
    auto forward = LiteMath::normalize(target-position);
    right = LiteMath::normalize(LiteMath::cross(forward, up));
    this->up = LiteMath::normalize(LiteMath::cross(right, forward));
  }
public:
  LiteMath::float3 position;
  LiteMath::float3 target;
  LiteMath::float3 up;
  LiteMath::float3 right;
  float aspect;
  float fov;
};

using ShadeFuncType = LiteMath::float4(
    const LiteMath::float3&,
    const LiteMath::float3&,
    const LiteMath::float3&,
    const LiteMath::float2&);

inline 
LiteMath::float4 shade_uv(
    const LiteMath::float3&,
    const LiteMath::float3&,
    const LiteMath::float3&,
    const LiteMath::float2& uv) {
  return LiteMath::float4{ uv.x, uv.y, 0.0f, 1.0f }; 
}

inline 
LiteMath::float4 shade_normals(
    const LiteMath::float3&,
    const LiteMath::float3&,
    const LiteMath::float3& normal,
    const LiteMath::float2&) {
  auto normal_col = (normal + 1.0f)/2.0f;
  return LiteMath::to_float4(normal_col, 1.0f);
}

void draw_newton(
    const RBezierGrid &surface,
    const Camera &camera,
    LiteImage::Image2D<uint32_t> &image,
    std::function<ShadeFuncType> shade_function = shade_uv);

void draw_points(
    const RBezierGrid &surface,
    const Camera &camera,
    LiteImage::Image2D<uint32_t> &image,
    int samples_per_parameter = 250,
    std::function<ShadeFuncType> shade_function = shade_uv);
#endif