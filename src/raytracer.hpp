#ifndef NURBS_SRC_RAYTRACER
#define NURBS_SRC_RAYTRACER

#include <optional>

#include <LiteMath.h>
#include <Image2d.h>
#include <functional>

#include "Surface.hpp"

struct FrameBuffer 
{
  LiteImage::Image2D<uint32_t> col_buf;
  LiteImage::Image2D<float> z_buf;
};

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

struct HitInfo
{
  bool hitten = false;
  LiteMath::float4 pos;
  LiteMath::float3 uder;
  LiteMath::float3 vder;
  LiteMath::float2 uv;
};

using ShadeFuncType = LiteMath::float4(
    const RBezierGrid &grid, 
    const HitInfo &info,
    const LiteMath::float3 &camera_pos);

inline 
LiteMath::float4 shade_uv(
    const RBezierGrid &grid, 
    const HitInfo &info,
    const LiteMath::float3 &camera_pos) {
  return LiteMath::float4{ info.uv.x, info.uv.y, 0.0f, 1.0f }; 
}

inline 
LiteMath::float4 shade_normals(
    const RBezierGrid &grid, 
    const HitInfo &info,
    const LiteMath::float3 &camera_pos) {
  auto normal = LiteMath::normalize(LiteMath::cross(info.uder, info.vder));
  auto normal_col = (normal+1.0f)/2.0f;
  return LiteMath::to_float4(normal_col, 1.0f);
}

void draw_newton(
    const RBezierGrid &surface,
    const Camera &camera,
    FrameBuffer &fb,
    std::function<ShadeFuncType> shade_function = shade_uv);

void draw_points(
    const RBezierGrid &surface,
    const Camera &camera,
    FrameBuffer &fb,
    int samples_per_parameter = 250,
    std::function<ShadeFuncType> shade_function = shade_uv);

void draw_boxes(
    const std::vector<BoundingBox3d> &bboxes,
    const std::vector<LiteMath::float2> &uvs,
    const Camera &camera,
    FrameBuffer &fb);
#endif