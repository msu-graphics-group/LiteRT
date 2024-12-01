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
  Camera() = default;
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
  LiteMath::float3 pos;
  LiteMath::float3 normal;
  LiteMath::float2 uv;
};

using ShadeFuncType = LiteMath::float4( 
    const HitInfo &info,
    const LiteMath::float3 &camera_pos);

inline 
LiteMath::float4 shade_uv(
    const HitInfo &info,
    const LiteMath::float3 &camera_pos) {
  return LiteMath::float4{ info.uv.x, info.uv.y, 0.0f, 1.0f }; 
}

inline 
LiteMath::float4 shade_lambert(
    const HitInfo &info,
    const LiteMath::float3 &camera_pos) {
  LiteMath::float3 light_dir = { -1, -1, -1};
  light_dir = LiteMath::normalize(light_dir);

  float br = std::max(LiteMath::dot(-light_dir, info.normal), 0.0f);

  LiteMath::float3 color = br * LiteMath::float3{1.0f};
  color = LiteMath::clamp(color, 0.0f, 1.0f);

  return LiteMath::to_float4(color, 1.0f); 
}

inline 
LiteMath::float4 shade_normals(
    const HitInfo &info,
    const LiteMath::float3 &camera_pos) {
  auto normal_col = (info.normal+1.0f)/2.0f;
  return LiteMath::to_float4(normal_col, 1.0f);
}

extern int max_steps;
extern float EPS;
HitInfo trace_surface_newton(
    const LiteMath::float3 &pos,
    const LiteMath::float3 &ray,
    const RBezierGrid &surf,
    LiteMath::float2 uv);

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