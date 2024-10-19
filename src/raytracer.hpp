#ifndef NURBS_SRC_RAYTRACER
#define NURBS_SRC_RAYTRACER

#include <optional>

#include <LiteMath.h>
#include <Image2d.h>

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

void draw_newton(
    const Surface &surface,
    const Camera &camera,
    LiteImage::Image2D<uint32_t> &image, 
    float col[4]);

void draw_newton(
    const RBezierGrid &surface,
    const Camera &camera,
    LiteImage::Image2D<uint32_t> &image, 
    float col[4]);

void draw_bezier(
    const Surface &surface,
    const Camera &camera,
    LiteImage::Image2D<uint32_t> &image, 
    float col[4]);

void draw_points(
    const Surface &surface,
    const Camera &camera,
    LiteImage::Image2D<uint32_t> &image, 
    float col[4]);

void draw_points(
    const RBezierGrid &surface,
    const Camera &camera,
    LiteImage::Image2D<uint32_t> &image, 
    float col[4]);
#endif