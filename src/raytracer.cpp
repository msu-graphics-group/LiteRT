#include <random>
#include <cmath>
#include <iostream>
#include <cassert>
#include <limits>
#include <omp.h>
#include <vector>

#include "raytracer.hpp"


constexpr float EPS = 1e-3f;
constexpr int max_steps = 16;

using namespace LiteMath;
using namespace LiteImage;

static float2
mul2x2x2(float2 m[2], const float2 &v)
{
  return m[0]*v[0]+m[1]*v[1];
}

float closed_clamp(float val) {
  val -= static_cast<int>(val);
  if (val < 0.0f)
    return 1.0f+val;
  if (val > 1.0f)
    return val-1.0f;
  return val;
}

float2 first_approx(
    const float4 &P1,
    const float4 &P2,
    const Surface &surf) {
  int mx = static_cast<int>(std::sqrt(max_steps));
  float2 D = { std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
  float2 best_uv;
  for (int ui = 0; ui <= mx; ++ui)
  for (int vi = 0; vi <= mx; ++vi)
  {
    float u = 1.0f/mx*ui;
    float v = 1.0f/mx*vi;
    float2 new_D = { dot(P1, surf.get_point(u, v)), dot(P2, surf.get_point(u, v)) };
    if (length(new_D) < length(D)) {
      D = new_D;
      best_uv = { u, v };
    }
  }
  return best_uv;
}

std::optional<float3> trace_surface_newton(
    const float3 &pos,
    const float3 &ray,
    const Surface &surf) {
  bool u_closed = surf.u_closed();
  bool v_closed = surf.v_closed();

  float3 ortho_dir1 = (ray.x || ray.z) ? float3{ 0, 1, 0 } : float3{ 1, 0, 0 };
  float3 ortho_dir2 = normalize(cross(ortho_dir1, ray));
  ortho_dir1 = normalize(cross(ray, ortho_dir2));
  assert(dot(ortho_dir1, ortho_dir2) < 1e-2f);
  assert(dot(ortho_dir1, ray) < 1e-2f);
  assert(dot(ortho_dir2, ray) < 1e-2f);

  float4 P1 = to_float4(ortho_dir1, -dot(ortho_dir1, pos));
  float4 P2 = to_float4(ortho_dir2, -dot(ortho_dir2, pos));
  assert(dot(P1, to_float4(pos, 1.0f)) < 1e-2);
  assert(dot(P2, to_float4(pos, 1.0f)) < 1e-2);
  
  float2 uv = { rand()*1.0f/RAND_MAX, rand()*1.0f/RAND_MAX };
  float2 D = { dot(P1, surf.get_point(uv.x, uv.y)), dot(P2, surf.get_point(uv.x, uv.y)) };
  
  int steps_left = max_steps-1;
  while(length(D) > EPS && steps_left--) {
    float2 J[2] = 
    { 
      { dot(P1, surf.uderivative(uv.x, uv.y)), dot(P2, surf.uderivative(uv.x, uv.y)) }, //col1
      { dot(P1, surf.vderivative(uv.x, uv.y)), dot(P2, surf.vderivative(uv.x, uv.y)) } //col2
    };

    float det = J[0][0]*J[1][1] - J[0][1] * J[1][0];

    float2 J_inversed[2] = 
    {
      { J[1][1]/det, -J[1][0]/det },
      { -J[0][1]/det, J[0][0]/det }
    };

    uv = uv - mul2x2x2(J_inversed, D);
    uv.x = u_closed ? closed_clamp(uv.x) : clamp(uv.x, 0.0f, 1.0f);
    uv.y = v_closed ? closed_clamp(uv.y) : clamp(uv.y, 0.0f, 1.0f);
    assert(0 <= uv.x && uv.x <= 1);
    assert(0 <= uv.y && uv.y <= 1);

    float2 new_D = { dot(P1, surf.get_point(uv.x, uv.y)), dot(P2, surf.get_point(uv.x, uv.y)) };
    
    if (length(new_D) > length(D))
      return {};
    
    D = new_D;
  }

  if (length(D) > EPS)
    return {};
  
  return float3(uv.x, uv.y, 0.0f);
}

float pseudo_dot(const float2 &a, const float2 &b) {
  return a.x * b.y - a.y * b.x;
}

float2 bezier_project(
  const float4 &P1,
  const float4 &P2,
  const float4 &point) {
  return float2 { dot(point, P1), dot(point, P2) };
}

float2 new_bound(
    const Surface &surf,
    const float4 &P1,
    const float4 &P2,
    const float2 &nL,
    const float2 &ubounds,
    const float2 &vbounds,
    const float2 &bounds,
    bool isu) {
  int n = surf.points.get_n()-1;
  int m = surf.points.get_m()-1;

  float2 bl = { -1e16f, bounds.y};
  float2 br = { -1e16f, bounds.x };
  float2 tl = { 1e16f, bounds.y };
  float2 tr = { 1e16f, bounds.x };

  for (int i = 0; i < surf.points.get_n()-1; ++i)
  for (int j = 0; j < surf.points.get_m()-1; ++j)
  {
    float u = ubounds.x + (ubounds.y-ubounds.x)*1.0f/n*i;
    float v = vbounds.x + (vbounds.y-vbounds.x)*1.0f/m*i;
    float2 pij = bezier_project(P1, P2, surf.get_point(u, v));
    float dij = dot(pij, nL);
    float par = isu ? u : v;

    if (dij < 0) {
      if (pseudo_dot(tl-bl, float2{dij, par}-bl) < 0)
        bl = float2{dij, par};
      if (pseudo_dot(tr-br, float2{dij, par}-br) > 0)
        br = float2{dij, par};
    } else {
      if (pseudo_dot(tl-bl, float2{dij, par}-bl) < 0)
        tl = float2{dij, par};
      if (pseudo_dot(tr-br, float2{dij, par}-br) > 0)
        tr = float2{dij, par};
    }
  }

  float mn = tl.x-bl.y/(tl.y-bl.y)*(tl.x-bl.x);
  float mx = tr.x-br.y/(tr.y-br.y)*(tr.x-br.x);

  return { mn, mx };
}

std::optional<float3> trace_surface_bezier(
    const float3 &pos,
    const float3 &ray,
    const Surface &surf) {
  int n = surf.points.get_n()-1;
  int m = surf.points.get_m()-1;

  float3 ortho_dir1 = (ray.x || ray.z) ? float3{ 0, 1, 0 } : float3{ 1, 0, 0 };
  float3 ortho_dir2 = normalize(cross(ortho_dir1, ray));
  ortho_dir1 = normalize(cross(ray, ortho_dir2));
  assert(dot(ortho_dir1, ortho_dir2) < 1e-2f);
  assert(dot(ortho_dir1, ray) < 1e-2f);
  assert(dot(ortho_dir2, ray) < 1e-2f);

  float4 P1 = to_float4(ortho_dir1, -dot(ortho_dir1, pos));
  float4 P2 = to_float4(ortho_dir2, -dot(ortho_dir2, pos));
  assert(dot(P1, to_float4(pos, 1.0f)) < 1e-2);
  assert(dot(P2, to_float4(pos, 1.0f)) < 1e-2);

  float2 Lu = 
            (
              bezier_project(P1, P2, surf.points[{n, 0}])
            - bezier_project(P1, P2, surf.points[{0, 0}])
            + bezier_project(P1, P2, surf.points[{n, m}])
            - bezier_project(P1, P2, surf.points[{0, m}])
            )
            * 0.5f;
  float2 Lv = 
            (
              bezier_project(P1, P2, surf.points[{0, m}])
            - bezier_project(P1, P2, surf.points[{0, 0}])
            + bezier_project(P1, P2, surf.points[{n, m}])
            - bezier_project(P1, P2, surf.points[{n, 0}])
            )
            * 0.5f;
  if (length(Lu) < EPS && length(Lv) < EPS) {
    float2 Lu = { 1, 1 };
    float2 Lv = { -1, 1 };
  } else if (length(Lu) < EPS) {
    Lu = float2{ -Lv.y, Lv.x };
  } else if (length(Lv) < EPS) {
    Lv = float2{ -Lu.y, Lu.x };
  }
  Lu = normalize(Lu);
  float2 nLu = float2{ -Lu.y, Lu.x };
  Lv = normalize(Lv);
  float2 nLv = float2{ -Lv.y, Lv.x };

  std::vector<std::tuple<float2, float2, bool>> bounds;
  bounds.emplace_back(float2{0, 1}, float2{0, 1}, true);

  float3 res;
  float2 res_uv;
  float t = -1.0f;

  while(!bounds.empty()) {
    auto [ubounds, vbounds, isu] = bounds.back();
    bounds.pop_back();
    if (isu) {
      float2 new_ubounds = new_bound(
          surf, 
          P1, P2, 
          nLu, 
          ubounds, vbounds, 
          ubounds, 
          true);
      if (new_ubounds.y-new_ubounds.x < EPS && vbounds.y-vbounds.x < EPS) {
        
      }
    } 

  }
  
}

void draw_points(
    const Surface &surface,
    const Camera &camera,
    Image2D<uint32_t> &image, 
    float col[4]) {
  image.clear(LiteMath::uchar4{ 153, 153, 153, 255 }.u32);
  if (surface.u_knots.size() < 2 || surface.v_knots.size() < 2)
    return;
  float3 forward = normalize(camera.target - camera.position);
  float4x4 view = lookAt(camera.position, camera.target, camera.up);
  float4x4 proj = perspectiveMatrix(camera.fov*180*M_1_PI, camera.aspect, 0.01f, 150.0f);
  int count = 250;
  for (int ui = 0; ui < count; ++ui)
  for (int vi = 0; vi < count; ++vi)
  {
    float u = ui * 1.0f/count;
    float v = vi * 1.0f/count;
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
        static_cast<u_char>(u*255.0f),
        static_cast<u_char>(v*255.0f),
        static_cast<u_char>(0*255.0f),
        static_cast<u_char>(1*255.0f) }.u32;
  }
}

void draw_newton(
    const Surface &surface,
    const Camera &camera,
    Image2D<uint32_t> &image, 
    float col[4]) {
  image.clear(LiteMath::uchar4{ 153, 153, 153, 255 }.u32);
  if (surface.u_knots.size() < 2 || surface.v_knots.size() < 2)
    return;
  float4x4 mat  = perspectiveMatrix(camera.fov*180*M_1_PI, camera.aspect, 0.001f, 100.0f)
                * lookAt(camera.position, camera.target, camera.up);
  float4x4 inversed_mat = inverse4x4(mat);

  for (uint32_t y = 0; y < image.height(); ++y)
  for (uint32_t x = 0; x < image.width();  ++x)
  {
    float2 ndc_point  = float2{ x+0.5f, y+0.5f } 
                      / float2{ image.width()*1.0f, image.height()*1.0f }
                      * 2.0f
                      - 1.0f;
    float4 ndc_point4 = { ndc_point.x, ndc_point.y, 0.0f, 1.0f };
    float4 point = inversed_mat * ndc_point4;
    point /= point.w;

    float3 ray = normalize(to_float3(point)-camera.position);
    float3 pos = camera.position;
    auto intersect_point = trace_surface_newton(pos, ray, surface);
    if (intersect_point.has_value()) {
      float3 new_col = intersect_point.value();
      image[uint2{ x, image.height()-1-y }] = uchar4{ 
        static_cast<u_char>(new_col[0]*255.0f),
        static_cast<u_char>(new_col[1]*255.0f),
        static_cast<u_char>(new_col[2]*255.0f),
        static_cast<u_char>(new_col[3]*255.0f) }.u32;
    }
  }
}

void draw_bezier(
    const Surface &surface,
    const Camera &camera,
    Image2D<uint32_t> &image, 
    float col[4]) {
  image.clear(LiteMath::uchar4{ 153, 153, 153, 255 }.u32);
  if (surface.u_knots.size() < 2 || surface.v_knots.size() < 2)
    return;
  float4x4 mat  = perspectiveMatrix(camera.fov*180*M_1_PI, camera.aspect, 0.001f, 100.0f)
                * lookAt(camera.position, camera.target, camera.up);
  float4x4 inversed_mat = inverse4x4(mat);

  for (uint32_t y = 0; y < image.height(); ++y)
  for (uint32_t x = 0; x < image.width();  ++x)
  {
    float2 ndc_point  = float2{ x+0.5f, y+0.5f } 
                      / float2{ image.width()*1.0f, image.height()*1.0f }
                      * 2.0f
                      - 1.0f;
    float4 ndc_point4 = { ndc_point.x, ndc_point.y, 0.0f, 1.0f };
    float4 point = inversed_mat * ndc_point4;
    point /= point.w;

    float3 ray = normalize(to_float3(point)-camera.position);
    float3 pos = camera.position;
    auto intersect_point = trace_surface_newton(pos, ray, surface);
    if (intersect_point.has_value()) {
      float3 new_col = intersect_point.value();
      image[uint2{ x, image.height()-1-y }] = uchar4{ 
        static_cast<u_char>(new_col[0]*255.0f),
        static_cast<u_char>(new_col[1]*255.0f),
        static_cast<u_char>(new_col[2]*255.0f),
        static_cast<u_char>(new_col[3]*255.0f) }.u32;
    }
  }
}