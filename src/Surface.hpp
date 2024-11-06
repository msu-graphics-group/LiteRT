#ifndef NURBS_SRC_SURFACE
#define NURBS_SRC_SURFACE

#include <cinttypes>
#include <vector>
#include <cmath>
#include <filesystem>
#include <limits>
#include <iostream>

#include <LiteMath.h>
#include <Image2d.h>

#include "utils.hpp"

struct BoundingBox3d
{
public:
  BoundingBox3d() = default;
  BoundingBox3d(const LiteMath::float4 *points, int count) {
    for (int i = 0; i < count; ++i) {
      LiteMath::float3 point = LiteMath::to_float3(points[i]/points[i].w);
      mn = LiteMath::min(mn, point);
      mx = LiteMath::max(mx, point);
    }
  }
public:
  LiteMath::float3 mn = LiteMath::float3{std::numeric_limits<float>::infinity()};
  LiteMath::float3 mx = -LiteMath::float3{std::numeric_limits<float>::infinity()};
  LiteMath::float3 center() const { return (mn+mx)/2; }
  LiteMath::float3 shape() const { return (mn-mx); }
public:
  LiteMath::float2 tbounds(
      const LiteMath::float3 &pos,
      const LiteMath::float3 &ray) const {
    LiteMath::float3 lo = (mn-pos)/ray;
    LiteMath::float3 ho = (mx-pos)/ray;
    LiteMath::float3 tmin3 = LiteMath::min(lo, ho);
    LiteMath::float3 tmax3 = LiteMath::max(lo, ho);
    float tmin = LiteMath::hmax(tmin3);
    float tmax = LiteMath::hmin(tmax3);
    return { tmin, tmax };
  }
  bool intersects(
      const LiteMath::float3 &pos,
      const LiteMath::float3 &ray) const {
    auto t_bounds = tbounds(pos, ray);
    if (t_bounds[1] < t_bounds[0])
      return false;
    return t_bounds[1] >= 0.0f;
  }
};

class NURBS_Surface {
public:
  NURBS_Surface() = default;
  NURBS_Surface(
      const Matrix2D<LiteMath::float4> &points,
      const Matrix2D<float> &weights,
      uint32_t deg_u, uint32_t deg_v,
      const std::vector<float> &u_knots,
      const std::vector<float> &v_knots)
      : points(points), weights(weights),
          deg_u(deg_u), deg_v(deg_v),
          u_knots(u_knots), v_knots(v_knots),
          bbox(points.data(), points.get_n()*points.get_m()){}
public:
  Matrix2D<LiteMath::float4> points;
  Matrix2D<float> weights;
  uint32_t deg_u;
  uint32_t deg_v;
  std::vector<float> u_knots;
  std::vector<float> v_knots;
  BoundingBox3d bbox;
};
NURBS_Surface load_nurbs(const std::filesystem::path &path);

enum class SurfaceParameter { U, V };
struct RBezier
{
public:
  Matrix2D<LiteMath::float4> weighted_points;
public:
  LiteMath::float4 get_point(float u, float v) const;
  LiteMath::float4 uder(float u, float v) const;
  LiteMath::float4 uder(float u, float v, const LiteMath::float4 &Sw) const;
  LiteMath::float4 vder(float u, float v) const;
  LiteMath::float4 vder(float u, float v, const LiteMath::float4 &Sw) const;
};

struct RBezierGrid
{
public:
  std::vector<float> uniq_uknots;
  std::vector<float> uniq_vknots;
  Matrix2D<RBezier> grid;
  BoundingBox3d bbox;
  bool is_visible = true;
public:
  LiteMath::float4 get_point(float u, float v) const;
  LiteMath::float4 uder(float u, float v) const;
  LiteMath::float4 uder(float u, float v, const LiteMath::float4 &Sw) const;
  LiteMath::float4 vder(float u, float v) const;
  LiteMath::float4 vder(float u, float v, const LiteMath::float4 &Sw) const;
  LiteMath::float3 normal(float u, float v) const;
  LiteMath::float3 normal(float u, float v, const LiteMath::float4 &Sw) const;
  LiteMath::int2 get_spans(float u, float v) const;
};

RBezierGrid
nurbs2rbezier(const NURBS_Surface &nurbs);

std::vector<RBezierGrid>
load_rbeziers(const std::filesystem::path &path);

std::tuple<std::vector<BoundingBox3d>, std::vector<LiteMath::float2>>
get_bvh_leaves(const RBezierGrid &rbezier);
#endif