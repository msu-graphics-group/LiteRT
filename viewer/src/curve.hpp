#ifndef NURBS_SRC_CURVE
#define NURBS_SRC_CURVE

#include <vector>
#include <optional>
#include <functional>
#include <filesystem>
#include <limits>
#include <array>

#include "utils.hpp"
#include "LiteMath.h"

struct BCurve3D
{
  LiteMath::uint degree() const;

  BCurve3D() {};
  BCurve3D(std::vector<LiteMath::float3> points,
      float tmin = 0.0f, float tmax = 1.0f);
  LiteMath::float3 get_point(float u) const;
  LiteMath::float3 der(float u, int order = 1) const;
  LiteMath::float3 operator()(float u) const;

  Matrix2D<LiteMath::float3> points;
  float tmin;
  float tmax;
};

struct RBCurve2D : public BCurve3D
{
  // Stores points and weights as homorgeneous map to 3D coodinates
  // Pi(xi, yi) -> Pi(wi * xi, wi * yi, wi).
  // tmin <= u <= tmax is linearly mapped to [0, 1]

  RBCurve2D() {}; 
  RBCurve2D(std::vector<LiteMath::float2> points,
            std::vector<float> weights,
            float tmin = 0.0f, float tmax = 1.0f);
  RBCurve2D(std::vector<LiteMath::float3> Hpoints,
            float tmin = 0.0f, float tmax = 1.0f);
  static std::vector<LiteMath::float3> Hmap(
      std::vector<LiteMath::float2> points,
      std::vector<float> weights);

  LiteMath::float3 get_point(float u) const;
  LiteMath::float3 der(float u, int order = 1) const;
  LiteMath::float3 fg_gf(float u, int order = 0) const;
  std::vector<float> monotonic_parts(int axes, int order = 0) const;
  std::vector<float> intersections(float u0) const;
  LiteMath::float3 operator()(float u) const;

  // Monotonic knots
  std::vector<float> knots;
};

struct NURBSCurve2D
{
public:
  int degree(void) const;
  int npoints(void) const;
  int nknots(void) const;
  std::vector<RBCurve2D> decompose(void) const;
public:
  std::vector<LiteMath::float3> pw;
  std::vector<float> knots;
};
NURBSCurve2D load_nurbs_curve(std::filesystem::path path);

std::optional<float>
bisection(std::function<float(float)> f, float u1, float u2);

struct KdTreeBox
{
  LiteMath::float2 boxMin{ std::numeric_limits<float>::infinity() };
  LiteMath::float2 boxMax{ -std::numeric_limits<float>::infinity() };
};


struct CurveId
{
public:
  CurveId(uint16_t curve_id = 0xffff, uint16_t monotonic_span = 0xffff)
      : curve_id(curve_id), monotonic_span(monotonic_span) {}
  CurveId(uint32_t value): curve_id(value >> 16), monotonic_span(value & 0xffff) {}
public:
  operator uint32_t() const { return (curve_id << 16)|monotonic_span; }
public:
  uint16_t curve_id;
  uint16_t monotonic_span;
};
struct KdTreeLeave
{
  uint32_t curve_id = -1u;
  bool precalc = 0;
};

std::array<RBCurve2D, 2>
de_casteljau_division(const RBCurve2D &c, float t);

std::tuple<std::vector<KdTreeBox>, std::vector<KdTreeLeave>>
get_kdtree_leaves(const std::vector<RBCurve2D> &curves);

#endif 
