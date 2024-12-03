#ifndef NURBS_SRC_CURVE
#define NURBS_SRC_CURVE

#include <vector>
#include <optional>
#include <functional>

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
  // Pi(xi, yi) -> Pi(wi * xi, wi * yi, wi)

  RBCurve2D() {};
  RBCurve2D(std::vector<LiteMath::float2> points,
            std::vector<float> weights,
            float tmin = 0.0f, float tmax = 1.0f);
  static std::vector<LiteMath::float3> Hmap(
      std::vector<LiteMath::float2> points,
      std::vector<float> weights);

  LiteMath::float3 get_point(float u) const;
  LiteMath::float3 der(float u, int order = 1) const;
  LiteMath::float3 fg_gf(float u, int order = 0) const;
  std::vector<float> monotonic_parts(int axes, int order = 0) const; 
  LiteMath::float3 operator()(float u) const;
};

std::optional<float>
bisection(std::function<float(float)> f, float u1, float u2);

#endif 
