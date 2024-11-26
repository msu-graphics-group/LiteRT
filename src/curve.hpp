#ifndef NURBS_SRC_CURVE
#define NURBS_SRC_CURVE

#include <vector>

#include "utils.hpp"
#include "LiteMath.h"

struct BCurve3D
{
  LiteMath::uint degree() const;

  BCurve3D() {};
  BCurve3D(std::vector<LiteMath::float3> points);
  LiteMath::float3 get_point(float u) const;
  LiteMath::float3 der(float u, int order = 1) const;
  Matrix2D<LiteMath::float3> ders(float u, int order = 1) const;
  LiteMath::float3 operator()(float u) const;

  Matrix2D<LiteMath::float3> points;
};

struct RBCurve2D
{
public:
  LiteMath::uint degree() const;
public:
  RBCurve2D() {};
  RBCurve2D(std::vector<LiteMath::float2> points,
            std::vector<float> weights);
  LiteMath::float3 get_point(float u) const;
  LiteMath::float3 der(float u, int order = 1) const;
  LiteMath::float3 fg_gf(float u, int order = 0);
  LiteMath::float3 operator()(float u) const;
public:
  std::vector<LiteMath::float3> points;
  std::vector<float> weights;

  // Homorgeneous map to 3D coodinates
  // Pi(xi, yi) -> Pi(wi * xi, wi * yi, wi)
  BCurve3D H;
};

#endif 
