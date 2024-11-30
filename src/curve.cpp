#include <vector>
#include <cassert>
#include <iostream>

#include "utils.hpp"
#include "curve.hpp"

using namespace LiteMath;

// *********************** Bezier curve 3D *********************** //
uint BCurve3D::degree() const {
  return points.get_n() - 1;
}

LiteMath::float3 BCurve3D::operator()(float u) const {
    return get_point(u);
}

BCurve3D::BCurve3D(std::vector<LiteMath::float3> points) {
  assert(points.size() > 0);
  uint p = points.size() - 1;
  this->points = Matrix2D<float3>(p + 1, p + 1);

  for (uint j = 0; j <= p; j++) {
    auto index = std::make_pair(0, j);
    this->points[index] = points[j];
  }

  for (uint i = 1; i <= p; i++) {
    for (uint j = 0; j <= p - i; j++) {
      auto index = std::make_pair(i,   j);
      auto left  = std::make_pair(i-1, j);
      auto right = std::make_pair(i-1, j+1);
      this->points[index] = (p - i + 1) * (this->points[right] - this->points[left]);
    }
  }
}

LiteMath::float3 BCurve3D::get_point(float u) const {
  return der(u, 0);
}

LiteMath::float3 BCurve3D::der(float u, int order) const {
  uint p = degree();
  if (order < 0 || order > p)
    return float3(0.0f);

  float u_n = 1.0f;
  float _1_u = 1.0f - u;

  // Note: C(n, k) can store only up to n = 67 for 64-bit integer.
  // Critical no-overflow value appears for C(67, 33).
  unsigned long long comb = 1;
  auto points = this->points.row(order);
  p -= order;

  if (p == 0) {
    return points[0];
  }

  float3 res = points[0] * _1_u;
  for (uint i = 1; i <= p-1; ++i) {
    u_n *= u;
    comb = comb * (p-i+1) / i;
    res = (res + u_n * comb * points[i]) * _1_u;
  }
  res += (u_n * u) * points[p];
  return res;
}


// *********************** RBezier curve 2D *********************** //
RBCurve2D::RBCurve2D(
    std::vector<float2> points,
    std::vector<float> weights) : BCurve3D( RBCurve2D::Hmap(points, weights) ) {}

std::vector<float3> RBCurve2D::Hmap(
    std::vector<float2> points,
    std::vector<float> weights) {
  // Homorgeneous map to 3D space coodinates

  assert(points.size() > 0 && points.size() == weights.size());

  std::vector<float3> Hpoints(points.size());
  for (size_t i = 0; i < points.size(); i++) {
    float3 point = float3(points[i].x, points[i].y, 1);
    Hpoints[i] = weights[i] * point;
  }
  return Hpoints;
}

LiteMath::float3 RBCurve2D::get_point(float u) const {
  float3 p = BCurve3D::get_point(u);
  return p / p.z;
}

LiteMath::float3 RBCurve2D::fg_gf(float u, int order) {
  // Returns n-order derivative of F(u) = f(u)g'(u)-f'(u)g(u),
  // where f(u) - numerator of RBezier and g(u) is the denominator
  // Complexity: O(n^2)

  assert(order >= 0);
  uint p = order;
  std::vector<float3> ders(p + 2);

  for (uint i = 0; i <= p + 1; i++) {
    ders[i] = BCurve3D::der(u, i);
  }

  float3 result = ders[0] * ders[p+1].z - ders[1] * ders[p].z;
  if (order == 0) return result;

  unsigned long long comb = 1;
  for (uint i = 1; i <= order - 1; i++) {
    comb = comb * (p-i+1) / i;
    result += comb * (ders[i] * ders[p-i+1].z - ders[i+1] * ders[p-i].z);
  }
  result += ders[p] * ders[1].z - ders[p+1] * ders[0].z;
  return result;
}

LiteMath::float3 RBCurve2D::der(float u, int order) const {
  assert(order >= 0);
  if (order == 0) {
    return get_point(u);
  }

  if (order == 1) {
    float3 p = get_point(u);
    float3 d = BCurve3D::der(u);
    return (d*p.z - d.z*p) / (p.z * p.z);
  }

  unsigned long long comb = 1;
  float3 left = float3(0.0f);
  for (int i = 0; i < order; ++i) {
    float3 a = der(u, i);
    float b = BCurve3D::der(u, order-i).z;
    left += comb * a * b;
    comb = comb * (order-i) / (i+1);
  }

  float3 right = BCurve3D::der(u, order);

  float3 res = (right - left) / (BCurve3D::get_point(u).z * comb);
  assert(res.z == 0.0f);

  return res;
}

LiteMath::float3 RBCurve2D::operator()(float u) const {
    return get_point(u);
}
