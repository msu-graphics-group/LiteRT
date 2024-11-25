#include <vector>
#include <cassert>

#include "curve.hpp"

using namespace LiteMath;

// *********************** Bezier curve 3D *********************** //
uint BCurve3D::degree() const {
  return points.size() - 1;
}

LiteMath::float3 BCurve3D::operator()(float u) const {
    return get_point(u);
}

LiteMath::float3 BCurve3D::get_point(float u) const {
  uint p = degree();

  float u_n = 1.0f;
  float _1_u = 1.0f - u;

  // Note: C(n, k) can store only up to n = 67 for 64-bit integer.
  // Critical no-overflow value appears for C(67, 33).
  unsigned long long comb = 1;

  float3 res = points[0] * _1_u;
  for (int i = 1; i <= p-1; ++i) {
    u_n *= u;
    comb *= (p-i+1) / i;
    res = (res + u_n * comb * points[i]) * _1_u;
  }
  res += (u_n * u) * points[p];
  return res;
}

LiteMath::float3 BCurve3D::der(float u, int order) const {
  assert(order >= 0);
  if (order == 0)
    return get_point(u);

  uint p = degree();

  BCurve3D der = { std::vector<float3>(p + 1) };
  std::copy(points.begin(), points.end(), der.points.begin());

  for (int i = 0; i < order; ++i) {
    for (int j = 0; j < p-i; ++i) {
      der.points[j] = (der.points[j+1] - der.points[j]) * (p-i);
    }
  }

  der.points.resize(p+1-order);
  return der.get_point(u);
}

// *********************** RBezier curve 2D *********************** //
uint RBCurve2D::degree() const {
  return points.size() - 1;
}

LiteMath::float3 RBCurve2D::operator()(float u) const {
    return get_point(u);
}

RBCurve2D::RBCurve2D(
    std::vector<float2> points,
    std::vector<float> weights) {
  assert(points.size() == weights.size());

  this->points.resize(points.size());
  this->weights.resize(weights.size());

  for (size_t i = 0; i < points.size(); i++)
    this->points[i] = float3(points[i].x, points[i].y, 1);
  this->weights = weights;

  this->H.points.resize(points.size());
  for (size_t i = 0; i < points.size(); i++)
    this->H.points[i] = this->weights[i] * this->points[i];
}

LiteMath::float3 RBCurve2D::get_point(float u) const {
  float3 p = H.get_point(u);
  p /= p.z;
  return p;
}

LiteMath::float3 RBCurve2D::der(float u, int order) const {
  assert(order >= 0);
  if (order == 0) {
    return get_point(u);
  }

  if (order == 1) {
    float3 p = get_point(u);
    float3 d = H.der(u);
    return (d*p.z - d.z*p) / (p.z * p.z);
  }

  unsigned long long comb = 1;
  float3 left = float3(0.0f);
  for (int i = 0; i < order; ++i) {
    float3 a = der(u, i);
    float b = H.der(u, order-i).z;
    left += comb * a * b;
    comb *= (order-i) / (i+1);
  }

  float3 right = H.der(u, order);

  float3 res = (right - left) / (H.get_point(u).z * comb);
  assert(res.z == 0.0f);

  return res;
}
