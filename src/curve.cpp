#include <vector>
#include <cassert>

#include "curve.hpp"

using namespace LiteMath;

int RBCurve2D::degree() const {
  return pw.size()-1;
}

LiteMath::float3 RBCurve2D::get_point(float u) const {
  int p = degree();

  float u_n = 1.0f;
  float _1_u = 1.0f - u;
  int bc = 1.0f;

  float3 res = pw[0] * _1_u;
  for (int i = 1; i <= p-1; ++i) {
    u_n *= u;
    bc = bc * (p-i+1)/i;
    res = (res + u_n * bc * pw[i]) * _1_u;
  }
  res += (u_n * u) * pw[p];
  return res;
}

LiteMath::float3 RBCurve2D::non_rat_der(float u, int order) const {
  if (order == 0)
    return get_point(u);

  int p = degree();

  RBCurve2D der = { std::vector<float3>(degree()+1) };
  std::copy(pw.begin(), pw.end(), der.pw.begin());

  for (int i = 0; i < order; ++i) {
    for (int j = 0; j < p-i; ++i) {
      der.pw[j] = (der.pw[j+1] - der.pw[j]) * (p-i);
    }
  }

  der.pw.resize(p+1-order);
  return der.get_point(u);
}

LiteMath::float3 RBCurve2D::der(float u) const {
  float3 p = get_point(u);
  float3 d = non_rat_der(u, 1);
  return (d*p.z - d.z*p)/(p.z * p.z);
}

LiteMath::float3 RBCurve2D::der(float u, int order) const {
  if (order == 0) {
    float3 res = get_point(u);
    res /= res.z;
    return res;
  }

  int bc = 1;
  float3 left = {};
  for (int i = 0; i < order; ++i) {
    float3 a = der(u, i);
    float b = non_rat_der(u, order-i).z;
    left += bc * a * b;
    bc = bc * (order-i) / (i+1);
  }

  float3 right = non_rat_der(u, order);

  float3 res = (right - left) / (get_point(u).z * bc);
  assert(res.z == 0.0f);

  return res;
}

