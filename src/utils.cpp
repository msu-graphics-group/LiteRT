#include <vector>
#include <cmath>

#include "LiteMath.h"
#include "utils.hpp"

using namespace LiteMath;

// Inverse lerp
float ilerp(float a, float b, float t) {
  return (t - a) / (b - a);
}

std::vector<float> lerp(float a, float b, std::vector<float> t) {
  for (size_t i = 0; i < t.size(); i++)
    t[i] = lerp(a, b, t[i]);
  return t;
}

bool isclose(float a, float b, float eps) {
  return std::abs(a - b) < eps;
}

bool isclose(float3 a, float3 b, float eps) {
  return length(a - b) < eps;
}

bool allclose(std::vector<float> v, std::vector<float> w, float eps) {
  if (v.size() != w.size()) return false;
  for (size_t i = 0; i < v.size(); i++) {
    if (!isclose(v[i], w[i], eps))
      return false;
  }
  return true;
}
