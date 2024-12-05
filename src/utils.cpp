#include <vector>
#include <cmath>

#include "LiteMath.h"
#include "utils.hpp"

using namespace LiteMath;

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
