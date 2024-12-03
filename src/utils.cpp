#include <vector>
#include <cmath>

#include "utils.hpp"

// Inverse lerp
//float ilerp(float a, float b, float t) {
//  return (t - a) / (b - a);
//}

bool isclose(float a, float b, float eps) {
  return std::abs(a - b) < eps;
}

bool allclose(std::vector<float> v, std::vector<float> w, float eps) {
  if (v.size() != w.size()) return false;
  for (size_t i = 0; i < v.size(); i++) {
    if (!isclose(v[i], w[i], eps))
      return false;
  }
  return true;
}
