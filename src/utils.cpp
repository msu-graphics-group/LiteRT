#include <cmath>

#include "utils.hpp"

bool isclose(float a, float b, float eps) {
  return std::abs(a - b) < eps;
}
