#include <vector>
#include <iostream>

#include "curve.hpp"
#include "LiteMath.h"

using namespace LiteMath;

std::ostream& operator<<(std::ostream& cout, float3 v) {
  return cout << "(" << v.x << " " << v.y << " " << v.z << ")";
}

float F1(float u) {
  return 0.0f;
}

int main() {
  std::optional<float> root = bisection(F1, 1.0f, 10.0f);
  if (root.has_value())
    std::cout << "u = " << root.value() << std::endl;
  else std::cout << "No root" << std::endl;
 
  return 0;
}
