#include <vector>
#include <iostream>

#include "curve.hpp"
#include "LiteMath.h"
#include "debug.hpp"

using namespace LiteMath;

float F1(float u) {
  return 1 - u * u;
}

int main() {
  std::optional<float> root = bisection(F1, 0.0f, 10.0f);
  if (root.has_value())
    std::cout << "u = " << root.value() << std::endl;
  else std::cout << "No root" << std::endl;
 
  return 0;
}
