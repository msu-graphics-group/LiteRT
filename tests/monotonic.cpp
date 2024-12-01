#include <vector>
#include <iostream>

#include "curve.hpp"
#include "LiteMath.h"

using namespace LiteMath;

std::ostream& operator<<(std::ostream& cout, float3 v) {
  return cout << "(" << v.x << " " << v.y << " " << v.z << ")";
}

int main() {
  // 1/2 of Circle
  /*
  std::vector<float2> points = {
    {-1, -1},
    {-1,  1},
    { 1,  1},
    { 1, -1},
  };
  std::vector<float> weights = {2, 1, 1, 2};
  */

  // Cubic-like curve
  std::vector<float2> points = {
    {-5.73, -3.989},
    {-2.756, 6.709},
    {0.87, -7.689},
    {5.295, 1.777},
  };
  std::vector<float> weights = {1, 1, 1, 1};

  RBCurve2D curve(points, weights);

  auto X = curve.monotonic_parts(0);
  std::cout << "X: ";
  for (auto x : X)
    std::cout << x << " ";
  std::cout << std::endl;

  auto Y = curve.monotonic_parts(1);
  std::cout << "Y: ";
  for (auto y : Y)
    std::cout << y << " ";
  std::cout << std::endl;
  return 0;
}
