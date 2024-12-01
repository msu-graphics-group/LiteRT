#include <vector>
#include <iostream>

#include "LiteMath.h"
#include "curve.hpp"
#include "debug.hpp"

using namespace LiteMath;

int main() {
  std::vector<float2> points = {
    {-1, -1},
    {-1,  1},
    { 1,  1},
    { 1, -1},
  };
  std::vector<float> weights = {1, 1, 1, 1};
  RBCurve2D curve(points, weights);

  srand(time(NULL));
  float u = (float)(rand() % 100) / 100;
  auto val = curve(u);
  auto der = curve.der(u, 1);
  auto fg_gf = curve.fg_gf(u);
  auto fg_gf1 = curve.fg_gf(u, 1);
  auto fg_gf2 = curve.fg_gf(u, 2);
  auto fg_gf3 = curve.fg_gf(u, 3);
  std::cout << "u = " << u << std::endl;
  std::cout << "C(u) = " << val << std::endl;
  std::cout << "C'(u) = " << der << std::endl;
  std::cout << "(fg'-f'g)(u) = " << fg_gf << std::endl;
  std::cout << "(fg'-f'g)(1)(u) = " << fg_gf1 << std::endl;
  std::cout << "(fg'-f'g)(2)(u) = " << fg_gf2 << std::endl;
  std::cout << "(fg'-f'g)(3)(u) = " << fg_gf3 << std::endl;
  return 0;
}
