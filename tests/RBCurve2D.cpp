#include <vector>
#include <iostream>

#include "curve.hpp"
#include "LiteMath.h"

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
  std::cout << "u = " << u << std::endl;
  std::cout << "C(u) = (" << val.x << " " << val.y << " " << val.z << ")" << std::endl;
  std::cout << "C'(u) = (" << der.x << " " << der.y << " " << der.z << ")" << std::endl;
  return 0;
}
