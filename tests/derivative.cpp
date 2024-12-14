#include <vector>
#include <iostream>
#include <functional>

#include "LiteMath.h"
#include "constants.hpp"
#include "curve.hpp"
#include "debug.hpp"

namespace c = constants;

// *********************** Simple Monotonic Tests *********************** //

namespace derivative_tests {

bool test_fg_gf(
    const RBCurve2D &curve,
    int samples = 1000) { 

  int n = curve.degree();
  int MAX_ORDER;
  if (n == 3)
    MAX_ORDER = 5;
  else MAX_ORDER = 3;

  auto C = BCurve3D(curve);
  bool passed = true;

  for (int m = 0; m <= MAX_ORDER; m++) {

    int mfact = 1;
    for (int k = 1; k <= m; k++)
      mfact *= k;

    auto Ftest = [&](float u) {
      return mfact * curve.fg_gf(u, m);
    };

    std::function<float3(float)> Ftruth;
    if (m == 0) {
      
      // f'g - fg'
      Ftruth = [&](float u) {
        return n * (C.der(u) * C(u).z - C(u) * C.der(u).z);
      };

    } else if (m == 1) {

      // f''g - fg''
      Ftruth = [&](float u) {
        return n * (n-1) * (C.der(u, 2) * C(u).z - C(u) * C.der(u, 2).z);
      };

    } else if (m == 2) {
      
      // f'''g + f''g' - f'g'' - fg'''
      Ftruth = [&](float u) {
        float3 result = n * (n-1) * (n-2) * (C.der(u, 3) * C(u).z - C(u) * C.der(u, 3).z);
        result += n * n * (n-1) * (C.der(u, 2) * C.der(u).z - C.der(u) * C.der(u, 2).z);
        return result;
      };

    } else if (m == 3) {
    
      // f(4)g + 2f(3)g' - 2f'g(3) - fg(4)
      Ftruth = [&](float u) {
        float3 result = n * (n-1) * (n-2) * (n-3) * (C.der(u, 4) * C(u).z - C(u) * C.der(u, 4).z);
        result += 2 * n * n * (n-1) * (n-2) * (C.der(u, 3) * C.der(u).z - C.der(u) * C.der(u, 3).z);
        return result;
      };

    } else if (n == 3 && m == 4) {
    
      // 2f(3)g(2) - 2f(2)g(3)
      Ftruth = [&](float u) {
        int mult = 2 * n * (n-1) * (n-2) * n * (n-1);
        return mult * (C.der(u, 3) * C.der(u, 2).z - C.der(u, 2) * C.der(u, 3).z);
      };
    } else if (n == 3 && m == 5) {

      Ftruth = [&](float u) {
        return float3(0.0f);
      };
    }

    float step = (float)1 / samples;
    for (float u = 0.0f; u <= 1.0f; u += step) {
      //std::cout << "order = " <<  m << ", u = " << u << std::endl;
      if (!isclose(Ftest(u).x, Ftruth(u).x, 2.0f * c::BISECTION_EPS) ||
          !isclose(Ftest(u).y, Ftruth(u).y, 2.0f * c::BISECTION_EPS)) {
        std::cout << debug::LOG::INFO << "Failed at order = " << m << ", u = " << u << std::endl;
        std::cout << debug::LOG::INFO << "Test: " << Ftest(u) << " | Truth: " << Ftruth(u) << std::endl;
        std::cout << debug::LOG::INFO << Ftest(u) - Ftruth(u) << std::endl;
        passed = false;
        break;
      }
    }

    if (!passed) break;
  }
  return passed;
}

bool cubic_bezier_1() {
  std::vector<float2> points = {
    {-4, -4},
    {-2,  4},
    { 2, -4},
    { 4,  4},
  };
  std::vector<float> weights = {1, 1, 1, 1};

  RBCurve2D curve(points, weights);
  return test_fg_gf(curve);
}

bool cubic_bezier_2() {
  std::vector<float2> points = {
    {-4, -4},
    {-4,  4},
    { 4, -4},
    { 4,  4},
  };
  std::vector<float> weights = {1, 1, 1, 1};

  RBCurve2D curve(points, weights);
  return test_fg_gf(curve);
}

bool half_circle() {
  std::vector<float2> points = {
    {-1, -1},
    {-1,  1},
    { 1,  1},
    { 1, -1},
  };
  std::vector<float> weights = {2, 1, 1, 2};

  RBCurve2D curve(points, weights);
  return test_fg_gf(curve);
}

bool egg() {
  std::vector<float2> points = {
    { 0,  1},
    { 1,  1},
    { 1,  0},
    { 1, -1},
    { 0, -1},
    {-1, -1},
    {-1,  0},
    {-1,  1},
    { 0,  1}
  };
  std::vector<float> weights = {1, 1, 1, 2, 1, 2, 1, 1, 1};

  RBCurve2D curve(points, weights);
  return test_fg_gf(curve);
}

} // derivative_tests

int main() {
  debug::title("Simple f'g - fg' Tests");
  //debug::test("Slope Line", derivative_tests::slope_line());
  //debug::test("Vertical Line", derivative_tests::vertical_line());
  //debug::test("Horizontal Line", derivative_tests::horizontal_line());
  debug::test("Cubic Bezier 1", derivative_tests::cubic_bezier_1());
  debug::test("Cubic Bezier 2", derivative_tests::cubic_bezier_2());
  debug::test("Half Circle", derivative_tests::half_circle());
  debug::test("Egg", derivative_tests::egg());
  //debug::test("Water Drop", derivative_tests::waterdrop());
  //debug::test("Curl", derivative_tests::curl());
  //debug::test("Triangle", derivative_tests::triangle());

  //debug::title("Mapped Monotonic Tests");
  //debug::test("Mapped Cubic Bezier 1", derivative_tests::mapped_cubic_bezier_1());
  //debug::test("Mapped Cubic Bezier 2", derivative_tests::mapped_cubic_bezier_2());
  //debug::test("Mapped Egg", derivative_tests::mapped_egg());
  return 0;
}
