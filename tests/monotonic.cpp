#include <vector>
#include <iostream>

#include "LiteMath.h"
#include "constants.hpp"
#include "curve.hpp"
#include "debug.hpp"

namespace c = constants;

// *********************** Simple Monotonic Tests *********************** //

namespace monotonic_tests {

bool test_monotonic(
    const RBCurve2D &curve,
    std::vector<float> Xtruth,
    std::vector<float> Ytruth,
    float tmin = 0.0f,
    float tmax = 1.0f) {
  
  Xtruth = lerp(tmin, tmax, Xtruth);
  std::vector<float> Xtest = curve.monotonic_parts(0);
  bool Xclose = allclose(Xtest, Xtruth, c::BISECTION_EPS);
  //std::cout << Xtest << std::endl;

  Ytruth = lerp(tmin, tmax, Ytruth);
  std::vector<float> Ytest = curve.monotonic_parts(1);
  bool Yclose = allclose(Ytest, Ytruth, c::BISECTION_EPS);
  //std::cout << Ytest << std::endl;
  
  return Xclose && Yclose;
}

bool slope_line() {
  std::vector<float2> points = {
    {-4, -4},
    { 4,  4},
  };
  std::vector<float> weights = {1, 1};

  RBCurve2D curve(points, weights);
  std::vector<float> Xtruth = { 0.0f, 1.0f };
  std::vector<float> Ytruth = { 0.0f, 1.0f };
  return test_monotonic(curve, Xtruth, Ytruth);
}

bool vertical_line() {
  std::vector<float2> points = {
    {-4, -4},
    {-4,  4},
  };
  std::vector<float> weights = {1, 1};

  RBCurve2D curve(points, weights);
  std::vector<float> Xtruth = { 0.0f, 1.0f };
  std::vector<float> Ytruth = { 0.0f, 1.0f };
  return test_monotonic(curve, Xtruth, Ytruth);
}

bool horizontal_line() {
  std::vector<float2> points = {
    {-4, -4},
    { 4, -4},
  };
  std::vector<float> weights = {1, 1};

  RBCurve2D curve(points, weights);
  std::vector<float> Xtruth = { 0.0f, 1.0f };
  std::vector<float> Ytruth = { 0.0f, 1.0f };
  return test_monotonic(curve, Xtruth, Ytruth);
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
  std::vector<float> Xtruth = { 0.0f, 1.0f };
  std::vector<float> Ytruth = { 0.0f, 0.5f, 1.0f };
  return test_monotonic(curve, Xtruth, Ytruth);
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
  std::vector<float> Xtruth = { 0.0f, 1.0f };
  std::vector<float> Ytruth = { 0.0f, 0.5f, 1.0f };
  return test_monotonic(curve, Xtruth, Ytruth);
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
  std::vector<float> Xtruth = { 0.0f, 1.0f };
  std::vector<float> Ytruth = { 0.0f, 0.5f, 1.0f };
  return test_monotonic(curve, Xtruth, Ytruth);
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
  std::vector<float> Xtruth = { 0.0f, 0.214f, 0.786f, 1.0f };
  std::vector<float> Ytruth = { 0.0f, 0.5f, 1.0f };
  return test_monotonic(curve, Xtruth, Ytruth);
}

bool waterdrop() {
  std::vector<float2> points = {
    {-1,  1},
    { 1,  1},
    { 1, -1},
    { 1, -3},
    {-1, -3},
    {-1, -1},
    {-1,  1}
  };
  std::vector<float> weights = {2, 1, 2, 1, 1, 1, 1};

  RBCurve2D curve(points, weights);
  std::vector<float> Xtruth = { 0.0f, 0.358f, 1.0f };
  std::vector<float> Ytruth = { 0.0f, 0.614f, 1.0f };
  return test_monotonic(curve, Xtruth, Ytruth);
}

bool curl() {
  std::vector<float2> points = {
    {-4, -4},
    {-4,  4},
    { 0,  4},
    { 4,  4},
    { 4, -4},
    {-2, -0.5f},
  };
  std::vector<float> weights = {1, 1, 1, 1, 1, 1};

  RBCurve2D curve(points, weights);
  std::vector<float> Xtruth = { 0.0f, 0.694f, 1.0f };
  std::vector<float> Ytruth = { 0.0f, 0.392f, 0.902f, 1.0f };
  return test_monotonic(curve, Xtruth, Ytruth);
}

bool triangle() {
  std::vector<float2> points = {
    {-4, -4},
    {-4,  4},
    { 0,  4},
    { 4,  4},
    { 4, -4},
  };

  // Failes when w3 = 1e8
  std::vector<float> weights = {1, 1, 1e3, 1, 1};

  RBCurve2D curve(points, weights);
  std::vector<float> Xtruth = { 0.0f, 1.0f };
  std::vector<float> Ytruth = { 0.0f, 0.5f, 1.0f };
  return test_monotonic(curve, Xtruth, Ytruth);
}

// *********************** Mapped Monotonic Tests *********************** //
bool mapped_cubic_bezier_1() {
  std::vector<float2> points = {
    {-4, -4},
    {-2,  4},
    { 2, -4},
    { 4,  4},
  };
  std::vector<float> weights = {1, 1, 1, 1};
  float tmin = 0.4f;
  float tmax = 0.8f;

  RBCurve2D curve(points, weights, tmin, tmax);
  std::vector<float> Xtruth = { 0.0f, 1.0f };
  std::vector<float> Ytruth = { 0.0f, 0.5f, 1.0f };
  return test_monotonic(curve, Xtruth, Ytruth, tmin, tmax);
}

bool mapped_cubic_bezier_2() {
  std::vector<float2> points = {
    {-4, -4},
    {-4,  4},
    { 4, -4},
    { 4,  4},
  };
  std::vector<float> weights = {1, 1, 1, 1};
  float tmin = 0.6f;
  float tmax = 0.9f;

  RBCurve2D curve(points, weights, tmin, tmax);
  std::vector<float> Xtruth = { 0.0f, 1.0f };
  std::vector<float> Ytruth = { 0.0f, 0.5f, 1.0f };
  return test_monotonic(curve, Xtruth, Ytruth, tmin, tmax);
}

bool mapped_egg() {
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
  float tmin = 0.5f;
  float tmax = 1.0f;

  RBCurve2D curve(points, weights, tmin, tmax);
  std::vector<float> Xtruth = { 0.0f, 0.214f, 0.786f, 1.0f };;
  std::vector<float> Ytruth = { 0.0f, 0.5f, 1.0f };
  return test_monotonic(curve, Xtruth, Ytruth, tmin, tmax);
}

} // monotonic_tests

int main() {
  debug::title("Simple Monotonic Tests");
  debug::test("Slope Line", monotonic_tests::slope_line());
  debug::test("Vertical Line", monotonic_tests::vertical_line());
  debug::test("Horizontal Line", monotonic_tests::horizontal_line());
  debug::test("Cubic Bezier 1", monotonic_tests::cubic_bezier_1());
  debug::test("Cubic Bezier 2", monotonic_tests::cubic_bezier_2());
  debug::test("Half Circle", monotonic_tests::half_circle());
  debug::test("Egg", monotonic_tests::egg());
  debug::test("Water Drop", monotonic_tests::waterdrop());
  debug::test("Curl", monotonic_tests::curl());
  debug::test("Triangle", monotonic_tests::triangle());

  debug::title("Mapped Monotonic Tests");
  debug::test("Mapped Cubic Bezier 1", monotonic_tests::mapped_cubic_bezier_1());
  debug::test("Mapped Cubic Bezier 2", monotonic_tests::mapped_cubic_bezier_2());
  debug::test("Mapped Egg", monotonic_tests::mapped_egg());
  return 0;
}
