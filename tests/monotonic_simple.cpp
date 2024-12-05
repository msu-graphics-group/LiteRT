#include <vector>
#include <iostream>

#include "LiteMath.h"
#include "constants.hpp"
#include "curve.hpp"
#include "debug.hpp"

namespace c = constants;

// *********************** Simple Monotonic Tests *********************** //
bool test_slope_line() {
  std::vector<float2> points = {
    {-4, -4},
    { 4,  4},
  };
  std::vector<float> weights = {1, 1};

  RBCurve2D curve(points, weights);
  std::vector<float> Xtest = curve.monotonic_parts(0);
  std::vector<float> Xtruth = { 0.0f, 1.0f };
  bool Xclose = allclose(Xtest, Xtruth, c::TEST_EPS);
  //std::cout << debug::LOG::INFO << Xtest << std::endl;

  auto Y = curve.monotonic_parts(1);
  std::vector<float> Ytest = curve.monotonic_parts(1);
  std::vector<float> Ytruth = { 0.0f, 1.0f };
  bool Yclose = allclose(Ytest, Ytruth, c::TEST_EPS);
  //std::cout << debug::LOG::INFO << Ytest << std::endl;

  return Xclose && Yclose;
}

bool test_vertical_line() {
  std::vector<float2> points = {
    {-4, -4},
    {-4,  4},
  };
  std::vector<float> weights = {1, 1};

  RBCurve2D curve(points, weights);
  std::vector<float> Xtest = curve.monotonic_parts(0);
  std::vector<float> Xtruth = { 0.0f, 1.0f };
  bool Xclose = allclose(Xtest, Xtruth, c::TEST_EPS);
  //std::cout << debug::LOG::INFO << Xtest << std::endl;

  auto Y = curve.monotonic_parts(1);
  std::vector<float> Ytest = curve.monotonic_parts(1);
  std::vector<float> Ytruth = { 0.0f, 1.0f };
  bool Yclose = allclose(Ytest, Ytruth, c::TEST_EPS);
  //std::cout << debug::LOG::INFO << Ytest << std::endl;

  return Xclose && Yclose;
}

bool test_horizontal_line() {
  std::vector<float2> points = {
    {-4, -4},
    { 4, -4},
  };
  std::vector<float> weights = {1, 1};

  RBCurve2D curve(points, weights);
  std::vector<float> Xtest = curve.monotonic_parts(0);
  std::vector<float> Xtruth = { 0.0f, 1.0f };
  bool Xclose = allclose(Xtest, Xtruth, c::TEST_EPS);
  //std::cout << debug::LOG::INFO << Xtest << std::endl;

  auto Y = curve.monotonic_parts(1);
  std::vector<float> Ytest = curve.monotonic_parts(1);
  std::vector<float> Ytruth = { 0.0f, 1.0f };
  bool Yclose = allclose(Ytest, Ytruth, c::TEST_EPS);
  //std::cout << debug::LOG::INFO << Ytest << std::endl;

  return Xclose && Yclose;
}

bool test_cubic_bezier_1() {
  std::vector<float2> points = {
    {-4, -4},
    {-2,  4},
    { 2, -4},
    { 4,  4},
  };
  std::vector<float> weights = {1, 1, 1, 1};

  RBCurve2D curve(points, weights);
  std::vector<float> Xtest = curve.monotonic_parts(0);
  std::vector<float> Xtruth = { 0.0f, 1.0f };
  bool Xclose = allclose(Xtest, Xtruth, c::TEST_EPS);
  //std::cout << debug::LOG::INFO << Xtest << std::endl;

  auto Y = curve.monotonic_parts(1);
  std::vector<float> Ytest = curve.monotonic_parts(1);
  std::vector<float> Ytruth = { 0.0f, 0.5f, 1.0f };
  bool Yclose = allclose(Ytest, Ytruth, c::TEST_EPS);
  //std::cout << debug::LOG::INFO << Ytest << std::endl;
  
  return Xclose && Yclose;
}

bool test_cubic_bezier_2() {
  std::vector<float2> points = {
    {-4, -4},
    {-4,  4},
    { 4, -4},
    { 4,  4},
  };
  std::vector<float> weights = {1, 1, 1, 1};

  RBCurve2D curve(points, weights);
  std::vector<float> Xtest = curve.monotonic_parts(0);
  std::vector<float> Xtruth = { 0.0f, 1.0f };
  bool Xclose = allclose(Xtest, Xtruth, c::TEST_EPS);
  //std::cout << debug::LOG::INFO << Xtest << std::endl;

  auto Y = curve.monotonic_parts(1);
  std::vector<float> Ytest = curve.monotonic_parts(1);
  std::vector<float> Ytruth = { 0.0f, 0.5f, 1.0f };
  bool Yclose = allclose(Ytest, Ytruth, c::TEST_EPS);
  //std::cout << debug::LOG::INFO << Ytest << std::endl;

  return Xclose && Yclose;
}


bool test_half_circle() {
  std::vector<float2> points = {
    {-1, -1},
    {-1,  1},
    { 1,  1},
    { 1, -1},
  };
  std::vector<float> weights = {2, 1, 1, 2};

  RBCurve2D curve(points, weights);
  std::vector<float> Xtest = curve.monotonic_parts(0);
  std::vector<float> Xtruth = { 0.0f, 1.0f };
  bool Xclose = allclose(Xtest, Xtruth, c::TEST_EPS);

  auto Y = curve.monotonic_parts(1);
  std::vector<float> Ytest = curve.monotonic_parts(1);
  std::vector<float> Ytruth = { 0.0f, 0.5f, 1.0f };
  bool Yclose = allclose(Ytest, Ytruth, c::TEST_EPS);
  
  return Xclose && Yclose;
}

bool test_egg() {
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
  std::vector<float> Xtest = curve.monotonic_parts(0);
  std::vector<float> Xtruth = { 0.0f, 0.214f, 0.786f, 1.0f };
  bool Xclose = allclose(Xtest, Xtruth, c::TEST_EPS);
  //std::cout << debug::LOG::INFO << Xtest << std::endl;

  auto Y = curve.monotonic_parts(1);
  std::vector<float> Ytest = curve.monotonic_parts(1);
  std::vector<float> Ytruth = { 0.0f, 0.5f, 1.0f };
  bool Yclose = allclose(Ytest, Ytruth, c::TEST_EPS);
  //std::cout << debug::LOG::INFO << Ytest << std::endl;
  
  return Xclose && Yclose;
}

bool test_waterdrop() {
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
  std::vector<float> Xtest = curve.monotonic_parts(0);
  std::vector<float> Xtruth = { 0.0f, 0.358f, 1.0f };
  bool Xclose = allclose(Xtest, Xtruth, c::TEST_EPS);

  auto Y = curve.monotonic_parts(1);
  std::vector<float> Ytest = curve.monotonic_parts(1);
  std::vector<float> Ytruth = { 0.0f, 0.614f, 1.0f };
  bool Yclose = allclose(Ytest, Ytruth, c::TEST_EPS);

  return Xclose && Yclose;
}

bool test_curl() {
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
  std::vector<float> Xtest = curve.monotonic_parts(0);
  std::vector<float> Xtruth = { 0.0f, 0.694f, 1.0f };
  bool Xclose = allclose(Xtest, Xtruth, c::TEST_EPS);
  //std::cout << debug::LOG::INFO << Xtest << std::endl;

  auto Y = curve.monotonic_parts(1);
  std::vector<float> Ytest = curve.monotonic_parts(1);
  std::vector<float> Ytruth = { 0.0f, 0.392f, 0.902f, 1.0f };
  bool Yclose = allclose(Ytest, Ytruth, c::TEST_EPS);
  //std::cout << debug::LOG::INFO << Ytest << std::endl;

  return Xclose && Yclose;
}

bool test_triangle() {
  std::vector<float2> points = {
    {-4, -4},
    {-4,  4},
    { 0,  4},
    { 4,  4},
    { 4, -4},
  };

  // Failes when w3 = 1e8
  std::vector<float> weights = {1, 1, 1e6, 1, 1};

  RBCurve2D curve(points, weights);
  std::vector<float> Xtest = curve.monotonic_parts(0);
  std::vector<float> Xtruth = { 0.0f, 1.0f };
  bool Xclose = allclose(Xtest, Xtruth, c::TEST_EPS);
  //std::cout << debug::LOG::INFO << Xtest << std::endl;

  auto Y = curve.monotonic_parts(1);
  std::vector<float> Ytest = curve.monotonic_parts(1);
  std::vector<float> Ytruth = { 0.0f, 0.5f, 1.0f };
  bool Yclose = allclose(Ytest, Ytruth, c::TEST_EPS);
  //std::cout << debug::LOG::INFO << Ytest << std::endl;

  return Xclose && Yclose;
}

// *********************** Mapped Monotonic Tests *********************** //
bool test_mapped_cubic_bezier_1() {
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
  std::vector<float> Xtest = curve.monotonic_parts(0);
  std::vector<float> Xtruth = { 0.0f, 1.0f };
  Xtruth = lerp(tmin, tmax, Xtruth);
  bool Xclose = allclose(Xtest, Xtruth, c::TEST_EPS);
  //std::cout << debug::LOG::INFO << Xtruth << std::endl;
  //std::cout << debug::LOG::INFO << Xtest << std::endl;

  auto Y = curve.monotonic_parts(1);
  std::vector<float> Ytest = curve.monotonic_parts(1);
  std::vector<float> Ytruth = { 0.0f, 0.5f, 1.0f };
  Ytruth = lerp(tmin, tmax, Ytruth);
  bool Yclose = allclose(Ytest, Ytruth, c::TEST_EPS);
  //std::cout << debug::LOG::INFO << Ytruth << std::endl;
  //std::cout << debug::LOG::INFO << Ytest << std::endl;

  return Xclose && Yclose;
}

bool test_mapped_cubic_bezier_2() {
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
  std::vector<float> Xtest = curve.monotonic_parts(0);
  std::vector<float> Xtruth = { 0.0f, 1.0f };
  Xtruth = lerp(tmin, tmax, Xtruth);
  bool Xclose = allclose(Xtest, Xtruth, c::TEST_EPS);
  //std::cout << debug::LOG::INFO << Xtruth << std::endl;
  //std::cout << debug::LOG::INFO << Xtest << std::endl;

  auto Y = curve.monotonic_parts(1);
  std::vector<float> Ytest = curve.monotonic_parts(1);
  std::vector<float> Ytruth = { 0.0f, 0.5f, 1.0f };
  Ytruth = lerp(tmin, tmax, Ytruth);
  bool Yclose = allclose(Ytest, Ytruth, c::TEST_EPS);
  //std::cout << debug::LOG::INFO << Ytruth << std::endl;
  //std::cout << debug::LOG::INFO << Ytest << std::endl;

  return Xclose && Yclose;
}

bool test_mapped_egg() {
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
  std::vector<float> Xtest = curve.monotonic_parts(0);
  std::vector<float> Xtruth = { 0.0f, 0.214f, 0.786f, 1.0f };;
  Xtruth = lerp(tmin, tmax, Xtruth);
  bool Xclose = allclose(Xtest, Xtruth, c::TEST_EPS);
  //std::cout << debug::LOG::INFO << Xtest << std::endl;

  auto Y = curve.monotonic_parts(1);
  std::vector<float> Ytest = curve.monotonic_parts(1);
  std::vector<float> Ytruth = { 0.0f, 0.5f, 1.0f };
  Ytruth = lerp(tmin, tmax, Ytruth);
  bool Yclose = allclose(Ytest, Ytruth, c::TEST_EPS);
  //std::cout << debug::LOG::INFO << Ytest << std::endl;

  return Xclose && Yclose;
}

int main() {
  debug::title("Simple Monotonic Tests");
  debug::test("Slope Line", test_slope_line());
  debug::test("Vertical Line", test_vertical_line());
  debug::test("Horizontal Line", test_horizontal_line());
  debug::test("Cubic Bezier 1", test_cubic_bezier_1());
  debug::test("Cubic Bezier 2", test_cubic_bezier_2());
  debug::test("Half Circle", test_half_circle());
  debug::test("Egg", test_egg());
  debug::test("Water Drop", test_waterdrop());
  debug::test("Curl", test_curl());
  debug::test("Triangle", test_triangle());

  debug::title("Mapped Monotonic Tests");
  debug::test("Mapped Cubic Bezier 1", test_mapped_cubic_bezier_1());
  debug::test("Mapped Cubic Bezier 2", test_mapped_cubic_bezier_2());
  debug::test("Mapped Egg", test_mapped_egg());
  return 0;
}
