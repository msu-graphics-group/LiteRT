#include <vector>
#include <iostream>

#include "LiteMath.h"
#include "constants.hpp"
#include "curve.hpp"
#include "debug.hpp"

using namespace LiteMath;

namespace c = constants;

namespace intersection_tests {

bool test_intersections(
    const RBCurve2D &curve,
    const std::vector<float> &intersections,
    float u0,
    int nintersections) {
  if (intersections.size() != nintersections)
    return false;

  bool passed = true;
  for (const auto &u : intersections) {
    float x0  = curve(u).x;
    //std::cout << x0 - u0 << std::endl;
    // TODO: derive the formula for eps. This is heuristic.
    bool close = isclose(x0, u0, 4.0f * sqrt(2.0f) * c::BISECTION_EPS);
    passed = passed && close;
  }
  return passed;
}

// *********************** Intersection open-curve Tests *********************** //
bool slope_line() {
  std::vector<float2> points = {
    {-4, -4},
    { 4,  4},
  };
  std::vector<float> weights = {1, 1};
  float u0  = 2.0f;
  int total = 1;

  RBCurve2D curve(points, weights);
  auto intersections = curve.intersections(u0);
  return test_intersections(curve, intersections, u0, total);
}

bool cubic_bezier_1() {
  std::vector<float2> points = {
    {-4, -4},
    {-2,  4},
    { 2, -4},
    { 4,  4},
  };
  std::vector<float> weights = {1, 1, 1, 1};
  float u0  = 0.0f;
  int total = 1;

  RBCurve2D curve(points, weights);
  auto intersections = curve.intersections(u0);
  return test_intersections(curve, intersections, u0, total);
}

bool cubic_bezier_2() {
  std::vector<float2> points = {
    {-4, -4},
    {-4,  4},
    { 4, -4},
    { 4,  4},
  };
  std::vector<float> weights = {1, 1, 1, 1};
  float u0  = -3.5f;
  int total = 1;

  RBCurve2D curve(points, weights);
  auto intersections = curve.intersections(u0);
  return test_intersections(curve, intersections, u0, total);
}

bool half_circle() {
  std::vector<float2> points = {
    {-1, -1},
    {-1,  1},
    { 1,  1},
    { 1, -1},
  };
  std::vector<float> weights = {2, 1, 1, 2};
  float u0  = 0.65f;
  int total = 1;

  RBCurve2D curve(points, weights);
  auto intersections = curve.intersections(u0);
  return test_intersections(curve, intersections, u0, total);
}

bool half_circle_edge() {
  std::vector<float2> points = {
    {-1, -1},
    {-1,  1},
    { 1,  1},
    { 1, -1},
  };
  std::vector<float> weights = {2, 1, 1, 2};
  float u0  = 1.0f;
  int total = 1; // Should be zero?

  RBCurve2D curve(points, weights);
  auto intersections = curve.intersections(u0);
  return test_intersections(curve, intersections, u0, total);
}

// *********************** Intersection closed-curve Tests *********************** //
bool egg_mid() {
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
  float u0  = 0.5f;
  int total = 2;

  RBCurve2D curve(points, weights);
  auto intersections = curve.intersections(u0);
  return test_intersections(curve, intersections, u0, total);
}

bool egg_mid_offset() {
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
  float u0  = 0.4f;
  int total = 2;

  RBCurve2D curve(points, weights);
  auto intersections = curve.intersections(u0);
  return test_intersections(curve, intersections, u0, total);
}

bool egg_edge() {
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
  float u0  = -1.0f;
  int total = 0;

  RBCurve2D curve(points, weights);
  auto intersections = curve.intersections(u0);
  return test_intersections(curve, intersections, u0, total);
}

} // namespace intersections_tests

int main() {
  debug::title("Intersection open-curve Tests");
  debug::test("Slope Line", intersection_tests::slope_line());
  debug::test("Cubic Bezier 1", intersection_tests::cubic_bezier_1());
  debug::test("Cubic Bezier 2", intersection_tests::cubic_bezier_2());
  debug::test("Half Circle", intersection_tests::half_circle());
  debug::test("Half Circle Edge", intersection_tests::half_circle_edge());

  debug::title("Intersection closed-curve Tests");
  debug::test("Egg Mid", intersection_tests::egg_mid());
  debug::test("Egg Mid-Offset", intersection_tests::egg_mid_offset());
  debug::test("Egg Edge", intersection_tests::egg_edge());

  return 0;
}
