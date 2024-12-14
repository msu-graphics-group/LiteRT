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
    bool close = isclose(x0, u0, 2.0f * c::INTERSECTION_EPS);
    passed = passed && close;
  }
  return passed;
}

// *********************** Intersection open-curve Tests *********************** //
bool slope_line() {
  std::vector<float2> points = {
    {-1, -1},
    { 1,  1},
  };
  std::vector<float> weights = {1, 1};
  float u0  = 0.5f;
  int total = 1;

  RBCurve2D curve(points, weights);
  curve.preset();
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
  curve.preset();
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
  curve.preset();
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
  curve.preset();
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
  curve.preset();
  auto intersections = curve.intersections(u0);
  return test_intersections(curve, intersections, u0, total);
}

bool curl_mid() {
  std::vector<float2> points = {
    {-4, -4},
    {-4,  4},
    { 0,  4},
    { 4,  4},
    { 4, -4},
    {-2, -0.5f},
  };
  std::vector<float> weights = {1, 1, 1, 1, 1, 1};
  float u0  = 0;
  int total = 2;

  RBCurve2D curve(points, weights);
  curve.preset();
  auto intersections = curve.intersections(u0);
  return test_intersections(curve, intersections, u0, total);
}

bool curl_left_quater_offset() {
  std::vector<float2> points = {
    {-4, -4},
    {-4,  4},
    { 0,  4},
    { 4,  4},
    { 4, -4},
    {-2, -0.5f},
  };
  std::vector<float> weights = {1, 1, 1, 1, 1, 1};
  float u0  = -2.0f;
  int total = 2; // Should be one?

  RBCurve2D curve(points, weights);
  curve.preset();
  auto intersections = curve.intersections(u0);
  return test_intersections(curve, intersections, u0, total);
}

bool curl_right_quater_offset() {
  std::vector<float2> points = {
    {-4, -4},
    {-4,  4},
    { 0,  4},
    { 4,  4},
    { 4, -4},
    {-2, -0.5f},
  };
  std::vector<float> weights = {1, 1, 1, 1, 1, 1};
  float u0  = 2.0f;
  int total = 2;

  RBCurve2D curve(points, weights);
  curve.preset();
  auto intersections = curve.intersections(u0);
  return test_intersections(curve, intersections, u0, total);
}

bool curl_left_offset() {
  std::vector<float2> points = {
    {-4, -4},
    {-4,  4},
    { 0,  4},
    { 4,  4},
    { 4, -4},
    {-2, -0.5f},
  };
  std::vector<float> weights = {1, 1, 1, 1, 1, 1};
  float u0  = -2.5f;
  int total = 1;

  RBCurve2D curve(points, weights);
  curve.preset();
  auto intersections = curve.intersections(u0);
  return test_intersections(curve, intersections, u0, total);
}

bool curl_left_edge() {
  std::vector<float2> points = {
    {-4, -4},
    {-4,  4},
    { 0,  4},
    { 4,  4},
    { 4, -4},
    {-2, -0.5f},
  };
  std::vector<float> weights = {1, 1, 1, 1, 1, 1};
  float u0  = -4.0f;
  int total = 1; // Should be zero?

  RBCurve2D curve(points, weights);
  curve.preset();
  auto intersections = curve.intersections(u0);
  return test_intersections(curve, intersections, u0, total);
}

bool triangle() {
  std::vector<float2> points = {
    {-4, -4},
    {-4,  4},
    { 0,  4},
    { 4,  4},
    { 4, -4},
  };
  std::vector<float> weights = {1, 1, 1e2, 1, 1};
  float u0  = 2.0f;
  int total = 1;

  RBCurve2D curve(points, weights);
  curve.preset();
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
  curve.preset();
  auto intersections = curve.intersections(u0);
  return test_intersections(curve, intersections, u0, total);
}

bool egg_offset() {
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
  curve.preset();
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
  curve.preset();
  auto intersections = curve.intersections(u0);
  return test_intersections(curve, intersections, u0, total);
}

bool waterdrop_mid() {
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
  float u0  = 0.0f;
  int total = 2;

  RBCurve2D curve(points, weights);
  curve.preset();
  auto intersections = curve.intersections(u0);
  return test_intersections(curve, intersections, u0, total);
}

bool waterdrop_right_offset() {
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
  float u0  = 0.5f;
  int total = 2;

  RBCurve2D curve(points, weights);
  curve.preset();
  auto intersections = curve.intersections(u0);
  return test_intersections(curve, intersections, u0, total);
}

bool waterdrop_left_offset() {
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
  float u0  = -0.5f;
  int total = 2;

  RBCurve2D curve(points, weights);
  curve.preset();
  auto intersections = curve.intersections(u0);
  return test_intersections(curve, intersections, u0, total);
}

bool waterdrop_left_edge() {
  std::vector<float2> points = {
    {-1,  1},
    { 1,  1},
    { 1, -1},
    { 1, -3},
    {-1, -3},
    {-1, -1},
    {-1,  1}
  };

  // Finds two interections on the edge!
  std::vector<float> weights = {2, 1, 2, 1, 1, 1, 1};
  float u0  = -1.0f;
  int total = 2;

  RBCurve2D curve(points, weights);
  curve.preset();
  auto intersections = curve.intersections(u0);
  return test_intersections(curve, intersections, u0, total);
}

bool waterdrop_left_edge_offset() {
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
  float u0  = -0.9f;
  int total = 2;

  RBCurve2D curve(points, weights);
  curve.preset();
  auto intersections = curve.intersections(u0);
  return test_intersections(curve, intersections, u0, total);
}

} // namespace intersections_tests

int main() {
  using namespace intersection_tests;

  debug::title("Intersection open-curve Tests");
  debug::test("Slope Line", slope_line());
  debug::test("Cubic Bezier 1", cubic_bezier_1());
  debug::test("Cubic Bezier 2", cubic_bezier_2());
  debug::test("Half Circle", half_circle());
  debug::test("Half Circle Edge", half_circle_edge(), true);
  debug::test("Curl Mid", curl_mid());
  debug::test("Curl Left-Quater-Offset", curl_left_quater_offset(), true);
  debug::test("Curl Right-Quater-Offset", curl_right_quater_offset());
  debug::test("Curl Left-Offset", curl_left_offset());
  debug::test("Curl Left-Edge", curl_left_edge(), true);
  debug::test("Triangle", triangle());

  debug::title("Intersection closed-curve Tests");
  debug::test("Egg Mid", egg_mid());
  debug::test("Egg Offset", egg_offset());
  debug::test("Egg Edge", egg_edge(), true);
  debug::test("Waterdrop Mid", waterdrop_mid());
  debug::test("Waterdrop Right-Offset", waterdrop_right_offset());
  debug::test("Waterdrop Left-Offset", waterdrop_left_offset());
  debug::test("Waterdrop Left-Edge", waterdrop_left_edge());
  debug::test("Waterdrop Left-Edge-Offset", waterdrop_left_edge_offset(), true);
  
  return 0;
}
