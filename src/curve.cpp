#include <vector>
#include <cassert>
#include <iostream>
#include <optional>
#include <functional>
#include <filesystem>
#include <fstream>
#include <numeric>

#include "utils.hpp"
#include "constants.hpp"
#include "curve.hpp"

using namespace LiteMath;

namespace c = constants;

// *********************** Bezier curve 3D *********************** //
uint BCurve3D::degree() const {
  return points.get_n() - 1;
}

LiteMath::float3 BCurve3D::operator()(float u) const {
    return get_point(u);
}

BCurve3D::BCurve3D(
    std::vector<LiteMath::float3> points,
    float tmin, float tmax) : tmin(tmin), tmax(tmax) {
  assert(points.size() > 0);
  uint p = points.size() - 1;
  this->points = Matrix2D<float3>(p + 1, p + 1);

  for (uint j = 0; j <= p; j++) {
    auto index = std::make_pair(0, j);
    this->points[index] = points[j];
  }

  for (uint i = 1; i <= p; i++) {
    for (uint j = 0; j <= p - i; j++) {
      auto index = std::make_pair(i,   j);
      auto left  = std::make_pair(i-1, j);
      auto right = std::make_pair(i-1, j+1);
      this->points[index] = (p - i + 1) * (this->points[right] - this->points[left]);
    }
  }
}

LiteMath::float3 BCurve3D::get_point(float u) const {
  return der(u, 0);
}

LiteMath::float3 BCurve3D::der(float u, int order) const {
  u = lerp(tmin, tmax, u);
  uint p = degree();
  if (order < 0 || order > p)
    return float3(0.0f);

  float u_n = 1.0f;
  float _1_u = 1.0f - u;

  // Note: C(n, k) can store only up to n = 67 for 64-bit integer.
  // Critical no-overflow value appears for C(67, 33).
  unsigned long long comb = 1;
  auto points = this->points.row(order);
  p -= order;

  if (p == 0) {
    return points[0];
  }

  float3 res = points[0] * _1_u;
  for (uint i = 1; i <= p-1; ++i) {
    u_n *= u;
    comb = comb * (p-i+1) / i;
    res = (res + u_n * comb * points[i]) * _1_u;
  }
  res += (u_n * u) * points[p];
  return res;
}


// *********************** RBezier curve 2D *********************** //
RBCurve2D::RBCurve2D(
    std::vector<float2> points,
    std::vector<float> weights,
    float tmin,
    float tmax) :
  BCurve3D( RBCurve2D::Hmap(points, weights), tmin, tmax ) {
  knots = monotonic_parts(0);
}

RBCurve2D::RBCurve2D(
    std::vector<LiteMath::float3> Hpoints,
    float tmin,
    float tmax) :
  BCurve3D( Hpoints, tmin, tmax ) {
  knots = monotonic_parts(0);
}

// Homorgeneous map to 3D space coodinates
std::vector<float3> RBCurve2D::Hmap(
    std::vector<float2> points,
    std::vector<float> weights) {

  assert(points.size() > 0 && points.size() == weights.size());

  std::vector<float3> Hpoints(points.size());
  for (size_t i = 0; i < points.size(); i++) {
    float3 point = float3(points[i].x, points[i].y, 1);
    Hpoints[i] = weights[i] * point;
  }
  return Hpoints;
}

LiteMath::float3 RBCurve2D::get_point(float u) const {
  float3 p = BCurve3D::get_point(u);
  return p / p.z;
}

// Returns n-order derivative of F(u) = f(u)g'(u)-f'(u)g(u),
// where f(u) - numerator of RBezier and g(u) is the denominator
// Complexity: O(n^2)
LiteMath::float3 RBCurve2D::fg_gf(float u, int order) const {
  assert(order >= 0);
  uint p = order;
  std::vector<float3> ders(p + 2);

  for (uint i = 0; i <= p + 1; i++) {
    ders[i] = BCurve3D::der(u, i);
  }

  float3 result = ders[0] * ders[p+1].z - ders[1] * ders[p].z;
  if (order == 0) return result;

  unsigned long long comb = 1;
  for (uint i = 1; i <= order - 1; i++) {
    comb = comb * (p-i+1) / i;
    result += comb * (ders[i] * ders[p-i+1].z - ders[i+1] * ders[p-i].z);
  }
  result += ders[p] * ders[1].z - ders[p+1] * ders[0].z;
  return result;
}

// Unused function
LiteMath::float3 RBCurve2D::der(float u, int order) const {
  assert(order >= 0);
  if (order == 0) {
    return get_point(u);
  }

  if (order == 1) {
    float3 p = get_point(u);
    float3 d = BCurve3D::der(u);
    return (d*p.z - d.z*p) / (p.z * p.z);
  }

  unsigned long long comb = 1;
  float3 left = float3(0.0f);
  for (int i = 0; i < order; ++i) {
    float3 a = der(u, i);
    float b = BCurve3D::der(u, order-i).z;
    left += comb * a * b;
    comb = comb * (order-i) / (i+1);
  }

  float3 right = BCurve3D::der(u, order);

  float3 res = (right - left) / (BCurve3D::get_point(u).z * comb);
  assert(res.z == 0.0f);

  return res;
}

// If     axes = 0, returns monotonic parts on x-axes
// elseif axes = 1, returns monotonic parts on y-axis
std::vector<float>
RBCurve2D::monotonic_parts(int axes, int order) const {
  int p = degree();
  if (order == 2*p-1) {
    return { 0.0f, 1.0f };
  }

  std::vector<float> result = { 0.0f };

  auto F = [&](float u) {
    return fg_gf(u, order)[axes];
  };

  // TODO: remove recursion
  auto knots = monotonic_parts(axes, order+1);
  //std::cout << "ORDER = " << order << " AXES = " << axes << std::endl;
  for (int span = 0; span < knots.size() - 1; ++span) {
    float a = knots[span], b = knots[span+1];
    //std::cout << a << " " << b << std::endl;
    auto potential_root = bisection(F, a, b);
    if (potential_root.has_value()) {
      float root = potential_root.value();
      if (!isclose(root, result.back(), 2.0f * c::BISECTION_EPS)) {
        result.push_back(root);
      }
    }
  }

  if (!isclose(1.0f, result.back(), c::BISECTION_EPS)) {
    result.push_back(1.0f);
  }

  return result;
}

std::vector<float>
RBCurve2D::intersections(float u0) const {
  std::vector<float> result;
  for (int span = 0; span < knots.size() - 1; ++span) {
    float umin = knots[span];
    float umax = knots[span+1];
    auto f = [&](float t) {
      auto p = get_point(t);
      return p.x - u0;
    };

    auto potential_hit = bisection(f, umin, umax);
    if (potential_hit.has_value()) {
      auto u = potential_hit.value();
      result.push_back(u);
    }
  }
  return result;
}

LiteMath::float3 RBCurve2D::operator()(float u) const {
    return get_point(u);
}

// *********************** NURBS curve 2D *********************** //
int NURBSCurve2D::degree() const {
  return knots.size() - pw.size() - 1;
}

int NURBSCurve2D::npoints() const {
  return pw.size();
}

int NURBSCurve2D::nknots() const {
  return knots.size();
}

std::vector<RBCurve2D>
NURBSCurve2D::decompose() const {
  int n = npoints() - 1; // Maybe just npoints
  int m = nknots() - 1; // Maybe just nknots
  int p = degree();

  std::vector<float> unique_knots;
  for (int i = 0; i < m; i++) {
    if (!isclose(knots[i], knots[i+1], c::KNOTS_EPS)) {
      unique_knots.push_back(knots[i]);
    }
  }
  unique_knots.push_back(knots[m]);

  int a = p;
  int b = p+1;
  int nb = 0;
  std::vector<std::vector<float3>> Qw(unique_knots.size() - 1, 
                                      std::vector<float3>(p + 1));
  std::vector<float> alphas(p+1);
  for (int i = 0; i <= p; ++i) {
    Qw[nb][i] = pw[i];
  }

  while (b < m) {
    int i = b;
    // Maybe should compare knots with close-function
    while (b < m && knots[b+1] == knots[b])
      b++;
    int mult = b-i+1;
    if (mult < p) {
      float numer = knots[b]-knots[a]; /* Numerator of alpha */
      /* Compute and store alphas */
      for (int j = p; j > mult; j--)
        alphas[j-mult-1] = numer / (knots[a+j] - knots[a]);
      int r = p-mult; /* Insert knot r times */
      for (int j = 1; j <= r; j++) {
        int save = r-j;
        int s = mult+j; /* This many new points */
        for (int k = p; k >= s; k--) {
          float alpha = alphas[k-s];
          Qw[nb][k] = alpha * Qw[nb][k] + (1.0f - alpha) * Qw[nb][k-1];
        }
        if (b < m) { /* Control point of */
          Qw[nb+1][save] = Qw[nb][p]; /* next segment */
        }
      }
    }
    nb = nb+1; /* Bezier segment completed */
    if (b < m) {
      /* Initialize for next segment */
      for (i = p - mult; i <= p; i++)
        Qw[nb][i] = pw[b-p+i];
      a = b;
      b = b+1;
    }
  }

  std::vector<RBCurve2D> result;
  for (int i = 0; i < Qw.size(); ++i) {
    auto curve = RBCurve2D(Qw[i], knots[i], knots[i+1]);
    result.push_back(curve);
  }
  return result;
}

NURBSCurve2D load_nurbs_curve(std::filesystem::path path) {
  std::fstream fin;
  fin.exceptions(std::ios::failbit|std::ios::badbit);
  fin.open(path);
  if (!fin.good()) throw std::runtime_error("Failed to open the file.");

  std::string tmp_str;
  char tmp_chr;

  NURBSCurve2D curve;

  int n;
  fin >> tmp_chr >> tmp_chr >> n; // n = ...

  curve.pw = std::vector<float3>(n+1, { 0, 0, 1.0f });
  float min_u = std::numeric_limits<float>::infinity();
  float max_u = -min_u;
  float min_v = min_u;
  float max_v = max_u;

  fin >> tmp_str; // "points:"
  for (int i = 0; i <= n; ++i)
  {
    auto &point = curve.pw[i];
    fin >> tmp_chr >> point.x >> tmp_chr >> point.y >> tmp_chr; // { ..., ..., ... }
    min_u = std::min(min_u, point.x);
    max_u = std::max(max_u, point.x);
    min_v = std::min(min_v, point.y);
    max_v = std::max(max_v, point.y);
  }
  //std::cout << "normalized points:" << std::endl;
  for (int i = 0; i <= n; ++i)
  {
    auto &point = curve.pw[i];
    point.x = (point.x-min_u) / (max_u-min_u);
    point.y = (point.y-min_v) / (max_v-min_v);
    std::cout << "{" << point.x << ", " << point.y << "} ";
  }
  std::cout << std::endl;


  fin >> tmp_str; // "weights:"
  for (int i = 0; i <= n; ++i)
  {
    float w;
    fin >> w;
    curve.pw[i] *= w;
  }

  fin >> tmp_str; // "degree:"
  int deg;
  fin >> deg;

  curve.knots.resize(n+deg+2);
  fin >> tmp_str; // "knots:"
  for (size_t i = 0; i < curve.knots.size(); ++i) {
    fin >> curve.knots[i];
  }

  float u_min = curve.knots.front();
  float u_max = curve.knots.back();
  for (auto &elem: curve.knots)
    elem = (elem-u_min) / (u_max-u_min);

  return curve;
}

// *********************** Utilities *********************** //
std::optional<float>
bisection(std::function<float(float)> f, float u1, float u2) {
  float l = u1;
  float r = u2;
  {
    float f1 = f(u1);
    float f2 = f(u2);
    if (std::abs(f1) < c::BISECTION_EPS && std::abs(f2) < c::BISECTION_EPS) {
      //This should be tested
      return {};//(l + r) / 2;
    }
    if (f1 > c::BISECTION_EPS  && f2 > c::BISECTION_EPS ) {
      return {};
    }
    if (f1 < -c::BISECTION_EPS  && f2 < -c::BISECTION_EPS ) {
      return {};
    }
    if (f1 > f2) {
      std::swap(l, r);
    }
  }

  // I tried using error of function instead of error of l-r
  // (Don't do this)
  while (std::abs(l-r) > c::BISECTION_EPS) { 
    float m = (l+r) / 2.0f;
    float value = f(m);
    if (value < 0.0f)
      l = m;
    else
      r = m;
  }

  return (l+r) / 2.0f;
}

std::array<RBCurve2D, 2>
de_casteljau_division(const RBCurve2D &c, float t) {
  int p = static_cast<int>(c.degree());
  float t_effective = (t - c.tmin) / (c.tmax - c.tmin);
  std::vector<float3> tmp(p+1);

  std::array res = { 
    std::vector<float3>(p+1),
    std::vector<float3>(p+1)
  };

  for (int i = 0; i <= p; ++i)
    tmp[i] = c.points[{0, i}];
  for (int i = 0; i <= p; ++i) {
    res[0][i] = tmp[0];
    res[1][p-i] = tmp[p-i];
    for (int j = 0; j <= p-i-1; ++j)
      tmp[j] += (tmp[j+1]-tmp[j])*t_effective;
  }
  return std::array{ RBCurve2D(res[0], c.tmin, t), RBCurve2D(res[1], c.tmax, t) };
}

KdTreeBox calc_box(const RBCurve2D &curve) {
  KdTreeBox res;
  for (int i = 0; i <= curve.degree(); ++i) {
    float3 p3 = curve.points[{0, i}];
    p3 /= p3.z;
    float2 p2 = float2{ p3.x, p3.y };
    res.boxMin = min(res.boxMin, p2);
    res.boxMax = max(res.boxMax, p2);
  }
  return res;
}

void 
get_kdtree_leaves_helper(
    std::vector<RBCurve2D> curves,
    std::vector<uint32_t> curv_ids, 
    std::vector<KdTreeBox> &res_boxes,
    std::vector<KdTreeLeave> &res_leaves,
    float2 box_min,
    float2 box_max,
    int axes = 0) {
  if (curves.size() == 0) {
    res_leaves.push_back(KdTreeLeave{ 0.0f, 0.0f, -1u }); 
    res_boxes.push_back(KdTreeBox{box_min, box_max});
    return;
  } 
  if (curves.size() == 1) {
    res_leaves.push_back(KdTreeLeave{ curves[0].tmin, curves[0].tmax, curv_ids[0] });
    res_boxes.push_back(KdTreeBox{box_min, box_max});
    return;
  }
  
  std::vector<RBCurve2D> left_child_curves, right_child_curves;
  std::vector<uint32_t> left_child_ids, right_child_ids;
  float middle = ((box_min+box_max)/2)[axes];
  for (int i = 0; i < curves.size(); ++i) {
    auto box = calc_box(curves[i]);
    if (box.boxMin[axes] >= middle) {
      right_child_curves.push_back(curves[i]);
      right_child_ids.push_back(curv_ids[i]);
    } else if (box.boxMax[axes] < middle) {
      left_child_curves.push_back(curves[i]);
      left_child_ids.push_back(curv_ids[i]);
    } else {
      auto &curve = curves[i];
      auto f = [&](float t) {
        auto p = curve.get_point(t);
        p /= p.z;
        return p[axes] - middle;
      };
      float t = bisection(f, curve.tmin, curve.tmax).value(); // TODO define chile to put curve if no intersections
      auto parts = de_casteljau_division(curve, t);
      auto test_point = parts[0].points[{0, 0}];
      test_point /= test_point.z;
      if (test_point[axes] < middle) {
        left_child_curves.push_back(parts[0]);
        right_child_curves.push_back(parts[1]);
      } else {
        left_child_curves.push_back(parts[1]);
        right_child_curves.push_back(parts[0]);
      }
      left_child_ids.push_back(curv_ids[i]);
      right_child_ids.push_back(curv_ids[i]);
    }
  }

  curves.clear();
  curves.reserve(0);

  auto lmn = box_min, rmn = box_min;
  auto lmx = box_max, rmx = box_max;
  lmx[axes] = middle;
  rmn[axes] = middle;

  get_kdtree_leaves_helper(
      std::move(left_child_curves), std::move(left_child_ids), 
      res_boxes, res_leaves,
      lmn, lmx, axes^1);
  get_kdtree_leaves_helper(
      std::move(right_child_curves), std::move(right_child_ids), 
      res_boxes, res_leaves,
      rmn, rmx, axes^1);
}

std::vector<RBCurve2D>
decompose_to_monotonic(const RBCurve2D &curve) {
  std::vector<float> monotonic_parts;
  auto monotonic_parts_u = curve.monotonic_parts(0);
  auto monotonic_parts_v = curve.monotonic_parts(1);
  std::copy(monotonic_parts_u.begin(), monotonic_parts_u.end(), std::back_inserter(monotonic_parts));
  std::copy(monotonic_parts_v.begin(), monotonic_parts_v.end(), std::back_inserter(monotonic_parts));
  std::sort(monotonic_parts.begin(), monotonic_parts.end());
  monotonic_parts.resize(std::unique(monotonic_parts.begin(), monotonic_parts.end())-monotonic_parts.begin());
  
  if (monotonic_parts.size() == 2) {
    return { curve };
  }

  std::vector<RBCurve2D> result = { curve };

  for (int span = 0; span < monotonic_parts.size()-1; ++span) {
    float tmin = monotonic_parts[span];
    float tmax = curve.tmax;
    float t_div = monotonic_parts[span+1];
    t_div = (t_div - tmin)/(tmax-tmin);
    auto parts = de_casteljau_division(result.back(), t_div);
    result.back() = parts[0];
    result.push_back(parts[1]);
  }

  return result;
}

std::tuple<std::vector<KdTreeBox>, std::vector<KdTreeLeave>>
get_kdtree_leaves(const std::vector<RBCurve2D> &curves) {
  std::vector<uint32_t> ids;
  std::vector<RBCurve2D> monotonic_curves;
  for (uint32_t i = 0; i < curves.size(); ++i) {
    auto cur_monotonic_curves = decompose_to_monotonic(curves[i]);
    std::copy(cur_monotonic_curves.begin(), cur_monotonic_curves.end(), std::back_inserter(monotonic_curves));
    size_t offset = ids.size();
    ids.resize(offset + cur_monotonic_curves.size());
    std::fill(ids.begin()+offset, ids.begin()+offset+cur_monotonic_curves.size(), i);
  }

  std::vector<KdTreeBox> res_boxes;
  std::vector<KdTreeLeave> res_leaves;

  KdTreeBox box;
  for (auto &curve: monotonic_curves) {
    auto cur_box = calc_box(curve);
    box.boxMin = min(box.boxMin, cur_box.boxMin);
    box.boxMax = max(box.boxMax, cur_box.boxMax);
  }

  get_kdtree_leaves_helper(
    std::move(monotonic_curves), std::move(ids), 
    res_boxes, res_leaves, box.boxMin, box.boxMax);

  return { res_boxes, res_leaves };
}
