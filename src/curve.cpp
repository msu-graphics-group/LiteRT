#include <vector>
#include <cassert>
#include <iostream>
#include <optional>
#include <functional>
#include <filesystem>
#include <fstream>

#include "utils.hpp"
#include "constants.hpp"
#include "curve.hpp"

#include "debug.hpp"

using namespace LiteMath;

namespace c = constants;

// *********************** Bezier curve 3D *********************** //
int BCurve3D::degree() const {
  return points.get_n() - 1;
}

LiteMath::float3 BCurve3D::operator()(float u) const {
    return get_point(u);
}

BCurve3D::BCurve3D(
    std::vector<LiteMath::float3> points,
    float tmin, float tmax) : tmin(tmin), tmax(tmax) {
  assert(points.size() > 0);
  int p = points.size() - 1;
  this->points = Matrix2D<float3>(p + 1, p + 1);

  for (int j = 0; j <= p; j++) {
    auto index = std::make_pair(0, j);
    this->points[index] = points[j];
  }

  // Store just the differences of points
  // P(i, j) = P(i-1, j+1) - P(i-1, j)
  // without multiplying by (p-i+1)/(tmax-tmin) to avoid overflow
  for (int i = 1; i <= p; i++) {
    for (int j = 0; j <= p - i; j++) {
      auto index = std::make_pair(i,   j);
      auto left  = std::make_pair(i-1, j);
      auto right = std::make_pair(i-1, j+1);
      this->points[index] = this->points[right] - this->points[left];
    }
  }
}

LiteMath::float3 BCurve3D::get_point(float u) const {
  return der(u, 0);
}

LiteMath::float3 BCurve3D::der(float u, int order) const {
  u = ilerp(tmin, tmax, u);
  int p = degree();
  if (order < 0 || order > p)
    return float3(0.0f);

  float u_n = 1.0f;
  float _1_u = 1.0f - u;

  // Note: C(n, k) can store only up to n = 67 for 64-bit integer.
  // Critical no-overflow value appears for C(67, 33).
  long long comb = 1;
  auto points = this->points.row(order);
  p -= order;

  if (p == 0) {
    return points[0];
  }

  float3 res = points[0] * _1_u;
  for (int i = 1; i <= p-1; ++i) {
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

// Returns n-order derivative of F(u) = f'(u)g(u)-f(u)g'(u),
// where f(u) - numerator of RBezier and g(u) is the denominator.
// Complexity: O(n^2)
LiteMath::float3 RBCurve2D::fg_gf(float u, int order) const {
  assert(order >= 0);
  int n = degree();
  int m = order;

  // If m < n  => size(ders) = m + 2
  // If m >= n => size(ders) = 2n - m + 2
  // Therefore, size(ders) <= n + 2 and reaches that max when m = n
  float3 result = float3(0.0f);
  if (m < n) {
    //std::cout << "m < n\n";
    //int size = min(m, 2*n-m) + 2;
    std::vector<float3> ders(m + 2);

    for (int k = 0; k <= m + 1; k++) {
      ders[k] = BCurve3D::der(u, k); 
    }

    long long mcomb = 1;
    for (int k = 1; k <= m; k++) {
      mcomb = mcomb * (n-k+1) / k;
    }

    result = mcomb * (n * ders[1] * ders[m].z - (n-m) * ders[0] * ders[m+1].z);
    if (m == 0) return result;

    //k <= min(n, m) && k >= max(0, m - n)
    //k <= n && m - k <= n
    //k <= n && k >= m - n

    long long kcomb = 1;
    for (int k = 1; k <= m - 1; k++) {
      kcomb = kcomb * (n-k+1) / k;
      mcomb = mcomb * (m-k+1) / (n-m+k);
      float3 diff = (n-k) * ders[k+1] * ders[m-k].z - (n-m+k) * ders[k] * ders[m-k+1].z;
      result += kcomb * (mcomb * diff);
    }
    kcomb = kcomb * (n-m+1) / m;
    result += kcomb * ((n-m) * ders[m+1] * ders[0].z - n * ders[m] * ders[1].z);
  } else if (m >= n && m < 2*n) {
    //std::cout << "m >= n\n";
    //int size = min(m, 2*n-m) + 2;
    std::vector<float3> ders(2*n - m + 2);

    for (int k = m - n; k <= n + 1; k++) {
      ders[k-m+n] = BCurve3D::der(u, k);
      //if (order == 3)
      //  std::cout << k << " " << ders[k-m+n] << std::endl;
    }

    long long kcomb = 1;
    for (int k = 1; k <= m - n; k++) {
      kcomb = kcomb * (n-k+1) / k;
    }

    result = kcomb * (2*n-m) * ders[m-n+1-m+n] * ders[m-m+n-m+n].z;
    //                                  (k0-k0)+1   m
    //if (m == 2*n-1) return result;

    //k <= n && m - k <= n
    //k <= n && k >= m - n
    //k <= min(n, m) && k >= max(0, m - n)

    long long mcomb = 1;
    for (int k = m - n + 1; k <= n - 1; k++) {
      kcomb = kcomb * (n-k+1) / k;
      mcomb = mcomb * (m-k+1) / (n-m+k);
      float3 diff = (n-k) * ders[k+1-m+n] * ders[m-k-m+n].z - (n-m+k) * ders[k-m+n] * ders[m-k+1-m+n].z;
      result += kcomb * (mcomb * diff); 
    }
    mcomb = mcomb * (m-n+1) / (2*n-m); 

    result -= mcomb * (2*n-m) * ders[n-m+n] * ders[m-n+1-m+n].z;
    //if (order == 3)
    //  std::cout << " " << result << std::endl;
  }
  //std::cout << result << std::endl;
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

  long long comb = 1;
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
    return { tmin, tmax };
  }

  std::vector<float> result = { tmin };

  auto F = [&](float u) {
    return fg_gf(u, order)[axes];
  };

  // TODO: remove recursions
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

  if (!isclose(tmax, result.back(), c::BISECTION_EPS)) {
    result.push_back(tmax);
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
    //auto curve = RBCurve2D(Qw[i], knots[i], knots[i+1]);
    auto curve = RBCurve2D(Qw[i]);
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
      //std::swap(f1, f2);
    }
  }

  // I tried using error of function instead of error of l-r
  // (Don't do this)
  while (std::abs(l-r) > c::BISECTION_EPS) {
    float m = (l+r) / 2.0f;
    float value = f(m);
    if (value < 0.0f) {
      //std::cout << l << " " << r << std::endl;
      //std::cout << "LEFT " << f1 << " " << f2 << std::endl;
      l = m;
      //f1 = value;
    } else {
      //std::cout << l << " " << r << std::endl;
      //std::cout << "RIGHT  " << f1 << " " << f2 << std::endl;
      r = m;
      //f2 = value;
    }
  }

  return (l+r) / 2.0f;
}
