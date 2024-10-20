#include <vector>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <functional>

#include <Image2d.h>

#include "Surface.hpp"

NURBS_Surface load_nurbs(const std::filesystem::path &path) {
  std::fstream fin;
  fin.exceptions(std::ios::failbit|std::ios::badbit);
  fin.open(path);
  std::string tmp_str;
  char tmp_chr;

  NURBS_Surface surf;

  int n, m;
  fin >> tmp_chr >> tmp_chr >> n; // n = ...
  fin >> tmp_chr >> tmp_chr >> m; // m = ...

  surf.points = Matrix2D<LiteMath::float4>(n+1, m+1, { 0, 0, 0, 1.0f });
  surf.weights = Matrix2D<float>(n+1, m+1, 1.0f);

  fin >> tmp_str; // "points:"
  for (int i = 0; i <= n; ++i)
  for (int j = 0; j <= m; ++j)
  {
    auto &point = surf.points[{i, j}];
    fin >> tmp_chr >> point.x >> tmp_chr >> point.y >> tmp_chr >> point.z >> tmp_chr; // { ..., ..., ... }
  }
  surf.bbox = BoundingBox3d(surf.points.data(), surf.points.get_n()*surf.points.get_m());

  fin >> tmp_str; // "weights:"
  for (int i = 0; i <= n; ++i)
  for (int j = 0; j <= m; ++j)
  {
    fin >> surf.weights[{i, j}];
  }

  fin >> tmp_str; // "u_degree:"
  fin >> surf.deg_u;

  fin >> tmp_str; // "v_degree:"
  fin >> surf.deg_v;

  surf.u_knots.resize(n+surf.deg_u+2);
  fin >> tmp_str; // "u_knots:"
  for (size_t i = 0; i < surf.u_knots.size(); ++i) {
    fin >> surf.u_knots[i];
  }
  float u_min = surf.u_knots.front();
  float u_max = surf.u_knots.back();
  for (auto &elem: surf.u_knots)
    elem = (elem-u_min)/u_max;

  surf.v_knots.resize(m+surf.deg_v+2);
  fin >> tmp_str; // "v_knots:"
  for (size_t i = 0; i < surf.v_knots.size(); ++i) {
    fin >> surf.v_knots[i];
  }
  float v_min = surf.v_knots.front();
  float v_max = surf.v_knots.back();
  for (auto &elem: surf.v_knots)
    elem = (elem-v_min)/v_max;

  return surf;
}

std::vector<std::vector<LiteMath::float4> >
decompose_curve(
    int n, int p,
    const float *U,
    const LiteMath::float4 *Pw) {
  int m = n+p+1;
  int a = p;
  int b = p+1;
  int nb = 0;
  std::vector<std::vector<LiteMath::float4> > Qw(1, std::vector<LiteMath::float4>(p+1));
  std::vector<float> alphas(p+1);
  for (int i=0; i <= p; ++i) 
    Qw[nb][i] = Pw[i];
  while (b < m) {
    int i = b;
    while (b < m && U[b+1] == U[b]) 
      b++;
    int mult = b-i+1;
    if (mult < p) {
      float numer = U[b]-U[a]; /* Numerator of alpha */
      /* Compute and store alphas */
      for (int j = p; j > mult; j--)
        alphas [j-mult-1] = numer/(U[a+j] - U[a]);
      int r = p-mult; /* Insert knot r times */
      for (int j= 1; j <= r; j++) {
        int save = r-j;
        int s = mult+j; /* This many new points */
        for (int k=p; k>=s; k--) {
          float alpha = alphas[k-s];
          Qw[nb][k] = alpha*Qw[nb][k] + (1.0f - alpha)*Qw[nb][k - 1];
        }
        if (b < m) { /* Control point of */ 
          if (Qw.size() == nb+1)
            Qw.emplace_back(p+1);
          Qw [nb+1] [save] = Qw[nb][p]; /* next segment */
        }
      }
    }
    nb = nb+1; /* Bezier segment completed */
    if (b < m) { 
      if (Qw.size() == nb)
        Qw.emplace_back(p+1);
      /* Initialize for next segment */
      for (i=p-mult; i<=p; i++) 
        Qw[nb][i] = Pw[b-p+i];
      a = b;
      b = b+1;
    }
  }
  return Qw;
}

using namespace LiteMath;

std::vector<Matrix2D<LiteMath::float4> >
decompose_surface(
    int n, int p,
    const float *U,
    int m,int q,
    const float *V,
    const Matrix2D<float4> &Pw,
    SurfaceParameter dir) {
  if (dir == SurfaceParameter::U) {
    std::vector<Matrix2D<float4>> Qw;
    for (int coli = 0; coli <= m; ++coli) {
      std::vector<float4> col(n+1);
      for (int rowi = 0; rowi <= n; ++rowi)
        col[rowi] = Pw[{rowi, coli}];

      auto decomposed_col = decompose_curve(n, p, U, col.data());
      Qw.resize(decomposed_col.size(), Matrix2D<float4>(p+1, m+1));

      for (int stripi = 0; stripi < Qw.size(); ++stripi)
      for (int rowi = 0; rowi <= p; ++rowi)
        Qw[stripi][{rowi, coli}] = decomposed_col[stripi][rowi];
    }
    return Qw;
  } else {
    std::vector<Matrix2D<float4>> Qw;
    for (int rowi = 0; rowi <= n; ++rowi) {
      auto decomposed_row = decompose_curve(m, q, V, &Pw[{rowi, 0}]);
      Qw.resize(decomposed_row.size(), Matrix2D<float4>(n+1, q+1));

      for (int stripi = 0; stripi < Qw.size(); ++stripi)
      for (int coli = 0; coli <= q; ++coli)
        Qw[stripi][{rowi, coli}] = decomposed_row[stripi][coli];
    }
    return Qw;
  }
}

RBezierGrid
nurbs2rbezier(const NURBS_Surface &nurbs) {
  int n = nurbs.points.get_n()-1;
  int m = nurbs.points.get_m()-1;
  int p = nurbs.deg_u;
  int q = nurbs.deg_v;

  auto weighted_points = nurbs.points;
  for (int i = 0; i <= n; ++i)
  for (int j = 0; j <= m; ++j)
    weighted_points[{i, j}] *= nurbs.weights[{i, j}];

  auto u_decomposed = decompose_surface(
      n, p, nurbs.u_knots.data(), 
      m, q, nullptr, 
      weighted_points, SurfaceParameter::U);

  Matrix2D<RBezier> ans;
  for (int stripi = 0; stripi < u_decomposed.size(); ++stripi) {
    auto v_decomposed = decompose_surface(
      p, p, nullptr, 
      m, q, nurbs.v_knots.data(), 
      u_decomposed[stripi], SurfaceParameter::V);

    if (ans.get_n() == 0) { // need to resize to correct shape
      ans = Matrix2D<RBezier>(u_decomposed.size(), v_decomposed.size());
    }

    for (int stripj = 0; stripj < v_decomposed.size(); ++stripj)
      ans[{stripi, stripj}] = { v_decomposed[stripj] };
  }
  
  auto u_knots = nurbs.u_knots;
  auto v_knots = nurbs.v_knots;
  u_knots.resize(std::unique(u_knots.begin(), u_knots.end())-u_knots.begin());
  v_knots.resize(std::unique(v_knots.begin(), v_knots.end())-v_knots.begin());
  assert(u_knots.size()-1 == ans.get_n() && v_knots.size()-1 == ans.get_m());

  return RBezierGrid{ u_knots, v_knots, ans, nurbs.bbox };
}


LiteMath::float4 rbezier_curve_point(
    float u,
    int p,
    const float4 *pw) {
  float u_n = 1.0f;
  float _1_u = 1.0f - u;
  int bc = 1.0f;
  float4 res = pw[0] * _1_u;
  for (int i = 1; i <= p-1; ++i) {
    u_n *= u;
    bc = bc * (p-i+1)/i;
    res = (res + u_n * bc * pw[i]) * _1_u;
  }
  res += (u_n * u) * pw[p];
  return res;
}

LiteMath::float4 rbezier_curve_der(
    float u,
    int p,
    const float4 *pw) {
  float u_n = 1.0f;
  float _1_u = 1.0f - u;
  int bc = 1.0f;
  float4 res = (pw[1]-pw[0]) * _1_u;
  for (int i = 1; i <= p-2; ++i) {
    u_n *= u;
    bc = bc * (p-i+1)/i;
    res = (res + u_n * bc * (pw[i+1]-pw[i])) * _1_u;
  }
  res += (u_n * u) * (pw[p]-pw[p-1]);

  res *= p;
  return res;
}


LiteMath::float4 RBezier::get_point(float u, float v) const {
  int n = weighted_points.get_n()-1;
  int m = weighted_points.get_m()-1;
  float u_n = 1.0f;
  float _1_u = 1.0f - u;
  int bc = 1.0f;
  float4 res = rbezier_curve_point(v, m, &weighted_points[{0, 0}]) * _1_u;
  for (int i = 1; i <= n-1; ++i)
  {
    u_n *= u;
    bc = bc * (n-i+1)/i;
    float4 curve_point = rbezier_curve_point(v, m, &weighted_points[{i, 0}]);
    res = (res + u_n * bc * curve_point) * _1_u;
  }
  res += (u_n*u) * rbezier_curve_point(v, m, &weighted_points[{n, 0}]);
  return res;
}

LiteMath::float4 RBezier::vder(float u, float v) const {
  float4 Sw = get_point(u, v);

  int n = weighted_points.get_n()-1;
  int m = weighted_points.get_m()-1;
  float u_n = 1.0f;
  float _1_u = 1.0f - u;
  int bc = 1.0f;
  float4 Sw_der = rbezier_curve_der(v, m, &weighted_points[{0, 0}]) * _1_u;
  for (int i = 1; i <= n-1; ++i)
  {
    u_n *= u;
    bc = bc * (n-i+1)/i;
    float4 curve_der = rbezier_curve_der(v, m, &weighted_points[{i, 0}]);
    Sw_der = (Sw_der + u_n * bc * curve_der) * _1_u;
  }
  Sw_der += (u_n*u) * rbezier_curve_der(v, m, &weighted_points[{n, 0}]);
  
  float4 res = (Sw_der * Sw.w - Sw * Sw_der.w)/(Sw.w * Sw.w);
  return res;
}

LiteMath::float4 RBezier::uder(float u, float v) const {
  float4 Sw = get_point(u, v);

  int n = weighted_points.get_n()-1;
  int m = weighted_points.get_m()-1;
  float u_n = 1.0f;
  float _1_u = 1.0f - u;
  int bc = 1.0f;
  
  float4 Sw_der = std::invoke([&]() {
    float4 next = rbezier_curve_point(v, m, &weighted_points[{1, 0}]);
    float4 cur = rbezier_curve_point(v, m, &weighted_points[{0, 0}]);
    return (next-cur) * _1_u;
  });

  for (int i = 1; i <= n-2; ++i)
  {
    u_n *= u;
    bc = bc * (n-i+1)/i;
    float4 next = rbezier_curve_point(v, m, &weighted_points[{i+1, 0}]);
    float4 cur = rbezier_curve_point(v, m, &weighted_points[{i, 0}]);
    Sw_der = (Sw_der + u_n * bc * (next-cur)) * _1_u;
  }

  Sw_der += std::invoke([&]() {
    float4 next = rbezier_curve_point(v, m, &weighted_points[{n, 0}]);
    float4 cur = rbezier_curve_point(v, m, &weighted_points[{n-1, 0}]);
    return (u_n*u) * (next-cur);
  });

  Sw_der *= n;
  
  float4 res = (Sw_der * Sw.w - Sw * Sw_der.w)/(Sw.w * Sw.w);
  return res;
}

static
int find_span(float u, int sz, const float *U)
{
  if (u == U[sz-1])
    return sz-2;

  int l = 0;
  int r = sz-1;
  while(r-l > 1) {
    int m = (l+r)/2;
    if (u < U[m])
      r = m;
    else 
      l = m;
  }

  assert(U[l] <= u && u < U[l+1]);
  return l;
}

LiteMath::float4 RBezierGrid::get_point(float u, float v) const {
  int uspan = find_span(u, uniq_uknots.size(), uniq_uknots.data());
  int vspan = find_span(v, uniq_vknots.size(), uniq_vknots.data());
  float umin = uniq_uknots[uspan], umax = uniq_uknots[uspan+1];
  float vmin = uniq_vknots[vspan], vmax = uniq_vknots[vspan+1];
  u = (u-umin)/(umax-umin);
  v = (v-vmin)/(vmax-vmin);
  assert(0 <= u && u <= 1);
  assert(0 <= v && v <= 1);
  return grid[{uspan, vspan}].get_point(u, v);
}

LiteMath::int2 RBezierGrid::get_spans(float u, float v) const {
  int uspan = find_span(u, uniq_uknots.size(), uniq_uknots.data());
  int vspan = find_span(v, uniq_vknots.size(), uniq_vknots.data());
  return int2{ uspan, vspan };
}

LiteMath::float4 RBezierGrid::uder(float u, float v) const {
  int uspan = find_span(u, uniq_uknots.size(), uniq_uknots.data());
  int vspan = find_span(v, uniq_vknots.size(), uniq_vknots.data());
  float umin = uniq_uknots[uspan], umax = uniq_uknots[uspan+1];
  float vmin = uniq_vknots[vspan], vmax = uniq_vknots[vspan+1];
  u = (u-umin)/(umax-umin);
  v = (v-vmin)/(vmax-vmin);
  assert(0 <= u && u <= 1);
  assert(0 <= v && v <= 1);
  float4 res =  grid[{uspan, vspan}].uder(u, v) / (umax - umin);
  return res;
}

LiteMath::float4 RBezierGrid::vder(float u, float v) const {
  int uspan = find_span(u, uniq_uknots.size(), uniq_uknots.data());
  int vspan = find_span(v, uniq_vknots.size(), uniq_vknots.data());
  float umin = uniq_uknots[uspan], umax = uniq_uknots[uspan+1];
  float vmin = uniq_vknots[vspan], vmax = uniq_vknots[vspan+1];
  u = (u-umin)/(umax-umin);
  v = (v-vmin)/(vmax-vmin);
  assert(0 <= u && u <= 1);
  assert(0 <= v && v <= 1);
  float4 res =  grid[{uspan, vspan}].vder(u, v) / (vmax - vmin);
  return res;
}

LiteMath::float3 RBezierGrid::normal(float u, float v) const {
  float4 uderiv = uder(u, v);
  float4 vderiv = vder(u, v);
  float3 normal = normalize(cross(to_float3(uderiv), to_float3(vderiv)));
  return normal;
}


#include <step_parser.h>
std::vector<RBezierGrid>
load_rbeziers(const std::filesystem::path &path) {
  if (path.extension() == ".nurbss") {
    return { nurbs2rbezier(load_nurbs(path)) };
  } 

  assert(path.extension() == ".step");

  std::vector<RBezierGrid> res;
  auto parsed_nurbs = STEP::allNURBS(STEP::parse(path));
  for (auto *p_raw_nurbs: parsed_nurbs) {
    auto [n, m] = p_raw_nurbs->points.shape2D();
    int p = p_raw_nurbs->u_knots.size()-n-1;
    int q = p_raw_nurbs->v_knots.size()-m-1;
    Matrix2D<float4> points(n, m);
    Matrix2D<float> weights(n, m);
    std::vector<float> u_knots = p_raw_nurbs->u_knots;
    float umin = u_knots.front(), umax = u_knots.back();
    for (float &knot: u_knots)
      knot = (knot - umin)/(umax-umin);
    std::vector<float> v_knots = p_raw_nurbs->v_knots;
    float vmin = v_knots.front(), vmax = v_knots.back();
    for (float &knot: v_knots)
      knot = (knot - vmin)/(vmax-vmin);
    std::copy(p_raw_nurbs->points.data(), p_raw_nurbs->points.data()+n*m, points.data());
    std::copy(p_raw_nurbs->weights.data(), p_raw_nurbs->weights.data()+n*m, weights.data());
    NURBS_Surface surf(
      points, weights, 
      p, q,
      u_knots,
      v_knots);
    res.push_back(nurbs2rbezier(surf));
  }

  return res;
}

