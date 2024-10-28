#include <cassert>
#include <fstream>
#include "nurbs_common_host.h"

using namespace LiteMath;

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

std::vector<Vector2D<LiteMath::float4> >
decompose_surface(
    int n, int p,
    const float *U,
    int m,int q,
    const float *V,
    const Vector2D<float4> &Pw,
    SurfaceParameter dir) {
  if (dir == SurfaceParameter::U) {
    std::vector<Vector2D<float4>> Qw;
    for (int col_idx = 0; col_idx <= m; ++col_idx) {
      std::vector<float4> col(n+1);
      for (int row_idx = 0; row_idx <= n; ++row_idx)
        col[row_idx] = Pw[{row_idx, col_idx}];
      auto col_qw = decompose_curve(n, p, U, col.data());
      Qw.resize(col_qw.size(), Vector2D<float4>(p+1, m+1));
      for (int stripi = 0; stripi < Qw.size(); ++stripi) 
      for (int rowi = 0; rowi < p+1; ++rowi)
      {
        Qw[stripi][{rowi, col_idx}] = col_qw[stripi][rowi];
      }
    }
    return Qw;
  } else {
    std::vector<Vector2D<float4>> Qw;
    for (int row_idx = 0; row_idx < Pw.shape2D().first; ++row_idx) {
      auto row_qw = decompose_curve(m, q, V, &Pw[{row_idx, 0}]);
      Qw.resize(row_qw.size(), Vector2D<float4>(p+1, q+1));
      for (int stripi = 0; stripi < Qw.size(); ++stripi) 
      for (int coli = 0; coli < q+1; ++coli)
      {
        Qw[stripi][{row_idx, coli}] = row_qw[stripi][coli];
      }
    }
    return Qw;
  }
}

RBezierGrid
nurbs2rbezier(RawNURBS nurbs) {
  int n = nurbs.get_n();
  int m = nurbs.get_m();
  int p = nurbs.get_p();
  int q = nurbs.get_q();

  auto weighted_points = nurbs.points;
  for (int i = 0; i <= n; ++i)
  for (int j = 0; j <= m; ++j)
    weighted_points[{i, j}] *= nurbs.weights[{i, j}];

  auto u_decomposed = decompose_surface(
      n, p, nurbs.u_knots.data(), 
      m, q, nullptr, 
      weighted_points, SurfaceParameter::U);

   Vector2D<RBezier> ans;
  for (int stripi = 0; stripi < u_decomposed.size(); ++stripi) {
    auto v_decomposed = decompose_surface(
      p, p, nullptr, 
      m, q, nurbs.v_knots.data(), 
      u_decomposed[stripi], SurfaceParameter::V);

    if (ans.shape2D() == std::make_pair(0u, 0u)) { // need to resize to correct shape
      ans = Vector2D<RBezier>(u_decomposed.size(), v_decomposed.size());
    }

    for (int stripj = 0; stripj < v_decomposed.size(); ++stripj)
      ans[{stripi, stripj}] = { v_decomposed[stripj] };
  }
  
  auto u_knots = nurbs.u_knots;
  auto v_knots = nurbs.v_knots;
  u_knots.resize(std::unique(u_knots.begin(), u_knots.end())-u_knots.begin());
  v_knots.resize(std::unique(v_knots.begin(), v_knots.end())-v_knots.begin());
  assert((ans.shape2D() == 
      std::make_pair<uint32_t, uint32_t>(u_knots.size()-1, v_knots.size()-1)));

  return RBezierGrid{ u_knots, v_knots, ans, nurbs.bbox };
}

RawNURBS load_nurbs (const std::filesystem::path &path) {
  std::fstream fin(path);
  fin.exceptions(std::ifstream::badbit|std::ifstream::failbit);
  std::string tmp_str;
  char tmp_chr;

  RawNURBS surf;

  int n, m, p, q;
  fin >> tmp_chr >> tmp_chr >> n; // n = ...
  fin >> tmp_chr >> tmp_chr >> m; // m = ...

  surf.points = Vector2D<LiteMath::float4>(n+1, m+1, { 0, 0, 0, 1.0f });
  surf.weights = Vector2D<float>(n+1, m+1, 1.0f);

  fin >> tmp_str; // "points:"
  for (int i = 0; i <= n; ++i)
  for (int j = 0; j <= m; ++j)
  {
    auto &point = surf.points[{i, j}];
    // { ..., ..., ... }
    fin >> tmp_chr >> point.x >> tmp_chr >> point.y >> tmp_chr >> point.z >> tmp_chr; 
    surf.bbox.boxMin = LiteMath::min(surf.bbox.boxMin, point);
    surf.bbox.boxMin = LiteMath::max(surf.bbox.boxMin, point);
  }

  fin >> tmp_str; // "weights:"
  for (int i = 0; i <= n; ++i)
  for (int j = 0; j <= m; ++j)
  {
    fin >> surf.weights[{i, j}];
  }

  fin >> tmp_str; // "u_degree:"
  fin >> p; 

  fin >> tmp_str; // "v_degree:"
  fin >> q;

  surf.u_knots.resize(n+p+2);
  fin >> tmp_str; // "u_knots:"
  for (size_t i = 0; i < surf.u_knots.size(); ++i) {
    fin >> surf.u_knots[i];
  }
  float u_min = surf.u_knots.front();
  float u_max = surf.u_knots.back();
  for (auto &elem: surf.u_knots)
    elem = (elem-u_min)/(u_max-u_min); // map to [0, 1]

  surf.v_knots.resize(m+q+2);
  fin >> tmp_str; // "v_knots:"
  for (size_t i = 0; i < surf.v_knots.size(); ++i) {
    fin >> surf.v_knots[i];
  }
  float v_min = surf.v_knots.front();
  float v_max = surf.v_knots.back();
  for (auto &elem: surf.v_knots)
    elem = (elem-v_min)/(v_max-v_min); // map to [0, 1]

  return surf;
}

