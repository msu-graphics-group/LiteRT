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
    while (b < m && U[b+1] == U[b]) b++;
    int mult = b-i+1;
    if (mult < p) {
      int numer = U[b]-U[a]; /* Numerator of alpha */
      /* Compute and store alphas */
      for (int j = p; j > mult; j--)
        alphas [j-mult-1] = numer/(U[a+j] -U[a]);
      int r = p-mult; /* Insert knot r times */
      for (int j= 1; j <= r; j++) {
        int save = r-j;
        int s = mult+j; /* This many new points */
        for (int k=p; k>=s; k--) {
          int alpha = alphas[k-s];
          Qw[nb][k] = alpha*Qw[nb][k] + (1.0f - alpha)*Qw[nb][k - 1];
        }
        if (b < m) /* Control point of */
          Qw [nb+1] [save] = Qw[nb][p]; /* next segment */
      }
    }
    nb = nb+1; /* Bezier segment completed */
    if (b < m) { 
      Qw.emplace_back(p+1); // new vector of size p+1
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

Vector2D<RBezier>
nurbs2rbezier(RawNURBS nurbs) {
  auto weighted_points = nurbs.points;
  for (int i = 0; i <= nurbs.get_n(); ++i)
  for (int j = 0; j <= nurbs.get_m(); ++j)
    weighted_points[{i, j}] = nurbs.points[{i, j}] * nurbs.weights[{i, j}];
  auto u_decomposed = decompose_surface(
      nurbs.get_n(), nurbs.get_p(), nurbs.u_knots.data(), 
      nurbs.get_m(), nurbs.get_q(), nurbs.v_knots.data(), 
      weighted_points, SurfaceParameter::U);
  Vector2D<RBezier> ans;
  for (int stripi = 0; stripi < u_decomposed.size(); ++stripi) {
    auto v_decomposed = decompose_surface(
      nurbs.get_n(), nurbs.get_p(), nurbs.u_knots.data(), 
      nurbs.get_m(), nurbs.get_q(), nurbs.v_knots.data(), 
      u_decomposed[stripi], SurfaceParameter::V);
    if (ans.shape2D() == std::make_pair(0u, 0u)) { // need to resize to correct shape
      ans = Vector2D<RBezier>(u_decomposed.size(), v_decomposed.size());
    }
    for (int stripj = 0; stripj < v_decomposed.size(); ++stripj)
      ans[{stripi, stripj}] = { .weighted_points = v_decomposed[stripj] };
  }
  return ans;
}