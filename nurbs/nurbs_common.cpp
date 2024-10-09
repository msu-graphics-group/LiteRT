#include <cassert>
#include <random>
#include <ctime>

#include "nurbs_common.h"

using namespace LiteMath;

#ifndef KERNEL_SLICER

int DecomposedNURBS::find_span(int n, int p, float u, const float *U) const {
  if (u == U[n+1])
    return n;

  int l = p-1;
  int r = n+2;
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

void DecomposedNURBS::basis_funs(int i, float u, int p, const float *U, float *N) const {
  N[0] = 1.0f;
  std::vector<float> left(p+1, 0);
  std::vector<float> right(p+1, 0);
  for (int j = 1; j <= p; ++j) {
    left[j] = u - U[i+1-j];
    right[j] = U[i+j]-u;
    float saved = 0.0f;
    for (int r = 0; r < j; ++r) {
      float temp = N[r] / (right[r+1]+left[j-r]);
      N[r] = saved + right[r+1] * temp;
      saved = left[j-r] * temp;
    }
    N[j] = saved;
  }
}

LiteMath::float4 DecomposedNURBS::surface_point(float u, float v) const {
  auto [_, n, m, p, q] = header;
  int uspan = find_span(n, p, u, u_knots);
  std::vector<float> Nu(p+1);
  basis_funs(uspan, u, p, u_knots, Nu.data());
  
  int vspan = find_span(m, q, v, v_knots);
  std::vector<float> Nv(q+1);
  basis_funs(vspan, v, q, v_knots, Nv.data());

  std::vector<float4> temp(q+1);
  for (int l = 0; l <= q; ++l) 
  for (int k = 0; k <= p; ++k) 
  {
    temp[l] += Nu[k] * point(uspan-p+k, vspan-q+l) * w(uspan-p+k, vspan-q+l);
  }

  float4 res = {};
  for (int l = 0; l <= q; ++l)
    res += Nv[l] * temp[l];
  
  return res/res.w;
}

//TODO calculate accurate derivatives
LiteMath::float4 DecomposedNURBS::uder(float u, float v) const {
    constexpr float EPS = 1e-2f;
  LiteMath::float4 res = {};
  res += (u+EPS > 1.0f) ? surface_point(u, v) : surface_point(u+EPS, v);
  res -= (u-EPS < 0.0f) ? surface_point(u, v) : surface_point(u-EPS, v);

  return res / (EPS * (1 + ((u+EPS <= 1.0f) && (u-EPS >= 0.0f))));
}

//TODO calculate accurate derivatives 
LiteMath::float4 DecomposedNURBS::vder(float u, float v) const {
    constexpr float EPS = 1e-2f;
  LiteMath::float4 res = {};
  res += (v+EPS > 1.0f) ? surface_point(u, v) : surface_point(u, v+EPS);
  res -= (v-EPS < 0.0f) ? surface_point(u, v) : surface_point(u, v-EPS);

  return res / (EPS * (1 + ((v+EPS <= 1.0f) && (v-EPS >= 0.0f))));
}

bool DecomposedNURBS::u_closed() const {
  constexpr float EPS = 1e-2;
  for (int j = 0; j <= header.m; ++j) 
    if (length(point(0, j) - point(header.n, j)) > EPS)
      return false;
  return true;
}

bool DecomposedNURBS::v_closed() const {
  constexpr float EPS = 1e-2;
  for (int i = 0; i <= header.n; ++i) 
    if (length(point(i, 0) - point(i, header.m)) > EPS)
      return false;
  return true;
}

float closed_clamp(float val) {
  val -= static_cast<int>(val);
  if (val < 0.0f)
    return 1.0f+val;
  if (val > 1.0f)
    return val-1.0f;
  return val;
}

static float2
mul2x2x2(float2 m[2], const float2 &v)
{
  return m[0]*v[0]+m[1]*v[1];
}

std::mt19937 generator;

std::optional<LiteMath::float2> ray_nurbs_newton_intersection(
    const LiteMath::float3 &pos,
    const LiteMath::float3 &ray,
    const DecomposedNURBS &surf) {
  constexpr int max_steps = 16;
  constexpr float EPS = 0.001f;

  bool u_closed = surf.u_closed();
  bool v_closed = surf.v_closed();

  float3 ortho_dir1 = (ray.x || ray.z) ? float3{ 0, 1, 0 } : float3{ 1, 0, 0 };
  float3 ortho_dir2 = normalize(cross(ortho_dir1, ray));
  ortho_dir1 = normalize(cross(ray, ortho_dir2));
  assert(dot(ortho_dir1, ortho_dir2) < EPS);
  assert(dot(ortho_dir1, ray) < EPS);
  assert(dot(ortho_dir2, ray) < EPS);

  float4 P1 = to_float4(ortho_dir1, -dot(ortho_dir1, pos));
  float4 P2 = to_float4(ortho_dir2, -dot(ortho_dir2, pos));
  assert(dot(P1, to_float4(pos, 1.0f)) < EPS);
  assert(dot(P2, to_float4(pos, 1.0f)) < EPS);

  std::uniform_real_distribution<float> distr(0.0f, 1.0f);
  float2 uv = { distr(generator), distr(generator) };
  float2 D = { dot(P1, surf.surface_point(uv.x, uv.y)), dot(P2, surf.surface_point(uv.x, uv.y)) };
  
  int steps_left = max_steps-1;
  while(length(D) > EPS && steps_left--) {
    float2 J[2] = 
    { 
      { dot(P1, surf.uder(uv.x, uv.y)), dot(P2, surf.uder(uv.x, uv.y)) }, //col1
      { dot(P1, surf.vder(uv.x, uv.y)), dot(P2, surf.vder(uv.x, uv.y)) } //col2
    };

    float det = J[0][0]*J[1][1] - J[0][1] * J[1][0];

    float2 J_inversed[2] = 
    {
      { J[1][1]/det, -J[0][1]/det },
      { -J[1][0]/det, J[0][0]/det }
    };

    uv = uv - mul2x2x2(J_inversed, D);
    uv.x = u_closed ? closed_clamp(uv.x) : clamp(uv.x, 0.0f, 1.0f);
    uv.y = v_closed ? closed_clamp(uv.y) : clamp(uv.y, 0.0f, 1.0f);
    assert(0 <= uv.x && uv.x <= 1);
    assert(0 <= uv.y && uv.y <= 1);

    float2 new_D = { dot(P1, surf.surface_point(uv.x, uv.y)), dot(P2, surf.surface_point(uv.x, uv.y)) };
    
    if (length(new_D) > length(D))
      return {};
    
    D = new_D;
  }

  if (length(D) > EPS)
    return {};
  
  return uv;
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


#endif