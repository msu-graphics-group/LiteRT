#include <vector>
#include <iostream>
#include <fstream>

#include <Image2d.h>

#include "Surface.hpp"

int SurfaceView::find_span(int n, int p, float u, const float *U) const {
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

void SurfaceView::basis_funs(int i, float u, int p, const float *U, float *N) const {
  N[0] = 1.0f;
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

LiteMath::float4 SurfaceView::get_point(float u, float v) const {
  const float *U = p_surf->u_knots.data();
  const float *V = p_surf->v_knots.data();

  int uspan = find_span(n(), p(), u, U);
  basis_funs(uspan, u, p(), U, Nu.data());
  
  int vspan = find_span(m(), q(), v, V);
  basis_funs(vspan, v, q(), V, Nv.data());

  std::fill(temp.begin(), temp.end(), LiteMath::float4{0.0f});
  for (int l = 0; l <= q(); ++l) 
  for (int k = 0; k <= p(); ++k) 
  {
    temp[l] += Nu[k] * (p_surf->points[{uspan-p()+k, vspan-q()+l}])
                * (p_surf->weights[{uspan-p()+k, vspan-q()+l}]);
  }

  LiteMath::float4 res = {};
  for (int l = 0; l <= q(); ++l)
    res += Nv[l] * temp[l];
  
  return res/res.w;
}

LiteMath::float4 SurfaceView::uderivative(float u, float v) const {
  constexpr float EPS = 1e-2f;
  LiteMath::float4 res = {};
  res += (u+EPS > 1.0f) ? get_point(u, v) : get_point(u+EPS, v);
  res -= (u-EPS < 0.0f) ? get_point(u, v) : get_point(u-EPS, v);

  return res / (EPS * (1 + ((u+EPS <= 1.0f) && (u-EPS >= 0.0f))));
}

LiteMath::float4 SurfaceView::vderivative(float u, float v) const {
  constexpr float EPS = 1e-2f;
  LiteMath::float4 res = {};
  res += (v+EPS > 1.0f) ? get_point(u, v) : get_point(u, v+EPS);
  res -= (v-EPS < 0.0f) ? get_point(u, v) : get_point(u, v-EPS);

  return res / (EPS * (1 + ((v+EPS <= 1.0f) && (v-EPS >= 0.0f))));
}

LiteMath::float3 SurfaceView::get_normal(float u, float v) const {
  return LiteMath::normalize(LiteMath::cross(
        LiteMath::to_float3(uderivative(u, v)),
        LiteMath::to_float3(vderivative(u, v))));
}

bool SurfaceView::u_closed() const {
  constexpr float EPS = 1e-2;
    for (int j = 0; j <= m(); ++j) 
      if (length(p_surf->points[{0, j}] - p_surf->points[{n(), j}]) > EPS)
        return false;
    return true;
}

bool SurfaceView::v_closed() const {
  constexpr float EPS = 1e-2;
    for (int i = 0; i <= n(); ++i) 
      if (length(p_surf->points[{i, 0}] - p_surf->points[{i, m()}]) > EPS)
        return false;
    return true;
}

Surface load_surface(const std::filesystem::path &path) {
  std::fstream fin(path);
  std::string tmp_str;
  char tmp_chr;

  Surface surf;

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