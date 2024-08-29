#include <vector>
#include <iostream>
#include <fstream>

#include <Image2d.h>

#include "Surface.hpp"

int find_span(int n, int p, float u, const float *U) {
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

void basis_funs(int i, float u, int p, const float *U, float *N) {
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

LiteMath::float4 surface_point(
    int n, int p, 
    const float *U, 
    int m, int q,
    const float *V,
    const Matrix2D<LiteMath::float4> &P,
    const Matrix2D<float> &W,
    float u, float v) {
  int uspan = find_span(n, p, u, U);
  std::vector<float> Nu(p+1);
  basis_funs(uspan, u, p, U, Nu.data());
  
  int vspan = find_span(m, q, v, V);
  std::vector<float> Nv(q+1);
  basis_funs(vspan, v, q, V, Nv.data());

  std::vector<LiteMath::float4> temp(q+1);
  for (int l = std::max(0, q-vspan); l <= q; ++l) 
  for (int k = std::max(0, p-uspan); k <= p; ++k) 
  {
    temp[l] += Nu[k] * P[{uspan-p+k, vspan-q+l}] * W[{uspan-p+k, vspan-q+l}];
  }

  LiteMath::float4 res = {};
  for (int l = 0; l <= q; ++l)
    res += Nv[l] * temp[l];
  
  return res/res.w;
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

  surf.v_knots.resize(m+surf.deg_v+2);
  fin >> tmp_str; // "v_knots:"
  for (size_t i = 0; i < surf.v_knots.size(); ++i) {
    fin >> surf.v_knots[i];
  }

  return surf;
}