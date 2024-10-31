#include <cassert>
#include <fstream>
#include <array>
#include "nurbs_common_host.h"

using namespace LiteMath;

std::vector<std::vector<LiteMath::float4> >
decompose_curve(
    int n, int p,
    const float *U,
    StrideView<const float4> Pw) {
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
    for (int col_idx = 0; col_idx <= m; ++col_idx) {
      auto col_qw = decompose_curve(n, p, U, Pw.col(col_idx));
      Qw.resize(col_qw.size(), Matrix2D<float4>(p+1, m+1));
      for (int stripi = 0; stripi < Qw.size(); ++stripi) 
      for (int rowi = 0; rowi < p+1; ++rowi)
      {
        Qw[stripi][{rowi, col_idx}] = col_qw[stripi][rowi];
      }
    }
    return Qw;
  } else {
    std::vector<Matrix2D<float4>> Qw;
    for (int row_idx = 0; row_idx < Pw.shape2D().first; ++row_idx) {
      auto row_qw = decompose_curve(m, q, V, Pw.row(row_idx));
      Qw.resize(row_qw.size(), Matrix2D<float4>(p+1, q+1));
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

   Matrix2D<RBezier> ans;
  for (int stripi = 0; stripi < u_decomposed.size(); ++stripi) {
    auto v_decomposed = decompose_surface(
      p, p, nullptr, 
      m, q, nurbs.v_knots.data(), 
      u_decomposed[stripi], SurfaceParameter::V);

    if (ans.shape2D() == std::make_pair(0u, 0u)) { // need to resize to correct shape
      ans = Matrix2D<RBezier>(u_decomposed.size(), v_decomposed.size());
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

std::array<std::vector<float4>, 2>
de_castelaju_divide_curve(
    int p, float u,
    StrideView<const float4> Pw) {
  std::vector<float4> tmp(p+1);
  std::array<std::vector<float4>, 2> res = { 
    std::vector<float4>(p+1),
    std::vector<float4>(p+1)
  };
  for (int i = 0; i <= p; ++i)
    tmp[i] = Pw[i];
  for (int i = 0; i <= p; ++i) {
    res[0][i] = tmp[0];
    res[1][p-i] = tmp[p-i];
    for (int j = 0; j <= p-i-1; ++j)
      tmp[j] += (tmp[j+1]-tmp[j])*u;
  }
  return res;
}

constexpr float div_constant = 1.0f;
int flatteing_div_count(
    int p,
    StrideView<const float4> Pw) {
  if (p == 1)
    return 0;

  float avg_v = 0.0f;
  float mx_a = 0.0f;
  
  for (int i = 0; i <= p-1; ++i) {
    float3 p0 = to_float3(Pw[i]/Pw[i].w);
    float3 p1 = to_float3(Pw[i+1]/Pw[i+1].w);
    avg_v += p * length(p1-p0);
  }
  avg_v /= p;

  for (int i = 0; i <= p-2; ++i) {
    float3 p0 = to_float3(Pw[i]/Pw[i].w);
    float3 p1 = to_float3(Pw[i+1]/Pw[i+1].w);
    float3 p2 = to_float3(Pw[i+2]/Pw[i+2].w);
    float3 res_p = p0 - 2*p1 + p2;
    mx_a = std::max(mx_a, length(res_p) * p * (p-1));
  }

  return static_cast<int>(div_constant * mx_a / std::sqrt(avg_v));
}

std::vector<std::vector<float4>>
de_castelaju_uniformly_divide_curve(
    int p, int div_count,
    StrideView<const float4> control_points) {
  std::vector<float4> Pw(p+1);
  for (int i = 0; i <= p; ++i)
    Pw[i] = control_points[i];
  if (div_count == 0) {
    return { Pw };
  }
  std::vector<std::vector<float4>> res(div_count+1, std::vector<float4>(p+1));
  for (int i = 0; i < div_count; ++i) {
    float u = 1.0f/(div_count+1)*(i+1);
    auto [res1, res2] = de_castelaju_divide_curve(p, u, StrideView{ Pw.data(), 1 });
    res[i] = res1;
    res[i+1] = res2;
    Pw = std::move(res2);
  }

  return res;
}

std::vector<Matrix2D<LiteMath::float4> >
decompose_rbezier(
    int p, int q,
    const Matrix2D<float4> &Pw,
    int divs_count, SurfaceParameter dir) {
  if (dir == SurfaceParameter::U) {
    std::vector<Matrix2D<float4>> Qw(divs_count+1, Matrix2D<float4>(p+1, q+1));
    for (int coli = 0; coli <= q; ++coli) {
      auto decomposed_col = de_castelaju_uniformly_divide_curve(
          p, divs_count, Pw.col(coli));
      for (int stripi = 0; stripi < Qw.size(); ++stripi)
      for (int rowi = 0; rowi <= p; ++rowi)
        Qw[stripi][{rowi, coli}] = decomposed_col[stripi][rowi];
    }
    return Qw;
  } else {
    std::vector<Matrix2D<float4>> Qw(divs_count+1, Matrix2D<float4>(p+1, q+1));
    for (int rowi = 0; rowi <= p; ++rowi) {
      auto decomposed_row = de_castelaju_uniformly_divide_curve(
          q, divs_count, Pw.row(rowi));
      for (int stripi = 0; stripi < Qw.size(); ++stripi)
      for (int coli = 0; coli <= q; ++coli)
        Qw[stripi][{rowi, coli}] = decomposed_row[stripi][coli];
    }
    return Qw;
  }
}

static Box4f calc_bbox(const float4 *Pw, size_t count) {
  Box4f ans;
  for (int i = 0; i < count; ++i) {
    float4 point = Pw[i]/Pw[i].w;
    ans.boxMin = min(ans.boxMin, point);
    ans.boxMax = min(ans.boxMax, point);
  }
  return ans;
}

std::tuple<std::vector<LiteMath::Box4f>, std::vector<float2>>
get_bvh_leaves(const RBezier &rbezier, float2 ubounds, float2 vbounds) {
  int p = rbezier.weighted_points.rows_count()-1;
  int q = rbezier.weighted_points.cols_count()-1;

  int udivs = 0;
  int vdivs = 0;
  for (int ri = 0; ri <= p; ++ri) {
    int cur = flatteing_div_count(q, rbezier.weighted_points.row(ri));
    vdivs = std::max(cur, vdivs);
  }
  for (int ci = 0; ci <= q; ++ci) {
    int cur = flatteing_div_count(p, rbezier.weighted_points.col(ci));
    udivs = std::max(cur, udivs);
  }

  auto u_decomposed = decompose_rbezier(
      p, q, rbezier.weighted_points, udivs, SurfaceParameter::U);

  Matrix2D<RBezier> ans(udivs+1, vdivs+1);
  std::vector<Box4f> ans_boxes;
  std::vector<float2> ans_uv;
  for (int stripi = 0; stripi < u_decomposed.size(); ++stripi) {
    auto v_decomposed = decompose_rbezier(
      p, q,
      u_decomposed[stripi], 
      vdivs, SurfaceParameter::V);
    for (int stripj = 0; stripj < v_decomposed.size(); ++stripj) {
      ans[{stripi, stripj}] = { v_decomposed[stripj] };
      ans_boxes.push_back(calc_bbox(v_decomposed[stripj].data(), (p+1)*(q+1)));
      float u = lerp(ubounds[0], ubounds[1], 1.0f*stripi/(udivs+1)+0.5f/(udivs+1));
      float v = lerp(vbounds[0], vbounds[1], 1.0f*stripj/(vdivs+1)+0.5f/(vdivs+1));
      ans_uv.push_back(float2{ u, v });
    }   
  }

  return { ans_boxes, ans_uv };
}

std::tuple<std::vector<LiteMath::Box4f>, std::vector<float2>>
get_bvh_leaves(const RBezierGrid &rbezier) {
  std::vector<Box4f> ans_boxes;
  std::vector<float2> ans_uv;
  for (int patchi = 0; patchi < rbezier.grid.rows_count(); ++patchi)
  for (int patchj = 0; patchj < rbezier.grid.cols_count(); ++patchj)
  {
    auto [cur_boxes, cur_uv] = get_bvh_leaves(
      rbezier.grid[{patchi, patchj}], 
      float2{ rbezier.uniq_uknots[patchi], rbezier.uniq_uknots[patchi+1] },
      float2{ rbezier.uniq_vknots[patchj], rbezier.uniq_vknots[patchj+1] });
    ans_boxes.insert(ans_boxes.end(), cur_boxes.begin(), cur_boxes.end());
    ans_uv.insert(ans_uv.end(), cur_uv.begin(), cur_uv.end());
  }
  return { ans_boxes, ans_uv };
}

