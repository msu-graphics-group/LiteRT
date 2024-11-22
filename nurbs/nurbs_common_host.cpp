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
de_casteljau_divide_curve(
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

constexpr float div_constant = 3.0f;
int flatteing_div_count(
    int p,
    StrideView<const float4> Pw) {
  if (p <= 1)
    return 0;

  float avg_v = 0.0f;
  float mx_a = 0.0f;
  
  for (int i = 0; i <= p-1; ++i) {
    float3 p0 = to_float3(Pw[i]/Pw[i].w);
    float3 p1 = to_float3(Pw[i+1]/Pw[i+1].w);
    avg_v += length(p1-p0);
  }

  for (int i = 0; i <= p-2; ++i) {
    float3 p0 = to_float3(Pw[i]/Pw[i].w);
    float3 p1 = to_float3(Pw[i+1]/Pw[i+1].w);
    float3 p2 = to_float3(Pw[i+2]/Pw[i+2].w);
    float3 res_p = p0 - 2*p1 + p2;
    mx_a = std::max(mx_a, length(res_p) * p * (p-1));
  }

  return static_cast<int>(
      std::ceil(div_constant * mx_a / std::sqrt(avg_v)));
}

std::vector<std::vector<float4>>
de_casteljau_uniformly_divide_curve(
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
    float u = 1.0f/(div_count+1)*(i+1); // in domain of full rbezier curve
    //but in current part u is different, because it's domain [0, 1] maps to [umin, 1]
    //need to map global u e [umin, 1] to local [0, 1]
    float umin = 1.0f/(div_count+1)*i;
    u = (u-umin)/(1.0f-umin);
    auto [res1, res2] = de_casteljau_divide_curve(p, u, StrideView{ Pw.data(), 1 });
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
      auto decomposed_col = de_casteljau_uniformly_divide_curve(
          p, divs_count, Pw.col(coli));
      for (int stripi = 0; stripi < Qw.size(); ++stripi)
      for (int rowi = 0; rowi <= p; ++rowi)
        Qw[stripi][{rowi, coli}] = decomposed_col[stripi][rowi];
    }
    return Qw;
  } else {
    std::vector<Matrix2D<float4>> Qw(divs_count+1, Matrix2D<float4>(p+1, q+1));
    for (int rowi = 0; rowi <= p; ++rowi) {
      auto decomposed_row = de_casteljau_uniformly_divide_curve(
          q, divs_count, Pw.row(rowi));
      for (int stripi = 0; stripi < Qw.size(); ++stripi)
      for (int coli = 0; coli <= q; ++coli)
        Qw[stripi][{rowi, coli}] = decomposed_row[stripi][coli];
    }
    return Qw;
  }
}

Box4f calc_bbox(const float4 *Pw, int count) {
  Box4f ans;
  for (int i = 0; i < count; ++i) {
    float4 point = Pw[i];
    point /= point.w;
    ans.boxMin = min(ans.boxMin, point);
    ans.boxMax = max(ans.boxMax, point);
  }
  return ans;
}

std::tuple<std::vector<Box4f>, std::vector<float2>>
get_nurbs_bvh_leaves(const RBezier &rbezier, float2 ubounds, float2 vbounds) {
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

std::tuple<std::vector<Box4f>, std::vector<float2>>
get_nurbs_bvh_leaves(const RBezierGrid &rbezier) {
  std::vector<Box4f> ans_boxes;
  std::vector<float2> ans_uv;
  for (int patchi = 0; patchi < rbezier.grid.rows_count(); ++patchi)
  for (int patchj = 0; patchj < rbezier.grid.cols_count(); ++patchj)
  {
    auto [cur_boxes, cur_uv] = get_nurbs_bvh_leaves(
      rbezier.grid[{patchi, patchj}], 
      float2{ rbezier.uniq_uknots[patchi], rbezier.uniq_uknots[patchi+1] },
      float2{ rbezier.uniq_vknots[patchj], rbezier.uniq_vknots[patchj+1] });
    ans_boxes.insert(ans_boxes.end(), cur_boxes.begin(), cur_boxes.end());
    ans_uv.insert(ans_uv.end(), cur_uv.begin(), cur_uv.end());
  }
  return { ans_boxes, ans_uv };
}


//copy from Basic NURBS Viewer https://github.com/iMacsimus/Basic-NURBS-Viewer
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
  if (p == 1) {
    return p * (pw[1]-pw[0]);
  }

  float u_n = 1.0f;
  float _1_u = 1.0f - u;
  int bc = 1.0f;
  
  float4 next = pw[1];
  float4 cur = pw[0];
  float4 res = (next-cur) * _1_u;
  cur = next;

  for (int i = 1; i <= p-2; ++i) {
    u_n *= u;
    bc = bc * (p-i)/i;
    next = pw[i+1];
    res = (res + u_n * bc * (next-cur)) * _1_u;
    cur = next;
  }

  next = pw[p];
  res += (u_n * u) * (next-cur);

  res *= p;
  return res;
}


LiteMath::float4 RBezier::get_point(float u, float v) const {
  int n = weighted_points.rows_count()-1;
  int m = weighted_points.cols_count()-1;
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
  return vder(u, v, Sw);
}

LiteMath::float4 RBezier::vder(float u, float v, const LiteMath::float4 &Sw) const {
  int n = weighted_points.rows_count()-1;
  int m = weighted_points.cols_count()-1;
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
  return uder(u, v, Sw);
}

LiteMath::float4 RBezier::uder(float u, float v, const LiteMath::float4 &Sw) const {
  int n = weighted_points.rows_count()-1;
  int m = weighted_points.cols_count()-1;
  if (n == 1) {
    float4 next = rbezier_curve_point(v, m, &weighted_points[{1, 0}]);
    float4 cur = rbezier_curve_point(v, m, &weighted_points[{0, 0}]);
    float4 Sw_der = n * (next - cur);
    float4 res = (Sw_der * Sw.w - Sw * Sw_der.w)/(Sw.w * Sw.w);
    return res;
  }

  float u_n = 1.0f;
  float _1_u = 1.0f - u;
  int bc = 1.0f;
  
  float4 next = rbezier_curve_point(v, m, &weighted_points[{1, 0}]);
  float4 cur = rbezier_curve_point(v, m, &weighted_points[{0, 0}]);
  float4 Sw_der = (next-cur) * _1_u;
  cur = next;

  for (int i = 1; i <= n-2; ++i)
  {
    u_n *= u;
    bc = bc * (n-i)/i;
    next = rbezier_curve_point(v, m, &weighted_points[{i+1, 0}]);
    Sw_der = (Sw_der + u_n * bc * (next-cur)) * _1_u;
    cur = next;
  }

  next = rbezier_curve_point(v, m, &weighted_points[{n, 0}]);
  Sw_der += (u_n*u) * (next-cur);

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
  float4 Sw = get_point(u, v);
  return uder(u, v, Sw);
}

LiteMath::float4 RBezierGrid::uder(float u, float v, const LiteMath::float4 &Sw) const {
  int uspan = find_span(u, uniq_uknots.size(), uniq_uknots.data());
  int vspan = find_span(v, uniq_vknots.size(), uniq_vknots.data());
  float umin = uniq_uknots[uspan], umax = uniq_uknots[uspan+1];
  float vmin = uniq_vknots[vspan], vmax = uniq_vknots[vspan+1];
  u = (u-umin)/(umax-umin);
  v = (v-vmin)/(vmax-vmin);
  assert(0 <= u && u <= 1);
  assert(0 <= v && v <= 1);
  float4 res =  grid[{uspan, vspan}].uder(u, v, Sw) / (umax - umin);
  return res;
}

LiteMath::float4 RBezierGrid::vder(float u, float v) const {
  float4 Sw = get_point(u, v);
  return vder(u, v, Sw);
}

LiteMath::float4 RBezierGrid::vder(float u, float v, const LiteMath::float4 &Sw) const {
  int uspan = find_span(u, uniq_uknots.size(), uniq_uknots.data());
  int vspan = find_span(v, uniq_vknots.size(), uniq_vknots.data());
  float umin = uniq_uknots[uspan], umax = uniq_uknots[uspan+1];
  float vmin = uniq_vknots[vspan], vmax = uniq_vknots[vspan+1];
  u = (u-umin)/(umax-umin);
  v = (v-vmin)/(vmax-vmin);
  assert(0 <= u && u <= 1);
  assert(0 <= v && v <= 1);
  float4 res =  grid[{uspan, vspan}].vder(u, v, Sw) / (vmax - vmin);
  return res;
}

LiteMath::float3 RBezierGrid::normal(float u, float v) const {
  float4 Sw = get_point(u, v);
  return normal(u, v, Sw);
}

LiteMath::float3 RBezierGrid::normal(float u, float v, const float4 &Sw) const {
  float4 uderiv = uder(u, v, Sw);
  float4 vderiv = vder(u, v, Sw);
  float3 normal = normalize(cross(to_float3(uderiv), to_float3(vderiv)));
  return normal;
}
//end copy Basic NURBS Viewer

cmesh4::SimpleMesh
get_nurbs_control_mesh(const RBezier &rbezier, float2 ubounds, float2 vbounds, int counter)
{
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

  cmesh4::SimpleMesh res;
  for (int stripi = 0; stripi < u_decomposed.size(); ++stripi) {
    auto v_decomposed = decompose_rbezier(
      p, q,
      u_decomposed[stripi], 
      vdivs, SurfaceParameter::V);
    for (int stripj = 0; stripj < v_decomposed.size(); ++stripj) {
      auto &points = v_decomposed[stripj];
      float umin = lerp(ubounds[0], ubounds[1], stripi*1.0f/(udivs+1));
      float umax = lerp(ubounds[0], ubounds[1], (stripi+1.0f)/(udivs+1));
      float vmin = lerp(vbounds[0], vbounds[1], stripj*1.0f/(vdivs+1));
      float vmax = lerp(vbounds[0], vbounds[1], (stripj+1.0f)/(vdivs+1));
      for (int quadi = 0; quadi <= p-1; ++quadi)
      for (int quadj = 0; quadj <= q-1; ++quadj)
      {
        float4 cPw[] = {
          points[{quadi, quadj}],
          points[{quadi+1, quadj}],
          points[{quadi+1, quadj+1}],

          points[{quadi+1, quadj+1}],
          points[{quadi, quadj+1}],
          points[{quadi, quadj}]
        };
        for (auto &p: cPw) { p /= p.w; }
        float2 UV[] = {
          float2{ quadi*1.0f/p, quadj*1.0f/q },
          float2{ (quadi+1.0f)/p, quadj*1.0f/q },
          float2{ (quadi+1.0f)/p, (quadj+1.0f)/q },

          float2{ (quadi+1.0f)/p, (quadj+1.0f)/q },
          float2{ quadi*1.0f/p, (quadj+1.0f)/q },
          float2{ quadi*1.0f/p, quadj*1.0f/q },
        };
        auto calc_normal_f = [&](float2 uv) {
          uv.x = lerp(stripi*1.0f/(udivs+1), (stripi+1.0f)/(udivs+1), uv.x);
          uv.y = lerp(stripj*1.0f/(vdivs+1), (stripj+1.0f)/(vdivs+1), uv.y);
          float3 uder = to_float3(rbezier.uder(uv.x, uv.y));
          float3 vder = to_float3(rbezier.vder(uv.x, uv.y));
          float3 normal = normalize(cross(uder, vder));
          return to_float4(normal, 0.0f);
        };
        float4 normals[] = {
          calc_normal_f(UV[0]),
          calc_normal_f(UV[1]),
          calc_normal_f(UV[2]),
          
          calc_normal_f(UV[3]),
          calc_normal_f(UV[4]),
          calc_normal_f(UV[5]),
        };
        for (auto &uv: UV) {
          uv.x = lerp(umin, umax, uv.x);
          uv.y = lerp(vmin, vmax, uv.y);
        }
        std::copy(cPw, cPw+6, std::back_inserter(res.vPos4f));
        std::copy(UV, UV+6, std::back_inserter(res.vTexCoord2f));
        std::copy(normals, normals+6, std::back_inserter(res.vNorm4f));
        for (int i = 0; i < 6; ++i)
          res.indices.push_back(counter++);
      }
    }   
  }

  return res;
}

cmesh4::SimpleMesh
get_nurbs_control_mesh(const RBezierGrid &rbezier)
{
  cmesh4::SimpleMesh res;
  for (int patchi = 0; patchi < rbezier.grid.rows_count(); ++patchi)
  for (int patchj = 0; patchj < rbezier.grid.cols_count(); ++patchj)
  {
    auto cur_mesh = get_nurbs_control_mesh(
      rbezier.grid[{patchi, patchj}], 
      float2{ rbezier.uniq_uknots[patchi], rbezier.uniq_uknots[patchi+1] },
      float2{ rbezier.uniq_vknots[patchj], rbezier.uniq_vknots[patchj+1] },
      res.indices.size());
    std::copy(cur_mesh.vPos4f.begin(), cur_mesh.vPos4f.end(), std::back_inserter(res.vPos4f));
    std::copy(cur_mesh.vTexCoord2f.begin(), cur_mesh.vTexCoord2f.end(), std::back_inserter(res.vTexCoord2f));
    std::copy(cur_mesh.vNorm4f.begin(), cur_mesh.vNorm4f.end(), std::back_inserter(res.vNorm4f));
    std::copy(cur_mesh.indices.begin(), cur_mesh.indices.end(), std::back_inserter(res.indices));
  }
  return res;
}

#include <vector>
#include <cassert>

using namespace LiteMath;

int RBCurve2D::degree() const {
  return pw.size()-1;
}

LiteMath::float3 RBCurve2D::get_point(float u) const {
  int p = degree();

  float u_n = 1.0f;
  float _1_u = 1.0f - u;
  int bc = 1.0f;

  float3 res = pw[0] * _1_u;
  for (int i = 1; i <= p-1; ++i) {
    u_n *= u;
    bc = bc * (p-i+1)/i;
    res = (res + u_n * bc * pw[i]) * _1_u;
  }
  res += (u_n * u) * pw[p];
  return res;
}

LiteMath::float3 RBCurve2D::non_rat_der(float u, int order) const {
  if (order == 0)
    return get_point(u);

  int p = degree();

  RBCurve2D der = { std::vector<float3>(degree()+1) };
  std::copy(pw.begin(), pw.end(), der.pw.begin());

  for (int i = 0; i < order; ++i) {
    for (int j = 0; j < p-i; ++i) {
      der.pw[j] = (der.pw[j+1] - der.pw[j]) * (p-i);
    }
  }

  der.pw.resize(p+1-order);
  return der.get_point(u);
}

LiteMath::float3 RBCurve2D::der(float u) const {
  float3 p = get_point(u);
  float3 d = non_rat_der(u, 1);
  return (d*p.z - d.z*p)/(p.z * p.z);
}

LiteMath::float3 RBCurve2D::der(float u, int order) const {
  if (order == 0) {
    float3 res = get_point(u);
    res /= res.z;
    return res;
  }

  int bc = 1;
  float3 left = {};
  for (int i = 0; i < order; ++i) {
    float3 a = der(u, i);
    float b = non_rat_der(u, order-i).z;
    left += bc * a * b;
    bc = bc * (order-i) / (i+1);
  }

  float3 right = non_rat_der(u, order);

  float3 res = (right - left) / (bc * get_point(u).z);
  assert(res.z == 0.0f);

  return res;
}

