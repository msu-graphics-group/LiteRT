#include <cassert>
#include <random>

#include "nurbs_common.h"

using namespace LiteMath;

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
  res += (u+EPS > 1.0f) ? point(u, v) : point(u+EPS, v);
  res -= (u-EPS < 0.0f) ? point(u, v) : point(u-EPS, v);

  return res / (EPS * (1 + ((u+EPS <= 1.0f) && (u-EPS >= 0.0f))));
}

//TODO calculate accurate derivatives 
LiteMath::float4 DecomposedNURBS::vder(float u, float v) const {
    constexpr float EPS = 1e-2f;
  LiteMath::float4 res = {};
  res += (v+EPS > 1.0f) ? point(u, v) : point(u, v+EPS);
  res -= (v-EPS < 0.0f) ? point(u, v) : point(u, v-EPS);

  return res / (EPS * (1 + ((v+EPS <= 1.0f) && (v-EPS >= 0.0f))));
}

static float2
mul2x2x2(float2 m[2], const float2 &v)
{
  return m[0]*v[0]+m[1]*v[1];
}

std::optional<LiteMath::float2> ray_nurbs_newton_intersection(
    const LiteMath::float3 &pos,
    const LiteMath::float3 &ray,
    const DecomposedNURBS &surf) {
  constexpr int max_steps = 16;
  constexpr float EPS = 1e-2;

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
  
  float2 uv = { rand()*1.0f/static_cast<float>(RAND_MAX), rand()*1.0f/static_cast<float>(RAND_MAX) };
  float2 D = { dot(P1, surf.point(uv.x, uv.y)), dot(P2, surf.point(uv.x, uv.y)) };
  
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
      { J[1][1]/det, -J[1][0]/det },
      { -J[0][1]/det, J[0][0]/det }
    };

    uv = uv - mul2x2x2(J_inversed, D);
    uv.x = clamp(uv.x, 0.0f, 1.0f);
    uv.y = clamp(uv.y, 0.0f, 1.0f);
    assert(0 <= uv.x && uv.x <= 1);
    assert(0 <= uv.y && uv.y <= 1);

    float2 new_D = { dot(P1, surf.point(uv.x, uv.y)), dot(P2, surf.point(uv.x, uv.y)) };
    
    if (length(new_D) > length(D))
      return {};
    
    D = new_D;
  }

  if (length(D) > EPS)
    return {};
  
  return uv;
}