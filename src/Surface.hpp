#ifndef NURBS_SRC_SURFACE
#define NURBS_SRC_SURFACE

#include <cinttypes>
#include <vector>
#include <cmath>
#include <filesystem>

#include <LiteMath.h>
#include <Image2d.h>

#include <LiteMath.h>
#include <Image2d.h>

template<typename T>
struct Matrix2D 
{
public:
  Matrix2D(int n = 0, int m = 0, T value = T()): n(n), m(m), values(n * m, value) {}
  T& operator[](std::pair<int, int> ids) { return values[ids.first*m+ids.second]; }
  const T& operator[](std::pair<int, int> ids) const { return values[ids.first*m+ids.second]; }
  int get_n() const { return n; }
  int get_m() const { return m; }
private:
  int n; int m;
  std::vector<T> values;
};

// WRITTEN WITH THE NURBS BOOK
int find_span(int n, int p, float u, const float *U);
void basis_funs(int i, float u, int p, const float *U, float *N);
LiteMath::float4 surface_point(
    int n, int p, 
    const float *U, 
    int m, int q,
    const float *V,
    const Matrix2D<LiteMath::float4> &P,
    const Matrix2D<float> &W,
    float u, float v);

class Surface {
public:
  Surface() = default;
  Surface(
      const Matrix2D<LiteMath::float4> &points,
      const Matrix2D<float> &weights,
      uint32_t deg_u, uint32_t deg_v,
      const std::vector<float> &u_knots,
      const std::vector<float> &v_knots)
      : points(points), weights(weights),
          deg_u(deg_u), deg_v(deg_v),
          u_knots(u_knots), v_knots(v_knots) {}
public:
  Matrix2D<LiteMath::float4> points;
  Matrix2D<float> weights;
  uint32_t deg_u;
  uint32_t deg_v;
  std::vector<float> u_knots;
  std::vector<float> v_knots;
public:
  LiteMath::float4 get_point(float u, float v) const { 
    return surface_point(
      points.get_n()-1, deg_u, u_knots.data(), 
      points.get_m()-1, deg_v, v_knots.data(), 
      points, weights,
      u, v);
  }
  LiteMath::float4 uderivative(float u, float v) const {
    constexpr float EPS = 1e-2f;
    return (get_point(u+EPS, v)-get_point(u-EPS, v)) / 2*EPS;
  }
  LiteMath::float4 vderivative(float u, float v) const {
    constexpr float EPS = 1e-2f;
    return (get_point(u, v+EPS)-get_point(u, v-EPS)) / 2*EPS;
  }
  LiteMath::float3 get_normal(float u, float v) const {
    return LiteMath::normalize(LiteMath::cross(
        LiteMath::to_float3(uderivative(u, v)),
        LiteMath::to_float3(vderivative(u, v))));
  }
};

Surface load_surface(const std::filesystem::path &path);

#endif