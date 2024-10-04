#pragma once

#include <vector>
#include <tuple>
#include <cinttypes>
#include <optional>

#include <LiteMath.h>

struct NURBSHeader
{
  uint32_t offset;
  uint32_t n, m, p, q;
};

#ifndef KERNEL_SLICER
template<typename T>
struct Vector2D
{
public:
  Vector2D() = default;
  Vector2D(size_t rows, size_t cols, const T& value = T{})
      : data_(rows*cols, value), n_(rows), m_(cols) {}
public:
  T& operator[](std::pair<size_t, size_t> idx) { return data_[m_*idx.first+idx.second]; }
  const T& operator[](std::pair<size_t, size_t> idx) const { return data_[m_*idx.first+idx.second]; }
public:
  uint32_t rows_count() const { return n_; }
  uint32_t cols_count() const { return m_; }
  std::tuple<uint32_t, uint32_t> shape() const { return { n_, m_ }; }
  std::pair<uint32_t, uint32_t> shape2D() const { return { n_, m_ }; }
public:
  T* data() { return data_.data(); }
  const T* data() const { return data_.data(); }
private:
  std::vector<T> data_;
  uint32_t n_, m_;
};

struct RawNURBS
{
public:
  Vector2D<LiteMath::float4> points;
  Vector2D<float> weights;
  std::vector<float> u_knots;
  std::vector<float> v_knots;
public:
  uint32_t get_n() const { return points.rows_count() - 1; }
  uint32_t get_m() const { return points.cols_count() - 1; }
  uint32_t get_p() const { return u_knots.size() - get_n() - 2; }
  uint32_t get_q() const { return v_knots.size() - get_m() - 2; }
};

inline
uint32_t points_offset(NURBSHeader h) { return 0; }
inline
uint32_t weights_offset(NURBSHeader h) { 
  return (h.n + 1)*(h.m + 1)*4; 
}
inline
uint32_t u_knots_offset(NURBSHeader h) { 
  return weights_offset(h) + (h.n + 1)*(h.m + 1); 
}
inline
uint32_t v_knots_offset(NURBSHeader h) {
  return u_knots_offset(h) + (h.n + h.p + 2);
}
struct DecomposedNURBS
{
public:
  DecomposedNURBS(const float *data, NURBSHeader h)
      : points(reinterpret_cast<const LiteMath::float4*>(data)),
        weights(data+weights_offset(h)),
        u_knots(data+u_knots_offset(h)),
        v_knots(data+v_knots_offset(h)),
        header(h) {}
public:
  LiteMath::float4 point(uint i, uint j) const { return points[i*(header.m+1) + j]; }
  float w(uint i, uint j) const { return weights[i*(header.m+1) + j]; }
  float u(uint i) const { return u_knots[i]; }
  float v(uint i) const { return v_knots[i]; }
public:
  int find_span(int n, int p, float u, const float *U) const;
  void basis_funs(int i, float u, int p, const float *U, float *N) const;
  LiteMath::float4 surface_point(float u, float v) const;
  LiteMath::float4 uder(float u, float v) const;
  LiteMath::float4 vder(float u, float v) const;
  bool u_closed() const;
  bool v_closed() const;
public:
  const LiteMath::float4 *points;
  const float *weights;
  const float *u_knots;
  const float *v_knots;
public:
  NURBSHeader header;
};

std::optional<LiteMath::float2> ray_nurbs_newton_intersection(
    const LiteMath::float3 &pos,
    const LiteMath::float3 &ray,
    const DecomposedNURBS &surf);

#endif