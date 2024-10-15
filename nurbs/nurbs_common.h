#pragma once

#include <LiteMath.h>
using LiteMath::float2;

struct NURBSHeader
{
  uint32_t offset;
  uint32_t n, m, p, q;
};

inline
uint32_t weights_offset(NURBSHeader h) { 
  return h.offset+(h.n + 1)*(h.m + 1)*4; 
}
inline
uint32_t u_knots_offset(NURBSHeader h) { 
  return h.offset+(h.n + 1)*(h.m + 1)*5; 
}
inline
uint32_t v_knots_offset(NURBSHeader h) {
  return h.offset+(h.n + 1)*(h.m + 1)*5 + (h.n + h.p + 2);
}

struct NURBS_HitInfo
{
  bool hitten;
  float2 uv;
};

//HOST PART
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
#endif