#pragma once

#include <LiteMath.h>
using LiteMath::float2, LiteMath::float3;

struct NURBSHeader
{
  int offset;
  int p, q;
  int uknots_cnt, vknots_cnt;
};

inline
int pts_offset(NURBSHeader h, int uspan, int vspan) { 
  return h.offset+(h.p + 1)*(h.q + 1) * 4 * ((h.vknots_cnt-1)*uspan + vspan); 
}
inline
int uknots_offset(NURBSHeader h) { 
  return h.offset+(h.p + 1)*(h.q + 1) * 4 * (h.uknots_cnt-1) * (h.vknots_cnt-1);
}
inline
int vknots_offset(NURBSHeader h) {
  return uknots_offset(h)+h.uknots_cnt;
}

struct NURBS_HitInfo
{
  bool hitten;
  float3 point;
  float3 normal;
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
  uint32_t n_ = 0, m_ = 0;
};

struct RawNURBS
{
public:
  Vector2D<LiteMath::float4> points;
  Vector2D<float> weights;
  std::vector<float> u_knots;
  std::vector<float> v_knots;
  LiteMath::Box4f bbox;
public:
  uint32_t get_n() const { return points.rows_count() - 1; }
  uint32_t get_m() const { return points.cols_count() - 1; }
  uint32_t get_p() const { return u_knots.size() - get_n() - 2; }
  uint32_t get_q() const { return v_knots.size() - get_m() - 2; }
};

struct RBezier 
{ 
  Vector2D<LiteMath::float4> weighted_points;
};

struct RBezierGrid
{
  std::vector<float> uniq_uknots;
  std::vector<float> uniq_vknots;
  Vector2D<RBezier> grid;
  LiteMath::Box4f bbox;
};


#endif