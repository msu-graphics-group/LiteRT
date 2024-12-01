#ifndef NURBS_SRC_UTILS
#define NURBS_SRC_UTILS
#include <vector>
#include <type_traits>
#include <cstddef>

template<typename T>
struct StrideView
{
public:
  StrideView(T *data = nullptr, size_t stride = 1)
      : data_(data), stride_(stride) {}
  StrideView(const StrideView &) = default;
public:
  T &operator[](size_t indx) const { return data_[indx*stride_]; }
  StrideView suffix(size_t begin) const {
    return StrideView(data_+stride_*begin, stride_);
  }
public:
  friend struct StrideView<const T>;
private:
  T *data_;
  size_t stride_;
};

template<typename T>
struct StrideView<const T>
{
public:
  StrideView(const T *data = nullptr, size_t stride = 1)
      : data_(data), stride_(stride) {}
  StrideView(const StrideView &) = default;
  StrideView(StrideView<T> view)
      : data_(view.data_), stride_(view.stride_) {}
public:
  const T &operator[](size_t indx) const { return data_[indx*stride_]; }
  StrideView suffix(size_t begin) const {
    return StrideView(data_+stride_*begin, stride_);
  }
private:
  const T *data_;
  size_t stride_;
};

template<typename T> StrideView(T*) -> StrideView<T>;
template<typename T> StrideView(const T*) -> StrideView<const T>;

template<typename T>
struct Matrix2D 
{
public:
  Matrix2D(int n = 0, int m = 0, T value = T())
      : n(n), m(m), values(n * m, value) {}
public:
  T& operator[](std::pair<int, int> ids) { return values[ids.first*m+ids.second]; }
  const T& operator[](std::pair<int, int> ids) const { return values[ids.first*m+ids.second]; }
public:
  const T* data() const { return values.data(); }
  T* data() { return values.data(); }
public:
  StrideView<T> row(size_t indx) { return StrideView(values.data()+indx*m); }
  StrideView<const T> row(size_t indx) const { return StrideView(values.data()+indx*m); }
public:
  StrideView<T> col(size_t indx) { return StrideView(values.data()+indx, m); }
  StrideView<const T> col(size_t indx) const { return StrideView(values.data()+indx, m); }
public:
  int get_n() const { return n; }
  int get_m() const { return m; }
private:
  int n; int m;
  std::vector<T> values;
};

#endif 