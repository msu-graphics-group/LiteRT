#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <utility>
#include <regex>
#include <fstream>
#include <sstream>
#include <cassert>

namespace LiteMath {
  struct float3;
  struct float4;
}

namespace STEP
{

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
  Vector2D<LiteMath::float4> points;
  Vector2D<float> weights;
  std::vector<float> u_knots;
  std::vector<float> v_knots;
  uint u_degree;
  uint v_degree;
};

enum class Type {
    UNDEFINED,
    POINT,
    BSPLINE_SURFACE,
};

struct Entity {
    uint id;
    Type type;
    std::vector<std::string> args;
};

Type str2type(std::string name);
std::vector<std::string> argsplit(const std::string &rawargs);
uint parseID(std::string rawID);
uint parseU(std::string raw);
std::vector<uint> parseUVector1D(std::string raw);
std::vector<float> parseFVector1D(std::string raw);
LiteMath::float3 tofloat3(std::map<uint, Entity> &entities, uint id);
Vector2D<LiteMath::float4> parsePointVector2D(
          std::map<uint, STEP::Entity> &entities,
          std::string raw);
bool try_parse_entity(const std::string &entry, Entity &res);
std::vector<float> decompressKnots(
    std::vector<float> knots_comp,
    std::vector<uint> knots_mult);
RawNURBS toNURBS(std::map<uint, Entity> &entities, uint id);
std::vector<RawNURBS> allNURBS(std::map<uint, Entity> &entities);
std::map<uint, RawNURBS> allIDNurbs(std::map<uint, Entity> &entities);
std::map<uint, Entity> parse(const std::string &filename);

} // namespace STEP

std::ostream& operator<<(std::ostream& cout, const STEP::RawNURBS &nurbs);
