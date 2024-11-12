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
    REPRESENTATION_ITEM,
    GEOMETRIC_REPRESENTATION_ITEM,
    BSPLINE_SURFACE_WITH_KNOTS,
    RATIONAL_BSPLINE_SURFACE,
    BSPLINE_SURFACE,
    BOUNDED_SURFACE,
    SURFACE,
    COMPLEX,
};

struct Entity {
    uint id;
    Type type;
    std::vector<std::string> args;
};

class Parser {
public:
    Parser(const std::string &filename);
    LiteMath::float3 tofloat3(uint id);
    
    RawNURBS RationalBSplineSurfaceToNURBS(uint id);
    RawNURBS BSplineSurfaceWithKnotsToNURBS(uint id);
    RawNURBS toNURBS(uint id);
    std::vector<RawNURBS> allNURBS(void);
    std::map<uint, RawNURBS> allIDNurbs(void);

    bool isBSplineWithKnots(uint id);
    bool isRationalBSpline(uint id);
    bool isConvertableToNurbs(uint id);

    Entity getEntity(uint id);
private:
    Vector2D<float> parseFVector2D(std::string raw);
    Vector2D<LiteMath::float4> parsePointVector2D(std::string raw); 
    Entity parseComplexArg(const std::string &arg);
    Entity parseComplexEntity(const std::string &entity, uint id);
    Entity parseSimpleEntity(const std::string &entity, uint id); 
    Entity parseEntity(const std::string &entry);
    uint parseID(std::string rawID);
    uint parseF(std::string raw);
    uint parseU(std::string raw);
    std::vector<uint> parseUVector1D(std::string raw);
    std::vector<float> parseFVector1D(std::string raw);

    void storeBSplineSurface(Entity &entity, RawNURBS &nurbs);
    void storeBSplineSurfaceWithKnots(Entity &entity, RawNURBS &nurbs);
    void storeRationalBSplineSurface(Entity &entity, RawNURBS &nurbs);
    
    std::string readEntry(const std::string &text, size_t &offset);
    std::map<uint, Entity> entities;
};

// Utils functions
Type str2type(std::string name);
std::string type2str(Type type);
std::vector<std::string> argsplit(const std::string &rawargs, bool bycomma = true);
std::vector<float> decompressKnots(
        std::vector<float> &knots_comp,
        std::vector<uint> &knots_mult);
void trimKnots(std::vector<float> &knots, const std::vector<uint> &knots_mult, uint degree);

std::ostream& operator<<(std::ostream& cout, const RawNURBS &nurbs);

} // namespace STEP

