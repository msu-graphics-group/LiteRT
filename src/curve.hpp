#ifndef NURBS_SRC_CURVE
#define NURBS_SRC_CURVE

#include <vector>

#include "utils.hpp"
#include "LiteMath.h"

struct RBCurve2D
{
public:
  int degree() const;
public:
  LiteMath::float3 get_point(float u) const;
  LiteMath::float3 der(float u, int order) const;
  LiteMath::float3 non_rat_der(float u, int order) const;
  LiteMath::float3 der(float u) const;
public:
  std::vector<LiteMath::float3> pw;
};

#endif 