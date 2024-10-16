#pragma once

#include <vector>
#include <tuple>
#include <cinttypes>
#include <optional>

#include <LiteMath.h>

#include "nurbs_common.h"

enum class SurfaceParameter { U, V };

std::vector<std::vector<LiteMath::float4> >
decompose_curve(
    int n, int p,
    const float *U,
    const LiteMath::float4 *Pw);

std::vector<Vector2D<LiteMath::float4> >
decompose_surface(
    int n, int p,
    const float *U,
    int m,int q,
    const float *V,
    const Vector2D<LiteMath::float4> &Pw,
    SurfaceParameter dir);

struct RBezier
{
public:
  Vector2D<LiteMath::float4> weighted_points;
};

Vector2D<RBezier>
nurbs2rbezier(RawNURBS nurbs);