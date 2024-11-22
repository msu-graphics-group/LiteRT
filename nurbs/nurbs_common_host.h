#pragma once

#include <vector>
#include <tuple>
#include <cinttypes>
#include <optional>
#include <filesystem>

#include <LiteMath.h>
#include <cmesh4.h>

#include "nurbs_common.h"

enum class SurfaceParameter { U, V };
RBezierGrid nurbs2rbezier(RawNURBS nurbs);
RawNURBS load_nurbs (const std::filesystem::path &path);

std::tuple<std::vector<LiteMath::Box4f>, std::vector<LiteMath::float2>>
get_nurbs_bvh_leaves(const RBezierGrid &rbezier);

cmesh4::SimpleMesh
get_nurbs_control_mesh(const RBezierGrid &rbezier);

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
