#pragma once

#include <vector>
#include <tuple>
#include <cinttypes>
#include <optional>
#include <filesystem>

#include <LiteMath.h>

#include "nurbs_common.h"

enum class SurfaceParameter { U, V };
RBezierGrid nurbs2rbezier(RawNURBS nurbs);
RawNURBS load_nurbs (const std::filesystem::path &path);
std::tuple<std::vector<LiteMath::Box4f>, std::vector<LiteMath::float2>>
get_nurbs_bvh_leaves(const RBezierGrid &rbezier);