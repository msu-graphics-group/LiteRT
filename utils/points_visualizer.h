#pragma once
#include "LiteMath/Image2d.h"

#include <vector>

void draw_points(std::vector<LiteMath::float4> &point_positions,
                 std::vector<LiteMath::float4> &point_colors,
                 LiteImage::Image2D<LiteMath::float4> &out_image);