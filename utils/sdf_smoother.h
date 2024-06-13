#pragma once
#include "sdf_converter.h"

namespace sdf_converter
{
  SdfGrid sdf_grid_smoother(const SdfGrid &original_grid, float power, float lr, float lambda, unsigned steps);
}