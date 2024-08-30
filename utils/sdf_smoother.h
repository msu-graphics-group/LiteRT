#pragma once
#include "sdf_converter.h"

namespace sdf_converter
{
  SdfGrid sdf_grid_smoother(const SdfGrid &original_grid, float power, float lr, float lambda, unsigned steps);
  SdfSBS sdf_SBS_smoother(const SdfSBS &original_sbs, float power, float lr, float lambda, unsigned steps);
}