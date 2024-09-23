#pragma once
#include <vector>

#ifndef KERNEL_SLICER
struct RawNURBS
{
    std::vector<float> knots;
    std::vector<float> control_points;
    std::vector<float> weights;
    unsigned power;
};
#endif
struct NURBSHeader
{
  unsigned power;
  unsigned offset;
  unsigned size;
};