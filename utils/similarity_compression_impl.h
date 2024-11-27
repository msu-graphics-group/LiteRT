#pragma once
#include "similarity_compression.h"

namespace scom
{
  struct DataPoint
  {
    uint32_t original_id;
    uint32_t data_offset;
    uint32_t rotation_id;
    float    average_val;
  };

  struct Dataset
  {
    std::vector<float> all_points; //R^n vector for each data point
    std::vector<DataPoint> data_points;
  };
}