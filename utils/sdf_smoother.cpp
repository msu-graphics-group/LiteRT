#include "sdf_smoother.h"

namespace sdf_converter
{
  SdfGrid sdf_grid_smoother(const SdfGrid &original_grid, float power, float lr, float lambda, unsigned steps)
  {
    SdfGrid grid;
    grid.data = original_grid.data;
    grid.size = original_grid.size;

    std::vector<float> diff(grid.data.size());

    for (int step = 0; step < steps; step++)
    {
      double total_loss = 0.0;
      double total_reg_loss = 0.0;
      std::fill(diff.begin(), diff.end(), 0.0f);
      
      for (int i0 = 0; i0 < grid.size.x; i0++)
      {
        for (int j0 = 0; j0 < grid.size.y; j0++)
        {
          for (int k0 = 0; k0 < grid.size.z; k0++)
          {
            float idx = i0 * grid.size.y * grid.size.z + j0 * grid.size.z + k0;
            float loss = (grid.data[idx] - original_grid.data[idx]) * (grid.data[idx] - original_grid.data[idx]);
            total_loss += loss;
            diff[idx] = 2.0f * (grid.data[idx] - original_grid.data[idx]);
            
            if (i0 > 0)
            {
              float d = grid.data[idx] - grid.data[idx - grid.size.y * grid.size.z];
              float sgn_d = d > 0.0f ? 1.0f : -1.0f;
              d = std::abs(d);

              float reg_loss = std::pow(d, power);
              total_reg_loss += reg_loss;
              diff[idx] += lambda * power * sgn_d * std::pow(d, power - 1);
            }

            if (i0 < grid.size.x - 1)
            {
              float d = grid.data[idx] - grid.data[idx + grid.size.y * grid.size.z];
              float sgn_d = d > 0.0f ? 1.0f : -1.0f;
              d = std::abs(d);

              float reg_loss = std::pow(d, power);
              total_reg_loss += reg_loss;
              diff[idx] += lambda * power * sgn_d * std::pow(d, power - 1);
            }

            if (j0 > 0)
            {
              float d = grid.data[idx] - grid.data[idx - grid.size.z];
              float sgn_d = d > 0.0f ? 1.0f : -1.0f;
              d = std::abs(d);

              float reg_loss = std::pow(d, power);
              total_reg_loss += reg_loss;
              diff[idx] += lambda * power * sgn_d * std::pow(d, power - 1);
            } 

            if (j0 < grid.size.y - 1)
            {
              float d = grid.data[idx] - grid.data[idx + grid.size.z];
              float sgn_d = d > 0.0f ? 1.0f : -1.0f;
              d = std::abs(d);

              float reg_loss = std::pow(d, power);
              total_reg_loss += reg_loss;
              diff[idx] += lambda * power * sgn_d * std::pow(d, power - 1);
            }

            if (k0 > 0)
            {
              float d = grid.data[idx] - grid.data[idx - 1];
              float sgn_d = d > 0.0f ? 1.0f : -1.0f;
              d = std::abs(d);

              float reg_loss = std::pow(d, power);
              total_reg_loss += reg_loss;
              diff[idx] += lambda * power * sgn_d * std::pow(d, power - 1);
            }

            if (k0 < grid.size.z - 1)
            {
              float d = grid.data[idx] - grid.data[idx + 1];
              float sgn_d = d > 0.0f ? 1.0f : -1.0f;
              d = std::abs(d);  

              float reg_loss = std::pow(d, power);
              total_reg_loss += reg_loss;
              diff[idx] += lambda * power * sgn_d * std::pow(d, power - 1);
            }
          }
        }
      }

      //printf("step = %d, loss = %f, reg_loss = %f\n", step, total_loss, total_reg_loss);

      for (int i = 0; i < grid.data.size(); i++)
      {
        grid.data[i] -= lr * diff[i];
      }
    }

    return grid;
  }
}