#include "sdf_smoother.h"

namespace sdf_converter
{
  template<typename T>
  class AdamOptimizer
  {
  public:
    AdamOptimizer(int _params_count, T _lr = T(0.15f), T _beta_1 = T(0.9f), T _beta_2 = T(0.999f), T _eps = T(1e-8)) 
    {
      lr = _lr;
      beta_1 = _beta_1;
      beta_2 = _beta_2;
      eps = _eps;
      V = std::vector<T>(_params_count, 0);
      S = std::vector<T>(_params_count, 0);
      params_count = _params_count;
    }
    
    void step(T *params_ptr, const T* grad_ptr, int iter)
    {
      const auto b1 = std::pow(beta_1, iter + 1);
      const auto b2 = std::pow(beta_2, iter + 1);
      for (size_t i = 0; i < params_count; i++)
      {
        auto g = grad_ptr[i];
        V[i] = beta_1 * V[i] + (1 - beta_1) * g;
        auto Vh = V[i] / (T(1) - b1);
        S[i] = beta_2 * S[i] + (1 - beta_2) * g * g;
        auto Sh = S[i] / (T(1) - b2);
        params_ptr[i] -= lr * Vh / (std::sqrt(Sh) + eps);
      }
    }
  private:
    T lr, beta_1, beta_2, eps;
    std::vector<T> V;
    std::vector<T> S;
    size_t params_count;
  };

  SdfGrid sdf_grid_smoother(const SdfGrid &original_grid, float power, float lr, float lambda, unsigned steps)
  {
    SdfGrid grid;
    grid.data = original_grid.data;
    grid.size = original_grid.size;

    std::vector<float> diff(grid.data.size());

    AdamOptimizer optimizer(grid.data.size(), lr);

    for (int step = 0; step < steps; step++)
    {
      double total_loss = 0.0;
      double total_reg_loss = 0.0;
      std::fill(diff.begin(), diff.end(), 0.0f);
      
      #pragma omp parallel for
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

      optimizer.step(grid.data.data(), diff.data(), step);

      printf("step = %d, loss = %f(%f + %f*%f)\n", step, total_loss + lambda*total_reg_loss, total_loss, lambda, total_reg_loss);

      //for (int i = 0; i < grid.data.size(); i++)
      //{
      //  grid.data[i] -= lr * diff[i];
      //}
    }

    return grid;
  }
}