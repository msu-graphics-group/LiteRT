#include "sdf_smoother.h"
#include <set>

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
      
      //#pragma omp parallel for
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

            //if (step == 0) fprintf(file_x, "%d -- %d %d %d %d - %f\n", step, i0, j0, k0, grid.size.x - 1, grid.data[idx]);
            
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

      //printf("step = %d, loss = %f(%f + %f*%f)\n", step, total_loss + lambda*total_reg_loss, total_loss, lambda, total_reg_loss);

      //for (int i = 0; i < grid.data.size(); i++)
      //{
      //  grid.data[i] -= lr * diff[i];
      //}
    }

    //fclose(file_x);

    return grid;
  }
  
  enum coord
  {
    X, Y, Z
  };

  /*typedef struct temp_sbs_value_data
  {
    uint32_t idx;
    uint32_t coord_1;//X or Y
    uint32_t coord_2;//Y or Z
    float dist_another_p;
    float val_another_p;
    int8_t weight;//4 2 or 1, when 0 element deleted
    bool dir;//false is - and true is +
    bool counted;//true if already counted diff
  } TSBSVD;

  void check_all_points(const SdfSBS &original_sbs,
                        SdfSBS &sbs,
                        SdfSBSNode &node,
                        std::pair<coord, uint32_t> key, 
                        std::map<std::pair<coord, uint32_t>, std::vector<TSBSVD>> &val_buf, 
                        std::vector<float> &diff,
                        TSBSVD temp,
                        int32_t off_another,
                        uint32_t off_1,
                        uint32_t off_2,
                        uint32_t coord_1_vpos,
                        uint32_t coord_2_vpos,
                        uint32_t coord_1_idx,
                        uint32_t coord_2_idx,
                        uint32_t val_size,
                        uint32_t size,
                        uint32_t idx,
                        double &total_reg_loss,
                        float power, 
                        float lambda)
  {
    if (val_buf.find(key) != val_buf.end())//searching points on this plane
    {
      for (int elem = 0; elem < val_buf[key].size(); )//for each previous point on this plane
      {
        if (val_buf[key][elem].dir == true && 
            ((val_buf[key][elem].coord_1 >= coord_1_vpos && val_buf[key][elem].coord_1 < coord_1_vpos + (size / sbs.header.brick_size) && coord_1_idx < val_size - 1) ||
            (val_buf[key][elem].coord_1 == coord_1_vpos && coord_1_idx == val_size - 1)) && 
            ((val_buf[key][elem].coord_2 >= coord_2_vpos && val_buf[key][elem].coord_2 < coord_2_vpos + (size / sbs.header.brick_size) && coord_2_idx < val_size - 1) ||
            (val_buf[key][elem].coord_2 == coord_2_vpos && coord_2_idx == val_size - 1)))//is point on the side of this voxel
        {

          //count for each point from val_buf and delete if it not on the border

          //can change then (counter: corner - 1 weight, border - 2 weight, side - 4 weight)

          if (val_buf[key][elem].coord_1 == coord_1_vpos && val_buf[key][elem].coord_2 == coord_2_vpos)//is point on the corner
          {
            //count temp
            temp.weight -= 1;
            if (!temp.counted)
            {
              float d = (sbs.values_f[sbs.values[node.data_offset + idx]] - val_buf[key][elem].val_another_p) / val_buf[key][elem].dist_another_p;
              float sgn_d = d > 0.0f ? 1.0f : -1.0f;
              d = std::abs(d);

              float reg_loss = std::pow(d, power);
              total_reg_loss += reg_loss;
              diff[sbs.values[node.data_offset + idx]] += lambda * power * sgn_d * std::pow(d, power - 1);

              temp.counted = true;
            }

            val_buf[key][elem].weight -= 1;
            if (!val_buf[key][elem].counted)
            {
              val_buf[key][elem].counted = true;

              float d = (sbs.values_f[val_buf[key][elem].idx] - sbs.values_f[sbs.values[node.data_offset + idx + off_another]]) / size;
              float sgn_d = d > 0.0f ? 1.0f : -1.0f;
              d = std::abs(d);

              float reg_loss = std::pow(d, power);
              total_reg_loss += reg_loss;
              diff[val_buf[key][elem].idx] += lambda * power * sgn_d * std::pow(d, power - 1);
            }

            if (val_buf[key][elem].weight > 0)
            {
              ++elem;
            }
            else
            {
              val_buf[key].erase(val_buf[key].begin() + elem);
            }
          }
          else if (val_buf[key][elem].coord_1 == coord_1_vpos)//is point on the second border
          {
            val_buf[key][elem].weight -= 2;
            if (!val_buf[key][elem].counted)
            {
              val_buf[key][elem].counted = true;

              float mid = 
                (sbs.values_f[sbs.values[node.data_offset + idx + off_another]] * (val_buf[key][elem].coord_2 - coord_2_vpos) + 
                  sbs.values_f[sbs.values[node.data_offset + idx + off_another + off_2]] * (coord_2_vpos + (size / sbs.header.brick_size) - val_buf[key][elem].coord_2)) / 
                (size / sbs.header.brick_size);

              float d = (sbs.values_f[val_buf[key][elem].idx] - mid) / size;
              float sgn_d = d > 0.0f ? 1.0f : -1.0f;
              d = std::abs(d);

              float reg_loss = std::pow(d, power);
              total_reg_loss += reg_loss;
              diff[val_buf[key][elem].idx] += lambda * power * sgn_d * std::pow(d, power - 1);
            }

            if (val_buf[key][elem].weight > 0)
            {
              ++elem;
            }
            else
            {
              val_buf[key].erase(val_buf[key].begin() + elem);
            }
          }
          else if (val_buf[key][elem].coord_2 == coord_2_vpos)//is point on the first border
          {
            val_buf[key][elem].weight -= 2;
            if (!val_buf[key][elem].counted)
            {
              val_buf[key][elem].counted = true;

              float mid = 
                (sbs.values_f[sbs.values[node.data_offset + idx + off_another]] * (val_buf[key][elem].coord_1 - coord_1_vpos) + 
                  sbs.values_f[sbs.values[node.data_offset + idx + off_another + off_1]] * (coord_1_vpos + (size / sbs.header.brick_size) - val_buf[key][elem].coord_1)) / 
                (size / sbs.header.brick_size);

              float d = (sbs.values_f[val_buf[key][elem].idx] - mid) / size;
              float sgn_d = d > 0.0f ? 1.0f : -1.0f;
              d = std::abs(d);

              float reg_loss = std::pow(d, power);
              total_reg_loss += reg_loss;
              diff[val_buf[key][elem].idx] += lambda * power * sgn_d * std::pow(d, power - 1);
            }

            if (val_buf[key][elem].weight > 0)
            {
              ++elem;
            }
            else
            {
              val_buf[key].erase(val_buf[key].begin() + elem);
            }
          }
          else//another points on the side
          {
            val_buf[key][elem].weight -= 4;
            if (!val_buf[key][elem].counted)
            {
              val_buf[key][elem].counted = true;

              float mid_1_minus = 
                (sbs.values_f[sbs.values[node.data_offset + idx + off_another]] * (val_buf[key][elem].coord_2 - coord_2_vpos) + 
                  sbs.values_f[sbs.values[node.data_offset + idx + off_another + off_2]] * (coord_2_vpos + (size / sbs.header.brick_size) - val_buf[key][elem].coord_2)) / 
                (size / sbs.header.brick_size);

              float mid_1_plus = 
                (sbs.values_f[sbs.values[node.data_offset + idx + off_another + off_1]] * (val_buf[key][elem].coord_2 - coord_2_vpos) + 
                  sbs.values_f[sbs.values[node.data_offset + idx + off_another + off_1 + off_2]] * (coord_2_vpos + (size / sbs.header.brick_size) - val_buf[key][elem].coord_2)) / 
                (size / sbs.header.brick_size);

              float mid = 
                (mid_1_minus * (val_buf[key][elem].coord_1 - coord_1_vpos) + 
                  mid_1_plus * (coord_1_vpos + (size / sbs.header.brick_size) - val_buf[key][elem].coord_1)) / 
                (size / sbs.header.brick_size);

              float d = (sbs.values_f[val_buf[key][elem].idx] - mid) / size;
              float sgn_d = d > 0.0f ? 1.0f : -1.0f;
              d = std::abs(d);

              float reg_loss = std::pow(d, power);
              total_reg_loss += reg_loss;
              diff[val_buf[key][elem].idx] += lambda * power * sgn_d * std::pow(d, power - 1);
            }
            
            val_buf[key].erase(val_buf[key].begin() + elem);
          }


          //val_buf[key].erase(val_buf[key].begin() + elem);//only if not a border
        }
        else//point not on the side of voxel
        {
          ++elem;
        }
      }
      if (temp.weight > 0)
      {
        val_buf[key].push_back(temp);//add point to others on this plane
      }
      else if (val_buf[key].size() == 0)
      {
        val_buf.erase(key);//delete void vector
      }
    }
    else
    {
      val_buf[key] = {temp};//first point on this side
    }
  }

  SdfSBS sdf_SBS_smoother(const SdfSBS &original_sbs, float power, float lr, float lambda, unsigned steps)
  {
    SdfSBS sbs;
    sbs.header = original_sbs.header;
    sbs.nodes = original_sbs.nodes;
    sbs.values = original_sbs.values;
    sbs.values_f = original_sbs.values_f;

    if (original_sbs.header.aux_data & SDF_SBS_NODE_LAYOUT_MASK != SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F)
    {
      return sbs;
    }

    std::vector<float> diff(sbs.values_f.size());

    AdamOptimizer optimizer(sbs.values_f.size(), lr);

    std::map<std::pair<coord, uint32_t>, std::vector<TSBSVD>> val_buf;

    std::sort(sbs.nodes.begin(), sbs.nodes.end(), [](SdfSBSNode a, SdfSBSNode b){return (a.pos_z_lod_size & 0x0000FFFF) > (b.pos_z_lod_size & 0x0000FFFF);});

    for (int step = 0; step < steps; step++)
    {
      double total_loss = 0.0;
      double total_reg_loss = 0.0;
      std::fill(diff.begin(), diff.end(), 0.0f);

      uint32_t val_size = sbs.header.brick_size + 1;

      uint32_t node_counter = 0;
      for (auto node : sbs.nodes)
      {
        //printf("node: %d/%d\n", node_counter++, sbs.nodes.size());
        float size = (sbs.nodes[0].pos_z_lod_size & 0x0000FFFF) / (node.pos_z_lod_size & 0x0000FFFF);

        uint32_t x_pos = size * (node.pos_xy >> 16);
        uint32_t y_pos = size * (node.pos_xy & 0x0000FFFF);
        uint32_t z_pos = size * (node.pos_z_lod_size >> 16);

        //#pragma omp parallel for
        for (uint32_t i = 0; i < val_size; ++i)//IGNORING BRICK_PAD
        {
          for (uint32_t j = 0; j < val_size; ++j)//IGNORING BRICK_PAD
          {
            for (uint32_t k = 0; k < val_size; ++k)//IGNORING BRICK_PAD
            {
              int idx = i * val_size * val_size + j * val_size + k;

              float loss =  (sbs.values_f[sbs.values[node.data_offset + idx]] - original_sbs.values_f[original_sbs.values[node.data_offset + idx]]) * 
                            (sbs.values_f[sbs.values[node.data_offset + idx]] - original_sbs.values_f[original_sbs.values[node.data_offset + idx]]);
              total_loss += loss;
              diff[sbs.values[node.data_offset + idx]] += 2.0f * (sbs.values_f[sbs.values[node.data_offset + idx]] - original_sbs.values_f[original_sbs.values[node.data_offset + idx]]);

              uint32_t x_vpos = x_pos + i * (size / sbs.header.brick_size);
              uint32_t y_vpos = y_pos + j * (size / sbs.header.brick_size);
              uint32_t z_vpos = z_pos + k * (size / sbs.header.brick_size);

              //x- side of voxel/////////////////////////////////////////////////////////////////////////////////////////

              if (i == 0)//for points on the side of voxel
              {
                auto key = std::pair(X, size * (node.pos_xy >> 16));

                TSBSVD temp;
                temp.coord_1 = y_vpos;
                temp.coord_2 = z_vpos;
                temp.dir = false;
                temp.idx = sbs.values[node.data_offset + idx];
                temp.dist_another_p = size / sbs.header.brick_size;
                temp.val_another_p = sbs.values_f[sbs.values[node.data_offset + idx + val_size * val_size]];
                temp.weight = 4;
                temp.counted = false;

                check_all_points(original_sbs, sbs, node, key, val_buf, diff, temp, val_size * val_size, val_size, 1, 
                                 y_vpos, z_vpos, j, k, val_size, size, idx, total_reg_loss, power, lambda);

              }
              else//for points inside the voxel
              {
                float d = (sbs.values_f[sbs.values[node.data_offset + idx]] - sbs.values_f[sbs.values[node.data_offset + idx - val_size * val_size]]) / size;
                float sgn_d = d > 0.0f ? 1.0f : -1.0f;
                d = std::abs(d);

                float reg_loss = std::pow(d, power);
                total_reg_loss += reg_loss;
                diff[sbs.values[node.data_offset + idx]] += lambda * power * sgn_d * std::pow(d, power - 1);
              }

              //x+ side of voxel/////////////////////////////////////////////////////////////////////////////////////////

              if (i == val_size - 1)//for points on the side of voxel
              {
                auto key = std::pair(X, size * (node.pos_xy >> 16));

                TSBSVD temp;
                temp.coord_1 = y_vpos;
                temp.coord_2 = z_vpos;
                temp.dir = true;
                temp.idx = sbs.values[node.data_offset + idx];
                temp.dist_another_p = size / sbs.header.brick_size;
                temp.val_another_p = sbs.values_f[sbs.values[node.data_offset + idx - val_size * val_size]];
                temp.weight = 4;
                temp.counted = false;

                check_all_points(original_sbs, sbs, node, key, val_buf, diff, temp, -val_size * val_size, val_size, 1, 
                                 y_vpos, z_vpos, j, k, val_size, size, idx, total_reg_loss, power, lambda);

              }
              else
              {
                float d = (sbs.values_f[sbs.values[node.data_offset + idx]] - sbs.values_f[sbs.values[node.data_offset + idx + val_size * val_size]]) / size;
                float sgn_d = d > 0.0f ? 1.0f : -1.0f;
                d = std::abs(d);

                float reg_loss = std::pow(d, power);
                total_reg_loss += reg_loss;
                diff[sbs.values[node.data_offset + idx]] += lambda * power * sgn_d * std::pow(d, power - 1);
              }

              //y- side of voxel/////////////////////////////////////////////////////////////////////////////////////////

              if (j == 0)//for points on the side of voxel
              {
                auto key = std::pair(Y, size * (node.pos_xy & 0x0000FFFF));

                TSBSVD temp;
                temp.coord_1 = x_vpos;
                temp.coord_2 = z_vpos;
                temp.dir = false;
                temp.idx = sbs.values[node.data_offset + idx];
                temp.dist_another_p = size / sbs.header.brick_size;
                temp.val_another_p = sbs.values_f[sbs.values[node.data_offset + idx + val_size]];
                temp.weight = 4;
                temp.counted = false;

                check_all_points(original_sbs, sbs, node, key, val_buf, diff, temp, val_size, val_size * val_size, 1, 
                                 x_vpos, z_vpos, i, k, val_size, size, idx, total_reg_loss, power, lambda);

              }
              else
              {
                float d = (sbs.values_f[sbs.values[node.data_offset + idx]] - sbs.values_f[sbs.values[node.data_offset + idx - val_size]]) / size;
                float sgn_d = d > 0.0f ? 1.0f : -1.0f;
                d = std::abs(d);

                float reg_loss = std::pow(d, power);
                total_reg_loss += reg_loss;
                diff[sbs.values[node.data_offset + idx]] += lambda * power * sgn_d * std::pow(d, power - 1);
              }

              //y+ side of voxel/////////////////////////////////////////////////////////////////////////////////////////

              if (j == val_size - 1)//for points on the side of voxel
              {
                auto key = std::pair(Y, size * (node.pos_xy & 0x0000FFFF));

                TSBSVD temp;
                temp.coord_1 = x_vpos;
                temp.coord_2 = z_vpos;
                temp.dir = true;
                temp.idx = sbs.values[node.data_offset + idx];
                temp.dist_another_p = size / sbs.header.brick_size;
                temp.val_another_p = sbs.values_f[sbs.values[node.data_offset + idx - val_size]];
                temp.weight = 4;
                temp.counted = false;

                check_all_points(original_sbs, sbs, node, key, val_buf, diff, temp, -val_size, val_size * val_size, 1, 
                                 x_vpos, z_vpos, i, k, val_size, size, idx, total_reg_loss, power, lambda);

              }
              else
              {
                float d = (sbs.values_f[sbs.values[node.data_offset + idx]] - sbs.values_f[sbs.values[node.data_offset + idx + val_size]]) / size;
                float sgn_d = d > 0.0f ? 1.0f : -1.0f;
                d = std::abs(d);

                float reg_loss = std::pow(d, power);
                total_reg_loss += reg_loss;
                diff[sbs.values[node.data_offset + idx]] += lambda * power * sgn_d * std::pow(d, power - 1);
              }

              //z- side of voxel/////////////////////////////////////////////////////////////////////////////////////////

              if (k == 0)//for points on the side of voxel
              {
                auto key = std::pair(Z, size * (node.pos_z_lod_size >> 16));

                TSBSVD temp;
                temp.coord_1 = x_vpos;
                temp.coord_2 = z_vpos;
                temp.dir = false;
                temp.idx = sbs.values[node.data_offset + idx];
                temp.dist_another_p = size / sbs.header.brick_size;
                temp.val_another_p = sbs.values_f[sbs.values[node.data_offset + idx + 1]];
                temp.weight = 4;
                temp.counted = false;

                check_all_points(original_sbs, sbs, node, key, val_buf, diff, temp, 1, val_size * val_size, val_size, 
                                 x_vpos, y_vpos, i, j, val_size, size, idx, total_reg_loss, power, lambda);

              }
              else
              {
                float d = (sbs.values_f[sbs.values[node.data_offset + idx]] - sbs.values_f[sbs.values[node.data_offset + idx - 1]]) / size;
                float sgn_d = d > 0.0f ? 1.0f : -1.0f;
                d = std::abs(d);

                float reg_loss = std::pow(d, power);
                total_reg_loss += reg_loss;
                diff[sbs.values[node.data_offset + idx]] += lambda * power * sgn_d * std::pow(d, power - 1);
              }

              //z+ side of voxel/////////////////////////////////////////////////////////////////////////////////////////

              if (k == val_size - 1)//for points on the side of voxel
              {
                auto key = std::pair(Z, size * (node.pos_z_lod_size >> 16));

                TSBSVD temp;
                temp.coord_1 = x_vpos;
                temp.coord_2 = z_vpos;
                temp.dir = true;
                temp.idx = sbs.values[node.data_offset + idx];
                temp.dist_another_p = size / sbs.header.brick_size;
                temp.val_another_p = sbs.values_f[sbs.values[node.data_offset + idx - 1]];
                temp.weight = 4;
                temp.counted = false;

                check_all_points(original_sbs, sbs, node, key, val_buf, diff, temp, -1, val_size * val_size, val_size, 
                                 x_vpos, y_vpos, i, j, val_size, size, idx, total_reg_loss, power, lambda);

              }
              else
              {
                float d = (sbs.values_f[sbs.values[node.data_offset + idx]] - sbs.values_f[sbs.values[node.data_offset + idx + 1]]) / size;
                float sgn_d = d > 0.0f ? 1.0f : -1.0f;
                d = std::abs(d);

                float reg_loss = std::pow(d, power);
                total_reg_loss += reg_loss;
                diff[sbs.values[node.data_offset + idx]] += lambda * power * sgn_d * std::pow(d, power - 1);
              }
            }
          }
        }
      }

      val_buf.erase(val_buf.begin(), val_buf.end());

      optimizer.step(sbs.values_f.data(), diff.data(), step);

      printf("step = %d, loss = %f(%f + %f*%f)\n", step, total_loss + lambda*total_reg_loss, total_loss, lambda, total_reg_loss);
      printf("nodes: %d\n", sbs.nodes.size());
      //for (int i = 0; i < grid.data.size(); i++)
      //{
      //  grid.data[i] -= lr * diff[i];
      //}
    }

    return sbs;
  }*/

  //new function

  typedef struct direct_sbs_value_data
  {
    uint32_t val;
    uint32_t val_c1;
    uint32_t val_c2;
    uint32_t val_c12;
    float distance;
    float coef_1;
    float coef_2;
    bool is_filled;
    
  } DSBSVD;

  typedef struct precounted_sbs_value_data
  {
    std::set<uint32_t> idxs;
    DSBSVD X_minus;
    DSBSVD X_plus;
    DSBSVD Y_minus;
    DSBSVD Y_plus;
    DSBSVD Z_minus;
    DSBSVD Z_plus;
    
  } PCSBSVD;

  void add_point_data(std::map<std::pair<std::pair<uint32_t, uint32_t>, uint32_t>, PCSBSVD> &border_vals, 
                      std::map<std::pair<coord, uint32_t>, std::vector<std::pair<uint32_t, uint32_t>>> &not_full_data, 
                      SdfSBS sbs, 
                      SdfSBSNode node, 
                      std::pair<std::pair<uint32_t, uint32_t>, uint32_t> key,
                      uint32_t idx, 
                      uint32_t size,
                      float mult,
                      uint32_t val_size,
                      uint32_t x_off, uint32_t y_off, uint32_t z_off,
                      uint32_t x_pos, uint32_t y_pos, uint32_t z_pos,
                      uint32_t c1, uint32_t c2, coord plate)
  {
    bool is_new = false;
    if (border_vals.find(key) != border_vals.end())
    {
      border_vals[key].idxs.insert(sbs.values[node.data_offset + idx]);
      //if (sbs.values[node.data_offset + idx] == 0) printf("%d %d %d\n", key.first.first, key.first.second, key.second);
      //printf("%u, %u, %u, %u - AAA\n", key.first.first, key.first.second, key.second, sbs.values[node.data_offset + idx]);
    }
    else
    {
      //if (sbs.values[node.data_offset + idx] == 0) printf("%d %d %d\n", key.first.first, key.first.second, key.second);
      is_new = true;
      border_vals[key] = {};
      border_vals[key].idxs.clear();
      border_vals[key].idxs.insert(sbs.values[node.data_offset + idx]);
      border_vals[key].X_minus.is_filled = false;
      border_vals[key].X_plus.is_filled = false;
      border_vals[key].Y_minus.is_filled = false;
      border_vals[key].Y_plus.is_filled = false;
      border_vals[key].Z_minus.is_filled = false;
      border_vals[key].Z_plus.is_filled = false;

      
    }
    if (x_off != val_size - 1 && !border_vals[key].X_plus.is_filled)
    {
      border_vals[key].X_plus.coef_1 = 1;
      border_vals[key].X_plus.coef_2 = 1;
      border_vals[key].X_plus.distance = mult;
      border_vals[key].X_plus.val = sbs.values[node.data_offset + idx + val_size * val_size];
      border_vals[key].X_plus.val_c1 = border_vals[key].X_plus.val;
      border_vals[key].X_plus.val_c2 = border_vals[key].X_plus.val;
      border_vals[key].X_plus.val_c12 = border_vals[key].X_plus.val;
      border_vals[key].X_plus.is_filled = true;

      //if (sbs.values[node.data_offset + idx] == 0) printf("x+ %d\n", sbs.values[node.data_offset + idx + val_size * val_size]);
    }
    if (x_off != 0 && !border_vals[key].X_minus.is_filled)
    {
      border_vals[key].X_minus.coef_1 = 1;
      border_vals[key].X_minus.coef_2 = 1;
      border_vals[key].X_minus.distance = mult;
      border_vals[key].X_minus.val = sbs.values[node.data_offset + idx - val_size * val_size];
      border_vals[key].X_minus.val_c1 = border_vals[key].X_minus.val;
      border_vals[key].X_minus.val_c2 = border_vals[key].X_minus.val;
      border_vals[key].X_minus.val_c12 = border_vals[key].X_minus.val;
      border_vals[key].X_minus.is_filled = true;

      //if (sbs.values[node.data_offset + idx] == 0) printf("x- %d\n", sbs.values[node.data_offset + idx - val_size * val_size]);
    }

    if (y_off != 0 && !border_vals[key].Y_minus.is_filled) 
    {
      border_vals[key].Y_minus.coef_1 = 1;
      border_vals[key].Y_minus.coef_2 = 1;
      border_vals[key].Y_minus.distance = mult;
      border_vals[key].Y_minus.val = sbs.values[node.data_offset + idx - val_size];
      border_vals[key].Y_minus.val_c1 = border_vals[key].Y_minus.val;
      border_vals[key].Y_minus.val_c2 = border_vals[key].Y_minus.val;
      border_vals[key].Y_minus.val_c12 = border_vals[key].Y_minus.val;
      border_vals[key].Y_minus.is_filled = true;

      //if (sbs.values[node.data_offset + idx] == 0) printf("y- %d\n", sbs.values[node.data_offset + idx - val_size]);
    }
    
    if (y_off != val_size - 1 && !border_vals[key].Y_plus.is_filled) 
    {
      border_vals[key].Y_plus.coef_1 = 1;
      border_vals[key].Y_plus.coef_2 = 1;
      border_vals[key].Y_plus.distance = mult;
      border_vals[key].Y_plus.val = sbs.values[node.data_offset + idx + val_size];
      border_vals[key].Y_plus.val_c1 = border_vals[key].Y_plus.val;
      border_vals[key].Y_plus.val_c2 = border_vals[key].Y_plus.val;
      border_vals[key].Y_plus.val_c12 = border_vals[key].Y_plus.val;
      border_vals[key].Y_plus.is_filled = true;

      //if (sbs.values[node.data_offset + idx] == 0) printf("y+ %d\n", sbs.values[node.data_offset + idx + val_size]);
    }

    if (z_off != 0 && !border_vals[key].Z_minus.is_filled) 
    {
      border_vals[key].Z_minus.coef_1 = 1;
      border_vals[key].Z_minus.coef_2 = 1;
      border_vals[key].Z_minus.distance = mult;
      border_vals[key].Z_minus.val = sbs.values[node.data_offset + idx - 1];
      border_vals[key].Z_minus.val_c1 = border_vals[key].Z_minus.val;
      border_vals[key].Z_minus.val_c2 = border_vals[key].Z_minus.val;
      border_vals[key].Z_minus.val_c12 = border_vals[key].Z_minus.val;
      border_vals[key].Z_minus.is_filled = true;

      //if (sbs.values[node.data_offset + idx] == 0) printf("z- %d\n", sbs.values[node.data_offset + idx - 1]);
    }
    
    if (z_off != val_size - 1 && !border_vals[key].Z_plus.is_filled) 
    {
      border_vals[key].Z_plus.coef_1 = 1;
      border_vals[key].Z_plus.coef_2 = 1;
      border_vals[key].Z_plus.distance = mult;
      border_vals[key].Z_plus.val = sbs.values[node.data_offset + idx + 1];
      border_vals[key].Z_plus.val_c1 = border_vals[key].Z_plus.val;
      border_vals[key].Z_plus.val_c2 = border_vals[key].Z_plus.val;
      border_vals[key].Z_plus.val_c12 = border_vals[key].Z_plus.val;
      border_vals[key].Z_plus.is_filled = true;

      //if (sbs.values[node.data_offset + idx] == 0) printf("z+ %d\n", sbs.values[node.data_offset + idx + 1]);
    }

    //go on the X plate and delete all filled X elems and elems between our YZ's
    if (c1 != val_size - 1 && c2 != val_size - 1)
    {
      std::pair<coord, uint32_t> key_plate;
      if (plate == X)
      {
        key_plate = {X, x_pos + (size / sbs.header.brick_size) * x_off};
      }
      else if (plate == Y)
      {
        key_plate = {Y, y_pos + (size / sbs.header.brick_size) * y_off};
      }
      else
      {
        key_plate = {Z, z_pos + (size / sbs.header.brick_size) * z_off};
      }
      if (not_full_data.find(key_plate) != not_full_data.end())
      {
        for (int i = 0; i < not_full_data[key_plate].size(); )
        {
          auto p = not_full_data[key_plate][i];
          if (plate == X)
          {
            if (border_vals[{{x_pos + (size / sbs.header.brick_size) * x_off, p.first}, p.second}].X_minus.is_filled && border_vals[{{x_pos + (size / sbs.header.brick_size) * x_off, p.first}, p.second}].X_plus.is_filled)
            {
              not_full_data[key_plate].erase(not_full_data[key_plate].begin() + i);
              continue;
            }
            if (p.first >= y_pos + (size / sbs.header.brick_size) * c1 && p.first <= y_pos + (size / sbs.header.brick_size) * (c1 + 1) && 
                p.second >= z_pos + (size / sbs.header.brick_size) * c2 && p.second <= z_pos + (size / sbs.header.brick_size) * (c2 + 1))
            {
              if (x_off == 0 && !border_vals[{{x_pos + (size / sbs.header.brick_size) * x_off, p.first}, p.second}].X_plus.is_filled)
              {
                //if (*border_vals[{{x_pos + (size / sbs.header.brick_size) * x_off, p.first}, p.second}].idxs.begin() == 0) printf("new x+ %d\n", sbs.values[node.data_offset + idx + val_size * val_size]);
                border_vals[{{x_pos + (size / sbs.header.brick_size) * x_off, p.first}, p.second}].X_plus.is_filled = true;
                border_vals[{{x_pos + (size / sbs.header.brick_size) * x_off, p.first}, p.second}].X_plus.coef_1 = (y_pos + (size / sbs.header.brick_size) * (c1 + 1) - p.first) / (size / sbs.header.brick_size);
                border_vals[{{x_pos + (size / sbs.header.brick_size) * x_off, p.first}, p.second}].X_plus.coef_2 = (z_pos + (size / sbs.header.brick_size) * (c2 + 1) - p.second) / (size / sbs.header.brick_size);
                border_vals[{{x_pos + (size / sbs.header.brick_size) * x_off, p.first}, p.second}].X_plus.distance = mult;
                border_vals[{{x_pos + (size / sbs.header.brick_size) * x_off, p.first}, p.second}].X_plus.val = sbs.values[node.data_offset + idx + val_size * val_size];
                border_vals[{{x_pos + (size / sbs.header.brick_size) * x_off, p.first}, p.second}].X_plus.val_c1 = sbs.values[node.data_offset + idx + val_size * val_size + val_size];
                border_vals[{{x_pos + (size / sbs.header.brick_size) * x_off, p.first}, p.second}].X_plus.val_c2 = sbs.values[node.data_offset + idx + val_size * val_size + 1];
                border_vals[{{x_pos + (size / sbs.header.brick_size) * x_off, p.first}, p.second}].X_plus.val_c12 = sbs.values[node.data_offset + idx + val_size * val_size + val_size + 1];

                not_full_data[key_plate].erase(not_full_data[key_plate].begin() + i);
              }
              else if (x_off == val_size - 1 && !border_vals[{{x_pos + (size / sbs.header.brick_size) * x_off, p.first}, p.second}].X_minus.is_filled)
              {
                //if (*border_vals[{{x_pos + (size / sbs.header.brick_size) * x_off, p.first}, p.second}].idxs.begin() == 0) printf("new x- %d %d %d %d %d %d\n", sbs.values[node.data_offset + idx - val_size * val_size], x_pos + (size / sbs.header.brick_size) * x_off, p.first, p.second, idx, x_off);
                border_vals[{{x_pos + (size / sbs.header.brick_size) * x_off, p.first}, p.second}].X_minus.is_filled = true;
                border_vals[{{x_pos + (size / sbs.header.brick_size) * x_off, p.first}, p.second}].X_minus.coef_1 = (y_pos + (size / sbs.header.brick_size) * (c1 + 1) - p.first) / (size / sbs.header.brick_size);
                border_vals[{{x_pos + (size / sbs.header.brick_size) * x_off, p.first}, p.second}].X_minus.coef_2 = (z_pos + (size / sbs.header.brick_size) * (c2 + 1) - p.second) / (size / sbs.header.brick_size);
                border_vals[{{x_pos + (size / sbs.header.brick_size) * x_off, p.first}, p.second}].X_minus.distance = mult;
                border_vals[{{x_pos + (size / sbs.header.brick_size) * x_off, p.first}, p.second}].X_minus.val = sbs.values[node.data_offset + idx - val_size * val_size];
                border_vals[{{x_pos + (size / sbs.header.brick_size) * x_off, p.first}, p.second}].X_minus.val_c1 = sbs.values[node.data_offset + idx - val_size * val_size + val_size];
                border_vals[{{x_pos + (size / sbs.header.brick_size) * x_off, p.first}, p.second}].X_minus.val_c2 = sbs.values[node.data_offset + idx - val_size * val_size + 1];
                border_vals[{{x_pos + (size / sbs.header.brick_size) * x_off, p.first}, p.second}].X_minus.val_c12 = sbs.values[node.data_offset + idx - val_size * val_size + val_size + 1];

                not_full_data[key_plate].erase(not_full_data[key_plate].begin() + i);
              }
              else
              {
                ++i;
              }
            }
            else
            {
              ++i;
            }
          }
          else if (plate == Y)/////////////////////////////////////////////////////////////////////////////////////////////
          {
            if (border_vals[{{p.first, y_pos + (size / sbs.header.brick_size) * y_off}, p.second}].Y_minus.is_filled && border_vals[{{p.first, y_pos + (size / sbs.header.brick_size) * y_off}, p.second}].Y_plus.is_filled)
            {
              not_full_data[key_plate].erase(not_full_data[key_plate].begin() + i);
              continue;
            }
            if (p.first >= x_pos + (size / sbs.header.brick_size) * c1 && p.first <= x_pos + (size / sbs.header.brick_size) * (c1 + 1) && 
                p.second >= z_pos + (size / sbs.header.brick_size) * c2 && p.second <= z_pos + (size / sbs.header.brick_size) * (c2 + 1))
            {
              if (y_off == 0 && !border_vals[{{p.first, y_pos + (size / sbs.header.brick_size) * y_off}, p.second}].Y_plus.is_filled)
              {
                //if (*border_vals[{{p.first, y_pos + (size / sbs.header.brick_size) * y_off}, p.second}].idxs.begin() == 0) printf("new y+ %d %d %d %d\n", sbs.values[node.data_offset + idx + val_size]);
                border_vals[{{p.first, y_pos + (size / sbs.header.brick_size) * y_off}, p.second}].Y_plus.is_filled = true;
                border_vals[{{p.first, y_pos + (size / sbs.header.brick_size) * y_off}, p.second}].Y_plus.coef_1 = (x_pos + (size / sbs.header.brick_size) * (c1 + 1) - p.first) / (size / sbs.header.brick_size);
                border_vals[{{p.first, y_pos + (size / sbs.header.brick_size) * y_off}, p.second}].Y_plus.coef_2 = (z_pos + (size / sbs.header.brick_size) * (c2 + 1) - p.second) / (size / sbs.header.brick_size);
                border_vals[{{p.first, y_pos + (size / sbs.header.brick_size) * y_off}, p.second}].Y_plus.distance = mult;
                border_vals[{{p.first, y_pos + (size / sbs.header.brick_size) * y_off}, p.second}].Y_plus.val = sbs.values[node.data_offset + idx + val_size];
                border_vals[{{p.first, y_pos + (size / sbs.header.brick_size) * y_off}, p.second}].Y_plus.val_c1 = sbs.values[node.data_offset + idx + val_size + val_size * val_size];
                border_vals[{{p.first, y_pos + (size / sbs.header.brick_size) * y_off}, p.second}].Y_plus.val_c2 = sbs.values[node.data_offset + idx + val_size + 1];
                border_vals[{{p.first, y_pos + (size / sbs.header.brick_size) * y_off}, p.second}].Y_plus.val_c12 = sbs.values[node.data_offset + idx + val_size + val_size * val_size + 1];

                not_full_data[key_plate].erase(not_full_data[key_plate].begin() + i);
              }
              else if (y_off == val_size - 1 && !border_vals[{{p.first, y_pos + (size / sbs.header.brick_size) * y_off}, p.second}].Y_minus.is_filled)
              {
                //if (*border_vals[{{p.first, y_pos + (size / sbs.header.brick_size) * y_off}, p.second}].idxs.begin() == 0) printf("new y- %d %d %d %d %d\n", sbs.values[node.data_offset + idx - val_size], p.first, y_pos + (size / sbs.header.brick_size) * y_off, p.second, idx);
                border_vals[{{p.first, y_pos + (size / sbs.header.brick_size) * y_off}, p.second}].Y_minus.is_filled = true;
                border_vals[{{p.first, y_pos + (size / sbs.header.brick_size) * y_off}, p.second}].Y_minus.coef_1 = (x_pos + (size / sbs.header.brick_size) * (c1 + 1) - p.first) / (size / sbs.header.brick_size);
                border_vals[{{p.first, y_pos + (size / sbs.header.brick_size) * y_off}, p.second}].Y_minus.coef_2 = (z_pos + (size / sbs.header.brick_size) * (c2 + 1) - p.second) / (size / sbs.header.brick_size);
                border_vals[{{p.first, y_pos + (size / sbs.header.brick_size) * y_off}, p.second}].Y_minus.distance = mult;
                border_vals[{{p.first, y_pos + (size / sbs.header.brick_size) * y_off}, p.second}].Y_minus.val = sbs.values[node.data_offset + idx - val_size];
                border_vals[{{p.first, y_pos + (size / sbs.header.brick_size) * y_off}, p.second}].Y_minus.val_c1 = sbs.values[node.data_offset + idx - val_size + val_size * val_size];
                border_vals[{{p.first, y_pos + (size / sbs.header.brick_size) * y_off}, p.second}].Y_minus.val_c2 = sbs.values[node.data_offset + idx - val_size + 1];
                border_vals[{{p.first, y_pos + (size / sbs.header.brick_size) * y_off}, p.second}].Y_minus.val_c12 = sbs.values[node.data_offset + idx - val_size + val_size * val_size + 1];

                not_full_data[key_plate].erase(not_full_data[key_plate].begin() + i);
              }
              else
              {
                ++i;
              }
            }
            else
            {
              ++i;
            }
          }
          else/////////////////////////////////////////////////////////////////////////////////////////////////////////////
          {
            if (border_vals[{{p.first, p.second}, z_pos + (size / sbs.header.brick_size) * z_off}].Z_minus.is_filled && border_vals[{{p.first, p.second}, z_pos + (size / sbs.header.brick_size) * z_off}].Z_plus.is_filled)
            {
              not_full_data[key_plate].erase(not_full_data[key_plate].begin() + i);
              continue;
            }
            if (p.first >= x_pos + (size / sbs.header.brick_size) * c1 && p.first <= x_pos + (size / sbs.header.brick_size) * (c1 + 1) && 
                p.second >= y_pos + (size / sbs.header.brick_size) * c2 && p.second <= y_pos + (size / sbs.header.brick_size) * (c2 + 1))
            {
              if (z_off == 0 && !border_vals[{{p.first, p.second}, z_pos + (size / sbs.header.brick_size) * z_off}].Z_plus.is_filled)
              {
                //if (*border_vals[{{p.first, p.second}, z_pos + (size / sbs.header.brick_size) * z_off}].idxs.begin() == 0) printf("new z+ %d\n", sbs.values[node.data_offset + idx + 1]);
                border_vals[{{p.first, p.second}, z_pos + (size / sbs.header.brick_size) * z_off}].Z_plus.is_filled = true;
                border_vals[{{p.first, p.second}, z_pos + (size / sbs.header.brick_size) * z_off}].Z_plus.coef_1 = (x_pos + (size / sbs.header.brick_size) * (c1 + 1) - p.first) / (size / sbs.header.brick_size);
                border_vals[{{p.first, p.second}, z_pos + (size / sbs.header.brick_size) * z_off}].Z_plus.coef_2 = (y_pos + (size / sbs.header.brick_size) * (c2 + 1) - p.second) / (size / sbs.header.brick_size);
                border_vals[{{p.first, p.second}, z_pos + (size / sbs.header.brick_size) * z_off}].Z_plus.distance = mult;
                border_vals[{{p.first, p.second}, z_pos + (size / sbs.header.brick_size) * z_off}].Z_plus.val = sbs.values[node.data_offset + idx + 1];
                border_vals[{{p.first, p.second}, z_pos + (size / sbs.header.brick_size) * z_off}].Z_plus.val_c1 = sbs.values[node.data_offset + idx + 1 + val_size * val_size];
                border_vals[{{p.first, p.second}, z_pos + (size / sbs.header.brick_size) * z_off}].Z_plus.val_c2 = sbs.values[node.data_offset + idx + 1 + val_size];
                border_vals[{{p.first, p.second}, z_pos + (size / sbs.header.brick_size) * z_off}].Z_plus.val_c12 = sbs.values[node.data_offset + idx + 1 + val_size * val_size + val_size];

                not_full_data[key_plate].erase(not_full_data[key_plate].begin() + i);
              }
              else if (z_off == val_size - 1 && !border_vals[{{p.first, p.second}, z_pos + (size / sbs.header.brick_size) * z_off}].Z_minus.is_filled)
              {
                //if (*border_vals[{{p.first, p.second}, z_pos + (size / sbs.header.brick_size) * z_off}].idxs.begin() == 0) printf("new z- %d %d %d %d %d\n", sbs.values[node.data_offset + idx - 1], p.first, p.second, z_pos + (size / sbs.header.brick_size) * z_off, idx);
                border_vals[{{p.first, p.second}, z_pos + (size / sbs.header.brick_size) * z_off}].Z_minus.is_filled = true;
                border_vals[{{p.first, p.second}, z_pos + (size / sbs.header.brick_size) * z_off}].Z_minus.coef_1 = (x_pos + (size / sbs.header.brick_size) * (c1 + 1) - p.first) / (size / sbs.header.brick_size);
                border_vals[{{p.first, p.second}, z_pos + (size / sbs.header.brick_size) * z_off}].Z_minus.coef_2 = (y_pos + (size / sbs.header.brick_size) * (c2 + 1) - p.second) / (size / sbs.header.brick_size);
                border_vals[{{p.first, p.second}, z_pos + (size / sbs.header.brick_size) * z_off}].Z_minus.distance = mult;
                border_vals[{{p.first, p.second}, z_pos + (size / sbs.header.brick_size) * z_off}].Z_minus.val = sbs.values[node.data_offset + idx - 1];
                border_vals[{{p.first, p.second}, z_pos + (size / sbs.header.brick_size) * z_off}].Z_minus.val_c1 = sbs.values[node.data_offset + idx - 1 + val_size * val_size];
                border_vals[{{p.first, p.second}, z_pos + (size / sbs.header.brick_size) * z_off}].Z_minus.val_c2 = sbs.values[node.data_offset + idx - 1 + val_size];
                border_vals[{{p.first, p.second}, z_pos + (size / sbs.header.brick_size) * z_off}].Z_minus.val_c12 = sbs.values[node.data_offset + idx - 1 + val_size * val_size + val_size];

                not_full_data[key_plate].erase(not_full_data[key_plate].begin() + i);
              }
              else
              {
                ++i;
              }
            }
            else
            {
              ++i;
            }
          }
        }
      }
    }
    
    if (is_new && (z_off == 0 || z_off == val_size - 1))//paste at the end
    {
      if (not_full_data.find({Z, z_pos + (size / sbs.header.brick_size) * z_off}) != not_full_data.end())
      {
        not_full_data[{Z, z_pos + (size / sbs.header.brick_size) * z_off}].push_back({x_pos + (size / sbs.header.brick_size) * x_off, y_pos + (size / sbs.header.brick_size) * y_off});
      }
      else
      {
        not_full_data[{Z, z_pos + (size / sbs.header.brick_size) * z_off}] = {{x_pos + (size / sbs.header.brick_size) * x_off, y_pos + (size / sbs.header.brick_size) * y_off}};
      }
    }
    if (is_new && (y_off == 0 || y_off == val_size - 1))//paste at the end
    {
      if (not_full_data.find({Y, y_pos + (size / sbs.header.brick_size) * y_off}) != not_full_data.end())
      {
        not_full_data[{Y, y_pos + (size / sbs.header.brick_size) * y_off}].push_back({x_pos + (size / sbs.header.brick_size) * x_off, z_pos + (size / sbs.header.brick_size) * z_off});
      }
      else
      {
        not_full_data[{Y, y_pos + (size / sbs.header.brick_size) * y_off}] = {{x_pos + (size / sbs.header.brick_size) * x_off, z_pos + (size / sbs.header.brick_size) * z_off}};
      }
    }
    if (is_new && (x_off == 0 || x_off == val_size - 1))
    {
      if (not_full_data.find({X, x_pos + (size / sbs.header.brick_size) * x_off}) != not_full_data.end())//paste at the end
      {
        not_full_data[{X, x_pos + (size / sbs.header.brick_size) * x_off}].push_back({y_pos + (size / sbs.header.brick_size) * y_off, z_pos + (size / sbs.header.brick_size) * z_off});
      }
      else
      {
        not_full_data[{X, x_pos + (size / sbs.header.brick_size) * x_off}] = {{y_pos + (size / sbs.header.brick_size) * y_off, z_pos + (size / sbs.header.brick_size) * z_off}};
      }
    }
  }

  SdfSBS sdf_SBS_smoother(const SdfSBS &original_sbs, float power, float lr, float lambda, unsigned steps)
  {
    SdfSBS sbs;
    sbs.header = original_sbs.header;
    sbs.nodes = original_sbs.nodes;
    sbs.values = original_sbs.values;
    sbs.values_f = original_sbs.values_f;

    //printf("AAA %d\n", sbs.header.bytes_per_value);

    if ((original_sbs.header.aux_data & SDF_SBS_NODE_LAYOUT_MASK) != SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F)
    {
      return sbs;
    }

    std::vector<float> diff(sbs.values_f.size());

    AdamOptimizer optimizer(sbs.values_f.size(), lr);

    //std::map<std::pair<coord, uint32_t>, std::vector<TSBSVD>> val_buf;
    std::map<std::pair<std::pair<uint32_t, uint32_t>, uint32_t>, PCSBSVD> border_vals;
    std::map<std::pair<coord, uint32_t>, std::vector<std::pair<uint32_t, uint32_t>>> not_full_data;

    uint32_t val_size = sbs.header.brick_size + 1;

    std::sort(sbs.nodes.begin(), sbs.nodes.end(), [](SdfSBSNode a, SdfSBSNode b){return (a.pos_z_lod_size & 0x0000FFFF) > (b.pos_z_lod_size & 0x0000FFFF);});

    for (auto node : sbs.nodes)
    {

      uint32_t size = sbs.header.brick_size * (UINT16_MAX + 1) / (node.pos_z_lod_size & 0x0000FFFF);
      //printf("%d\n", node.pos_z_lod_size & 0x0000FFFF);
      float mult = (sbs.nodes[0].pos_z_lod_size & 0x0000FFFF) / (node.pos_z_lod_size & 0x0000FFFF);

      uint32_t x_pos = size * (node.pos_xy >> 16);
      uint32_t y_pos = size * (node.pos_xy & 0x0000FFFF);
      uint32_t z_pos = size * (node.pos_z_lod_size >> 16);

      for (int c1 = 0; c1 < val_size; ++c1)
      {
        for (int c2 = 0; c2 < val_size; ++c2)
        {
          {//X = 0
            std::pair<std::pair<uint32_t, uint32_t>, uint32_t> key = {{x_pos, y_pos + (size / sbs.header.brick_size) * c1}, z_pos + (size / sbs.header.brick_size) * c2};
            uint32_t idx = c1 * val_size + c2;
            add_point_data(border_vals, not_full_data, sbs, node, key, idx, size, mult, val_size, 
                           0, c1, c2, x_pos, y_pos, z_pos, c1, c2, X);
          }
          {//X = val_size - 1
            std::pair<std::pair<uint32_t, uint32_t>, uint32_t> key = {{x_pos + (size / sbs.header.brick_size) * (val_size - 1), y_pos + (size / sbs.header.brick_size) * c1}, z_pos + (size / sbs.header.brick_size) * c2};
            uint32_t idx = (val_size - 1) * val_size * val_size + c1 * val_size + c2;
            add_point_data(border_vals, not_full_data, sbs, node, key, idx, size, mult, val_size, 
                           val_size - 1, c1, c2, x_pos, y_pos, z_pos, c1, c2, X);
          }

          {//Y = 0
            std::pair<std::pair<uint32_t, uint32_t>, uint32_t> key = {{x_pos + (size / sbs.header.brick_size) * c1, y_pos}, z_pos + (size / sbs.header.brick_size) * c2};
            uint32_t idx = c1 * val_size * val_size + c2;
            add_point_data(border_vals, not_full_data, sbs, node, key, idx, size, mult, val_size, 
                           c1, 0, c2, x_pos, y_pos, z_pos, c1, c2, Y);
          }
          {//Y = val_size - 1
            std::pair<std::pair<uint32_t, uint32_t>, uint32_t> key = {{x_pos + (size / sbs.header.brick_size) * c1, y_pos + (size / sbs.header.brick_size) * (val_size - 1)}, z_pos + (size / sbs.header.brick_size) * c2};
            uint32_t idx = c1 * val_size * val_size + (val_size - 1) * val_size + c2;
            add_point_data(border_vals, not_full_data, sbs, node, key, idx, size, mult, val_size, 
                           c1, val_size - 1, c2, x_pos, y_pos, z_pos, c1, c2, Y);
          }

          {//Z = 0
            std::pair<std::pair<uint32_t, uint32_t>, uint32_t> key = {{x_pos + (size / sbs.header.brick_size) * c1, y_pos + (size / sbs.header.brick_size) * c2}, z_pos};
            uint32_t idx = c1 * val_size * val_size + c2 * val_size;
            add_point_data(border_vals, not_full_data, sbs, node, key, idx, size, mult, val_size, 
                           c1, c2, 0, x_pos, y_pos, z_pos, c1, c2, Z);
          }
          {//Z = val_size - 1
            std::pair<std::pair<uint32_t, uint32_t>, uint32_t> key = {{x_pos + (size / sbs.header.brick_size) * c1, y_pos + (size / sbs.header.brick_size) * c2}, z_pos + (size / sbs.header.brick_size) * (val_size - 1)};
            uint32_t idx = c1 * val_size * val_size + c2 * val_size + val_size - 1;
            add_point_data(border_vals, not_full_data, sbs, node, key, idx, size, mult, val_size, 
                           c1, c2, val_size - 1, x_pos, y_pos, z_pos, c1, c2, Z);
          }
        }
      }
    }

    not_full_data.clear();

    for (int step = 0; step < steps; step++)
    {
      double total_loss = 0.0;
      double total_reg_loss = 0.0;
      std::fill(diff.begin(), diff.end(), 0.0f);

      for (auto node : sbs.nodes)
      {
        //printf("node: %d/%d\n", node_counter++, sbs.nodes.size());
        //uint32_t size = sbs.header.brick_size * (UINT16_MAX + 1) / (node.pos_z_lod_size & 0x0000FFFF);
        float mult = (sbs.nodes[0].pos_z_lod_size & 0x0000FFFF) / (node.pos_z_lod_size & 0x0000FFFF);

        //uint32_t x_pos = size * (node.pos_xy >> 16);
        //uint32_t y_pos = size * (node.pos_xy & 0x0000FFFF);
        //uint32_t z_pos = size * (node.pos_z_lod_size >> 16);

        //if (step == 0) printf("%d %d %d %d - %f\n", (node.pos_xy >> 16), (node.pos_xy & 0x0000FFFF), (node.pos_z_lod_size >> 16), (node.pos_z_lod_size & 0x0000FFFF), sbs.values_f[sbs.values[node.data_offset]]);

        //if (step == 1) std::fill(diff.begin(), diff.end(), 0.0f);

        //#pragma omp parallel for
        for (uint32_t i = 1; i < val_size - 1; ++i)//IGNORING BRICK_PAD
        {
          for (uint32_t j = 1; j < val_size - 1; ++j)//IGNORING BRICK_PAD
          {
            for (uint32_t k = 1; k < val_size - 1; ++k)//IGNORING BRICK_PAD
            {
              int idx = i * val_size * val_size + j * val_size + k;

              float loss =  (sbs.values_f[sbs.values[node.data_offset + idx]] - original_sbs.values_f[original_sbs.values[node.data_offset + idx]]) * 
                            (sbs.values_f[sbs.values[node.data_offset + idx]] - original_sbs.values_f[original_sbs.values[node.data_offset + idx]]);
              total_loss += loss;
              diff[sbs.values[node.data_offset + idx]] += 2.0f * (sbs.values_f[sbs.values[node.data_offset + idx]] - original_sbs.values_f[original_sbs.values[node.data_offset + idx]]);

              //if (step == 0) printf("%d %d %d %d - %f\n", (node.pos_xy >> 16) + i, (node.pos_xy & 0x0000FFFF) + j, (node.pos_z_lod_size >> 16) + k, (node.pos_z_lod_size & 0x0000FFFF), sbs.values_f[sbs.values[node.data_offset + idx]]);

              //x- side of voxel/////////////////////////////////////////////////////////////////////////////////////////

              {
                float d = (sbs.values_f[sbs.values[node.data_offset + idx]] - sbs.values_f[sbs.values[node.data_offset + idx - val_size * val_size]]) / mult;
                float sgn_d = d > 0.0f ? 1.0f : -1.0f;
                d = std::abs(d);

                float reg_loss = std::pow(d, power);
                total_reg_loss += reg_loss;
                diff[sbs.values[node.data_offset + idx]] += lambda * power * sgn_d * std::pow(d, power - 1);
              }

              //x+ side of voxel/////////////////////////////////////////////////////////////////////////////////////////

              
              {
                float d = (sbs.values_f[sbs.values[node.data_offset + idx]] - sbs.values_f[sbs.values[node.data_offset + idx + val_size * val_size]]) / mult;
                float sgn_d = d > 0.0f ? 1.0f : -1.0f;
                d = std::abs(d);

                float reg_loss = std::pow(d, power);
                total_reg_loss += reg_loss;
                diff[sbs.values[node.data_offset + idx]] += lambda * power * sgn_d * std::pow(d, power - 1);
              }

              //y- side of voxel/////////////////////////////////////////////////////////////////////////////////////////

              
              {
                float d = (sbs.values_f[sbs.values[node.data_offset + idx]] - sbs.values_f[sbs.values[node.data_offset + idx - val_size]]) / mult;
                float sgn_d = d > 0.0f ? 1.0f : -1.0f;
                d = std::abs(d);

                float reg_loss = std::pow(d, power);
                total_reg_loss += reg_loss;
                diff[sbs.values[node.data_offset + idx]] += lambda * power * sgn_d * std::pow(d, power - 1);
              }

              //y+ side of voxel/////////////////////////////////////////////////////////////////////////////////////////

              
              {
                float d = (sbs.values_f[sbs.values[node.data_offset + idx]] - sbs.values_f[sbs.values[node.data_offset + idx + val_size]]) / mult;
                float sgn_d = d > 0.0f ? 1.0f : -1.0f;
                d = std::abs(d);

                float reg_loss = std::pow(d, power);
                total_reg_loss += reg_loss;
                diff[sbs.values[node.data_offset + idx]] += lambda * power * sgn_d * std::pow(d, power - 1);
              }

              //z- side of voxel/////////////////////////////////////////////////////////////////////////////////////////

              
              {
                float d = (sbs.values_f[sbs.values[node.data_offset + idx]] - sbs.values_f[sbs.values[node.data_offset + idx - 1]]) / mult;
                float sgn_d = d > 0.0f ? 1.0f : -1.0f;
                d = std::abs(d);

                float reg_loss = std::pow(d, power);
                total_reg_loss += reg_loss;
                diff[sbs.values[node.data_offset + idx]] += lambda * power * sgn_d * std::pow(d, power - 1);
              }

              //z+ side of voxel/////////////////////////////////////////////////////////////////////////////////////////

              
              {
                float d = (sbs.values_f[sbs.values[node.data_offset + idx]] - sbs.values_f[sbs.values[node.data_offset + idx + 1]]) / mult;
                float sgn_d = d > 0.0f ? 1.0f : -1.0f;
                d = std::abs(d);

                float reg_loss = std::pow(d, power);
                total_reg_loss += reg_loss;
                diff[sbs.values[node.data_offset + idx]] += lambda * power * sgn_d * std::pow(d, power - 1);
              }
            }
          }
        }
      }

      //if (step == 1) std::fill(diff.begin(), diff.end(), 0.0f);
      //std::fill(diff.begin(), diff.end(), 0.0f);

      for (auto point : border_vals)
      {
        int cnt = 0;
        for (auto val_idx : point.second.idxs)
        {

          /*if (cnt++ > 0)
          {
            printf("AAA\n");
          }*/

          float loss =  (sbs.values_f[val_idx] - original_sbs.values_f[val_idx]) * 
                            (sbs.values_f[val_idx] - original_sbs.values_f[val_idx]);
          total_loss += loss;
          diff[val_idx] += 2.0f * (sbs.values_f[val_idx] - original_sbs.values_f[val_idx]);
          
          //x- side of voxel/////////////////////////////////////////////////////////////////////////////////////////

              
          {

            auto point_neigh = point.second.X_minus;
            if (point_neigh.is_filled)
            {
              float mean = sbs.values_f[point_neigh.val] * point_neigh.coef_1 * point_neigh.coef_2 + 
                           sbs.values_f[point_neigh.val_c1] * (1 - point_neigh.coef_1) * point_neigh.coef_2 + 
                           sbs.values_f[point_neigh.val_c2] * point_neigh.coef_1 * (1 - point_neigh.coef_2) + 
                           sbs.values_f[point_neigh.val_c12] * (1 - point_neigh.coef_1) * (1 - point_neigh.coef_2);
              float d = (sbs.values_f[val_idx] - mean) / point_neigh.distance;
              float sgn_d = d > 0.0f ? 1.0f : -1.0f;
              d = std::abs(d);

              float reg_loss = std::pow(d, power);
              total_reg_loss += reg_loss;
              diff[val_idx] += lambda * power * sgn_d * std::pow(d, power - 1);
            }
          }

          //x+ side of voxel/////////////////////////////////////////////////////////////////////////////////////////

          
          {

            auto point_neigh = point.second.X_plus;
            if (point_neigh.is_filled)
            {
              float mean = sbs.values_f[point_neigh.val] * point_neigh.coef_1 * point_neigh.coef_2 + 
                           sbs.values_f[point_neigh.val_c1] * (1 - point_neigh.coef_1) * point_neigh.coef_2 + 
                           sbs.values_f[point_neigh.val_c2] * point_neigh.coef_1 * (1 - point_neigh.coef_2) + 
                           sbs.values_f[point_neigh.val_c12] * (1 - point_neigh.coef_1) * (1 - point_neigh.coef_2);
              float d = (sbs.values_f[val_idx] - mean) / point_neigh.distance;
              float sgn_d = d > 0.0f ? 1.0f : -1.0f;
              d = std::abs(d);

              float reg_loss = std::pow(d, power);
              total_reg_loss += reg_loss;
              diff[val_idx] += lambda * power * sgn_d * std::pow(d, power - 1);
            }
          }

          //y- side of voxel/////////////////////////////////////////////////////////////////////////////////////////

          
          {

            auto point_neigh = point.second.Y_minus;
            if (point_neigh.is_filled)
            {
              float mean = sbs.values_f[point_neigh.val] * point_neigh.coef_1 * point_neigh.coef_2 + 
                           sbs.values_f[point_neigh.val_c1] * (1 - point_neigh.coef_1) * point_neigh.coef_2 + 
                           sbs.values_f[point_neigh.val_c2] * point_neigh.coef_1 * (1 - point_neigh.coef_2) + 
                           sbs.values_f[point_neigh.val_c12] * (1 - point_neigh.coef_1) * (1 - point_neigh.coef_2);
              float d = (sbs.values_f[val_idx] - mean) / point_neigh.distance;
              float sgn_d = d > 0.0f ? 1.0f : -1.0f;
              d = std::abs(d);

              float reg_loss = std::pow(d, power);
              total_reg_loss += reg_loss;
              diff[val_idx] += lambda * power * sgn_d * std::pow(d, power - 1);
            }
          }

          //y+ side of voxel/////////////////////////////////////////////////////////////////////////////////////////

          
          {

            auto point_neigh = point.second.Y_plus;
            if (point_neigh.is_filled)
            {
              float mean = sbs.values_f[point_neigh.val] * point_neigh.coef_1 * point_neigh.coef_2 + 
                           sbs.values_f[point_neigh.val_c1] * (1 - point_neigh.coef_1) * point_neigh.coef_2 + 
                           sbs.values_f[point_neigh.val_c2] * point_neigh.coef_1 * (1 - point_neigh.coef_2) + 
                           sbs.values_f[point_neigh.val_c12] * (1 - point_neigh.coef_1) * (1 - point_neigh.coef_2);
              float d = (sbs.values_f[val_idx] - mean) / point_neigh.distance;
              float sgn_d = d > 0.0f ? 1.0f : -1.0f;
              d = std::abs(d);

              float reg_loss = std::pow(d, power);
              total_reg_loss += reg_loss;
              diff[val_idx] += lambda * power * sgn_d * std::pow(d, power - 1);
            }
          }

          //z- side of voxel/////////////////////////////////////////////////////////////////////////////////////////

          
          {

            auto point_neigh = point.second.Z_minus;
            if (point_neigh.is_filled)
            {
              float mean = sbs.values_f[point_neigh.val] * point_neigh.coef_1 * point_neigh.coef_2 + 
                           sbs.values_f[point_neigh.val_c1] * (1 - point_neigh.coef_1) * point_neigh.coef_2 + 
                           sbs.values_f[point_neigh.val_c2] * point_neigh.coef_1 * (1 - point_neigh.coef_2) + 
                           sbs.values_f[point_neigh.val_c12] * (1 - point_neigh.coef_1) * (1 - point_neigh.coef_2);
              float d = (sbs.values_f[val_idx] - mean) / point_neigh.distance;
              float sgn_d = d > 0.0f ? 1.0f : -1.0f;
              d = std::abs(d);

              float reg_loss = std::pow(d, power);
              total_reg_loss += reg_loss;
              diff[val_idx] += lambda * power * sgn_d * std::pow(d, power - 1);
            }
          }

          //z+ side of voxel/////////////////////////////////////////////////////////////////////////////////////////

          
          {

            auto point_neigh = point.second.Z_plus;
            if (point_neigh.is_filled)
            {
              float mean = sbs.values_f[point_neigh.val] * point_neigh.coef_1 * point_neigh.coef_2 + 
                           sbs.values_f[point_neigh.val_c1] * (1 - point_neigh.coef_1) * point_neigh.coef_2 + 
                           sbs.values_f[point_neigh.val_c2] * point_neigh.coef_1 * (1 - point_neigh.coef_2) + 
                           sbs.values_f[point_neigh.val_c12] * (1 - point_neigh.coef_1) * (1 - point_neigh.coef_2);
              float d = (sbs.values_f[val_idx] - mean) / point_neigh.distance;
              float sgn_d = d > 0.0f ? 1.0f : -1.0f;
              d = std::abs(d);

              float reg_loss = std::pow(d, power);
              total_reg_loss += reg_loss;
              diff[val_idx] += lambda * power * sgn_d * std::pow(d, power - 1);
            }
          }

        }
      }

      optimizer.step(sbs.values_f.data(), diff.data(), step);

      //printf("step = %d, loss = %f(%f + %f*%f)\n", step, total_loss + lambda*total_reg_loss, total_loss, lambda, total_reg_loss);
      //printf("nodes: %d\n", sbs.nodes.size());
      //for (int i = 0; i < grid.data.size(); i++)
      //{
      //  grid.data[i] -= lr * diff[i];
      //}

    }
    //fclose(file_x);
    border_vals.clear();

    return sbs;
  }
}
