#pragma once

#include <cstdint>
#include <iostream>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>
#include <fstream>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>
#include "LiteMath.h"
#include "similarity_compression_impl.h"
#include "near_neighbor_common.h"

namespace scom
{
  class BallTree : public INNSearchAS
  {
  public:
    struct Node
    {
      unsigned l_idx, r_idx; // left and right children, = 0 <=> leaf
      unsigned centroid_index;
      unsigned start_index;
      unsigned count;
      float radius;
    };

    void build(const Dataset &dataset, int max_leaf_size) override;
    const float *get_closest_point(const float *query, float max_dist, float *dist_to_nearest = nullptr) const override;
    int scan_near(const float *query, float max_dist, ScanFunction callback) const override;
  
  private:  
    int build_rec(const Dataset &dataset, int max_leaf_size, int n, int *index, float *tmp_vec);
    int find_furthest_id(const Dataset &dataset, int id_from, int n, int *index);

    int m_dim; // dimension count of data
    std::vector<Node> m_nodes;
    std::vector<float> m_centroids_data; //m_dim * m_nodes.size();
    
    std::vector<DataPoint> m_points;
    std::vector<unsigned>  m_original_ids; //m_points.size();
    std::vector<float> m_points_data; //m_dim * m_points.size();
  };
}