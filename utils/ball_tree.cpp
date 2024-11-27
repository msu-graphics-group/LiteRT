#include "ball_tree.h"
#include <chrono>

namespace scom
{
  float BallTree::distance_sqr(const float *a, const float *b) const
  {
    float d = 0;
    for (int i = 0; i < m_dim; ++i)
      d += (a[i] - b[i]) * (a[i] - b[i]);
    
    return d;
  }

  void BallTree::build(const Dataset &dataset, int max_leaf_size)
  {
    m_dim = dataset.all_points.size() / dataset.data_points.size();

    m_points_data.clear();
    m_points.clear();
    m_original_ids.clear();
    m_nodes.clear();

    m_points_data.reserve(dataset.all_points.size());
    m_points.reserve(dataset.data_points.size());
    m_original_ids.reserve(dataset.data_points.size());
    m_nodes.reserve(2*dataset.data_points.size()/max_leaf_size);
    m_centroids_data.reserve(2*dataset.data_points.size() * m_dim / max_leaf_size);

    std::vector<int> index(dataset.data_points.size());
    for (int i = 0; i < dataset.data_points.size(); ++i)
      index[i] = i;

    build_rec(dataset, max_leaf_size, dataset.data_points.size(), index.data());
  
    m_points_data.shrink_to_fit();
    m_points.shrink_to_fit();
    m_original_ids.shrink_to_fit();
    m_nodes.shrink_to_fit();
  }

  int BallTree::build_rec(const Dataset &dataset, int max_leaf_size, int n, int *index)
  {
    unsigned cur_node_id = m_nodes.size();
    m_nodes.emplace_back();
    m_centroids_data.resize(m_centroids_data.size() + m_dim);
    
    for (int j = 0; j < m_dim; ++j)
      m_centroids_data[cur_node_id*m_dim + j] = 0;
    for (int i = 0; i < n; ++i)
    {
      for (int j = 0; j < m_dim; ++j)
        m_centroids_data[cur_node_id*m_dim + j] += dataset.all_points[index[i] * m_dim + j];
    }
    for (int j = 0; j < m_dim; ++j)
      m_centroids_data[cur_node_id*m_dim + j] /= n;

    float max_dist_sq = 0;
    for (int i = 0; i < n; ++i)
    {
      float dist_sq = distance_sqr(m_centroids_data.data() + cur_node_id * m_dim, dataset.all_points.data() + index[i] * m_dim);
      max_dist_sq = std::max(max_dist_sq, dist_sq);
    }

    m_nodes[cur_node_id].centroid_index = cur_node_id;
    m_nodes[cur_node_id].radius = sqrtf(max_dist_sq);

    if (n <= max_leaf_size)
    {
      // build leaf node
      unsigned start_id = m_points.size(); 

      m_points.resize(start_id + n);
      m_original_ids.resize(start_id + n);
      m_points_data.resize(m_points_data.size() + n * m_dim);
      for (int i = 0; i < n; ++i)
      {
        m_points[start_id + i] = dataset.data_points[index[i]];
        m_original_ids[start_id + i] = index[i];
        memcpy(m_points_data.data() + (start_id + i)* m_dim, dataset.all_points.data() + index[i] * m_dim, m_dim * sizeof(float));
      }

      m_nodes[cur_node_id].start_index = start_id;
      m_nodes[cur_node_id].count = n;
      m_nodes[cur_node_id].l_idx = 0;
      m_nodes[cur_node_id].r_idx = 0;
    }
    else
    {
      float *w = new float[m_dim];
      int left = 0, right = n - 1, cnt = 0;
      do
      {
        // build internal node
        int x_p = rand() % n;
        int l_p = find_furthest_id(dataset, x_p, n, index);
        int r_p = find_furthest_id(dataset, l_p, n, index);
        assert(l_p != r_p);

        // note: we use l_p and r_p as two pivots
        const float *l_pivot = dataset.all_points.data() + index[l_p] * m_dim;
        const float *r_pivot = dataset.all_points.data() + index[r_p] * m_dim;
        float l_sqr = 0.0f, r_sqr = 0.0f;
        for (int i = 0; i < m_dim; ++i)
        {
          float l_v = l_pivot[i], r_v = r_pivot[i];
          w[i] = r_v - l_v;
          l_sqr += l_v*l_v;
          r_sqr += r_v*r_v;
        }
        float b = 0.5f * (l_sqr - r_sqr);

        left = 0, right = n - 1;
        while (left <= right)
        {
          const float *x = dataset.all_points.data() + index[left] * m_dim;
          float inner_product = 0;
          for (int i = 0; i < m_dim; ++i)
            inner_product += w[i] * x[i];
          float val = inner_product + b;
          if (val < 0.0f)
            ++left;
          else
          {
            std::swap(index[left], index[right]);
            --right;
          }
        }
        ++cnt;
      } while ((left <= 0 || left >= n) && cnt <= 3); // ensure split into two parts
      if (cnt > 3)
        left = n / 2;
      delete[] w;

      m_nodes[cur_node_id].l_idx = build_rec(dataset, max_leaf_size, left, index);
      m_nodes[cur_node_id].r_idx = build_rec(dataset, max_leaf_size, n - left, index + left);
  }
    return cur_node_id;
  }

  int BallTree::find_furthest_id(const Dataset &dataset, int id_from, int n, int *index)
  {
    int far_id = -1;
    float far_dist = -1.0f;

    unsigned off_a = index[id_from] * m_dim;
    for (int i = 0; i < n; ++i)
    {
      if (i == id_from)
        continue;

      unsigned off_b = index[i] * m_dim;
      float dist_sq = distance_sqr(dataset.all_points.data() + off_a, dataset.all_points.data() + off_b);
      if (far_dist < dist_sq)
      {
        far_dist = dist_sq;
        far_id = i;
      }
    }
    return far_id;
  }

  const float *BallTree::get_closest_point(const float *query, float max_dist, float *dist_to_nearest) const
  {
    constexpr unsigned MAX_STACK_SIZE = 128;
    std::array<unsigned, MAX_STACK_SIZE> stack;
    int top = 0;
    stack[top] = 0;
    top = 1;
    const float *cur_nearest = nullptr;
    float nearest_dist_sq = max_dist * max_dist;
    while (top > 0)
    {
      const Node &cur_node = m_nodes[stack[top - 1]];
      // printf("cur_node %p\n", cur_node);
      top--;

      if (cur_node.l_idx == 0) // is leaf
      {
        for (int i = 0; i < cur_node.count; ++i)
        {
          // compute the actual distance
          const float *point = m_points_data.data() + (cur_node.start_index + i) * m_dim;
          float dist_sq = distance_sqr(query, point);

          if (dist_sq < nearest_dist_sq)
          {
            nearest_dist_sq = dist_sq;
            cur_nearest = point;
          }
        }
      }
      else
      {
        const Node &left_node = m_nodes[cur_node.l_idx];
        float left_center_dist_sq = distance_sqr(query, m_centroids_data.data() + left_node.centroid_index * m_dim);
        if (sqrtf(left_center_dist_sq) < left_node.radius + sqrtf(nearest_dist_sq))
        {
          stack[top] = cur_node.l_idx;
          top++;
        }

        const Node &right_node = m_nodes[cur_node.r_idx];
        float right_center_dist_sq = distance_sqr(query, m_centroids_data.data() + right_node.centroid_index * m_dim);
        if (sqrtf(right_center_dist_sq) < right_node.radius + sqrtf(nearest_dist_sq))
        {
          stack[top] = cur_node.r_idx;
          top++;
        }
      }
    }

    if (cur_nearest && dist_to_nearest)
    {
      *dist_to_nearest = sqrtf(nearest_dist_sq);
    }
    return cur_nearest;
  }

  int BallTree::scan_near(const float *query, float max_dist, ScanFunction callback) const
  {
    constexpr unsigned MAX_STACK_SIZE = 128;
    std::array<unsigned, MAX_STACK_SIZE> stack;
    int top = 0;
    stack[top] = 0;
    top = 1;
    float max_dist_sq = max_dist * max_dist;
    int count = 0;

    while (top > 0)
    {
      const Node &cur_node = m_nodes[stack[top - 1]];
      top--;

      if (cur_node.l_idx == 0) // is leaf
      {
        for (int i = 0; i < cur_node.count; ++i)
        {
          // compute the actual distance
          const float *point = m_points_data.data() + (cur_node.start_index + i) * m_dim;
          float dist_sq = distance_sqr(query, point);

          if (dist_sq < max_dist_sq)
          {
            callback(m_original_ids[cur_node.start_index + i], m_points[cur_node.start_index + i], point);
            count++;
          }
        }
      }
      else
      {
        const Node &left_node = m_nodes[cur_node.l_idx];
        float left_center_dist_sq = distance_sqr(query, m_centroids_data.data() + left_node.centroid_index * m_dim);
        if (sqrtf(left_center_dist_sq) < left_node.radius + sqrtf(max_dist_sq))
        {
          stack[top] = cur_node.l_idx;
          top++;
        }

        const Node &right_node = m_nodes[cur_node.r_idx];
        float right_center_dist_sq = distance_sqr(query, m_centroids_data.data() + right_node.centroid_index * m_dim);
        if (sqrtf(right_center_dist_sq) < right_node.radius + sqrtf(max_dist_sq))
        {
          stack[top] = cur_node.r_idx;
          top++;
        }
      }
    }

    return count;
  }

  bool test_ball_tree()
  {
    srand(time(NULL));
    int points_count = 100000;
    constexpr int DIM = 8;
    scom::Dataset dataset;
    dataset.data_points.resize(points_count);
    dataset.all_points.resize(points_count*DIM);

    for (int i = 0; i < points_count; ++i) 
    {
      dataset.data_points[i].data_offset = i*DIM;
      dataset.data_points[i].original_id = i;
      dataset.data_points[i].rotation_id = 0;
      for (int j = 0; j < DIM; ++j)
        dataset.all_points[i*DIM + j] = 2*(float) rand() / (float) RAND_MAX + 1;
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    scom::BallTree tree;
    tree.build(dataset, 8);
    auto t2 = std::chrono::high_resolution_clock::now();
    printf("build took %d us\n", (int)std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count());
    
    double search_time_naive = 0;
    double search_time_tree = 0;

    int tries = 1000;
    int found_count = 0;

    bool correct = true;
    for (int i = 0; i < tries; ++i) 
    {
      float point[DIM];
      for (int j = 0; j < DIM; ++j)
        point[j] = 2*(float) rand() / (float) RAND_MAX + 1;

      float limit = 0.1f*DIM*((float)rand() / (float) RAND_MAX);
      //limit = 10;

      const float *min_point_naive = nullptr;
      float min_distance_naive = limit;

    auto t1 = std::chrono::high_resolution_clock::now();
      for (int j = 0; j < points_count; ++j) 
      {
        float dist = 0;
        for (int k = 0; k < DIM; ++k)
          dist += (point[k] - dataset.all_points[j*DIM + k]) * (point[k] - dataset.all_points[j*DIM + k]);
        dist = sqrt(dist);
        //printf("data %f %f %f, dist = %f\n", data[j*DIM + 0], data[j*DIM + 1], data[j*DIM + 2], dist);
        if (dist < min_distance_naive) 
        {
          min_distance_naive = dist;
          min_point_naive = dataset.all_points.data() + j*DIM;
        }
      }
    
    auto t2 = std::chrono::high_resolution_clock::now();

      float min_distance = 1000;
      //const float *min_point = tree.find_nearest_neighbor(point, limit, &min_distance);
      const float *min_point = tree.get_closest_point(point, limit, &min_distance);

    auto t3 = std::chrono::high_resolution_clock::now();

      search_time_naive += std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();
      search_time_tree += std::chrono::duration_cast<std::chrono::microseconds>(t3-t2).count();

      bool found_naive = min_point_naive != nullptr;
      bool found_tree = min_point != nullptr;

      if (found_naive != found_tree)
      {
        correct = false;
        printf("point %f %f %f, ", point[0], point[1], point[2]);
        if (min_point_naive)
          printf("naive %f %f %f, ", min_point_naive[0], min_point_naive[1], min_point_naive[2]);
        if (min_point)
          printf("tree %f %f %f", min_point[0], min_point[1], min_point[2]);
        printf("\n");
      }
      else if (found_naive && found_tree && std::abs(min_distance - min_distance_naive) > 1e-6f)
      {
        correct = false;
        printf("point %f %f %f, ", point[0], point[1], point[2]);
        printf("naive %f %f %f, ", min_point_naive[0], min_point_naive[1], min_point_naive[2]);
        printf("tree %f %f %f, ", min_point[0], min_point[1], min_point[2]);
        printf("dist %f %f\n", min_distance, min_distance_naive);
      }
      else
      {
        found_count += (found_naive && found_tree);
      }
    }

    search_time_naive /= tries;
    search_time_tree /= tries;

    printf("Search time naive = %.2f us\n", search_time_naive);
    printf("Search time tree = %.2f us\n", search_time_tree);

    if (correct)
      printf("All correct, found %d/%d\n", found_count, tries);
    
    return correct;
  }
}