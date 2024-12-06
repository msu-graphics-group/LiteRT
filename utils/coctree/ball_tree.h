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
#include <immintrin.h>

#include "LiteMath.h"
#include "similarity_compression_impl.h"
#include "near_neighbor_common.h"
#include "utils/common/constexpr_for.h"

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
    virtual float distance_sqr(int m_dim, const float *a, const float *b) const;

    size_t m_dim; // dimension count of data
    std::vector<Node> m_nodes;
    aligned_vector_f m_centroids_data; //m_dim * m_nodes.size();
    
    std::vector<DataPoint> m_points;
    std::vector<unsigned>  m_original_ids; //m_points.size();
    aligned_vector_f m_points_data; //m_dim * m_points.size();
  };

  template <int DIM>
  class BallTreeFD : public BallTree
  {
  private:
    static float sum8(__m256 x)
    {
      // x = ( x7, x6, x5, x4, x3, x2, x1, x0 )
    #ifdef __AVX2__
      // hiQuad = ( x7, x6, x5, x4 )
      const __m128 hiQuad = _mm256_extractf128_ps(x, 1);
      // loQuad = ( x3, x2, x1, x0 )
      const __m128 loQuad = _mm256_castps256_ps128(x);
      // sumQuad = ( x3 + x7, x2 + x6, x1 + x5, x0 + x4 )
      const __m128 sumQuad = _mm_add_ps(loQuad, hiQuad);
      // loDual = ( -, -, x1 + x5, x0 + x4 )
      const __m128 loDual = sumQuad;
      // hiDual = ( -, -, x3 + x7, x2 + x6 )
      const __m128 hiDual = _mm_movehl_ps(sumQuad, sumQuad);
      // sumDual = ( -, -, x1 + x3 + x5 + x7, x0 + x2 + x4 + x6 )
      const __m128 sumDual = _mm_add_ps(loDual, hiDual);
      // lo = ( -, -, -, x0 + x2 + x4 + x6 )
      const __m128 lo = sumDual;
      // hi = ( -, -, -, x1 + x3 + x5 + x7 )
      const __m128 hi = _mm_shuffle_ps(sumDual, sumDual, 0x1);
      // sum = ( -, -, -, x0 + x1 + x2 + x3 + x4 + x5 + x6 + x7 )
      const __m128 sum = _mm_add_ss(lo, hi);
      return _mm_cvtss_f32(sum);
    #else
      return 0;
    #endif
    }
    virtual float distance_sqr(int m_dim, const float *a, const float *b) const override
    {
      float d = 0;
      #ifdef __AVX2__
      static_assert(DIM % 8 == 0, "DIM must be a multiple of 8");
      constexpr_for<0,DIM/8>([&](auto i)
      {
        __m256 v1 = _mm256_load_ps(a + i * 8);
        __m256 v2 = _mm256_load_ps(b + i * 8);
        __m256 diff = _mm256_sub_ps(v1, v2);
        __m256 diff_sq = _mm256_mul_ps(diff, diff);
        d += sum8(diff_sq);
      });
      #else
      for (int i = 0; i < DIM; ++i)
        d += (a[i] - b[i]) * (a[i] - b[i]);
      #endif

      return d;
    }
  };
}