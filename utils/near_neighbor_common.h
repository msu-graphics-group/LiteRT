#pragma once
#include "similarity_compression_impl.h"
#include <functional>

namespace scom
{
  /// Callback function for near neighbor search
  /// @param dist distance to point
  /// @param idx index of point in dataset
  /// @param point point info
  /// @param data  N-dimensional vector
  using ScanFunction = std::function<void(float dist, unsigned idx, const DataPoint &, const float *)>;

  //Interface for acceleration structure for near neighbor search
  class INNSearchAS
  {
  public:
    virtual ~INNSearchAS() = default;

    /// Build acceleration structure. Dataset is copied inside acceleration structure
    //  Max leaf size can mean nothing for some acceleration structures
    virtual void build(const Dataset &dataset, int max_leaf_size) = 0; 

    /// Get closest point to query within max_dist
    //  Returns nullptr if no point is found
    //  writes to *dist_to_nearest distance to closest point
    virtual const float *get_closest_point(const float *query, float max_dist, float *dist_to_nearest) const = 0;

    /// Scan all points within max_dist
    //  Returns number of points found
    virtual int scan_near(const float *query, float max_dist, ScanFunction callback) const = 0;
  };

  class LinearSearchAS : public INNSearchAS
  {
  public:
    void build(const Dataset &dataset, int max_leaf_size) override
    {
      m_dim = dataset.all_points.size() / dataset.data_points.size();
      m_dataset = dataset;
    }
    const float *get_closest_point(const float *query, float max_dist, float *dist_to_nearest) const override
    {
      const float *closest_point = nullptr;
      float closest_dist_sq = max_dist*max_dist;
      for (int i = 0; i < m_dataset.data_points.size(); ++i)
      {
        float dist_sq = 0;
        for (int j = 0; j < m_dim; ++j)
          dist_sq += (query[j] - m_dataset.all_points[i*m_dim + j]) * (query[j] - m_dataset.all_points[i*m_dim + j]);

        if (dist_sq < closest_dist_sq)
        {
          closest_dist_sq = dist_sq;
          closest_point = m_dataset.all_points.data() + i*m_dim;
        }
      }

      if (dist_to_nearest && closest_point)
        *dist_to_nearest = sqrtf(closest_dist_sq);

      return closest_point;
    }

    int scan_near(const float *query, float max_dist, ScanFunction callback) const override
    {
      int scan_count = 0;
      float max_dist_sq = max_dist*max_dist;
      for (int i = 0; i < m_dataset.data_points.size(); ++i)
      {
        float dist_sq = 0;
        for (int j = 0; j < m_dim; ++j)
          dist_sq += (query[j] - m_dataset.all_points[i*m_dim + j]) * (query[j] - m_dataset.all_points[i*m_dim + j]);
        
        if (dist_sq < max_dist_sq)
        {
          callback(sqrtf(dist_sq), i, m_dataset.data_points[i], m_dataset.all_points.data() + i*m_dim);
          ++scan_count;
        }
      }

      return scan_count;
    }

  private:
    int m_dim;
    Dataset m_dataset;
  };
}