#pragma once
#include <vector>
#include <cassert>
#include "span.h"
/*
A collection of utils related to collection, calculation and
working with some statistics. Nothing complicated here, just
some functions that can be useful in different scenarios
*/
namespace stat
{
  template <typename T>
  double distance(const std::span<T> &a, const std::span<T> &b)
  {
    assert(a.size() == b.size());
    long double sum = 0;
    for (int i = 0; i < a.size(); i++)
    {
      sum += (a[i] - b[i]) * (a[i] - b[i]);
    }
    return std::sqrt(sum);
  }
  
  template <typename T>
  double length(const std::span<T> &a)
  {
    long double sum = 0;
    for (int i = 0; i < a.size(); i++)
    {
      sum += a[i] * a[i];
    }
    return std::sqrt(sum);
  }

  template <typename T>
  double mean(const std::span<T> &a)
  {
    assert(a.size() > 0);
    long double sum = 0;
    for (int i = 0; i < a.size(); i++)
    {
      sum += a[i];
    }
    return sum / a.size();
  }

  template <typename T>
  double variance(const std::span<T> &a)
  {
    assert(a.size() > 1);

    long double sum = 0;
    long double mean_val = mean(a);
    for (int i = 0; i < a.size(); i++)
    {
      sum += (a[i] - mean_val) * (a[i] - mean_val);
    }
    return sum / (a.size() - 1);
  }

  double z_score(double confidence_level)
  {
    assert(confidence_level > 1e-6f && confidence_level < 1-1e-6f);
    long double step = 0.001f;
    long double x = -10.0f; // chances of N(0,1) being < -10 are very low
    long double z = 0;

    // estimate integral of N(0,1) PDF from -inf to x
    while (z < confidence_level)
    {
      z += (1.0 / sqrt(2.0 * M_PI)) * exp(-(x * x) / 2.0) * step;
      x += step;
    }
    printf("z_score for confidence level %f is %f\n", (float)confidence_level, (float)x);
    return x;
  }

  template <typename T>
  double confidence(const std::span<T> &a, double confidence_level = 0.95f)
  {
    assert(a.size() > 1);

    double variance_val = variance(a);
    double std_error = std::sqrt(variance_val / a.size());

    return z_score(confidence_level) * std_error;
  }

  // mean for a sample with independent observations of vector-valued random variable
  // a is an array of observations
  template <typename T>
  std::vector<double> mean(const std::vector<std::vector<T>> &a)
  {
    assert(a.size() > 0);
    std::vector<double> result(a[0].size(), 0);
    for (int i = 0; i < a.size(); i++)
    {
      assert(a[i].size() == a[0].size());
      for (int j = 0; j < a[i].size(); j++)
      {
        result[j] += a[i][j];
      }
    }

    for (int i = 0; i < result.size(); i++)
    {
      result[i] /= a.size();
    }

    return result;
  }

  // assume that every variable is independent for others
  // calculate variance for every variable for a sample with independent observations
  // a is an array of observations
  template <typename T>
  std::vector<double> variance(const std::vector<std::vector<T>> &a)
  {
    assert(a.size() > 0);
    std::vector<double> means = mean(a);
    std::vector<double> result(a[0].size(), 0);
    for (int i = 0; i < a.size(); i++)
    {
      assert(a[i].size() == a[0].size());
      for (int j = 0; j < a[i].size(); j++)
      {
        result[j] += (a[i][j] - means[j]) * (a[i][j] - means[j]);
      }
    }

    for (int i = 0; i < result.size(); i++)
    {
      result[i] /= a.size() - 1;
    }

    return result;
  }
  
  
  // covariance matrix for a sample with independent observations of vector-valued random variable
  // a is an array of observations
  template <typename T>
  std::vector<double> cov_matrix(const std::vector<std::vector<T>> &a)
  {
    assert(a[0].size() > 1);
    std::vector<double> mean_v = mean(a);
    std::vector<double> result(a[0].size() * a[0].size(), 0);
    for (int i = 0; i < a.size(); i++)
    {
      assert(a[i].size() == a[0].size());
      for (int j = 0; j < a[i].size(); j++)
      {
        for (int k = 0; k < a[i].size(); k++)
        {
          result[j * a[i].size() + k] += (a[i][j] - mean_v[j]) * (a[i][k] - mean_v[k]);
        }
      }
    }
    for (int i = 0; i < result.size(); i++)
    {
      result[i] /= (a.size() - 1);
    }

    return result;
  }

  template <typename T>
  std::vector<double> confidence(const std::vector<std::vector<T>> &a, double confidence_level = 0.95f)
  {
    assert(a.size() > 1);

    std::vector<double> variance_val = variance(a);
    std::vector<double> result(a[0].size(), 0);
    
    double z = z_score(confidence_level);

    for (int i = 0; i < a[0].size(); i++)
    {
      double std_error = std::sqrt(variance_val[i] / a.size());
      result[i] = z * std_error;
    }

    return result;
  }
}