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
    long double mean = mean(a);
    for (int i = 0; i < a.size(); i++)
    {
      sum += (a[i] - mean) * (a[i] - mean);
    }
    return sum / (a.size() - 1);
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
}