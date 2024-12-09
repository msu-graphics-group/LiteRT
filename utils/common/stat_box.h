#pragma once
#include <vector>
#include <cassert>
/*
A collection of utils related to collection, calculation and
working with some statistics. Nothing complicated here, just
some functions that can be useful in different scenarios
*/
namespace stat
{
  /*Accumulates a range of values (unlimeted size) in a set of bins, 
    uniformly distributed over given range. Also counts outliers (values out of given range).
    Can be useful by itself or to calculate quantiles*/
  template <typename T>
  class Bins
  {
  public:
    Bins(T _range_from, T _range_to, unsigned _bin_count = 1000)
    {
      bins = std::vector<unsigned>(_bin_count, 0);
      bin_count = _bin_count;
      range_from = _range_from;
      range_to = _range_to;
      range_size = range_to - range_from;
      assert(range_size != (T)0);
    }
    void add(T value)
    {
      if (empty)
      {
        empty = false;
        min_val = value;
        max_val = value;
      }
      else
      {
        min_val = std::min<T>(min_val, value);
        max_val = std::max<T>(max_val, value);        
      }
      if (value >= range_to)
        count_more++;
      else if (value < range_from)
        count_less++;
      else
      {
        unsigned bin = bin_count*((value - range_from)/range_size);
        bins[bin]++;
      }
      total_count++;
    }
    void add(const std::vector<T> &range)
    {
      for (auto &val : range)
        add(val);
    }
    void print_bins() const
    {
      for (int i=0;i<80;i++) {printf("#");} printf("\n");
      printf("stat::Bins statistics\n");
      printf("Items: %lu\n",total_count);
      printf("Bins: %u\n", (unsigned)bins.size());
      printf("Minimum: %Lf\n", (long double)min_val);
      printf("Maximum: %Lf\n", (long double)max_val);
      printf("Range [%Lf, %Lf]\n", (long double)range_from, (long double)range_to);
      printf("Out of range (less): %u\n", count_less);
      printf("Out of range (more): %u\n", count_more);
      printf("Bins:\n");
      for (int i=0;i<bin_count;i++)
      {
        printf("%d[%Lf - %Lf]: %u\n",i, (long double)(range_from + i*range_size/bin_count), 
                                       (long double)(range_from + (i+1)*range_size/bin_count),
                                       bins[i]);
      }
      for (int i=0;i<80;i++) {printf("#");} printf("\n");
    }
    unsigned count_more = 0;
    unsigned count_less = 0;
    unsigned bin_count;
    unsigned long total_count = 0;
    std::vector<unsigned> bins;
    bool empty = true;
    T min_val, max_val;
    T range_from, range_to;
    T range_size;
  };
};