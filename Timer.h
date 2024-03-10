#ifndef TIMER_H
#define TIMER_H

#include <chrono>
namespace profiling
{
  class Timer
  {
    std::chrono::high_resolution_clock::time_point m_start;
  public:

    class Time
    {
      long long m_microseconds;
    public:
      Time(long long microseconds) : m_microseconds(microseconds)
      {};

      long long asMicroseconds() const;

      double asMilliseconds() const;

      double asSeconds() const;
    };


    Timer();

    Time restart();

    Time getElapsedTime() const;
  };
}
#endif