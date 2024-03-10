#include "Timer.h"

using namespace profiling;

Timer::Timer() : m_start()
{
  restart();
}


Timer::Time Timer::restart()
{
  auto now = std::chrono::high_resolution_clock::now();
  auto before = m_start;
  m_start = now;
  auto dur = now - before;

  return Time(std::chrono::duration_cast<std::chrono::microseconds>(dur).count());
}


Timer::Time Timer::getElapsedTime() const
{
  auto now = std::chrono::high_resolution_clock::now();
  auto dur = now - m_start;

  return Time(std::chrono::duration_cast<std::chrono::microseconds>(dur).count());
}


/////////////////////////////////////////////////
/////////////////////////////////////////////////


long long Timer::Time::asMicroseconds() const
{
  return m_microseconds;
}


double    Timer::Time::asMilliseconds() const
{
  return static_cast<double>(m_microseconds) * 1e-3;
}


double    Timer::Time::asSeconds()      const
{
  return static_cast<double>(m_microseconds) * 1e-6;
}