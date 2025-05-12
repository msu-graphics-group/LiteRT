#pragma once
#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <sstream>
#include <thread>
#include <optional>

class Timer 
{
  public:
    Timer() 
    { 
      start = clock::now(); 
    }
    double Seconds() const 
    {
      clock::time_point now = clock::now();
      int64_t microsecs = std::chrono::duration_cast<std::chrono::microseconds>(now - start).count();
      return static_cast<double>(microsecs) / 1000000.;
    }

  private:
    using clock = std::chrono::steady_clock;
    clock::time_point start;
};

class ConsoleProgressBar 
{
  public:
    ConsoleProgressBar(){};
    ConsoleProgressBar(int64_t totalWork);

    ~ConsoleProgressBar() { Done(); }

    void Start();
    void Update(int64_t num = 1) { if (num == 0) return; workDone += num;}
    double ElapsedSeconds() const { return finishTime ? *finishTime : timer.Seconds(); }

    void Done();

  private:
    void Print();

    int64_t totalWork;
    Timer timer;
    std::atomic<int64_t> workDone;
    std::atomic<bool> exitThread;
    std::thread updateThread;
    std::optional<double> finishTime;

};