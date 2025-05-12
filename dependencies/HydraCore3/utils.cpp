#include "utils.h"
#include <cstring>
#include <cmath>
#include <array>
#include <iostream>

static constexpr int gTerminalWidth = 80;

ConsoleProgressBar::ConsoleProgressBar(int64_t totalWork)
    : totalWork(std::max<int64_t>(1, totalWork))
{
  workDone = 0;
  exitThread = false;
}

void ConsoleProgressBar::Start()
{
  updateThread = std::thread([this]() { Print();});
}

void ConsoleProgressBar::Print() 
{
  static constexpr int barLength = gTerminalWidth - 30;
  int currPrinted = 0;

  static constexpr size_t bufLen = barLength + 4;
  std::array<char, bufLen> buf;
  int i = 0;
  buf[i++] = '\r';
  buf[i++] = '|';
  int currPos = i;
  for (; i < barLength; ++i)
    buf[i] = ' ';
  buf[i++] = '|';
  buf[i++] = '\0';

  std::cout << buf.data();
  std::cout.flush();

  const std::chrono::milliseconds baseInterval {250};
  std::chrono::milliseconds sleepDuration(baseInterval);

  // number of iterations corresponding to certain time points
  const int tenSeconds = 10000 / baseInterval.count();
  const int oneMinute  = 10000 * 6 / baseInterval.count();
  const int tenMinutes = 10000 * 6 * 10 / baseInterval.count();

  int iter = 0;
  bool finished = false;
  while (!finished) 
  {
    if (exitThread)
      finished = true;
    else
      std::this_thread::sleep_for(sleepDuration);

    ++iter;
    if (iter == tenSeconds || iter == oneMinute)
        sleepDuration *= 2;
    else if (iter == tenMinutes)
        sleepDuration *= 5;


    float percentDone = float(workDone) / float(totalWork);
    int needToPrint = static_cast<int>(std::round(barLength * percentDone));
    
    while (currPrinted < needToPrint) 
    {
      if(currPrinted >= barLength - 2)
        break;
      buf[currPos++] = '#';
      ++currPrinted;
    }
    fputs(buf.data(), stdout);

    float timeElapsed = static_cast<float>(ElapsedSeconds());
    float timeLeftEst = timeElapsed / percentDone - timeElapsed;

    std::stringstream ss;
    ss.setf(std::ios::fixed);
    ss.precision(1);
    ss << " (";
    if (exitThread)
      ss << *finishTime;
    else if (percentDone == 1.f)
      ss << timeElapsed;
    else if (!std::isinf(timeLeftEst))
      ss << timeElapsed << "s|" << std::max<float>(0, timeLeftEst);
    else
      ss << timeElapsed << "s|???";
    ss << "s)";

    ss << " Rendering...";

    std::cout << ss.str();
    std::cout.flush();
  }
}

void ConsoleProgressBar::Done() 
{
  bool fa = false;
  finishTime = ElapsedSeconds();
  if (exitThread.compare_exchange_strong(fa, true)) 
  {
    workDone = totalWork;
    exitThread = true;
    if (updateThread.joinable())
      updateThread.join();
    
    std::cout << std::endl;
  } 
}