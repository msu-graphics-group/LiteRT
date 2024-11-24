#pragma once
#include <chrono>
#include <string>

namespace testing
{

    /*
        Prints 
        Started <action-name> 
        Ended   <action-name> ... <duration>
    */
    class ScopedTimer
    {
    public:
        ScopedTimer();
        ScopedTimer(std::string action_name);
        ScopedTimer(const ScopedTimer&) = delete;
        ScopedTimer(ScopedTimer&&);
        ScopedTimer& operator=(const ScopedTimer&) = delete;
        ScopedTimer& operator=(ScopedTimer&&);

        ~ScopedTimer();

        void end();

    private:
        std::string action_name_;
        std::chrono::high_resolution_clock::time_point start_;
        bool ended_;
    };

}