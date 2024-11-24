#include <testing_framework/helpers/scoped_timer.h>
#include <testing_framework/core/logging.h>
#include <testing_framework/helpers/format.h>

namespace testing
{
    ScopedTimer::ScopedTimer() :
        ended_(true)
    {}

    ScopedTimer::ScopedTimer(std::string action_name) :
        action_name_(std::move(action_name)),
        ended_(false)
    {
        log(bar_info) << "Started " << action_name_ << std::endl;
        start_ = std::chrono::high_resolution_clock::now();
    }

    ScopedTimer::ScopedTimer(ScopedTimer&&other) :
        action_name_(std::move(other.action_name_)),
        start_(other.start_),
        ended_(other.ended_)
    {
        other.ended_ = true;
    }

    ScopedTimer& ScopedTimer::operator=(ScopedTimer&&other)
    {
        action_name_ = std::move(other.action_name_);
        start_ = other.start_;
        ended_ = other.ended_;
        other.ended_ = true;
        return *this;
    }

    ScopedTimer::~ScopedTimer()
    {
        end();
    }

    void ScopedTimer::end()
    {
        if (!ended_)
        {
            auto end = std::chrono::high_resolution_clock::now();
            auto time = end - start_;
            
            float time_ms = std::chrono::duration_cast<std::chrono::duration<float, std::milli>>(time).count();
            log(bar_info)
                << begin_aligned(MESSAGE_WIDTH, -1, 0) << "Ended   " <<  action_name_ << end_aligned 
                << foreground(highlight_color_3) << format_time(time_ms) << default_color
                << std::endl;

            ended_ = true;
        }
    }

}