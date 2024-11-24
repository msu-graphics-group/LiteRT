#pragma once
#include <ostream>
#include <iomanip>
#include <testing_framework/helpers/source_location.h>
#include <testing_framework/core/logging.h>

#include "LiteMath/LiteMath.h"

namespace testing
{

    constexpr size_t MESSAGE_WIDTH = 70;

    struct TimeFormatter
    {
        float ms;
    };

    inline TimeFormatter format_time(float ms)
    {
        return {ms};
    }

    inline std::ostream &operator<<(std::ostream &out, TimeFormatter time)
    {
        float ms = time.ms;
        if (ms < 1000) // ms
        {
            out << std::fixed << std::setprecision(1) << ms << "ms";
        }
        else // sec
        {
            out << std::fixed << std::setprecision(2) << ms / 1000 << "s";
        }
        return out;
    }

    struct SizeFormatter
    {
        float bytes;
    };

    inline SizeFormatter format_size(size_t bytes)
    {
        return {bytes};
    }

    inline std::ostream &operator<<(std::ostream &out, SizeFormatter size)
    {
        size_t bytes = size.bytes;
        out << std::fixed << std::setprecision(2);
        if (bytes < 1000)
        {
            out << bytes << "B";
        }
        else if (bytes < 1000 * 1000)
        {
            out << (float)bytes / 1000 << "KB";
        }
        else if (bytes < 1000 * 1000 * 1000)
        {
            out << (float)bytes / 1000 / 1000 << "MB";
        }
        else
        {
            out << (float)bytes / 1000 / 1000 / 1000 << "GB";
        }
        return out;
    }

    inline std::ostream& operator<<(std::ostream&out, const source_location&loc)
    {
        #if TESTING_ENABLE_SOURCE_LOCATION
            out
                << foreground(highlight_color_2)
                << "'" << loc.file_name() << ":" << loc.line()
                << "'" << default_color;
        #else
            // print nothing
        #endif
        return out;
    }

    template<typename T>
    struct ConditionalPrinter
    {
        T value;
        bool print;
    };

    template<typename T>
    std::ostream& operator<<(std::ostream&out, const ConditionalPrinter<T>&x)
    {
        if (x.print)
        {
            out << x.value;
        }
        return out;
    }

    template<typename T>
    ConditionalPrinter<T> print_conditional(T value, bool print)
    {
        return {std::move(value), print};
    }

}
namespace LiteMath
{

    inline std::ostream &operator<<(std::ostream &out, float2 v)
    {
        return out << "(" << v.x << ", " << v.y << ")";
    }

    inline std::ostream &operator<<(std::ostream &out, float3 v)
    {
        return out << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    }

    inline std::ostream &operator<<(std::ostream &out, float4 v)
    {
        return out << "(" << v.x << ", " << v.y << ", " << v.z << ", " << v.w << ")";
    }
    
}