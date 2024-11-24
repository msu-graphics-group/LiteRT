#pragma once

#ifdef __cplusplus

#ifndef TESTING_ENABLE_SOURCE_LOCATION
#   if __cplusplus > 201703L // >= c++20
#       define TESTING_ENABLE_SOURCE_LOCATION 1
#   else
#       define TESTING_ENABLE_SOURCE_LOCATION 0
#   endif
#endif

#if TESTING_ENABLE_SOURCE_LOCATION
#   include <source_location>

namespace testing
{
    constexpr bool ENABLE_SOURCE_LOCATION = true;
    using source_location = std::source_location;

}

#else

namespace testing
{
    constexpr bool ENABLE_SOURCE_LOCATION = false;
    
    /*
        Dummy
    */
    struct source_location
    {
        static source_location current() noexcept
        {
            return {};
        }
    };
}

#endif

#endif