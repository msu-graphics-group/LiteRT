#include <testing_framework/core/logging.h>
#include <iostream>

namespace testing
{

    static size_t level = 0;

    size_t logging_level()
    {
        return level;
    }

    void set_logging_level(size_t l)
    {
        level = l;
    }

    std::ostream& log(size_t level)
    {
        static std::ostream null_output(nullptr);
        if (level <= logging_level())
        {
            return std::cout;
        }
        else
        {
            return null_output;
        }
    }

}