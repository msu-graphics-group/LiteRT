#include <testing_framework/core/param_parsers.h>
#include <cstdlib>

namespace testing
{

    bool parse_param(std::string text, int64_t&out)
    {
        if (text.length() == 0)
        {
            return false;
        }
        const char *str = text.c_str();
        char *end = nullptr;
        errno = 0;
        out = std::strtoll(str, &end, 10);
        if (errno || *end != '\0')
        {
            return false;
        }
        return true;
    }

    bool parse_param(std::string text, std::string&out)
    {
        out = text;
        return true;
    }

}