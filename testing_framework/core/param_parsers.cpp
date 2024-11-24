#include <testing_framework/core/param_parsers.h>
#include <cstdlib>

namespace testing
{

    bool parse_param(std::string_view t, int64_t&out)
    {
        std::string text{t};
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

    bool parse_param(std::string_view text, std::string&out)
    {
        out = std::string(text);
        return true;
    }

}