#pragma once
#include <string>

namespace testing
{

    bool parse_param(std::string_view text, int64_t&);
    bool parse_param(std::string_view text, uint64_t&);

    bool parse_param(std::string_view text, std::string&s);

}