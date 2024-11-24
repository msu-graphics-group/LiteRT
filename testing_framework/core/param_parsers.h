#pragma once
#include <string>

namespace testing
{

    bool parse_param(std::string_view text, int64_t&out);

    bool parse_param(std::string_view text, std::string&out);

}