#pragma once
#include <string>

namespace testing
{

    bool parse_param(std::string text, int64_t&out);

    bool parse_param(std::string text, std::string&out);

}