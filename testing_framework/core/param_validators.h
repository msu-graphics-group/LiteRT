#pragma once
#include <testing_framework/core/param_parsers.h>

namespace testing
{

    bool validate_param(std::string_view text, int64_t&);
    bool validate_param(std::string_view text, uint64_t&);

    bool validate_param(std::string_view text, std::string&);

}