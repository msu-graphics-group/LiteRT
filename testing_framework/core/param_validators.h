#pragma once
#include <testing_framework/core/param_parsers.h>

namespace testing
{

    bool validate_param(std::string name ,std::string value, int64_t&out);

    bool validate_param(std::string name, std::string value, std::string&out);

    bool validate_is_positive_param(std::string name, int64_t value);
    bool validate_is_non_negative_param(std::string name, int64_t value);

    bool validate_is_not_empty_param(std::string name, const std::string&value);

}