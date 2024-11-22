#pragma once
#include <testing_framework/core/param_parsers.h>

namespace testing
{

    bool validate_param(std::string_view name ,std::string_view value, int64_t&out);

    bool validate_param(std::string_view name, std::string_view value, std::string&out);

    bool validate_is_positive_param(std::string_view name, int64_t value);
    bool validate_is_non_negative_param(std::string_view name, int64_t value);

    bool validate_is_not_empty_param(std::string_view name, const std::string&value);

}