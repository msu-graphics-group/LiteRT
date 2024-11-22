#include <testing_framework/core/param_validators.h>
#include <testing_framework/core/param_parsers.h>
#include <iostream>

namespace testing
{

    bool validate_param(std::string_view name ,std::string_view value, int64_t&out)
    {
        if (parse_param(value, out))
        {
            return true;
        }
        std::cerr << "'" << name << "' must be 64-bit integer" << std::endl;
        return false;
    }

    bool validate_param(std::string_view name, std::string_view value, std::string&out)
    {
        return parse_param(value, out);
    }

    bool validate_is_positive_param(std::string_view name, int64_t value)
    {
        if (value > 0)
        {
            return true;
        }
        std::cerr << "'" << name << "' must be positive" << std::endl;
        return false;
    }

    bool validate_is_non_negative_param(std::string_view name, int64_t value)
    {
        if (value >= 0)
        {
            return true;
        }
        std::cerr << "'" << name << "' must be non-negative" << std::endl;
        return false;
    }

    bool validate_is_not_empty_param(std::string_view name, const std::string&value)
    {
        if (value.length() > 0)
        {
            return true;
        }
        std::cerr << "'" << name << "'" << "must not be empty" << std::endl;
        return false;
    }

}