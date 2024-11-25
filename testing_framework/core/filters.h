#pragma once
#include <string>
#include <functional>

namespace testing
{

    /*
        Filter function returns true, if string should be filtered
    */
    void add_filter(std::function<bool(std::string_view)> f);

    void add_filter_if_contains(std::string_view s);

    bool should_filter(std::string_view);

}