#include <testing_framework/core/filters.h>
#include <vector>
#include <algorithm>

namespace testing
{

    static std::vector<std::function<bool(std::string_view)>> filters_;


    void add_filter(std::function<bool(std::string_view)> f)
    {
        filters_.push_back(f);
    }

    bool should_filter(std::string_view s)
    {
        if (std::all_of(s.begin(), s.end(), [](char x){ return std::isspace(x); }))
        {
            return true;
        }
        for (auto&f : filters_)
        {
            if (f(s))
            {
                return true;
            }
        }
        return false;
    }

    void add_filter_if_contains(std::string_view x)
    {
        add_filter([x](std::string_view s){
            return s.find(x) != std::string_view::npos;
        });
    }

}