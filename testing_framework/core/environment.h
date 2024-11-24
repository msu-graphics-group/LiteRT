#pragma once
#include <string>
#include <typeinfo>
#include <testing_framework/core/param_parsers.h>

namespace testing
{

    void skip();
    void add_check_result(bool passed);

    /*
        In rewrite mode asks user, does he want to rewrite <what> in file <path>
    */
    bool should_rewrite_saved_reference(std::string_view what, std::string_view path);

    std::string_view get_test_name();

    bool get_flag(std::string_view name);

    std::string_view get_param(std::string_view name, const std::type_info*parsing_type);

    template<typename T>
    T get_param(std::string_view name)
    {
        T out;
        std::string_view value = get_param(name, &typeid(T));
        parse_param(value, out); // always success
        return out;
    }

}