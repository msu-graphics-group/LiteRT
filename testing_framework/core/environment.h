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
    bool should_rewrite_saved_reference(std::string what, std::string path);

    bool get_flag(std::string name);

    std::string get_param(std::string name, const std::type_info*parsing_type);

    template<typename T>
    T get_param(std::string name)
    {
        T out;
        std::string value = get_param(name, &typeid(T));
        parse_param(value, out); // always success
        return out;
    }

}