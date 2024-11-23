#pragma once
#include <testing_framework/core/test.h>
#include <string>
#include <map>
#include <typeinfo>

namespace testing
{

    void skip_current_test();
    void add_current_test_check_result(bool passed);

    bool rewrite();

    std::string_view get_current_test_name();

    /*
        Skip test if types are mismatched
    */
    bool get_test_flag(std::string_view name);
    std::string_view get_test_param(std::string_view name, const std::type_info*parsing_type);

    /*
        Returns true, if test was executed successfully
        Returns false, if test was skipped
    */
    bool execute_test(
        const Test*test,
        bool rewrite,
        std::map<std::string, std::pair<const std::type_info*, std::string>> test_options,
        size_t&passed_checks,
        size_t&failed_checks
    );

}