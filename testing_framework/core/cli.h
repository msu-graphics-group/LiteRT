#pragma once
#include <vector>
#include <string>
#include <functional>
#include <map>
#include <typeinfo>
#include <testing_framework/core/param_validators.h>

namespace testing
{

    /*
        Asks user to answer yes or no
        Returns true if user has answered and false if end of stdin has been reached
    */
    bool yes_or_no_dialogue(bool&result);

    /*
        Returns args which should be put in cmdline to 
        run exec command with specified params
    */
    std::vector<std::string> cmdline_to_exec(
        bool enable_colors,
        size_t logging_level,
        std::string test_name,
        bool rewrite,
        std::map<std::string, std::pair<const std::type_info*, std::string>> params
    );

    void add_test_param(
        std::string name,
        std::string default_value,
        const std::type_info*type,
        std::function<bool(std::string_view)> validator,
        std::string short_name,
        std::string long_name
    );

    inline void add_test_flag(std::string name, std::string short_name, std::string long_name)
    {
        add_test_param(name, "", nullptr, {}, short_name, long_name);
    }
    
    template<typename T>
    void add_test_param(std::string name, std::string default_value, std::string short_name, std::string long_name)
    {
        add_test_param(name, default_value, &typeid(T), [](std::string_view text){
            T tmp;
            return validate_param(text, tmp);
        }, short_name, long_name);
    }

    bool handle_args(size_t argc, char**argv);

}
