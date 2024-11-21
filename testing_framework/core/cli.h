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

    bool handle_args(size_t argc, char**argv);

}
