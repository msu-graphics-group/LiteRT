#pragma once
#include <vector>
#include <map>
#include <typeinfo>
#include <testing_framework/core/test.h>

namespace testing
{

    bool list(
        const std::vector<const Test*>& tests,
        bool write_descriptions
    );
    
    bool run(
        size_t logging_level,
        size_t jobs,
        const std::vector<const Test*>& tests,
        std::map<std::string, std::pair<const std::type_info*, std::string>> test_options
    );
    
    bool exec(
        size_t logging_level,
        bool rewrite,
        const Test*test,
        std::map<std::string, std::pair<const std::type_info*, std::string>> test_options
    );

}