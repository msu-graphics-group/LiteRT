#pragma once
#include <string>

namespace testing
{

    // skips current test
    void skip();

    // adds score to current test
    void add_check_result(bool passed);

    /*
        If in rewrite mode
            Asks
                "Are you sure?"
                "Do you want to <comment>?"
                "Please, answer [y/n]"
        
        Always returns false in non-rewrite mode
    */
    bool should_rewrite(std::string_view comment);

}