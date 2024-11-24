#pragma once
#include <string>

namespace testing
{

    bool file_exists(const std::string&path);

    /*
        If there is no file, skips test
    */
    bool assert_file_existance(const std::string&path, bool treat_failure_as_error = true);

    /*
        Creates direcoty for path
    */
    void prepare_directoty_for_saving(const std::string&path);

}