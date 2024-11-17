#include "help.h"
#include <regex>
#include "exe.h"

namespace testing::help
{
    
    std::string create_message_from_template(const std::string&temp)
    {
        return std::regex_replace(temp, std::regex("EXE"), current_executable_name().string());
    }

    std::string help_message()
    {
        static const char temp[] = R"(  EXE help
        Prints this text.
    EXE list
        Prints all test names with additional information.
    EXE run <test_name_1> ... <test_name_n>
        Runs specified tests in supervised mode
            (each test is run in separate process, so crashes can be handled).
        If no tests is specified, then runs all tests.
    EXE rewrite <test_name>
        Rewrites saved references for <test_name>.
        Only one test can be specified.
    EXE unsafe <test_name>
        Runs <test_name> in unsupervised mode, so it can be debugged.
        Only one test can be specified.
)";
        return create_message_from_template(temp);
    }

    std::string read_help_message()
    {
        static const char temp[] = "Read help with 'EXE help'.";
        return create_message_from_template(temp);
    }

    std::string read_list_message()
    {
        static const char temp[] = "Run 'EXE list' to list all tests.";
        return create_message_from_template(temp);
    }

}