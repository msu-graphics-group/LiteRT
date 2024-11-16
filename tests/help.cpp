#include "help.h"

namespace test
{

    std::string read_help_message = "Read help with 'test help'";
    std::string help_message = R"(Tests for LiteRT.
    test help
        Print this text.
    test list
        Prints all test names with their descriptions.
    test run <test_name_1> ... <test_name_n>
        Runs specified tests in supervised mode
            (each test is run in separate process, so crashes can be handled).
        If no tests is specified, then runs all tests.
    test rewrite <test_name>
        Rewrites saved references for <test_name>.
        Only one test can be specified.
    test unsafe <test_name>
        Runs <test_name> in unsupervised mode, so it can be debugged.
        Only one test can be specified.
)";

}