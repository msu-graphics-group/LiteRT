#include "help.h"

namespace test
{

    std::string read_help_message = "Read help with 'test --help'";
    std::string help_message = R"(Tests for LiteRT.
How to use:
    test <cmd> <test-list>
Commands:
    --run:
        Runs specified tests and logs their results.
        Examples:
            test --run --unittest 1 2 --regression 3 4
            test --run --unittest
            test --run --all
    --list:
        Just prints description of tests
        Example:
            test --list
    --rewrite:
        Rewrites reference of specified regression test.
            Example:
                test --rewrite --regression 1
        Only one regression test must be specified
Test lists:
    Test list has form:
        <type1> <name1> <name2> <type2> <name3> <name4> <type3> ...
    where <type> is a test type of following <name>'s.
    <type> is one of '--unittest' and '--regression'.
    Example:
        --unittest 1 2 --regression 3 4 --unittest 5
        means test list of unittests with names "1", "2", "5" and
            regression tests with names "3" and "4".
    If no names are specified for <type> but <type> is specified, it means all tests of <type>
    Example:
        test --run --unittest --regression 1 2
        runs all unittests and regression tests with names "1" and "2".
    --all means all tests of all types
Shortcuts:
    If no command is specified --run is assumed
    If no test is specified --all is assumed
)";

}