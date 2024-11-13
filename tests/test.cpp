#include "test.h"
#include <iostream>
#include <map>
#include <optional>

namespace test
{
    std::vector<Unittest*> Unittest::tests_;
}

namespace test
{
    static char read_help_message[] = "Read help with 'test --help'.";
    static char help_message[] = R"(Tests for LiteRT.
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
        Rewrites reference of specified regression tests.
            Example:
                test --rewrite --regression 1
Test lists:
    Test list has form:
        <type1> <name1> <name2> <type2> <name3> <name4> <type3> ...
    where <type> is a test type of following <name>'s.
    <type> is one of '--unittest' and '--regression'.
    Example:
        --unittest 1 2 --regression 3 4 --unittest 5
        means test list of unittests with names "1", "2", "5" and
            regression tests witrh names "3" and "4".
    If no names are specified for <type> but <type>, it means all tests of <type>
    Example:
        test --run --unittest --regression 1 2
        runs all unittests and regression tests with names "1" and "2".
    --all means all tests of all types
Shotcuts:
    If no command is specified --run is assumed
    If no test is specified and <cmd> is --run or --list, then --all is assumend
)";
}

namespace test
{

    static bool is_flag_name(std::string_view x)
    {
        if (x.size() <= 2)
            return false;
        if (x[0] != '-' || x[1] != '-')
            return false;
        for (size_t i = 2; i < x.size(); i++)
            if (!std::isalnum(x[i]))
                return false;
        return true;
    }

    /*
        Returns std::nullopt if args does not start with flag, which is treated as bad case
        All other cases are Ok
     */
    static std::optional<std::map<std::string_view, std::vector<std::string_view>>> parse_args(size_t argc, char**argv)
    {
        std::map<std::string_view, std::vector<std::string_view>> out;
        std::string_view key;
        for (int i = 0; i < argc; i++)
        {
            std::string_view x = argv[i];
            if (is_flag_name(x)) {
                key = x;
                out[key];
            } else {
                if (is_flag_name(key)) {
                    out[key].push_back(x);
                } else {                    // there was no key; aka bad case
                    return std::nullopt;
                }
            }
        }
        return out;
    }

    static int get_arg(const std::map<std::string_view, std::vector<std::string_view>>&args, std::string_view name, bool&x)
    {
        auto it = args.find(name);
        if (it == args.end())
        {
            x = false;
            return 0;
        }
        if (it->second.size() == 0)
        {
            x = true;
            return 0;
        }
        std::cerr << "'" << name << "' is a flag and can not have any values, but some were specified." << std::endl;
        return 1;
    }

    static int get_arg(const std::map<std::string_view, std::vector<std::string_view>>&args, std::string_view name, std::optional<std::string_view>&x)
    {
        auto it = args.find(name);
        if (it == args.end())
        {
            x = std::nullopt;
            return 0;
        }
        if (it->second.size() == 1)
        {
            x = it->second[0];
            return 0;
        }
        std::cerr << "'" << name << "' is a one value argument and can have only one value, but zero or more were specified." << std::endl;
        return 1;
    }

    static int get_arg(const std::map<std::string_view, std::vector<std::string_view>>&args, std::string_view name, std::optional<std::vector<std::string_view>>&x)
    {
        auto it = args.find(name);
        if (it == args.end())
        {
            x = std::nullopt;
            return 0;
        }
        x = it->second;
        return 0;
    }

    template<typename Test>
    std::optional<std::vector<Test*>> collect_tests(const std::vector<std::string_view>&names)
    {
        return {};
    }

    template<typename Test>
    int list_tests(const std::vector<Test*> tests)
    {

    }

    template<typename Test>
    int run_tests(const std::vector<Test*> tests)
    {

    }

    static int handle_args(size_t argc, char**argv)
    {
        auto args = parse_args(argc, argv);
        if (!args)
        {
            std::cerr << "Command '" << argv[0] << "' is not recognized." << std::endl;
            return 1;
        }

        bool run, list, all, rewrite, help;
        std::optional<std::vector<std::string_view>> unittest, regression;
        
        if (get_arg(*args, "--help", help)) return 1;
        if (get_arg(*args, "--run", run)) return 1;
        if (get_arg(*args, "--list", list)) return 1;
        if (get_arg(*args, "--rewrite", rewrite)) return 1;
        if (get_arg(*args, "--all", all)) return 1;
        if (get_arg(*args, "--unittest", unittest)) return 1;
        if (get_arg(*args, "--regression", regression)) return 1;
        
        if (help)
        {
            std::cout << help_message;
            return 0;
        }

        if (run + list + rewrite > 1)
        {
            std::cerr << "More than one command is specified. " << read_help_message << std::endl;
            return 1;
        }
        if (run + list + rewrite == 0)
        {
            run = true;
        }
        if ((run || list) && (!unittest && !regression))
        {
            all = true;
        }

        if (all)
        {
            unittest = {};
            regression = {};
        }

        

        auto wanted_unittests = collect_tests<Unittest>(*unittest);

        if (!wanted_unittests)
        {
            return 1;
        }
        if (run)
        {
            if (run_tests(*wanted_unittests))
            {
                return 1;
            }
        } else if(list)
        {
            if (list_tests(*wanted_unittests)) 
            {
                return 1;
            }
        }

    }

}

ADD_UNITTEST(1, 01, "test 1")
{
    return true;
}

int main(int argc, char**argv)
{
    return test::handle_args(argc-1, argv + 1);
}