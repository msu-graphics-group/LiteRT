#include "test.h"
#include <iostream>
#include <map>
#include <optional>
#include "args.h"
#include "help.h"

namespace test
{
    std::vector<Unittest*> Unittest::tests_;
}

namespace test
{

    template<typename Test>
    std::optional<std::vector<Test*>> collect_tests(const std::vector<std::string_view>&names)
    {
        return {};
    }

    template<typename Test>
    bool list_tests(const std::vector<Test*>& tests)
    {
        return true;
    }

    template<typename Test>
    bool run_tests(const std::vector<Test*>& tests)
    {
        return true;
    }

    bool rewrite_regression(const Unittest*test)
    {
        return true;
    }

    static bool handle_args(size_t argc, char**argv)
    {
        auto args = Args::parse(argc, argv);
        if (!args)
        {
            return false;
        }

        bool run, list, all, rewrite, help;
        std::optional<std::vector<std::string_view>> unittest, regression;

        if (!(
            args->check_only({
                 "--help",
                "--run",
                "--list",
                "--rewrite",
                "--all",
                "--unittest",
                "--regression"
            }) &&
            args->get("--help", help) &&
            args->get("--run", run) &&
            args->get("--list", list) &&
            args->get("--rewrite", rewrite) &&
            args->get("--all", all) &&
            args->get("--unittest", unittest) &&
            args->get("--regression", regression)
        )) return false;

        if (help)
        {
            std::cout << help_message;
            return true;
        }

        if (run + list + rewrite > 1)
        {
            std::cerr << "More than one command is specified. " << std::endl;
            std::cerr << read_help_message << std::endl;
            return false;
        }
        if (run + list + rewrite == 0)
        {
            run = true;
        }

        if (!unittest && !regression)
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
            return false;
        }
        
        auto wanted_regressions = collect_tests<Unittest>(*regression);
        if (!wanted_regressions)
        {
            return false;
        }


        if (run)
        {
            if (run_tests(*wanted_unittests))
            {
                return false;
            }
            if (!run_tests(*wanted_regressions))
            {
                return false;
            }
            return true;
        }
        else if(list)
        {
            if (list_tests(*wanted_unittests))
            {
                return false;
            }
            if (!list_tests(*wanted_regressions))
            {
                return false;
            }
            return true;
        }
        else if(rewrite)
        {
            if (!(wanted_unittests->size() == 0) && !(wanted_regressions->size() == 1))
            {
                std::cerr << "To do rewrite only one regression test must be specified" << std::endl;
                std::cerr << read_help_message << std::endl;
                return false;
            }
            if (!rewrite_regression((*wanted_regressions)[0]))
            {
                return false;
            }
            return true;
        }

    }

}

ADD_UNITTEST(1, 01, "test 1")
{
    return true;
}

int main(int argc, char**argv)
{
    return !test::handle_args(argc-1, argv + 1);
}