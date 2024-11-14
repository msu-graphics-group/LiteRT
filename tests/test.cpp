#include "test.h"
#include <iostream>
#include <map>
#include <optional>
#include "args.h"
#include "help.h"
#include <iomanip>

namespace test
{

    enum class COLOR
    {
        RED,
        GREEN
    };

    const char* color_begin(COLOR c)
    {
        switch(c)
        {
            case COLOR::RED:
                return "\33[31m";
            case COLOR::GREEN:
                return "\33[32m";
            default:
                return "\33[97m"; // white
        }
    }

    const char* color_end()
    {
        return "\33[0m";
    }

    template<typename Test>
    std::optional<std::vector<const Test*>> collect_tests(const std::vector<std::string_view>&names)
    {
        if (names.size() == 0)
        {
            return Test::all();
        }

        std::map<std::string_view, const Test*> tests;

        for (auto i : Test::all())
        {
            tests[i->name()] = i;
        }

        std::vector<const Test*> out;
        for (auto name : names)
        {
            auto it  = tests.find(name);
            if (it == tests.end())
            {
                std::cerr << "Unrecognized test name '" << name << "'" << std::endl;
                return std::nullopt;
            }
            out.push_back(it->second);
        }

        return out;
    }

    template<typename Test>
    bool list_tests(const std::vector<const Test*>& tests)
    {

        for (auto i : tests)
        {
            std::cout << "[" << i->name() << "]" << " \"" << i->description() << "\"" << std::endl;
        }

        return true;
    }

    template<typename Test>
    bool run_tests(const std::vector<const Test*>& tests, size_t&total)
    {
        total = 0;
        for (auto i : tests)
        {
            std::cout << "[RUN   ] [" << i->name() << "] \"" << i->description() << "\"" << std::endl;
            bool res = i->execute();
            if (res)
            {
                std::cout << color_begin(COLOR::GREEN) << "[PASSED]" << color_end();
                total += 1;
            }
            else
            {
                std::cout << color_begin(COLOR::RED) <<  "[FAILED]" << color_end();
            }
            std::cout << " [" << i->name() << "]" << std::endl;
        }
        return true;
    }

    bool rewrite_regression(const Regression*test)
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
            unittest = std::vector<std::string_view>{};
            regression = std::vector<std::string_view>{};
        }

        auto wanted_unittests = (unittest ? collect_tests<Unittest>(*unittest) : std::vector<const Unittest*>{});
        if (!wanted_unittests)
        {
            return false;
        }
        
        auto wanted_regressions = regression ? collect_tests<Regression>(*regression) : std::vector<const Regression*>{};
        if (!wanted_regressions)
        {
            return false;
        }

        bool has_unittests = wanted_unittests->size() > 0;
        bool has_regressions = wanted_regressions->size() > 0;

        if (run)
        {
            if (has_unittests) {
                std::cout << "###################" << std::endl;
                std::cout << " Running unittests" << std::endl;
                std::cout << "###################" << std::endl;
                size_t passed_unittests = 0;
                if (!run_tests(*wanted_unittests, passed_unittests))
                {
                    return false;
                }
                std::cout << "Total: " << passed_unittests << "/" << wanted_unittests->size() << " passed." << std::endl;
            }

            if (has_unittests && has_regressions)
                std::cout << std::endl;
            
            if (has_regressions) {
                std::cout << "##########################" << std::endl;
                std::cout << " Running regression tests" << std::endl;
                std::cout << "##########################" << std::endl;
                size_t passed_regressions = 0;
                if (!run_tests(*wanted_regressions, passed_regressions))
                {
                    return false;
                }
                std::cout << "Total: " << passed_regressions << "/" << wanted_regressions->size() << " passed." << std::endl;
            }

            return true;
        }
        else if(list)
        {
            if (has_unittests) {
            std::cout << "###########" << std::endl;
            std::cout << " Unittests" << std::endl;
            std::cout << "###########" << std::endl;
            if (!list_tests(*wanted_unittests))
            {
                return false;
            }
            }

            if (has_unittests && has_regressions)
                std::cout << std::endl;
            
            if (has_regressions) {
                std::cout << "##################" << std::endl;
                std::cout << " Regression tests" << std::endl;
                std::cout << "##################" << std::endl;
                if (!list_tests(*wanted_regressions))
                {
                    return false;
                }
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

        // Should never be here
        return true;
    }

}

ADD_UNITTEST(1, 01, "test 1")
{
    return true;
}

ADD_REGRESSION(2, 10, "reg test 10")
{
    return false;
}

int main(int argc, char**argv)
{
    return !test::handle_args(argc-1, argv + 1);
}