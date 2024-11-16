#include <iostream>
#include <map>
#include "help.h"
#include "test.h"

namespace test
{

    std::vector<const Test*> collect_tests(char**names, size_t count)
    {
        std::map<std::string_view, const Test*> tests;
        if (count == 0)
        {
            return Test::all();
        }
        for (auto i : Test::all())
        {
            tests[i->run_name()] = i;
        }
        std::vector<const Test*> out;
        for (size_t i = 0; i < count; i++)
        {
            auto it = tests.find(names[i]);
            if (it == tests.end())
            {
                std::cerr << "Unrecognized test name '" << names[i] << "'." << std::endl;
                std::cerr << "Run 'test list' to list all tests" << std::endl;
                return {};
            }
            out.push_back(it->second);
        }
        return out;
    }

    bool handle_list(int argc, char**argv)
    {
        if (argc > 0)
        {
            std::cerr << "Command 'list' must have no arguments." << std::endl;
            std::cerr << read_help_message << std::endl;
            return false;
        }
        Test::list();
        return true;
    }

    bool handle_run(int argc, char**argv)
    {
        auto tests = collect_tests(argv, argc);
        if (tests.size() == 0)
        {
            return false;
        }
        return Test::run(tests);
    }

    bool handle_rewrite(int argc, char **argv)
    {
        if (argc == 0)
        {
            std::cerr << "Name of test to rewrite must be specified." << std::endl;
            std::cerr << read_help_message << std::endl;
            return false;
        }
        if (argc > 1)
        {
            std::cerr << "Only one test references can be rewritten at a time, but several were specified." << std::endl;
            std::cerr << read_help_message << std::endl;
            return false;
        }

        auto tests = collect_tests(argv, 1);
        if (tests.size() == 0)
        {
            return false;
        }
        test_execution_context ctx;
        ctx.rewrite = true;
        tests[0]->unsafe_run(ctx);
        return true;
    }

    bool handle_unsafe(int argc, char**argv)
    {
        if (argc == 0)
        {
            std::cerr << "Name of test to run must be specified." << std::endl;
            std::cerr << read_help_message << std::endl;
            return false;
        }
        if (argc > 1)
        {
            std::cerr << "Only one test can be run unsafe at a time, but several were specified." << std::endl;
            std::cerr << read_help_message << std::endl;
            return false;
        }

        auto tests = collect_tests(argv, 1);
        if (tests.size() == 0)
        {
            return false;
        }
        test_execution_context ctx;
        ctx.rewrite = false;
        tests[0]->unsafe_run(ctx);
        return true;
    }

    bool handle_args(int argc, char**argv)
    {
        
        if (argc < 2)
        {
            std::cout << read_help_message << std::endl;
            return false;
        }

        std::string_view cmd = argv[1];

        if (cmd == "help")
        {
            std::cout << help_message;
            return true;
        }
        else if (cmd == "list")
        {
            return handle_list(argc - 2, argv + 2);
        }
        else if(cmd == "run")
        {
            return handle_run(argc-2, argv + 2);
        }
        else if(cmd == "rewrite")
        {
            return handle_rewrite(argc-2, argv + 2);
        }
        else if(cmd == "unsafe")
        {
            return handle_unsafe(argc-2, argv + 2);
        }
        else
        {
            std::cerr << "Unrecognized command '" << cmd << "'." << std::endl;
            std::cerr << read_help_message << std::endl;
            return false;
        }

    }

}

ADD_TEST(x, "x")
{

}

int main(int argc, char**argv)
{
    return !test::handle_args(argc, argv);
}