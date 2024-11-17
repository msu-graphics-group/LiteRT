#include <iostream>
#include <map>
#include <optional>
#include "help.h"
#include "actions.h"

namespace testing
{

    std::optional<std::vector<const Test*>> collect_tests(char**names, size_t count)
    {
        std::map<std::string_view, const Test*> tests;
        if (count == 0)
        {
            return Test::all();
        }
        for (auto i : Test::all())
        {
            tests[i->name()] = i;
        }
        std::vector<const Test*> out;
        for (size_t i = 0; i < count; i++)
        {
            auto it = tests.find(names[i]);
            if (it == tests.end())
            {
                std::cerr << "Unrecognized test name '" << names[i] << "'." << std::endl;
                std::cerr << help::read_list_message() << std::endl;
                return std::nullopt;
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
            std::cerr << help::read_help_message() << std::endl;
            return false;
        }
        return list();
    }

    bool handle_run(int argc, char**argv)
    {
        auto tests = collect_tests(argv, argc);
        if (!tests)
        {
            return false;
        }
        ExecutionContext ctx;
        ctx.rewrite = false;
        return run(*tests, ctx);
    }

    bool handle_rewrite(int argc, char **argv)
    {
        if (argc == 0)
        {
            std::cerr << "Name of test to rewrite must be specified." << std::endl;
            std::cerr << help::read_help_message() << std::endl;
            return false;
        }
        if (argc > 1)
        {
            std::cerr << "Only one test's references can be rewritten at a time, but several tests were specified." << std::endl;
            std::cerr << help::read_help_message() << std::endl;
            return false;
        }

        auto tests = collect_tests(argv, 1);
        if (!tests)
        {
            return false;
        }
        ExecutionContext ctx;
        ctx.rewrite = true;
        return unsafe((*tests)[0], ctx);
    }

    bool handle_unsafe(int argc, char**argv)
    {
        if (argc == 0)
        {
            std::cerr << "Name of test to run must be specified." << std::endl;
            std::cerr << help::read_help_message() << std::endl;
            return false;
        }
        if (argc > 1)
        {
            std::cerr << "Only one test can be run unsafe at a time, but several tests were specified." << std::endl;
            std::cerr << help::read_help_message() << std::endl;
            return false;
        }

        auto tests = collect_tests(argv, 1);
        if (!tests)
        {
            return false;
        }
        ExecutionContext ctx;
        ctx.rewrite = false;
        return unsafe((*tests)[0], ctx);
    }

    bool handle_args(int argc, char**argv)
    {
        
        if (argc < 1)
        {
            std::cerr << help::read_help_message() << std::endl;
            return false;
        }

        std::string_view cmd = argv[0];

        if (cmd == "help")
        {
            std::cout << help::help_message();
            return true;
        }
        else if (cmd == "list")
        {
            return handle_list(argc - 1, argv + 1);
        }
        else if(cmd == "run")
        {
            return handle_run(argc-1, argv + 1);
        }
        else if(cmd == "rewrite")
        {
            return handle_rewrite(argc-1, argv + 1);
        }
        else if(cmd == "unsafe")
        {
            return handle_unsafe(argc-1, argv + 1);
        }
        else
        {
            std::cerr << "Unrecognized command '" << cmd << "'." << std::endl;
            std::cerr << help::read_help_message() << std::endl;
            return false;
        }

    }

}


int main(int argc, char**argv)
{
    return !testing::handle_args(argc - 1, argv + 1);
}