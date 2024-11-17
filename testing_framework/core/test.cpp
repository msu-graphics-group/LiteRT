#include "test.h"
#include <iostream>
#include "supervisor.h"
#include <filesystem>

namespace test
{

    // std::vector<const Test*> Test::tests_;

    Test::Test(std::string run_name) :
        run_name_(std::move(run_name))
    {
        tests().push_back(this);
    }

    std::vector<const Test*>& Test::tests()
    {
        static std::vector<const Test*> tests_;
        return tests_;
    }

    std::string_view Test::run_name() const 
    {
        return run_name_;
    }

    const std::vector<const Test*> Test::all()
    {
        return tests();
    }

    void Test::list()
    {

    }

    bool Test::run(const std::vector<const Test*>&tests)
    {
        for (auto i : tests)
        {
            test_execution_info info;
            bool res = i->supervised_execute(info);
            if (!res)
            {
                return false;
            }
        }
        return true;
    }
    
    void Test::unsafe_run(test_execution_context ctx) const
    {
        auto res = unsafe_execute(ctx);

        std::cout << res.passed_checks << " " << res.failed_checks<< " " << res.was_skipped << std::endl;
        
    }

    bool Test::supervised_execute(test_execution_info&info) const
    {
        auto test_executable = std::filesystem::canonical("/proc/self/exe");

        auto superviser =  Supervisor::spawn({test_executable.string(), "help"});

        if (!superviser)
        {
            return false;
        }

        while (true)
        {
            auto x = superviser->get_char();
            if (!x)
                break;
            if (*x == EOF)
                break;
            std::cout << char(*x);
        }

        if (superviser->exited())
        {
            std::cout << "Exited with status: " << superviser->exit_status() << std::endl;
        }
        else
        {
            std::cout << "runtime error" << std::endl;
        }
        
        return true;
    }

    test_execution_info Test::unsafe_execute(test_execution_context) const
    {

    }

}