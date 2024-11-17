#include "actions.h"
#include <iostream>
#include <algorithm>
#include "exe.h"
#include "supervisor.h"
#include <map>

namespace testing
{

    static ExecutionContext context;
    
    static size_t passed_checks = 0;
    static size_t failed_checks = 0;

    struct TestSkipException{};

    // Declared in env.cpp
    bool rewrite_is_enabled()
    {
        return context.rewrite;
    }

    // Declared in env.h
    void skip()
    {
        throw TestSkipException{};
    }

    // Declared in env.h
    void add_check_result(bool passed)
    {
        if (passed)
        {
            passed_checks++;
        }
        else
        {
            failed_checks++;
        }
    }

    enum class TEST_RESULT
    {
        PASSED,
        FAILED,
        SKIPPED,
        CRASHED
    };

    std::string to_string(TEST_RESULT r)
    {
        static std::string names[] = {
            "PASSED",
            "FAILED",
            "SKIPPED",
            "CRASHED"
        };
        return names[static_cast<int>(r)];
    }

    bool list()
    {
        for (auto i : Test::all())
        {

            std::cout << i->name() << std::endl;
            std::cout << "    \"" << i->description() << "\"" << std::endl;
            std::cout << "    " << i->file() << ":" << i->line() << std::endl;
        }
        return true;
    }

    std::string make_test_summary(size_t passed, size_t failed)
    {
        size_t total = passed + failed;

        return std::to_string(passed) + "/" + std::to_string(total) + " checks passed, " +
        std::to_string(failed) + "/" + std::to_string(total) + " checks failed";
    }

    bool parse_test_summary(std::string_view text, size_t&passed, size_t&failed)
    {
        static auto parse_number = [](std::string_view s)->size_t{
            size_t out = 0;
            for (auto c : s)
            {
                out *= 10;
                out += c - '0';
            }
            return out;
        };

        auto first_slash = std::find(text.begin(), text.end(), '/');

        if (first_slash == text.end())
        {
            return false;
        }

        auto second_slash = std::find(first_slash + 1, text.end(), '/');
        if (second_slash == text.end())
        {
            return false;
        }

        static auto parse_number_before = [](auto begin, auto end, size_t&number)->bool{
            if (end == begin)
            {
                return false;
            }
            auto it = end - 1;
            while (it != begin && std::isdigit(*it))
            {
                --it;
            }
            if (!isdigit(*it))
            {
                ++it;
            }
            if (it == end)
            {
                return false;
            }
            number = parse_number(std::string_view(it, end - it));
            return true;
        };

        return parse_number_before(text.begin(), first_slash, passed) && 
            parse_number_before(text.begin(), second_slash, failed);
    }

    bool run(const std::vector<const Test*>&tests, ExecutionContext ctx)
    {

        std::map<TEST_RESULT, std::vector<const Test*>> results;
        size_t passed_checks = 0;
        size_t failed_checks = 0;

        for (auto i : tests)
        {
            auto supervisor = Supervisor::spawn({
                current_executable_path(),
                "unsafe",
                std::string{i->name()}
            });
            if (!supervisor)
            {
                return false;
            }
            std::string prev_line, curr_line;
            char last_char = '\n';
            while (true)
            {
                auto x = supervisor->get_char();
                if (!x)break;
                if (*x == EOF) break;
                char c = *x;
                std::cout << c;
                last_char = c;
                if (c == '\n')
                {
                    prev_line = curr_line;
                    curr_line = "";
                }
                else
                {
                    curr_line.push_back(c);
                }
            }

            if (last_char != '\n')
            {
                std::cout << std::endl;
            }

            if (!supervisor->exited() || supervisor->exit_status() != 0) // killed or crashed
            {
                results[TEST_RESULT::CRASHED].push_back(i);
                std::cout << "[CRASHED] " << i->name() << std::endl;
            }
            else
            {

                std::string last_line = curr_line == "" ? prev_line : curr_line;
                
                // parsing last line to get results
                size_t passed, failed;
                bool res = parse_test_summary(last_line, passed, failed);
                if (last_line.find("PASSED") != std::string::npos && res)
                {
                    results[TEST_RESULT::PASSED].push_back(i);
                    passed_checks += passed;
                    failed_checks += failed;
                }
                else if (last_line.find("FAILED") != std::string::npos && res)
                {
                    results[TEST_RESULT::FAILED].push_back(i);
                    passed_checks += passed;
                    failed_checks += failed;
                }
                else if(last_line.find("SKIPPED") != std::string::npos)
                {
                    results[TEST_RESULT::SKIPPED].push_back(i);
                }
                else
                {
                    std::cout << "[ERROR] Failed to parse test output. Assuming runtime error." << std::endl;
                    results[TEST_RESULT::CRASHED].push_back(i);
                    std::cout << "[CRASHED] " << i->name() << std::endl;
                }
            }

        }

        size_t total_checks = passed_checks + failed_checks;

        std::cout << "Passed: " << results[TEST_RESULT::PASSED].size() << std::endl;
        std::cout << "Failed: " << results[TEST_RESULT::FAILED].size() << std::endl;
        std::cout << "Skipped: " << results[TEST_RESULT::SKIPPED].size() << std::endl;
        std::cout << "Crashed: " << results[TEST_RESULT::CRASHED].size() << std::endl;
        std::cout << "Total passed checkes: " << passed_checks << "/" << total_checks << std::endl;
        std::cout << "Total failed checkes: " << failed_checks << "/" << total_checks << std::endl;

        return true;
    }

    bool unsafe(const Test*test, ExecutionContext ctx)
    {
       
        context = ctx;

        std::cout << "[RUN] " << test->name() << std::endl;

        try {
            test->run();
        }
        catch(TestSkipException)
        {
            std::cout << "[SKIPPED] " << test->name() << std::endl;
            return true;    
        }

        std::cout << (failed_checks > 0 ? "[FAILED]" : "[PASSED]") << " " << test->name() << " (" + 
            make_test_summary(passed_checks, failed_checks) << ") " << std::endl;

        return true;
    }

}