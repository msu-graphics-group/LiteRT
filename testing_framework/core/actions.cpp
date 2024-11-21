#include "actions.h"
#include <iostream>
#include <algorithm>
#include "exe.h"
#include "supervisor.h"
#include <map>
#include <testing_framework/core/colors.h>

namespace testing
{

    static ExecutionContext context;
    
    static size_t passed_checks = 0;
    static size_t failed_checks = 0;

    struct TestSkipException{};
    /*
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
    }*/

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

    /*
        Reads stdout of supervised process line by line with \n
        Empty string means eof
        Returns false if error happend
    */
    bool get_line(Supervisor&s, std::string&line)
    {
        line = "";
        while (true)
        {
            auto x = s.get_char();
            if (!x) // read error
            {
                return false;
            }
            if (*x == EOF)
            {
                break;
            }
            char c = *x;
            line.push_back(c);
            if (c == '\n')
            {
                break;
            }
        }
        return true;
    }

    bool run_and_get_last_line(Supervisor&supervisor, std::string&last_line)
    {
        std::string prev_line, curr_line;
        while (true)
        {
            prev_line = curr_line;
            if (!get_line(supervisor, curr_line))
            {
                return false;
            }
            if (curr_line == "")
            {
                break;
            }
            else
            {
                if (curr_line.back() != '\n')
                {
                    curr_line.push_back('\n');
                }
                std::cout << curr_line;
            }
        }
        last_line = (curr_line == "" ? prev_line : curr_line);
        return true;
    }

    bool parse_last_line(std::string_view line, TEST_RESULT&result, size_t&passed, size_t&failed)
    {
        bool res = parse_test_summary(line, passed, failed);
        if (line.find("PASSED") != std::string::npos && res)
        {
            result = TEST_RESULT::PASSED;
            return true;
        }
        if (line.find("FAILED") != std::string::npos && res)
        {
            result = TEST_RESULT::FAILED;
            return true;
        }
        if (line.find("SKIPPED") != std::string::npos)
        {
            result = TEST_RESULT::SKIPPED;
            return true;
        }
        return false;
    }

    bool run_supervised(const Test*test, TEST_RESULT&result, size_t&passed, size_t&failed)
    {

        auto supervisor = Supervisor::spawn({
            current_executable_path(),
            "unsafe",
            std::string{test->name()}
        });

        if (!supervisor)
        {
            return false;
        }
        std::string last_line;
        if (!run_and_get_last_line(*supervisor, last_line))
        {
            return false;
        }

        if (!supervisor->exited() || supervisor->exit_status() != 0)
        {
            result = TEST_RESULT::CRASHED;
            return true;
        }

        if (!parse_last_line(last_line, result, passed, failed))
        {
            std::cout << foreground(red) << "[ERROR]" << default_color << " Failed to parse test output. Assuming runtime error." << std::endl;
            result = TEST_RESULT::CRASHED;
            return true;
        }
        
        return true;
    }

    bool run(const std::vector<const Test*>&tests, ExecutionContext ctx)
    {

        std::map<TEST_RESULT, std::vector<const Test*>> results;
        size_t passed_checks = 0;
        size_t failed_checks = 0;

        for (auto i : tests)
        {
            TEST_RESULT result;
            size_t passed;
            size_t failed;
            if(!run_supervised(i, result, passed, failed))
            {
                return false;
            }
            results[result].push_back(i);
            if (result == TEST_RESULT::PASSED || result == TEST_RESULT::FAILED) 
            {
                passed_checks += passed;
                failed_checks += failed;
            }
            if (result == TEST_RESULT::CRASHED)
            {
                std::cout << foreground(red) << "[CRASHED]" << default_color << " " << i->name() << std::endl;
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