#include <testing_framework/core/commands.h>
#include <testing_framework/core/execution.h>
#include <testing_framework/core/supervisor.h>
#include <testing_framework/core/cli.h>
#include <testing_framework/core/colors.h>
#include <iostream>
#include <algorithm>

namespace testing
{   
    enum class TEST_RESULT
    {
        PASSED,
        FAILED,
        SKIPPED,
        CRASHED
    };

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
            if (!x) // reading error
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
    
    bool run_supervised(Supervisor&supervisor, TEST_RESULT&result, size_t&passed, size_t&failed)
    {

        std::string last_line;
        if (!run_and_get_last_line(supervisor, last_line))
        {
            return false;
        }

        if (!supervisor.exited() || supervisor.exit_status() != 0)
        {
            result = TEST_RESULT::CRASHED;
            return true;
        }

        if (!parse_last_line(last_line, result, passed, failed))
        {
            std::cout << "[ERROR]" << " Failed to parse test output. Assuming runtime error." << std::endl;
            result = TEST_RESULT::CRASHED;
            return true;
        }
        
        return true;
    }

    bool run(
        size_t logging_level,
        size_t jobs,
        const std::vector<const Test*>& tests,
        std::map<std::string, std::pair<const std::type_info*, std::string>> test_options
    )
    {
        for (auto i : tests)
        {
            auto args = cmdline_to_exec(
                colors_are_enabled(),
                logging_level,
                std::string(i->name()),
                false,
                test_options
            );
            auto supervisor = Supervisor::spawn(args);
            if (!supervisor)
            {
                return false;
            }
            TEST_RESULT result;
            size_t passed, failed;
            if (!run_supervised(*supervisor, result, passed, failed))
            {
                return false;
            }
            if (result == TEST_RESULT::CRASHED)
            {
                std::cout << "[CRASHED] " << i->name() << std::endl;
            }
        }

        return true;
    }

    bool exec(
        size_t logging_level,
        bool rewrite,
        const Test*test,
        std::map<std::string, std::pair<const std::type_info*, std::string>> test_options
    )
    {
        size_t passed_checks = 0;
        size_t failed_checks = 0;

        std::cout << "[RUN] " << test->name() << std::endl;
        bool was_skipped = !execute_test(test, rewrite, test_options, passed_checks, failed_checks);

        if (was_skipped)
        {
            std::cout << "[SKIPPED] " << test->name() << std::endl;
        }
        else
        {
            if (failed_checks > 0)
            {
                std::cout << "[FAILED]";
            }
            else
            {
                std::cout << "[PASSED]";
            }
            std::cout << " (" << make_test_summary(passed_checks, failed_checks) << ")" << std::endl;
        }

        return true;
    }

}