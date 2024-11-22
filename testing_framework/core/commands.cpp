#include <testing_framework/core/commands.h>
#include <testing_framework/core/execution.h>
#include <testing_framework/core/supervisor.h>
#include <testing_framework/core/cli.h>
#include <testing_framework/core/colors.h>
#include <testing_framework/core/logging.h>
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


    /*
        Finds
        \33[<n>;...;<n>m
        in line from offset
    */
    bool str_to_esc(std::string_view str, size_t&offset)
    {
        
        if (str.length() - offset < 2)
        {
            return false;
        }
        if (str[offset] != '\33')
        {
            return false;
        }
        offset++;
        if (str[offset] != '[')
        {
            return false;
        }
        offset++;

        while (true)
        {
            while (offset < str.length() && std::isdigit(str[offset]))
            {
                offset++;
            }
            if (offset == str.length())
            {
                return false;
            }
            if (str[offset] == ';')
            {
                offset++;
            }
            else if (str[offset] == 'm')
            {
                offset++;
                return true;
            }
            else
            {
                return false;
            }
        }
    }

    /*
        Finds
        <ESC>...<ESC>[<inside>]...
        where <ESC> is \33[<n>;...;<n>m
    */
    bool find_bar(std::string_view line, size_t&inside_begin, size_t& inside_end)
    {

        size_t offset = 0;

        while (offset < line.length())
        {
            if (str_to_esc(line, offset))
            {
                continue;
            }
            else // no more color escape sequences
            {
                break;
            }
        }

        if (offset == line.length() || line[offset] != '[')
        {
            return false;
        }
        inside_begin = offset + 1;
        offset++;
        while (offset < line.length() && line[offset] != ']')
        {
            offset++;
        }
        if (offset == line.length())
        {
            return false;
        }
        inside_end = offset;
        return true;
    }

    void reformat_output(std::string_view line)
    {
        size_t begin, end;
        if (find_bar(line, begin, end))
        {
            std::string_view bar_inside(line.begin() + begin, end - begin);
            auto has = [&](std::string_view str)->bool{ return bar_inside.find(str) != std::string::npos; };
           
            size_t level = has("RUN") || has("PASSED") || has("FAILED") || has("SKIPPED") ? TEST_RESULT_LOGGING_LEVEL :
                has("INFO")? INFO_LOGGING_LEVEL :
                has("WARNING") ? WARNING_LOGGING_LEVEL :
                has("ERROR") ? ERROR_LOGGING_LEVEL : 
                INFO_LOGGING_LEVEL;
            log(level) << foreground(gray)
                << std::string_view(line.begin(), begin-1)  // without '['
                << '[' << begin_aligned(BAR_WIDTH, 0, 0)
                << bar_inside << end_aligned
                << ']' << default_color
                << std::string_view(line.begin() + end + 1, line.length() - end - 1);
        }
        else
        {
            std::string lowercase(line);
            std::transform(lowercase.begin(), lowercase.end(), lowercase.begin(), [](char x)->char{ return std::tolower(x); });
            auto has = [&](std::string_view str)->bool{ return lowercase.find(str) != std::string::npos; };
            log(has("warning") ? bar_warning : has("error") ? bar_error : bar_info) << line;
        }
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
                reformat_output(curr_line);
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
            log(bar_error) << "Failed to parse test output. Assuming runtime error." << std::endl;
            result = TEST_RESULT::CRASHED;
            return true;
        }
        
        return true;
    }

    bool list(
        const std::vector<const Test*>& tests,
        bool write_descriptions
    )
    {
        auto line = bar_line;
        auto bold_line = bar_bold_line;
        line.color = info_color;
        bold_line.color = info_color;
        log(bold_line) << "Listing " << tests.size() << " tests." << std::endl;
        for (size_t i = 0; i < tests.size(); i++)
        {
            log(line) <<
                foreground(highlight_color_3) << i + 1 << default_color
                << "/" << tests.size() << std::endl;
            log(bar_info) << foreground(highlight_color_1) << tests[i]->name() << default_color
                << " defined at "
                << foreground(highlight_color_2) << tests[i]->file() << ":" << tests[i]->line() << default_color
                <<  std::endl;
            if (write_descriptions)
            {
                log(bar_info) << "\"" << tests[i]->description() << "\"" << std::endl;
            }
            log(line) << std::endl;
        }
        log(bold_line) << "Finished listing " << tests.size() << " tests" << std::endl;
        return true;
    }

    bool run(
        size_t logging_level,
        size_t jobs,
        const std::vector<const Test*>& tests,
        std::map<std::string, std::pair<const std::type_info*, std::string>> test_options
    )
    {
        set_logging_level(logging_level);
        log(bar_bold_line) << "running " << tests.size() << " tests" << std::endl;
        for (size_t i = 0; i < tests.size(); i++)
        {
            auto supervisor = Supervisor::spawn(cmdline_to_exec(
                colors_are_enabled(),
                logging_level,
                std::string(tests[i]->name()),
                false,
                test_options
            ));
            
            if (!supervisor)
            {
                return false;
            }
            
            log(bar_line) << "starting test " << i + 1 << "/" << tests.size() << std::endl;
            TEST_RESULT result;
            size_t passed, failed;
            if (!run_supervised(*supervisor, result, passed, failed))
            {
                return false;
            }
            if (result == TEST_RESULT::CRASHED)
            {
                log(bar_crashed) << tests[i]->name() << std::endl;
            }
            log(bar_line) << "ended test " << i + 1 << "/" << tests.size() << std::endl;
        }
        log(bar_bold_line) << "finished running" << std::endl;

        return true;
    }

    bool exec(
        size_t logging_level,
        bool rewrite,
        const Test*test,
        std::map<std::string, std::pair<const std::type_info*, std::string>> test_options
    )
    {
        set_logging_level(logging_level);
        size_t passed = 0;
        size_t failed = 0;

        log(bar_run) << test->name() << std::endl;
        bool was_skipped = !execute_test(test, rewrite, std::move(test_options), passed, failed);

        if (was_skipped)
        {
            log(bar_skipped) << test->name() << std::endl;
        }
        else
        {
            size_t total = passed + failed;
            log(failed > 0 ? bar_failed : bar_passed) 
                << test->name() << " "
                << "(" << foreground(success_color) << passed << "/" <<  total << " passed " << default_color << "checks, "
                <<  foreground(failure_color) << failed << "/" << total << " failed " << default_color << "checks" << ")" << std::endl;
        }

        return true;
    }

}