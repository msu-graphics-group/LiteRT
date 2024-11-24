#include <testing_framework/core/commands.h>
#include <testing_framework/core/execution.h>
#include <testing_framework/core/supervisor.h>
#include <testing_framework/core/cli.h>
#include <testing_framework/core/colors.h>
#include <testing_framework/core/logging.h>
#include <testing_framework/core/run_supervised.h>
#include <iostream>
#include <algorithm>
#include <list>

namespace testing
{   

    bool list(
        const std::vector<const Test*>& tests,
        bool write_descriptions
    )
    {
        auto line = bar_line;
        auto bold_line = bar_bold_line;
        line.color = info_color;
        bold_line.color = info_color;
        log(bold_line) << "Listing " << tests.size() << " tests" << std::endl;
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

        std::map<TEST_RESULT, std::vector<const Test*>> results;

        std::list<std::pair<size_t, Supervisor>> running_tests;

        size_t first_remaining_test = 0;

        log(bar_bold_line) << "Running " << tests.size() << " tests" << std::endl;

        while (true)
        {
            if (running_tests.size() < jobs && first_remaining_test < tests.size())
            {
                auto new_test = tests[first_remaining_test];
                first_remaining_test++;

                auto supervisor = Supervisor::spawn(cmdline_to_exec(
                    colors_are_enabled(),
                    logging_level,
                    new_test->name(),
                    false,
                    test_options
                ));
                if (!supervisor)
                {
                    return false;
                }
                running_tests.push_back({first_remaining_test-1, std::move(*supervisor)});
            }
            else if(running_tests.size() > 0)
            {
                auto[test_index, supervisor] = std::move(running_tests.front());
                running_tests.pop_front();
                log(bar_line) << "Starting test " << test_index + 1 << "/" << tests.size() << std::endl;
                TEST_RESULT result;
                size_t passed, failed;
                if (!run_supervised(supervisor, tests[test_index]->name(), result, passed, failed))
                {
                    return false;
                }
                log(bar_line) << "Ended test " << test_index + 1 << "/" << tests.size() << std::endl;
                results[result].push_back(tests[test_index]);
            }
            else
            {
                break;
            }
        }
        log(bar_bold_line) << "Finished running tests" << std::endl;

        
        auto log_result = [&](TEST_RESULT r, auto bar){
            auto line = bar_line;
            bar.align = 0;
            line.color = bar.color;
            std::string group(bar.text);
            std::transform(group.begin(), group.end(), group.begin(), [](char x){ return std::tolower(x); });
            group[0] = std::toupper(group[0]);
            log(line) << group << " tests" << std::endl;
            for (auto test : results[r])
            {
                log(bar) << test->name() << std::endl;
            }
            log(line) << group << " tests" << std::endl;
        };

        log(bar_bold_line) << "Listing results" << std::endl;
        
        log_result(TEST_RESULT::PASSED, bar_passed);
        log_result(TEST_RESULT::FAILED, bar_failed);
        log_result(TEST_RESULT::SKIPPED, bar_skipped);
        log_result(TEST_RESULT::CRASHED, bar_crashed);

        log(bar_bold_line) << "Finished listing results" << std::endl;

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