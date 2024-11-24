#include <testing_framework/core/execution.h>
#include <testing_framework/core/logging.h>
#include <iostream>

namespace testing
{

    struct SkipCurrentTestException{};

    static size_t passed_ = 0;
    static size_t failed_ = 0;

    void skip_current_test()
    {
        throw SkipCurrentTestException{};
    }

    void add_current_test_check_result(bool passed)
    {
        if (passed)
        {
            passed_++;
        }
        else
        {
            failed_++;
        }
    }

    static bool rewrite_ = false;

    bool rewrite()
    {
        return rewrite_;
    }

    static std::string_view test_name_;

    std::string_view get_current_test_name()
    {
        return test_name_;
    }

    static std::map<std::string, std::pair<const std::type_info*, std::string>> test_options_;

    /*
        Skip test if types are mismatched
    */
    bool get_test_flag(std::string_view name)
    {
        auto it = test_options_.find(std::string(name));
        if (it == test_options_.end())
        {
            log(bar_error) << "Trying to access non-existing option " << foreground(option_color) << name << default_color << "." << std::endl;
            skip_current_test();
        }
        if (it->second.first != nullptr)
        {
            log(bar_error) << "Trying to access option " << foreground(option_color) << name << default_color << " as a flag, but it is not." << std::endl;
            skip_current_test();
        }
        return it->second.second.length() > 0;
    }

    std::string_view get_test_param(std::string_view name, const std::type_info*parsing_type)
    {
        auto it = test_options_.find(std::string(name));
        if (it == test_options_.end())
        {
            log(bar_error) << "Trying to access non-existing option " << foreground(option_color) << name << default_color << "." << std::endl;
            skip_current_test();
        }
        if (it->second.first == nullptr)
        {
            log(bar_error) << "Trying to access value of option " << foreground(option_color) << name << default_color << ", but it is a flag." << std::endl;
            skip_current_test();
        }
        if (it->second.first != parsing_type)
        {
            log(bar_error) << "Trying to access value of option " << foreground(option_color) << name << default_color
                << ", but with type '" << foreground(highlight_color_2) << parsing_type->name() << default_color 
                << "' instead of declared '" << foreground(highlight_color_2) << it->second.first->name() << default_color << "'." << std::endl;
            skip_current_test();
        }
        return it->second.second;
    }

    /*
        Returns true, if test was executed successfully
        Returns false, if test was skipped
    */
    bool execute_test(
        const Test*test,
        bool rewrite,
        std::map<std::string, std::pair<const std::type_info*, std::string>> test_options,
        size_t&passed_checks,
        size_t&failed_checks
    )
    {
        passed_ = 0;
        failed_ = 0;
        rewrite_ = rewrite;
        test_name_ = test->name();
        test_options_ = std::move(test_options);
        try 
        {
            test->run();
            passed_checks = passed_;
            failed_checks = failed_;
            return true;
        }
        catch(SkipCurrentTestException)
        {
            return false;
        }
    }

}