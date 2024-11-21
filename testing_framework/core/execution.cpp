#include <testing_framework/core/execution.h>
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

    static std::map<std::string, std::pair<const std::type_info*, std::string>> test_options_;

    /*
        Skip test if types are mismatched
    */
    bool get_test_flag(std::string name)
    {
        auto it = test_options_.find(name);
        if (it == test_options_.end())
        {
            std::cerr << "Trying to access non-existing test option '" << name << "'" << std::endl;
            skip_current_test();
        }
        if (it->second.first != nullptr)
        {
            std::cerr << "Trying to access option '" << name << "' as a flag, but it is not" << std::endl;
            skip_current_test();
        }
        return it->second.second.length() > 0;
    }

    std::string get_test_param(std::string name, const std::type_info*parsing_type)
    {
        auto it = test_options_.find(name);
        if (it == test_options_.end())
        {
            std::cerr << "Trying to access non-existing test option '" << name << "'" << std::endl;
            skip_current_test();
        }
        if (it->second.first == nullptr)
        {
            std::cerr << "Trying to access value of option '" << name << "', but it is a flag" << std::endl;
            skip_current_test();
        }
        if (it->second.first != parsing_type)
        {
            std::cerr << "Trrying to access value of option '" << name << "', but with type different type"
                << "'" << parsing_type->name() << "' insted of declared '" << it->second.first->name() << "'" << std::endl;
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
        test_options_ = test_options;
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