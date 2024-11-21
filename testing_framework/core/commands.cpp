#include <testing_framework/core/commands.h>
#include <testing_framework/core/execution.h>

namespace testing
{

    bool exec(
        size_t logging_level,
        bool rewrite,
        const Test*test,
        std::map<std::string, std::pair<const std::type_info*, std::string>> test_options
    )
    {
        size_t passed_checks = 0;
        size_t failed_checks = 0;
        bool was_skipped = !execute_test(test, rewrite, test_options, passed_checks, failed_checks);

        return true;
    }

}