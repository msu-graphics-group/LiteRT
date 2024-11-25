#pragma once
#include <testing_framework/core/supervisor.h>
#include <string>

namespace testing
{

    enum class TEST_RESULT
    {
        PASSED,
        FAILED,
        SKIPPED,
        CRASHED
    };

    bool run_supervised(
        Supervisor&supervisor,
        std::string_view test_name,
        bool filter_output,
        TEST_RESULT&result,
        size_t&passed,
        size_t&failed
    );

}