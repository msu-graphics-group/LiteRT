#pragma once
#include "test.h"

namespace testing
{

    struct ExecutionContext
    {
        bool rewrite;
    };

    bool list();
    bool run(const std::vector<const Test*>&tests, ExecutionContext);
    bool unsafe(const Test*, ExecutionContext);
    
}