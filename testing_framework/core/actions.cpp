#include "actions.h"
#include <iostream>

namespace testing::actions
{

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
    bool run(const std::vector<const Test*>&tests, ExecutionContext)
    {
        return true;
    }
    bool unsafe(const Test*, ExecutionContext)
    {
        return true;
    }

}