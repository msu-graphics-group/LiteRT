#include <testing_framework/core/add_test.h>
#include <iostream>

namespace litert_tests
{

    ADD_TEST(test1, "Example test")
    {
        std::cout << "Code inside test" << std::endl;
    }

}