#include <testing_framework/core/add_test.h>
#include <testing_framework/core/env.h>
#include <iostream>

namespace litert_tests
{

    ADD_TEST(test1, "Example test")
    {
        std::cout << "Code inside test" << std::endl;

        testing::add_check_result(true);
        testing::add_check_result(false);
        testing::add_check_result(true);

    }

    ADD_TEST(test2, "Example for skipped test")
    {
        std::cout << "Before skip" << std::endl;
        testing::skip();
        std::cout << "After skip" << std::endl;
    }

    ADD_TEST(test3, "Example of rewrite")
    {
        std::cout << "Do smth" << std::endl;
        if (testing::should_rewrite("rewrite VERY IMPORTANT files"))
        {
            std::cout << "Files are rewritten!!!" << std::endl;
        }
        else
        {
            std::cout << "Nothing changed!" << std::endl; 
        }
        std::cout << "Just continue test" << std::endl;

    }

    ADD_TEST(test4, "Randomly crashing")
    {
        srand(time(NULL));
        bool crash = rand() % 2;
        
        srand(1);
        int x = rand();
        srand(1);
        int y = rand();
        if (crash)
        {
            int zero  =x - y;
            std::cout << "Dividing by zero!!" << std::endl;
            std::cout << "Result is: " << 1 / zero << std::endl;
        }
    }

    ADD_TEST(test5, "Invalid output example")
    {
        std::cout << "Bad output" << std::endl;
        exit(0);
    }

}