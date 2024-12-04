#include <testing_framework/core/add_test.h>
#include <testing_framework/core/environment.h>
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
    if (testing::should_rewrite_saved_reference("VERY IMPORTANT FILE", "./file.txt"))
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
      int zero = x - y;
      std::cout << "Dividing by zero!!" << std::endl;
      std::cout << "Result is: " << 1 / zero << std::endl;
    }
  }

  ADD_TEST(test5, "Invalid output example")
  {
    std::cout << "Bad output" << std::endl;
    exit(0);
  }

  ADD_TEST(test6, "Custom bar inside test")
  {

    std::cout << "[TEST6::LOG] " << "Important data: " << 123 << "!" << std::endl;
    printf("[TEST6::LOG] some other data: %d\n", 42);
    printf("error reading from file %s\n", "<path-to-file>");
    printf("warning something went wrong!\n");
  }

  ADD_TEST(test7, "Some globals")
  {
    bool ignore = testing::get_flag("ignore_saved_references");
    std::cout << "Ignore: " << ignore << std::endl;
    auto w = testing::get_param<int64_t>("image_width");
    std::cout << "Width: " << w << std::endl;
    auto h = testing::get_param<std::string>("image_height");
    std::cout << "Height: " << h << std::endl;
  }

  ADD_TEST(test8, "Some more globals")
  {
    auto some = testing::get_param<std::string>("some");
    std::cout << "Some: " << some << std::endl;
  }

}