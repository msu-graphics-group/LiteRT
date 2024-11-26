set(CMAKE_CXX_STANDARD 20)

add_executable(
    testrt
    new_tests.cpp
    tests_main.cpp
)

target_link_libraries(testrt PRIVATE testing_framework)

testing_discover_tests(testrt)
