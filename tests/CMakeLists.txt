set(CMAKE_CXX_STANDARD 20)

add_executable(
    test
    new_tests.cpp
    tests_main.cpp
)

target_link_libraries(test PRIVATE testing_framework)
