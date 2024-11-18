set(CMAKE_CXX_STANDARD 20)

add_executable(
    test
    example_tests.cpp
)

target_link_libraries(test PRIVATE testing_framework)
