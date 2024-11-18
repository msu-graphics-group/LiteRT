
set(CMAKE_CXX_STANDARD 20)

add_subdirectory(logging)
add_subdirectory(core)
add_subdirectory(helpers)

add_library(
    testing_framework
    INTERFACE
)

target_link_libraries(
    testing_framework INTERFACE
    testing_framework::logging
    testing_framework::core
    testing_framework::helpers
)
