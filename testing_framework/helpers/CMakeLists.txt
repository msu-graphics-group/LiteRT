
add_library(
    testing_framework_helpers INTERFACE
)


add_library(
    testing_framework::helpers ALIAS testing_framework_helpers
)

target_link_libraries(
    testing_framework_helpers INTERFACE
    render_lib
    testing_framework::core
)
