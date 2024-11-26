
set(CMAKE_CXX_STANDARD 20)

add_subdirectory(core)
add_subdirectory(helpers)

add_library(
    testing_framework
    INTERFACE
)

target_link_libraries(
    testing_framework INTERFACE
    testing_framework::core
    testing_framework::helpers
)

function(testing_discover_tests target)
    set(test_include_name ${target}_include.cmake)
    set(test_include ${CMAKE_CURRENT_BINARY_DIR}/${test_include_name})
    add_custom_command(
        TARGET ${target} POST_BUILD
        BYPRODUCTS ${test_include}
        COMMAND ${target} generate-ctest-files ${test_include} ${CMAKE_CURRENT_SOURCE_DIR}/..
    )
    set_property(DIRECTORY APPEND PROPERTY TEST_INCLUDE_FILES ${test_include})
    set_property(TARGET ${target} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..)
endfunction()
