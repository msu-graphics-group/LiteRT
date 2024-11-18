
add_library(
    testing_framework_logging

    log.h
    log.cpp

    color.h
    color.cpp

    colored_ostream.h

)

target_include_directories(testing_framework_logging PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../../)

target_compile_features(testing_framework_logging PUBLIC cxx_std_20)

add_library(testing_framework::logging ALIAS testing_framework_logging)

