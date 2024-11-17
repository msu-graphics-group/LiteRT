
add_library(
    testing_framework_core

    main.cpp

    actions.h
    actions.cpp

    colors.h

    help.h
    help.cpp
    exe.h
    exe.cpp

    test.h
    test.cpp

    supervisor.h
    supervisor.cpp

    env.h
    env.cpp

)

target_include_directories(testing_framework_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../../)

target_compile_features(testing_framework_core PUBLIC cxx_std_17)

add_library(testing_framework::core ALIAS testing_framework_core)

