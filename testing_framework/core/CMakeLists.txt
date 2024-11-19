
add_library(
    testing_framework_core

    main.cpp

    actions.h
    actions.cpp

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

    colors.h
    colors.cpp

)

add_library(testing_framework::core ALIAS testing_framework_core)

