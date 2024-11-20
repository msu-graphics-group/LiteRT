
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

    execution.h
    execution.cpp

    commands.h
    commands.cpp

    cli.h
    cli.cpp

    param_parsers.h

    param_validators.h

    cmdline_parser.h

)

add_library(testing_framework::core ALIAS testing_framework_core)
