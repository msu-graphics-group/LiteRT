
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
    param_parsers.cpp

    param_validators.h
    param_validators.cpp

    cmdline_parser.h

    test_options.h
    test_options.cpp

)

add_library(testing_framework::core ALIAS testing_framework_core)
