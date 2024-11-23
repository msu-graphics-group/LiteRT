
add_library(
    testing_framework_core

    # utils and helpers
    exe.h
    exe.cpp
    colors.h
    colors.cpp
    logging.h
    logging.cpp

    # command line interface
    cmdline_parser.h
    cli.h
    cli.cpp
    commands.h
    commands.cpp

    # test execution
    test.h
    test.cpp
    execution.h
    execution.cpp

    # supervised execution helpers
    supervisor.h
    supervisor.cpp
    run_supervised.h
    run_supervised.cpp

    # test options
    param_parsers.h
    param_parsers.cpp
    param_validators.h
    param_validators.cpp
    test_options.h
    test_options.cpp

    # inside-test api
    add_test.h
    environment.h
    environment.cpp

    # main function
    main.cpp

)

add_library(testing_framework::core ALIAS testing_framework_core)
