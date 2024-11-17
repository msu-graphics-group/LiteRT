

add_executable(
    test

    main.cpp

    colors.h

    help.h
    help.cpp

    test.h
    test.cpp

    supervisor.h
    supervisor.cpp
)

target_link_libraries(test PRIVATE render_lib)