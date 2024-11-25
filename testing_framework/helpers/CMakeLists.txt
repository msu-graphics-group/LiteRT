
add_library(
    testing_framework_helpers
    
    # inits helpers
    init.h
    init.cpp

    # accessing test options
    options.h
    options.cpp

    # rendering helpers
    render.h
    render.cpp
    
    # image helpers
    image.h
    image.cpp

    # mesh helpers
    mesh.h
    mesh.cpp

    # file helpers
    files.h
    files.cpp

    # formatters
    format.h

    # timer
    scoped_timer.h
    scoped_timer.cpp

    # checks
    check.h
    check.cpp

    source_location.h
    source_location.cpp
)


add_library(
    testing_framework::helpers ALIAS testing_framework_helpers
)

target_link_libraries(
    testing_framework_helpers PUBLIC
    render_lib
    testing_framework::core
)
