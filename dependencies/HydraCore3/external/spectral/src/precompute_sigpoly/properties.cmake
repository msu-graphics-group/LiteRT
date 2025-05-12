set(MODULE_NAME precompute_sigpoly)

set(MODULE_SOURCES
    main.cpp
    functions.cpp
    lutworks.cpp
)

set(MODULE_LIBS
    spectral ceres
)