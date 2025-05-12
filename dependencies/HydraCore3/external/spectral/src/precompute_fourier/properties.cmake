set(MODULE_NAME precompute_fourier)

set(MODULE_SOURCES
    main.cpp
    functions.cpp
    lutworks.cpp
)

set(MODULE_LIBS
    spectral ceres
)