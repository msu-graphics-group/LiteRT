set(MODULE_NAME serialization)
set(MODULE_PATH ${SRC}/spectral/serialization)

set(MODULE_SOURCES
    parsers.cpp
    binary.cpp
    envi.cpp
)

set(MODULE_LIBS
    math common
)