set(MODULE_NAME imageutil)
set(MODULE_PATH ${SRC}/spectral/imageutil)

set(MODULE_SOURCES
    pixel.cpp
    image.cpp
)

set(MODULE_LIBS
    stb spectral-internal
)