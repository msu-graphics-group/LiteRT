set(MODULE_NAME spectral)
set(MODULE_PATH ${SRC}/spectral)

add_subdirectory(imageutil)
add_subdirectory(internal)
add_subdirectory(spec)
add_subdirectory(upsample)
#set(MODULE_SOURCES

#)

set(MODULE_LIBS
    spectral-internal imageutil spec upsample
)