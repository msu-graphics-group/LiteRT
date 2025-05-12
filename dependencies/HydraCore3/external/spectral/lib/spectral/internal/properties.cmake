set(MODULE_NAME spectral-internal)
set(MODULE_PATH ${SRC}/spectral/internal)

add_subdirectory(common)
add_subdirectory(math)
add_subdirectory(serialization)

#set(MODULE_SOURCES

#)

set(MODULE_LIBS
    common math serialization
)