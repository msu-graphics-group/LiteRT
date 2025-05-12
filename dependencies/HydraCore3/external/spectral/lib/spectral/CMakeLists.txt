include(properties.cmake)

add_library(${MODULE_NAME} SHARED)

#target_sources(${MODULE_NAME} PRIVATE 
#    ${MODULE_SOURCES}
#)

target_link_libraries(${MODULE_NAME} PUBLIC
    ${MODULE_LIBS} spectral_lib_compile_options
)
