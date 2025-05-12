include(properties.cmake)


add_executable(${MODULE_NAME}
    ${MODULE_SOURCES}
)

target_compile_options(${MODULE_NAME} PRIVATE -Wno-error=unused-variable)

target_link_libraries(${MODULE_NAME}
    ${MODULE_LIBS} spectral_apps_compile_options
)
