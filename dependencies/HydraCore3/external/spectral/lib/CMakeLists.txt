enable_language(CXX)
enable_language(C)

add_library(spectral_lib_compile_options INTERFACE)
target_link_libraries(spectral_lib_compile_options INTERFACE spectral_proj_compile_options)
target_compile_options(spectral_lib_compile_options INTERFACE -m64 -fPIC)

if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(spectral_lib_compile_options INTERFACE -Wno-inconsistent-missing-override)
endif()

if(PROJECT_IS_TOP_LEVEL)
    target_compile_options(spectral_lib_compile_options INTERFACE -Wall -Wextra -Werror -Wcast-qual)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(spectral_lib_compile_options INTERFACE -O0 -g)
else()
    target_compile_options(spectral_lib_compile_options INTERFACE -O1)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    target_compile_options(spectral_lib_compile_options INTERFACE -g)
endif()


include(properties.cmake)
