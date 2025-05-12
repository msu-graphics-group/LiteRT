enable_language(CXX)
enable_language(C)

add_library(spectral_apps_compile_options INTERFACE)
target_link_libraries(spectral_apps_compile_options INTERFACE spectral_proj_compile_options)
target_compile_options(spectral_apps_compile_options INTERFACE -m64)

#Because eigen3 doesn't want to work without it
if(UNIX)
    include_directories(/usr/include/eigen3)
endif()

if(NOT SPECTRAL_NO_PRECOMPUTERS)
    find_package(Ceres REQUIRED)
endif()

if(PROJECT_IS_TOP_LEVEL)
    target_compile_options(spectral_apps_compile_options INTERFACE -Wall -Wextra -Werror -Wcast-qual)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(spectral_apps_compile_options INTERFACE -O0 -g)
else()
    target_compile_options(spectral_apps_compile_options INTERFACE -O1)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    target_compile_options(spectral_apps_compile_options INTERFACE -g)
endif()


include(properties.cmake)

