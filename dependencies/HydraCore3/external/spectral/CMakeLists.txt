cmake_minimum_required(VERSION 3.18)

project(Spectral CXX)

add_library(spectral_proj_compile_options INTERFACE)
target_compile_features(spectral_proj_compile_options INTERFACE cxx_std_17)
target_compile_options(spectral_proj_compile_options INTERFACE -Wno-unknown-pragmas)

find_package(OpenMP)
if(OpenMP_FOUND)
    target_compile_definitions(spectral_proj_compile_options INTERFACE SPECTRAL_ENABLE_OPENMP)
    target_link_libraries(spectral_proj_compile_options INTERFACE OpenMP::OpenMP_CXX)
endif()

option(SPECTRAL_LIB_ONLY "Only compile library")
option(SPECTRAL_NO_PRECOMPUTERS "Disable precompute modules")
option(SPECTRAL_DISABLE_PROGRESS_BAR "Disable progress printing")

set(SPECTRAL_PROJECT_ROOT ${PROJECT_SOURCE_DIR})
set(SPECTRAL_LIB_SRC_DIR ${SPECTRAL_PROJECT_ROOT}/lib)
set(SPECTRAL_APPS_SRC_DIR ${SPECTRAL_PROJECT_ROOT}/src)

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})

include(cmake/includes.cmake)

add_subdirectory(3rd_party)

add_subdirectory(${SPECTRAL_LIB_SRC_DIR})
if(NOT SPECTRAL_LIB_ONLY)
    add_subdirectory(${SPECTRAL_APPS_SRC_DIR})
endif()
if(SPECTRAL_DISABLE_PROGRESS_BAR)
    target_compile_definitions(spectral_proj_compile_options INTERFACE SPECTRAL_DISABLE_PROGRESS_BAR)
endif()
