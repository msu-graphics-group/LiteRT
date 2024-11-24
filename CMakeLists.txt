cmake_minimum_required(VERSION 3.13)

project("NURBSViewer" CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

if (APPLE)
    set(embree_DIR external/embree4/osx/lib/cmake/embree-4.3.3)
else()
    set(embree_DIR external/embree4/linux/lib/cmake/embree-4.3.3)
endif()
find_package(embree 4.3.3)

add_library(LiteMath external/LiteMath/Image2d.cpp)
target_include_directories(LiteMath PUBLIC external/LiteMath)

add_library(
  ImGui 
    external/imgui/imgui_demo.cpp
    external/imgui/imgui_draw.cpp
    external/imgui/imgui_tables.cpp
    external/imgui/imgui.cpp
    external/imgui/imgui_widgets.cpp
    external/imgui/backends/imgui_impl_sdlrenderer2.cpp 
    external/imgui/backends/imgui_impl_sdl2.cpp
)

target_include_directories(ImGui PUBLIC external/imgui external/imgui/backends)
if (APPLE)
    target_link_directories(ImGui PUBLIC
        /usr/local/Cellar/sdl2/2.30.9/lib
        /usr/local/Cellar/glfw/3.4/lib)
    target_include_directories(ImGui PUBLIC /usr/local/include/ /usr/local/include/SDL2)
else()
    target_include_directories(ImGui PUBLIC /usr/include/SDL2)
endif()
target_link_libraries(ImGui glfw SDL2main SDL2 embree)


add_executable(
  ${PROJECT_NAME} 
    src/main.cpp
    src/Surface.hpp
    src/Surface.cpp
    src/raytracer.hpp
    src/raytracer.cpp
    src/utils.hpp
    src/utils.cpp
    src/embree_adaptors.hpp
    src/embree_adaptors.cpp
    src/gui.hpp 
    src/gui.cpp
)

add_subdirectory(external/stp_parser)

if(APPLE)
    set(OpenMP_CXX_FLAGS
        "-Xpreprocessor -fopenmp -I/usr/local/opt/libomp/include")
    set(OpenMP_CXX_LIB_NAMES "omp")
    set(OpenMP_omp_LIBRARY "/usr/local/opt/libomp/lib/libomp.dylib")
endif()
find_package(OpenMP)

enable_language(ISPC)
add_library(ispc_ray_pack src/ispc_ray_pack.ispc)

if (APPLE)
    target_include_directories(ispc_ray_pack PUBLIC external/embree4/osx/include)
else()
    target_include_directories(ispc_ray_pack PUBLIC external/embree4/linux/include)
endif()

option(USE_ZENITY "Use zenity to browse step files" ON)
if (${USE_ZENITY})
    add_definitions(-DUSE_ZENITY)
endif()

target_link_libraries(
  ${PROJECT_NAME}
    LiteMath
    ImGui
    OpenMP::OpenMP_CXX
    stp_parser
    ispc_ray_pack
)
