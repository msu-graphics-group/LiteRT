cmake_minimum_required(VERSION 3.8)

project("NURBSViewer" CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

set(embree_DIR external/embree4/lib/cmake/embree-4.3.3)
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
target_link_libraries(ImGui glfw SDL2main SDL2 embree)
target_include_directories(ImGui PUBLIC external/imgui external/imgui/backends /usr/include/SDL2)

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
find_package(OpenMP)
target_link_libraries(
  ${PROJECT_NAME}
    LiteMath
    ImGui
    OpenMP::OpenMP_CXX
    stp_parser
)

enable_language(ISPC)
set(CMAKE_ISPC_HEADER_DIRECTORY ../src/)
add_library(ispc_ray_pack src/ispc_ray_pack.ispc)
target_include_directories(ispc_ray_pack PUBLIC external/embree4/include)
