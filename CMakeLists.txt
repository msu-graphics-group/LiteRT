cmake_minimum_required(VERSION 3.8)

project("NURBSViewer" CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

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
target_link_libraries(ImGui glfw SDL2main SDL2)
target_include_directories(ImGui PUBLIC external/imgui external/imgui/backends)

add_executable(
  ${PROJECT_NAME} 
    src/main.cpp
    src/Surface.hpp
    src/Surface.cpp
    src/raytracer.hpp
    src/raytracer.cpp
    src/utils.hpp
    src/utils.cpp
)
target_link_libraries(
  ${PROJECT_NAME}
    LiteMath
    ImGui
)
