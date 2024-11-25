cmake_minimum_required(VERSION 3.13)

project("NURBSViewer" CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

if (APPLE)
  set(embree_DIR /usr/local/Cellar/embree/4.3.3/lib/cmake/embree-4.3.3)
  set(EMBREE_INCLUDE_DIR /usr/local/Cellar/embree/4.3.3/include)
  set(SDL_DIR /usr/local/Cellar/sdl2/2.30.9/lib)
  set(SDL_INCLUDE_DIR /usr/local/include/SDL2)
  set(GLFW_DIR /usr/local/Cellar/glfw/3.4/lib)
  set(ImGUI_INCLUDE_DIRS ${SDL_INCLUDE_DIR} /usr/local/include/)
  set(OpenMP_CXX_FLAGS
        "-Xpreprocessor -fopenmp -I/usr/local/opt/libomp/include")
  set(OpenMP_CXX_LIB_NAMES "omp")
  set(OpenMP_omp_LIBRARY "/usr/local/opt/libomp/lib/libomp.dylib")
else()
  set(embree_DIR external/embree4/linux/lib/cmake/embree-4.3.3)
  set(EMBREE_INCLUDE_DIR external/embree4/linux/include)
  set(SDL_INCLUDE_DIR /usr/include/SDL2)
  set(ImGUI_INCLUDE_DIRS ${SDL_INCLUDE_DIR})
endif()

find_package(OpenMP)
find_package(embree 4.3.3)
add_subdirectory(external/step)

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
target_link_directories(ImGui PUBLIC ${SDL_DIR} ${GLFW_DIR})
target_link_libraries(ImGui glfw SDL2main SDL2)
target_include_directories(ImGui PUBLIC external/imgui external/imgui/backends ${ImGUI_INCLUDE_DIRS})

enable_language(ISPC)
add_library(ispc_ray_pack src/ispc_ray_pack.ispc)
target_include_directories(ispc_ray_pack PUBLIC ${EMBREE_INCLUDE_DIR})

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
    src/curve.hpp
    src/curve.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC ${SDL_INCLUDE_DIR})

target_link_libraries(
  ${PROJECT_NAME}
    LiteMath
    ImGui
    OpenMP::OpenMP_CXX
    ispc_ray_pack
    embree
    step
)

add_executable(
  RBCurve2D_test
  tests/RBCurve2D.cpp
    src/utils.hpp
    src/utils.cpp
    src/curve.hpp
    src/curve.cpp
)
target_link_libraries(RBCurve2D_test LiteMath)
target_include_directories(RBCurve2D_test PUBLIC ${PROJECT_SOURCE_DIR}/src/)
