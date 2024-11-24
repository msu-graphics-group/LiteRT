cmake_minimum_required(VERSION 3.15)

option(USE_ENZYME "Enable Enzyme" OFF)
if(USE_ENZYME)
  message("Enzyme is ON")
  add_definitions(-DUSE_ENZYME)
  set(CLANG_VERSION "17" CACHE STRING "clang and LLVM version to use")
  set(CMAKE_CXX_COMPILER "/usr/bin/clang++-${CLANG_VERSION}")
  set(ENZYME_LIB_PATH "${CMAKE_SOURCE_DIR}/dependencies/enzyme/ClangEnzyme-${CLANG_VERSION}.so")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fplugin=${ENZYME_LIB_PATH}")
else()
  set(CMAKE_CXX_COMPILER "/usr/bin/g++")
endif()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

project(loader_app CXX)

add_compile_definitions(LITERT_RENDERER)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/dependencies)
include_directories(${CMAKE_SOURCE_DIR}/dependencies/HydraCore3/external)
include_directories(${CMAKE_SOURCE_DIR}/dependencies/HydraCore3/external/LiteMath)
include_directories(${CMAKE_SOURCE_DIR}/dependencies/HydraCore3/external/LiteMath/external)
include_directories(${CMAKE_SOURCE_DIR}/dependencies/HydraCore3/external/LiteScene)
include_directories(${CMAKE_SOURCE_DIR}/dependencies/HydraCore3/external/CrossRT)
include_directories(${CMAKE_SOURCE_DIR}/dependencies/embree)
include_directories(${CMAKE_SOURCE_DIR}/sdfScene)
include_directories(${CMAKE_SOURCE_DIR}/BVH)
include_directories(${CMAKE_SOURCE_DIR}/Render)
include_directories(${CMAKE_SOURCE_DIR}/openvdb_structs)

#include_directories(SYSTEM ${CMAKE_SOURCE_DIR}/dependencies/HydraCore3/external/spectral/include)
#set(SPECTRAL_LIB_ONLY ON)
#add_subdirectory(${CMAKE_SOURCE_DIR}/dependencies/HydraCore3/external/spectral)

set(HYDRA_CORE3_LIB ON)

add_library(project_options INTERFACE)
target_compile_features(project_options INTERFACE cxx_std_17)

# compiler warnings
add_library(project_warnings INTERFACE)

include(${CMAKE_SOURCE_DIR}/dependencies/HydraCore3/cmake/CompilerWarnings.cmake)
#set_project_warnings(project_warnings)

# sanitizer options
include(${CMAKE_SOURCE_DIR}/dependencies/HydraCore3/cmake/Sanitizers.cmake)
enable_sanitizers(project_options)

include_directories(${CMAKE_SOURCE_DIR}/dependencies/HydraCore3)
include_directories(${CMAKE_SOURCE_DIR}/dependencies/HydraCore3/external)
include_directories(${CMAKE_SOURCE_DIR}/dependencies/HydraCore3/external/spectral)

add_subdirectory(${CMAKE_SOURCE_DIR}/dependencies/HydraCore3)
link_directories(${CMAKE_SOURCE_DIR}/dependencies/HydraCore3)

link_directories(${CMAKE_SOURCE_DIR}/dependencies/embree/lib)
set(RAYTRACING_EMBREE_LIBS embree4 tbb tbbmalloc)

find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -std=c++17 -Wall -Wno-unused-function \
                     -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unused-parameter -Wno-unused-result \
                     -Wno-int-to-pointer-cast -Wno-register -Wreturn-type -Wdisabled-optimization \
                     -Wno-narrowing -Wno-attributes -Wno-sign-compare")

set(SCENE_LOADER_SRC
    ${CMAKE_SOURCE_DIR}/dependencies/HydraCore3/external/LiteScene/pugixml.cpp
    ${CMAKE_SOURCE_DIR}/dependencies/HydraCore3/external/LiteScene/hydraxml.cpp
    ${CMAKE_SOURCE_DIR}/dependencies/HydraCore3/external/LiteScene/cmesh4.cpp
    ${CMAKE_SOURCE_DIR}/sdfScene/sdf_scene.cpp
    ${CMAKE_SOURCE_DIR}/utils/mesh_bvh.cpp
    ${CMAKE_SOURCE_DIR}/utils/mesh.cpp
    ${CMAKE_SOURCE_DIR}/utils/sparse_octree_builder.cpp
    ${CMAKE_SOURCE_DIR}/utils/radiance_field.cpp
    ${CMAKE_SOURCE_DIR}/utils/gaussian_field.cpp
    ${CMAKE_SOURCE_DIR}/utils/watertight_mesh.cpp
    ${CMAKE_SOURCE_DIR}/utils/sdf_converter.cpp
    ${CMAKE_SOURCE_DIR}/utils/hydra_stub.cpp
    ${CMAKE_SOURCE_DIR}/utils/marching_cubes.cpp
    ${CMAKE_SOURCE_DIR}/utils/sdf_smoother.cpp
    ${CMAKE_SOURCE_DIR}/utils/demo_meshes.cpp
    ${CMAKE_SOURCE_DIR}/utils/image_metrics.cpp
    ${CMAKE_SOURCE_DIR}/utils/iou.cpp
    ${CMAKE_SOURCE_DIR}/utils/points_visualizer.cpp
    ${CMAKE_SOURCE_DIR}/utils/compact_octree_builder.cpp
    ${CMAKE_SOURCE_DIR}/utils/similarity_compression.cpp
    ${CMAKE_SOURCE_DIR}/openvdb_structs/openvdb_common.cpp
    )

set(CORE_SRC
    ${CMAKE_SOURCE_DIR}/dependencies/HydraCore3/external/LiteMath/Image2d.cpp
    ${CMAKE_SOURCE_DIR}/Timer.cpp
    ${CMAKE_SOURCE_DIR}/Renderer/eye_ray.cpp
    ${CMAKE_SOURCE_DIR}/Renderer/eye_ray_host.cpp
    )

set(BUILDERS_SRC
    ${CMAKE_SOURCE_DIR}/BVH/BVH2Common.cpp
    ${CMAKE_SOURCE_DIR}/BVH/BVH2Common_host.cpp
    ${CMAKE_SOURCE_DIR}/BVH/cbvh.cpp
    ${CMAKE_SOURCE_DIR}/BVH/cbvh_fat.cpp
    ${CMAKE_SOURCE_DIR}/BVH/cbvh_embree2.cpp
    )

set(TESTS_SRC
    ${CMAKE_SOURCE_DIR}/tests/tests.cpp
    ${CMAKE_SOURCE_DIR}/tests/benchmark.cpp)

set(DR_SRC 
    ${CMAKE_SOURCE_DIR}/diff_render/DR_common.cpp
    ${CMAKE_SOURCE_DIR}/diff_render/BVH2DR.cpp
    ${CMAKE_SOURCE_DIR}/diff_render/MultiRendererDR.cpp
    ${CMAKE_SOURCE_DIR}/diff_render/benchmark_diff_render.cpp
    ${CMAKE_SOURCE_DIR}/diff_render/tests_diff_render.cpp)

set(HYDRA_SRC
    ${CMAKE_SOURCE_DIR}/tests/hydra_integration.cpp)

message(STATUS "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  message(STATUS "Debug is ON!")
  add_definitions(-D_DEBUG)
  #uncomment this to detect broken memory problems via gcc sanitizers
  #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fsanitize-address-use-after-scope -fno-omit-frame-pointer -fsanitize=leak -fsanitize=undefined -fsanitize=bounds-strict")
else()
  add_definitions(-DNDEBUG)
endif()

add_compile_definitions(ON_CPU=1)

option(USE_VULKAN "include GPU support" ON)
option(USE_RTX    "include RTX support" OFF)
option(USE_GPU_RQ "include RTX support using Ray Query in comnpute shader" OFF)

option(USE_STB_IMAGE "Enable STB image " ON)
if(USE_STB_IMAGE)
  add_definitions(-DUSE_STB_IMAGE)
endif()

option(DEBUG_PAYLOAD_STORE_SDF "Sdd a vector to store sdrf values along the ray to Payload struct" OFF)
if(DEBUG_PAYLOAD_STORE_SDF)
  add_definitions(-DDEBUG_PAYLOAD_STORE_SDF)
endif()

set(MODULES_SRC "")
set(MODULES_INCLUDE_DIRS "modules")
set(MODULES_LIBRARIES "")
set(MODULES_COMPILE_FLAGS "")

set(OpenMP_CXX "${CMAKE_CXX_COMPILER}")
set(OpenMP_CXX_FLAGS "-fopenmp=libomp")
set(OpenMP_CXX_LIB_NAMES "libomp")
set(OpenMP_libomp_LIBRARY ${OpenMP_CXX_LIB_NAMES})

if (USE_VULKAN)
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
  set(SLICER_DIR "/home/sammael/kernel_slicer" CACHE STRING "Custom diff render location")
  
  set(MODULES_INCLUDE_DIRS ${MODULES_INCLUDE_DIRS} ${Vulkan_INCLUDE_DIR}
                                                   ${CMAKE_SOURCE_DIR}/dependencies/HydraCore3/external/vkutils
                                                   ${CMAKE_SOURCE_DIR}/dependencies/HydraCore3/external/volk)

  #link_directories(${CMAKE_SOURCE_DIR}/dependencies/HydraCore3/external/volk)

  #add_subdirectory(${CMAKE_SOURCE_DIR}/dependencies/HydraCore3/external/volk
  #                 ${CMAKE_SOURCE_DIR}/dependencies/HydraCore3/external/volk/build)

  add_compile_definitions(USE_GPU)
  add_compile_definitions(USE_VOLK)

  set(VKGEN_SOURCE 
      ${CMAKE_SOURCE_DIR}/Renderer/eye_ray_gpu.cpp 
      ${CMAKE_SOURCE_DIR}/Renderer/eye_ray_gpu_ds.cpp 
      ${CMAKE_SOURCE_DIR}/Renderer/eye_ray_gpu_init.cpp)

  set(VKUTILS_SOURCE ${CMAKE_SOURCE_DIR}/dependencies/HydraCore3/external/vkutils/vk_utils.cpp
                     ${CMAKE_SOURCE_DIR}/dependencies/HydraCore3/external/vkutils/vk_copy.cpp
                     ${CMAKE_SOURCE_DIR}/dependencies/HydraCore3/external/vkutils/vk_buffers.cpp
                     ${CMAKE_SOURCE_DIR}/dependencies/HydraCore3/external/vkutils/vk_images.cpp
                     ${CMAKE_SOURCE_DIR}/dependencies/HydraCore3/external/vkutils/vk_context.cpp
                     ${CMAKE_SOURCE_DIR}/dependencies/HydraCore3/external/vkutils/vk_alloc_simple.cpp
                     ${CMAKE_SOURCE_DIR}/dependencies/HydraCore3/external/vkutils/vk_pipeline.cpp
                     ${CMAKE_SOURCE_DIR}/dependencies/HydraCore3/external/vkutils/vk_descriptor_sets.cpp
                     ${CMAKE_SOURCE_DIR}/dependencies/HydraCore3/external/vkutils/vk_swapchain.cpp
                     ${CMAKE_SOURCE_DIR}/dependencies/HydraCore3/external/vkutils/vk_quad.cpp
                     ${CMAKE_SOURCE_DIR}/dependencies/HydraCore3/external/vkutils/vk_fbuf_attachment.cpp
                     ${CMAKE_SOURCE_DIR}/dependencies/HydraCore3/external/vkutils/ray_tracing/vk_rt_utils.cpp
                     
                     ${CMAKE_SOURCE_DIR}/dependencies/HydraCore3/external/CrossRT/EmbreeRT.cpp
                     ${CMAKE_SOURCE_DIR}/dependencies/HydraCore3/external/CrossRT/VulkanRTX.cpp
                     ${CMAKE_SOURCE_DIR}/dependencies/HydraCore3/external/LiteScene/scene_mgr.cpp         
                     ${CMAKE_SOURCE_DIR}/dependencies/HydraCore3/external/vkutils/geom/cmesh.cpp
                     ${CMAKE_SOURCE_DIR}/dependencies/HydraCore3/external/vkutils/geom/vk_mesh.cpp
                     )    

  set(VKGEN_SOURCE_RTX "")
  set(VKGEN_SOURCE_GPU_RQ "")
  if (USE_RTX)
    add_compile_definitions(USE_RTX)
    set(VKGEN_SOURCE_RTX
        ${CMAKE_SOURCE_DIR}/Renderer/eye_ray_rtx.cpp 
        ${CMAKE_SOURCE_DIR}/Renderer/eye_ray_rtx_ds.cpp 
        ${CMAKE_SOURCE_DIR}/Renderer/eye_ray_rtx_init.cpp)
  endif()

  if (USE_GPU_RQ)
    add_compile_definitions(USE_GPU_RQ)
    set(VKGEN_SOURCE_GPU_RQ
        ${CMAKE_SOURCE_DIR}/Renderer/eye_ray_gpu_rq.cpp 
        ${CMAKE_SOURCE_DIR}/Renderer/eye_ray_gpu_rq_ds.cpp 
        ${CMAKE_SOURCE_DIR}/Renderer/eye_ray_gpu_rq_init.cpp) 
  endif()

  set(MODULES_SRC ${MODULES_SRC} ${VKGEN_SOURCE} ${VKUTILS_SOURCE} ${VKGEN_SOURCE_RTX} ${VKGEN_SOURCE_GPU_RQ})

  set(MODULES_LIBRARIES ${MODULES_LIBRARIES} ${Vulkan_LIBRARY} OpenMP::OpenMP_CXX volk dl)
endif()

set(
  NURBS_SRC
    nurbs/nurbs_common_host.cpp
)

set(
  CATMUL_CLARK_SRC
    catmul_clark/catmul_clark_host.cpp
)

set(
  RIBBON_SRC
    ribbon/ribbon_host.cpp
)

add_subdirectory(step)

add_executable(render_app)
target_include_directories(render_app PUBLIC ${MODULES_INCLUDE_DIRS})
target_link_directories(render_app PRIVATE step)

target_sources(render_app PRIVATE main.cpp
               ${MODULES_SRC}
               ${SCENE_LOADER_SRC}
               ${CORE_SRC}
               ${BUILDERS_SRC}
               ${NURBS_SRC}
               ${TESTS_SRC}
               ${DR_SRC}
               ${HYDRA_SRC}
               ${CATMUL_CLARK_SRC}
               ${RIBBON_SRC})

set_property(TARGET render_app PROPERTY CXX_STANDARD 17)
target_link_libraries(render_app PRIVATE Threads::Threads dl ${RAYTRACING_EMBREE_LIBS} ${MODULES_LIBRARIES})
target_link_libraries(render_app PUBLIC hydra OpenMP::OpenMP_CXX)

# Turn on/off OpenVDB
option(USE_OPENVDB "Enable OpenVDB" OFF)
if(USE_OPENVDB)
  message("-- OpenVDB is ON")
  target_link_libraries(render_app PUBLIC tbb openvdb)
else()
  message("-- OpenVDB is OFF")
  add_definitions(-DDISABLE_OPENVDB)
endif()

if (USE_VULKAN)
  set(IMGUI_SRC
          ${CMAKE_SOURCE_DIR}/dependencies/imgui/imgui.cpp
          ${CMAKE_SOURCE_DIR}/dependencies/imgui/imgui_draw.cpp
          ${CMAKE_SOURCE_DIR}/dependencies/imgui/imgui_tables.cpp
          ${CMAKE_SOURCE_DIR}/dependencies/imgui/imgui_widgets.cpp
          ${CMAKE_SOURCE_DIR}/dependencies/imgui/imgui_demo.cpp
          ${CMAKE_SOURCE_DIR}/dependencies/imgui/backends/imgui_impl_vulkan.cpp
          ${CMAKE_SOURCE_DIR}/dependencies/imgui/backends/imgui_impl_glfw.cpp
          ${CMAKE_SOURCE_DIR}/dependencies/imgui/misc/cpp/imgui_stdlib.cpp)

  set(ENGINE_SOURCE 
      ${CMAKE_SOURCE_DIR}/render_engine/glfw_window.cpp
      ${CMAKE_SOURCE_DIR}/render_engine/gltf_utils.cpp
      ${CMAKE_SOURCE_DIR}/render_engine/image_loader.cpp
      ${CMAKE_SOURCE_DIR}/render_engine/main.cpp
      ${CMAKE_SOURCE_DIR}/render_engine/render_imgui.cpp
      ${CMAKE_SOURCE_DIR}/render_engine/scn_manager_load.cpp
      ${CMAKE_SOURCE_DIR}/render_engine/simple_render_rt.cpp
      ${CMAKE_SOURCE_DIR}/render_engine/simple_render.cpp
  )   

  add_compile_definitions(IMGUI_USER_CONFIG="${CMAKE_SOURCE_DIR}/render_engine/my_imgui_config.h")

  add_executable(engine)
  target_include_directories(engine PUBLIC ${MODULES_INCLUDE_DIRS})
  target_include_directories(engine PUBLIC ${CMAKE_SOURCE_DIR}/dependencies/imgui)
  target_link_directories(engine PRIVATE step)
  find_package(glfw3 REQUIRED)

  target_sources(engine PRIVATE                
                ${MODULES_SRC}
                ${SCENE_LOADER_SRC}
                ${CORE_SRC}
                ${BUILDERS_SRC} 
                ${ENGINE_SOURCE} 
                ${IMGUI_SRC}
                ${NURBS_SRC}
                ${CATMUL_CLARK_SRC}
                ${RIBBON_SRC})

  set_property(TARGET engine PROPERTY CXX_STANDARD 17)
  target_link_libraries(engine PRIVATE Threads::Threads dl glfw ${RAYTRACING_EMBREE_LIBS} ${MODULES_LIBRARIES})
  target_link_libraries(engine PUBLIC OpenMP::OpenMP_CXX)
endif()
