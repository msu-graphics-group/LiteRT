cmake_minimum_required (VERSION 3.15)
project (HydraCore3)

option(USE_VULKAN "Enable Vulkan implementation" OFF)
option(CAM_PLUGIN "Enable Cam Plugin Demo" OFF)
option(USE_ENZYME "Enable Enzyme AD" OFF)

option(USE_STB_IMAGE "Enable STB image " ON)
if(USE_STB_IMAGE)
  add_definitions(-DUSE_STB_IMAGE)
endif()

if(USE_ENZYME)
  set(CLANG_VERSION "17" CACHE STRING "clang and LLVM version to use")
  set(CMAKE_C_COMPILER   "/usr/bin/clang-${CLANG_VERSION}")   # $ export CC=/usr/bin/clang-17
  set(CMAKE_CXX_COMPILER "/usr/bin/clang++-${CLANG_VERSION}") # $ export CXX=/usr/bin/clang++-17
  set(ENZYME_PLUGIN_DLL  "/home/frol/PROG/enzyme_103/build/Enzyme/ClangEnzyme-${CLANG_VERSION}.so")
  include_directories(${CMAKE_CURRENT_SOURCE_DIR})
  add_definitions(-DUSE_ENZYME)
endif()

# get rid of annoying MSVC warnings.
add_definitions(-D_CRT_SECURE_NO_WARNINGS)


if(HYDRA_CORE3_LIB)
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
else()
  add_library(project_options INTERFACE)
  target_compile_features(project_options INTERFACE cxx_std_17)

  # compiler warnings
  add_library(project_warnings INTERFACE)

  include(cmake/CompilerWarnings.cmake)
  set_project_warnings(project_warnings)

  # sanitizer options
  include(cmake/Sanitizers.cmake)
  enable_sanitizers(project_options)
endif()

add_subdirectory(ies_parser)

set(GENERATOR_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} CACHE PATH "Root output directory for all artifacts")
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin-debug)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin-debug)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin-debug)
  add_definitions(-D_DEBUG)
else()
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin-release)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin-release)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin-release)
endif()

find_package(OpenMP)

#LIBSPECTRAL
include(libspectral.cmake)
#//////////

if(USE_VULKAN)

  if(CMAKE_SYSTEM_NAME STREQUAL Windows)
    set(VOLK_STATIC_DEFINES VK_USE_PLATFORM_WIN32_KHR)
  elseif(CMAKE_SYSTEM_NAME STREQUAL Linux)
    set(VOLK_STATIC_DEFINES VK_USE_PLATFORM_XLIB_KHR)
  elseif(CMAKE_SYSTEM_NAME STREQUAL Darwin)
    set(VOLK_STATIC_DEFINES VK_USE_PLATFORM_MACOS_MVK)
  endif()

  add_subdirectory(external/volk
                   external/volk/build)

  add_compile_definitions(USE_VOLK)
  add_compile_definitions(USE_VULKAN)
  
  include_directories(${Vulkan_INCLUDE_DIR})
endif()

# try to silence warnings for include files from these dirs
include_directories(SYSTEM ${Vulkan_INCLUDE_DIR})
include_directories(SYSTEM "external/tinyexr"
                    "external/volk"
                    "external/LiteMath"
                    "external/LiteMath/external")
include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                    "external"
                    "external/vkutils"
                    "external/CrossRT")

link_directories("external/CrossRT/lib")

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -Wno-attributes")
  message(STATUS "Debug is ON!")
  add_definitions(-D_DEBUG)
  #uncomment this to detect broken memory problems via gcc sanitizers
  #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fsanitize-address-use-after-scope -fno-omit-frame-pointer -fsanitize=leak -fsanitize=undefined -fsanitize=bounds-strict")
else()
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-attributes -Wno-unused-parameter -Wno-useless-cast")
  message(STATUS "Debug is OFF!")
endif()


set(VULKAN_EXTERNAL_SRC
        external/vkutils/vk_utils.cpp
        external/vkutils/vk_copy.cpp
        external/vkutils/vk_buffers.cpp
        external/vkutils/vk_images.cpp
        external/vkutils/vk_context.cpp
        external/vkutils/vk_alloc_simple.cpp
        external/vkutils/vk_pipeline.cpp
        external/vkutils/vk_descriptor_sets.cpp
        external/vkutils/ray_tracing/vk_rt_utils.cpp
        external/vkutils/geom/vk_mesh.cpp
        external/CrossRT/VulkanRTX.cpp
        external/LiteScene/scene_mgr.cpp
        external/vkutils/geom/cmesh.cpp)

set(VULKAN_SRC
        integrator_pt_generated.cpp
        integrator_pt_generated_init.cpp
        integrator_pt_generated_ds.cpp )

set(VULKAN_CAM1
        cam_plugin/CamPinHole_pinhole_gpu.cpp
        cam_plugin/CamPinHole_pinhole_gpu_init.cpp
        cam_plugin/CamPinHole_pinhole_gpu_ds.cpp )

set(VULKAN_CAM2
        cam_plugin/CamTableLens_tablelens_gpu.cpp
        cam_plugin/CamTableLens_tablelens_gpu_init.cpp
        cam_plugin/CamTableLens_tablelens_gpu_ds.cpp )

add_library(miniz external/tinyexr/miniz.c)

set(LOADER_EXTERNAL_SRC
        external/LiteScene/pugixml.cpp
        external/LiteScene/hydraxml.cpp
        external/LiteScene/cmesh4.cpp)

set(OTHER_EXTERNAL_SRC
        external/LiteMath/Image2d.cpp
        external/CrossRT/EmbreeRT.cpp)

if(USE_VULKAN)
  set (CAM_MAIN cam_plugin/main_with_cam_gpu.cpp)   
else()
  set (CAM_MAIN cam_plugin/main_with_cam.cpp)
endif()

if(CAM_PLUGIN)
  set(MAIN_FILES ${CAM_MAIN}
                cam_plugin/CamPinHole.cpp
                cam_plugin/CamTableLens.cpp)
elseif(USE_ENZYME)
  set(MAIN_FILES diff_render/drmain.cpp
                 diff_render/integrator_dr.cpp
                 diff_render/integrator_dr2.cpp)
else()
  set(MAIN_FILES main.cpp)
endif()

set(CORE_SRC
        ${MAIN_FILES}
        integrator_rt.cpp
        integrator_pt.cpp
        integrator_pt_lgt.cpp
        integrator_pt_mat.cpp
        integrator_pt_host.cpp
        integrator_pt_scene.cpp
        integrator_pt_scene_mat.cpp
        integrator_pt_scene_lgt.cpp
        integrator_pt_scene_tex.cpp
        integrator_gbuffer.cpp
        integrator_spectrum.cpp
        mlt/integrator_qmc.cpp
        mlt/integrator_kmlt.cpp
        mlt/rnd_qmc.cpp
        mi_materials.cpp
        spectrum.cpp
        imageutils.cpp
        imageutils_gbuffer.cpp
        utils.cpp
        ies_parser/ies_render.cpp)

if(HYDRA_CORE3_LIB)
  if(USE_VULKAN)    
    if(CAM_PLUGIN)
        add_library(hydra ${CORE_SRC}
                          ${VULKAN_EXTERNAL_SRC}
                          ${VULKAN_SRC}
                          ${VULKAN_CAM1}
                          ${VULKAN_CAM2}
                          ${LOADER_EXTERNAL_SRC}
                          ${OTHER_EXTERNAL_SRC})
    else()
        add_library(hydra ${CORE_SRC}
                          ${VULKAN_EXTERNAL_SRC}
                          ${VULKAN_SRC}
                          ${LOADER_EXTERNAL_SRC}
                          ${OTHER_EXTERNAL_SRC})
    endif()
  else()
      add_library(hydra ${CORE_SRC}
                        ${LOADER_EXTERNAL_SRC}
                        ${OTHER_EXTERNAL_SRC})
  endif()
else()
  if(USE_VULKAN)    
    if(CAM_PLUGIN)
      add_executable(hydra ${CORE_SRC}
                          ${VULKAN_EXTERNAL_SRC}
                          ${VULKAN_SRC}
                          ${VULKAN_CAM1}
                          ${VULKAN_CAM2}
                          ${LOADER_EXTERNAL_SRC}
                          ${OTHER_EXTERNAL_SRC})
    else()
      add_executable(hydra ${CORE_SRC}
                          ${VULKAN_EXTERNAL_SRC}
                          ${VULKAN_SRC}
                          ${LOADER_EXTERNAL_SRC}
                          ${OTHER_EXTERNAL_SRC})
    endif()
  else()
    add_executable(hydra ${CORE_SRC}
                        ${LOADER_EXTERNAL_SRC}
                        ${OTHER_EXTERNAL_SRC})
  endif()
endif()

set_target_properties(hydra PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")

if(CMAKE_SYSTEM_NAME STREQUAL Windows)
  link_directories("${CMAKE_SOURCE_DIR}/external/CrossRT/lib_win64")
  set(EMBREE_LIBS ${CMAKE_CURRENT_SOURCE_DIR}/external/CrossRT/lib_win64/embree3.lib)
elseif(CMAKE_SYSTEM_NAME STREQUAL Linux)
  find_package(Threads REQUIRED)
  set(EMBREE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/CrossRT/lib)
  link_directories(${EMBREE_DIR})
  set(EMBREE_LIBS ${EMBREE_DIR}/libembree3.so 
                  ${EMBREE_DIR}/libtbb.so)
endif()


if(USE_ENZYME)
  set(ENZYME_FLAGS "-fplugin=${ENZYME_PLUGIN_DLL}") # -fplugin=${ENZYME_PLUGIN_DLL}
  set(OMP_FLAGS "-fopenmp=libomp ") # -fplugin=${ENZYME_PLUGIN_DLL}
  target_compile_options(hydra PUBLIC ${OMP_FLAGS} ${ENZYME_FLAGS})
else()
  set(CUSTOM_OMP "")
endif()

target_link_libraries(hydra LINK_PUBLIC project_options project_warnings miniz ies_parser spectral)

if(USE_VULKAN)
  target_link_libraries(hydra LINK_PUBLIC volk ${EMBREE_LIBS})
else()
  target_link_libraries(hydra LINK_PUBLIC ${EMBREE_LIBS})
endif ()

if(CMAKE_SYSTEM_NAME STREQUAL Windows)
  # add windows specific options here
  add_custom_command(TARGET hydra POST_BUILD COMMAND ${CMAKE_COMMAND}
          -E copy_directory "${PROJECT_SOURCE_DIR}/external/CrossRT/bin_win64" $<TARGET_FILE_DIR:hydra>)
else()
  target_link_libraries(hydra LINK_PUBLIC dl)
endif()

if (CMAKE_BUILD_TYPE STREQUAL "Debug")   # add debug specific options here
  target_link_libraries(hydra LINK_PUBLIC Threads::Threads OpenMP::OpenMP_CXX)
else()
  target_link_libraries(hydra LINK_PUBLIC OpenMP::OpenMP_CXX)
endif()