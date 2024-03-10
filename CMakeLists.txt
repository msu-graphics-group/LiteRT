cmake_minimum_required(VERSION 3.15)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

project(loader_app CXX)

add_library(project_warnings INTERFACE)
include(cmake/CompilerWarnings.cmake)
set_project_warnings(project_warnings)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/LiteMath)
include_directories(${CMAKE_SOURCE_DIR}/LiteScene)
include_directories(${CMAKE_SOURCE_DIR}/embree)

link_directories(${CMAKE_SOURCE_DIR}/embree/lib)
set(RAYTRACING_EMBREE_LIBS embree3 embree_sse42 embree_avx embree_avx2 lexers simd sys tasking) # embree_avx embree_avx2

find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(SCENE_LOADER_SRC
    ${CMAKE_SOURCE_DIR}/LiteScene/pugixml.cpp
    ${CMAKE_SOURCE_DIR}/LiteScene/hydraxml.cpp
    ${CMAKE_SOURCE_DIR}/LiteScene/cmesh4.cpp
    ${CMAKE_SOURCE_DIR}/sdfScene/sdf_scene.cpp
    )

set(CORE_SRC
    ${CMAKE_SOURCE_DIR}/LiteMath/Image2d.cpp
    ${CMAKE_SOURCE_DIR}/Timer.cpp
    ${CMAKE_SOURCE_DIR}/BruteForceRT.cpp
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

message(STATUS "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  message(STATUS "Debug is ON!")
  add_definitions(-D_DEBUG)
  #uncomment this to detect broken memory problems via gcc sanitizers
  #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fsanitize-address-use-after-scope -fno-omit-frame-pointer -fsanitize=leak -fsanitize=undefined -fsanitize=bounds-strict")
else()
  add_definitions(-DNDEBUG)
endif()


add_executable(render_app)

#add_subdirectory(${CMAKE_SOURCE_DIR}/volk ${CMAKE_SOURCE_DIR}/volk)

target_sources(render_app PRIVATE main.cpp
               ${SCENE_LOADER_SRC}
               ${CORE_SRC}
               ${BUILDERS_SRC})

set_property(TARGET render_app PROPERTY CXX_STANDARD 17)


if(CMAKE_SYSTEM_NAME STREQUAL Windows)
    set_target_properties(render_app PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")

    target_link_libraries(render_app PRIVATE project_warnings ${RAYTRACING_EMBREE_LIBS})

    add_custom_command(TARGET render_app POST_BUILD COMMAND ${CMAKE_COMMAND}
            -E copy_directory "${PROJECT_SOURCE_DIR}/external/embree/bin_win64" $<TARGET_FILE_DIR:render_app>)
elseif(ANDROID)
    find_library(log-lib log)
    find_library(android-lib android)
    target_link_libraries(render_app PRIVATE project_warnings Threads::Threads dl embree3 ${android-lib} ${log-lib} volk )
else()
    target_link_libraries(render_app PRIVATE project_warnings Threads::Threads dl ${RAYTRACING_EMBREE_LIBS})
endif()

if(OpenMP_CXX_FOUND)
  target_link_libraries(render_app PUBLIC OpenMP::OpenMP_CXX) ##
endif()

