cmake_minimum_required(VERSION 3.15)
project(step)
set(CMAKE_CXX_STANDARD 17)

add_library(
  ${PROJECT_NAME} # library name
    ${PROJECT_SOURCE_DIR}/src/step.cpp #sources...
    ${PROJECT_SOURCE_DIR}/include/step.h #just for vscode intellisence
)
#passing -Iinclude/ to compiler
#PUBLIC will be added to all dependent targets
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include/) 
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/../dependencies/HydraCore3/external/LiteMath)

add_library(
  Timer
     ${PROJECT_SOURCE_DIR}/../utils/common/timer.cpp
     ${PROJECT_SOURCE_DIR}/../utils/common/timer.h 
)
target_include_directories(Timer PUBLIC ${PROJECT_SOURCE_DIR}/../)

add_library(
    stl INTERFACE
    ${PROJECT_SOURCE_DIR}/include/stl.h
)
target_include_directories(stl INTERFACE ${PROJECT_SOURCE_DIR}/include/)

add_library(
  stlmesh
     ${PROJECT_SOURCE_DIR}/src/stlmesh.cpp
     ${PROJECT_SOURCE_DIR}/include/stlmesh.h
)
target_include_directories(stlmesh PUBLIC ${PROJECT_SOURCE_DIR}/include/)
target_include_directories(stlmesh PUBLIC ${PROJECT_SOURCE_DIR}/../dependencies/HydraCore3/external/LiteMath)
target_include_directories(stlmesh PUBLIC ${PROJECT_SOURCE_DIR}/../dependencies/HydraCore3/external/LiteScene)

option(COMPILE_EXAMPLES ON)
if (${COMPILE_EXAMPLES})
  add_executable(
    nurbss_stdout
      examples/nurbss_stdout.cpp
  )
  target_link_libraries(nurbss_stdout PRIVATE ${PROJECT_NAME} Timer)

  add_executable(
    nurbss_tofile
      examples/nurbss_tofile.cpp
  )
  target_link_libraries(nurbss_tofile PRIVATE ${PROJECT_NAME} Timer)

  add_executable(
    argsplit
      examples/argsplit.cpp
  )
  target_link_libraries(argsplit PRIVATE ${PROJECT_NAME} Timer)

  add_executable(
    stlout
      examples/stl.cpp
  )
  target_link_libraries(stlout PRIVATE stl Timer)

  add_executable(
    cmeshout
      examples/cmesh.cpp
  )
  target_link_libraries(cmeshout PRIVATE stlmesh Timer)

endif()

