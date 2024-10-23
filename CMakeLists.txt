cmake_minimum_required(VERSION 3.8)
project(stp_parser)
set(CMAKE_CXX_STANDARD 17)

add_library(
  ${PROJECT_NAME} # library name
    ${CMAKE_CURRENT_SOURCE_DIR}/src/stp_parser.cpp #sources...
    ${CMAKE_CURRENT_SOURCE_DIR}/include/stp_parser.hpp #just for vscode intellisence
)
#passing -Iinclude/ to compiler
#PUBLIC will be added to all dependent targets
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/) 
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/external/) 

option(compile_examples ON)
if (${compile_examples})
  add_executable(
    parse_and_print
      examples/example_print.cpp
  )
  target_link_libraries(parse_and_print ${PROJECT_NAME})
  target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/external/)

  add_executable(
    parse_and_div
      examples/example_div_individual.cpp
  )
  target_link_libraries(parse_and_div ${PROJECT_NAME})
  target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/external/)
endif()

