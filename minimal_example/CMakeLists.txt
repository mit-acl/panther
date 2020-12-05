cmake_minimum_required(VERSION 2.8.6)

project(example_casadi_demo)



set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH};${PROJECT_SOURCE_DIR}/cmake_finders)
find_package(CASADI REQUIRED)
include_directories(src src/examples ${CASADI_INCLUDE_DIR} )
add_executable(casadi_demo casadi_demo.cpp)
target_link_libraries(casadi_demo ${CASADI_LIBRARIES})


