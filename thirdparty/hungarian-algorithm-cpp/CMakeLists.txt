cmake_minimum_required(VERSION 2.8.3)
project(hungarian)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations")
find_package(catkin_simple REQUIRED)
# find_package(cmake_modules)
# find_package(Eigen3 REQUIRED)
# find_package(decomp_util REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS} include)

catkin_simple()

set(SOURCE_FILES
  src/Hungarian.cpp)

cs_add_library(hungarian ${SOURCE_FILES})
target_link_libraries(hungarian  ${catkin_LIBRARIES} )

cs_add_executable(test_hungarian src/test_hungarian.cpp)
target_link_libraries(test_hungarian hungarian)

cs_install()

cs_export()
