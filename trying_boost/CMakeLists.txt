cmake_minimum_required ( VERSION 2.8 )
project ( boost_a-star_cities )

set ( CMAKE_CXX_FLAGS "-std=c++11" )
set ( PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include )

find_package ( Boost COMPONENTS graph REQUIRED )
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories ( ${Boost_INCLUDE_DIR} )

file ( GLOB_RECURSE PROJ_SRCS src/*.cpp )

include_directories ( "${PROJECT_INCLUDE_DIR}" )

add_executable ( cities cities.cpp )
target_link_libraries ( cities ${Boost_LIBRARIES} )


add_executable ( tests_splines tests_splines.cpp )
target_link_libraries ( tests_splines Eigen3::Eigen)