cmake_minimum_required(VERSION 2.8.12)
project(panther)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++14)
# add_definitions(-std=c++11)
#add_definitions(-std=c99)

find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR})

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_finders")

find_package(CGAL REQUIRED)
include(${CGAL_USE_FILE})

find_package ( Boost COMPONENTS graph REQUIRED )
include_directories ( ${Boost_INCLUDE_DIR} )

find_package(CASADI REQUIRED)
include_directories( ${CASADI_INCLUDE_DIR} )

#From terminal: catkin config -DCMAKE_BUILD_TYPE=Release (or RelWithDebInfo or Debug)
set(CMAKE_BUILD_TYPE Release) 

find_package(catkin REQUIRED COMPONENTS 
	roscpp
	rospy
	snapstack_msgs
	panther_msgs
	sensor_msgs
	decomp_util
	decomp_ros_utils
	rviz_visual_tools
	separator
    pcl_ros
    hungarian
)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})

find_package(decomp_util REQUIRED)
include_directories(${DECOMP_UTIL_INCLUDE_DIRS})

# find_package(hungarian REQUIRED)
# include_directories(${HUNGARIAN_INCLUDE_DIRS}) //TODO

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} include)

add_executable(${PROJECT_NAME}_node src/panther_node.cpp src/panther_ros.cpp src/panther.cpp src/solver_ipopt.cpp src/solver_ipopt_utils.cpp src/utils.cpp src/solver_ipopt_guess.cpp src/yaw_guess_generator.cpp src/octopus_search.cpp src/bspline_utils.cpp src/cgal_utils.cpp)
target_include_directories (${PROJECT_NAME}_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${DECOMP_UTIL_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME}_node PUBLIC ${CASADI_LIBRARIES} ${catkin_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR} ${DECOMP_UTIL_LIBRARIES} ${Boost_LIBRARIES})  #${CGAL_LIBS}
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS} )

add_executable(test_octopus_search src/examples/test_octopus_search.cpp src/octopus_search.cpp src/bspline_utils.cpp src/utils.cpp src/cgal_utils.cpp) 
add_dependencies(test_octopus_search ${catkin_EXPORTED_TARGETS} )
target_link_libraries(test_octopus_search ${catkin_LIBRARIES}) 

add_executable(test_utils src/examples/test_utils.cpp src/utils.cpp)
add_dependencies(test_utils ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_utils ${catkin_LIBRARIES})

add_executable(test_bspline_utils src/examples/test_bspline_utils.cpp src/bspline_utils.cpp)
add_dependencies(test_bspline_utils ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_bspline_utils ${catkin_LIBRARIES})

add_executable(tracker_predictor_node src/tracker_predictor_node.cpp src/tracker_predictor.cpp src/utils.cpp)
add_dependencies(tracker_predictor_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(tracker_predictor_node ${catkin_LIBRARIES} ${CASADI_LIBRARIES})

add_executable(test_tracker_predictor src/examples/test_tracker_predictor.cpp src/tracker_predictor.cpp src/utils.cpp)
add_dependencies(test_tracker_predictor ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_tracker_predictor ${catkin_LIBRARIES} ${CASADI_LIBRARIES})

add_library(move_model src/move_model.cpp)
add_library(disable_physics src/disable_physics.cpp)

target_link_libraries(disable_physics ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(move_model ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

#############################################################################


#https://stackoverflow.com/questions/41361631/optimize-in-cmake-by-default
#set(CMAKE_CXX_FLAGS "-O1")

# See http://manpages.ubuntu.com/manpages/bionic/man1/ccache.1.html#precompiled%20headers
# Note also that we have added in the ~/.bashrc the line export CCACHE_SLOPPINESS=pch_defines,time_macros  
# add_compile_options(-fpch-preprocess)


# set (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_cotire")
# include(cotire)

#For precompile headers see example here
#https://gitlab.kitware.com/cmake/cmake/tree/375d01c6808713a0cfeef9ea092c8236ba063525/Tests/PrecompileHeaders
# target_precompile_headers(exprtk PUBLIC exprtk.hpp) #ONLY WORKS FOR CMAKE >=3.16


# add_library(cgal_utils STATIC src/cgal_utils)
# target_include_directories (cgal_utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
# target_link_libraries(cgal_utils PUBLIC ${DECOMP_UTIL_LIBRARIES} ${catkin_LIBRARIES})

# add_library(utils STATIC src/utils.cpp)
# target_include_directories (utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
# target_link_libraries(utils PUBLIC ${catkin_LIBRARIES})

# if (CMAKE_VERSION VERSION_LESS 3.16)
#   message(WARNING "Not using_precompiled headers")
#   message( ${CMAKE_VERSION} )
# else()
#   message("Using_precompiled headers")
# #   target_precompile_headers(cgal_utils PUBLIC include/cgal_utils.hpp)
#   target_precompile_headers(utils PUBLIC include/utils.hpp)
# endif()

# add_library(octopus_search STATIC src/octopus_search.cpp src/bspline_utils.cpp) #src/solvers/cvxgen/solver_cvxgen.cpp
# target_include_directories (octopus_search PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
# target_link_libraries(octopus_search PUBLIC ${catkin_LIBRARIES}) 

# if (CMAKE_VERSION VERSION_LESS 3.16)
#   message(WARNING "Not using_precompiled headers")
#   message( ${CMAKE_VERSION} )
# else()
#   message("Using_precompiled headers")
#   target_precompile_headers(utils PUBLIC exprtk.hpp)
# endif()



# add_library(solver_ipopt STATIC src/solver_ipopt.cpp src/solver_ipopt_utils.cpp src/solver_ipopt_guess.cpp src/yaw_guess_generator.cpp src/utils.cpp src/octopus_search.cpp src/bspline_utils.cpp)
# target_include_directories (solver_ipopt PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${DECOMP_UTIL_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
# target_link_libraries(solver_ipopt PUBLIC ${CASADI_LIBRARIES} ${DECOMP_UTIL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} cgal_utils)
# cotire(solver_ipopt)

# enable_unity_build(UNITY_BUILD_NAME src/panther_node.cpp)

# if (CMAKE_VERSION VERSION_LESS 3.16)
#   message(WARNING "Not using_precompiled headers")
# else()
#   message("Using_precompiled headers")
#   target_precompile_headers(${PROJECT_NAME}_node PUBLIC include/exprtk.hpp  PUBLIC include/utils.hpp PUBLIC include/cgal_utils.hpp include/utils.hpp include/termcolor.hpp include/panther.hpp include/panther_types.hpp include/solver_ipopt.hpp)
# #   target_precompile_headers(${PROJECT_NAME}_node REUSE_FROM cgal_utils)
# #   target_precompile_headers(${PROJECT_NAME}_node REUSE_FROM utils)
# endif()
#if("$ENV{ROS_DISTRO}" STREQUAL "melodic")
##	
#elseif("$ENV{ROS_DISTRO}" STREQUAL "kinetic")
##
#else()
#	message(WARNING "Using the disable_physics kinetic version")
##
#endif() 

# add_executable(solver_ipopt src/solver_ipopt.cpp)
# target_link_libraries(solver_ipopt ${CASADI_LIBRARIES})


# add_executable(predictor_node src/predictor_node.cpp src/predictor.cpp src/bspline_utils.cpp)
# add_dependencies(predictor_node ${catkin_EXPORTED_TARGETS})
# target_link_libraries(predictor_node ${catkin_LIBRARIES} utils)


# add_executable(test_predictor_node src/test_predictor.cpp src/predictor.cpp src/bspline_utils.cpp)
# add_dependencies(test_predictor_node ${catkin_EXPORTED_TARGETS})
# target_link_libraries(test_predictor_node ${catkin_LIBRARIES} utils)


# add_executable(test_cvxgen src/solvers/cvxgen/test_cvxgen.cpp src/solvers/cvxgen/solver_cvxgen.cpp)
# add_dependencies(test_cvxgen ${catkin_EXPORTED_TARGETS})
# target_link_libraries(test_cvxgen ${catkin_LIBRARIES} ${catkin_LIBRARIES} ${CMAKE_CURRENT_LIST_DIR}/src/solvers/cvxgen/cvxgen/libcvx.so)
