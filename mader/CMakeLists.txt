cmake_minimum_required(VERSION 2.8.3)
project(mader)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)
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

#From terminal: catkin config -DCMAKE_BUILD_TYPE=Release (or RelWithDebInfo) or Debug
set(CMAKE_BUILD_TYPE RelWithDebInfo) 

find_package(catkin REQUIRED COMPONENTS 
	roscpp
	rospy
	snapstack_msgs
	mader_msgs
	sensor_msgs
	decomp_util
	decomp_ros_utils
	rviz_visual_tools
	separator
  jsk_rviz_plugins
  pcl_ros
  hungarian
)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})

find_package(decomp_util REQUIRED)
include_directories(${DECOMP_UTIL_INCLUDE_DIRS})

# find_package(hungarian REQUIRED)
# include_directories(${HUNGARIAN_INCLUDE_DIRS}) //TODO!!

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} include)

#For precompile headers see example here
#https://gitlab.kitware.com/cmake/cmake/tree/375d01c6808713a0cfeef9ea092c8236ba063525/Tests/PrecompileHeaders

add_library(cgal_utils STATIC src/cgal_utils)
target_include_directories (cgal_utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(cgal_utils PUBLIC ${DECOMP_UTIL_LIBRARIES} ${catkin_LIBRARIES})

add_library(utils STATIC src/utils.cpp)
target_include_directories (utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(utils PUBLIC ${catkin_LIBRARIES})

add_library(octopus_search STATIC src/octopus_search.cpp src/bspline_utils.cpp) #src/solvers/cvxgen/solver_cvxgen.cpp
target_include_directories (octopus_search PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(octopus_search PUBLIC ${catkin_LIBRARIES}) # ${CMAKE_CURRENT_LIST_DIR}/src/solvers/cvxgen/cvxgen/libcvx.so


add_library(solver_ipopt STATIC src/solver_ipopt.cpp src/bspline_utils.cpp src/solver_ipopt_utils.cpp src/solver_ipopt_guess.cpp src/yaw_guess_generator.cpp)
target_include_directories (solver_ipopt PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${DECOMP_UTIL_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
target_link_libraries(solver_ipopt PUBLIC ${CASADI_LIBRARIES} ${DECOMP_UTIL_LIBRARIES} utils octopus_search ${catkin_LIBRARIES} cgal_utils ${Boost_LIBRARIES} )

# enable_unity_build(UNITY_BUILD_NAME src/mader_node.cpp)
add_executable(${PROJECT_NAME}_node src/mader_node.cpp src/mader_ros.cpp src/mader.cpp )
target_link_libraries(${PROJECT_NAME}_node PUBLIC ${catkin_LIBRARIES}  solver_ipopt  solver_ipopt utils)  #${CGAL_LIBS}
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS} )

add_executable(test_octopus_search src/examples/test_octopus_search.cpp src/octopus_search.cpp src/bspline_utils.cpp) # src/solvers/cvxgen/solver_cvxgen.cpp
add_dependencies(test_octopus_search ${catkin_EXPORTED_TARGETS} )
target_link_libraries(test_octopus_search ${catkin_LIBRARIES} utils cgal_utils) # ${CMAKE_CURRENT_LIST_DIR}/src/solvers/cvxgen/cvxgen/libcvx.so

add_executable(test_utils src/examples/test_utils.cpp)
add_dependencies(test_utils ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_utils ${catkin_LIBRARIES} utils)



add_executable(test_bspline_utils src/examples/test_bspline_utils.cpp src/bspline_utils.cpp)
add_dependencies(test_bspline_utils ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_bspline_utils ${catkin_LIBRARIES})


add_executable(tracker_predictor_node src/tracker_predictor_node.cpp src/tracker_predictor.cpp)
add_dependencies(tracker_predictor_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(tracker_predictor_node ${catkin_LIBRARIES} ${CASADI_LIBRARIES} utils)

add_executable(test_tracker_predictor src/examples/test_tracker_predictor.cpp src/tracker_predictor.cpp)
add_dependencies(test_tracker_predictor ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_tracker_predictor ${catkin_LIBRARIES} ${CASADI_LIBRARIES} utils)

add_library(disable_physics src/disable_physics.cpp)
target_link_libraries(disable_physics ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

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