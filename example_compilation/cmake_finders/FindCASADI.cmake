#This file should print something like 
#-- Found CasADi include dir: /usr/local/include;/usr/local/include/casadi
#-- Found CasADi libs: /usr/local/lib/libcasadi.so

#These folders where the pre-built binaries of Casadi are located, and I don't wanna compile against them 
set(CMAKE_IGNORE_PATH ${CMAKE_IGNORE_PATH};/home/jtorde/installations/matlab_related/casadi/include)
set(CMAKE_IGNORE_PATH ${CMAKE_IGNORE_PATH};/home/jtorde/installations/matlab_related/casadi/libcasadi.so)
set(CMAKE_IGNORE_PATH ${CMAKE_IGNORE_PATH};/home/jtorde/installations/matlab_related/casadi)

find_path(CASADI_INCLUDE_DIR
  casadi/casadi.hpp
  HINTS $ENV{CASADI_PREFIX}/include
)

if(CASADI_INCLUDE_DIR)
  set(CASADI_INCLUDE_DIR ${CASADI_INCLUDE_DIR} ${CASADI_INCLUDE_DIR}/casadi)
  set(CASADI_FOUND_INCLUDE TRUE)
  message(STATUS "Found CasADi include dir: ${CASADI_INCLUDE_DIR}")
else()
  message(STATUS "Could not find CasADi include dir")
endif()

find_library(CASADI_LIBRARY
    NAMES casadi
    HINTS ${CASADI_INCLUDE_DIR}/.. $ENV{CASADI_PREFIX})
if(CASADI_LIBRARY)
    set(CASADI_LIBRARIES ${CASADI_LIBRARIES} ${CASADI_LIBRARY})
endif()

if(CASADI_LIBRARIES)
  message(STATUS "Found CasADi libs: ${CASADI_LIBRARIES}")
else()
  message(STATUS "Could not find CasADi libs")
endif()

if(CASADI_FOUND_INCLUDE AND CASADI_LIBRARIES)
  set(CASADI_FOUND TRUE)
endif()
