CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(solver)

set(EIGEN_INCLUDE_DIR "/usr/local/include/eigen3")
include_directories(${EIGEN_INCLUDE_DIR})

option(EMBIND "Use emscripten with embind" OFF)
if(EMBIND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --bind")
endif()

option(PROFILE "Use emscripten profiling" OFF)
if(PROFILE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --profiling")
endif()

FIND_PACKAGE(Ceres REQUIRED)
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})

ADD_EXECUTABLE(solver src/solver.cc)
TARGET_LINK_LIBRARIES(solver ${CERES_LIBRARIES})

