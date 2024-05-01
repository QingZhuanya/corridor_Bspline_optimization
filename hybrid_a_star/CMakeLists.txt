cmake_minimum_required(VERSION 3.10)
project(hybrid_a_star LANGUAGES CXX C)

message(STATUS "Hybrid A star")

#set(CMAKE_VERBOSE_MAKEFILE "true")

# Compiler
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if (COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif (COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_FLAGS} doesn't have C++11 support.
                         Please use a different C++ compiler")
endif ()

set( CMAKE_BUILD_TYPE "Debug" )
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(ADDITIONAL_CXX_FLAG "-Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CXX_FLAG}")

# Eigen
#include(cmake/FindEigen.cmake)
find_package(Eigen3 CONFIG REQUIRED) #{CMAKE_MODULE_PATH} (/usr/share/cmake/Modules) eigen3 only have include in /usr/include/eigen3
#include_directories(${EIGEN_INCLUDE_DIR})

# glog
# include(cmake/glog.cmake)
find_package(glog CONFIG REQUIRED)
#include_directories(${GLOG_INCLUDE_DIRS})

# absl
# add_subdirectory(abseil-cpp)
# find_package(abseil-cpp CONFIG REQUIRED)

# catkin
find_package(
    catkin REQUIRED COMPONENTS
    nav_msgs
    roscpp
    tf
)

include_directories(
    # include
    # ${catkin_INCLUDE_DIRS}
)

catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES path_searcher
    #  CATKIN_DEPENDS nav_mags roscpp tf
    #  DEPENDS system_lib
)
file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h)
# add_library(
#     PATH_SEARCHER_LIB SHARED
#     src/rs_path.cpp
#     src/costmap_subscriber.cpp
#     src/goal_pose_subscriber.cpp
#     src/init_pose_subscriber.cpp
#     src/hybrid_a_star.cpp
#     src/hybrid_a_star_flow.cpp
# )
add_library(PATH_SEARCHER_LIB SHARED ${srcs})
target_link_libraries(
    PATH_SEARCHER_LIB
    ${catkin_LIBRARIES}
    #${GLOG_LIBRARIES}
    glog::glog
    COMMON_LIB
)
target_include_directories(PATH_SEARCHER_LIB PUBLIC include ${catkin_INCLUDE_DIRS}  ${EIGEN3_INCLUDE_DIR})

# add_executable(run_hybrid_astar app/run_hybrid_astar.cpp)
# target_link_libraries(run_hybrid_astar PATH_SEARCHER_LIB)
# target_include_directories(run_hybrid_astar PUBLIC include)