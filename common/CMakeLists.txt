cmake_minimum_required(VERSION 3.10)
project(common LANGUAGES CXX C)

message(STATUS "common part")

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
find_package(Eigen3 CONFIG REQUIRED) #{CMAKE_MODULE_PATH} (/usr/share/cmake/Modules) eigen3 only have include in /usr/include/eigen3
# PCL
# find_package(PCL 1.7 REQUIRED)
find_package(osqp CONFIG REQUIRED)
find_package(glog CONFIG REQUIRED)
# catkin
find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    visualization_msgs
    tf
)


catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES path_searcher
    #  CATKIN_DEPENDS nav_mags roscpp tf
    #  DEPENDS system_lib
)
file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp src/*.cc include/*.h)

add_library(COMMON_LIB SHARED ${srcs})
target_link_libraries(
    COMMON_LIB
    ${catkin_LIBRARIES}
    glog::glog
    osqp::osqp
)
target_include_directories(COMMON_LIB PUBLIC include ${catkin_INCLUDE_DIRS}  ${EIGEN3_INCLUDE_DIR} )
