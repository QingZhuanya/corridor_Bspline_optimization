cmake_minimum_required(VERSION 3.10)
project(opt_planner LANGUAGES CXX C)

message(STATUS "main:")
message(STATUS "opt_planner")
message(STATUS "Author: Long qinying")

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
set(CMAKE_BUILD_TYPE "RelWithDebInfo")
# set( CMAKE_BUILD_TYPE "Debug" )
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(ADDITIONAL_CXX_FLAG "-Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CXX_FLAG}")

find_package(Python3 COMPONENTS Development NumPy)

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

file(GLOB_RECURSE osrcs CONFIGURE_DEPENDS src/*.cpp )
add_executable(opt_planner_node ${osrcs})

target_include_directories(opt_planner_node PUBLIC include ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})
# target_include_directories(opt_planner_node PRIVATE ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})
target_link_libraries(opt_planner_node PATH_SEARCHER_LIB BSPLINE_OPT_LIB COMMON_LIB Python3::Python Python3::NumPy)

