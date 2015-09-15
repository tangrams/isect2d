cmake_minimum_required(VERSION 2.8)
project(isect2d)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11 -g -O0")

find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)

set(EXECUTABLE_NAME isect2d.out)

if(NOT GLFW_FOUND)
    message(SEND_ERROR "GLFW not found")
    return()
else()
    include_directories(${GLFW_INCLUDE_DIRS})
    message(STATUS "Found GLFW ${GLFW_PREFIX}")
endif()

include_directories(include)

add_executable(${EXECUTABLE_NAME} tests/main.cpp include/isect2d.h)

target_link_libraries(${EXECUTABLE_NAME} ${GLFW_STATIC_LIBRARIES})

if(APPLE)
    target_link_libraries(${EXECUTABLE_NAME} "-framework OpenGL")
endif()

if(UNIX AND NOT APPLE)
	target_link_libraries(${EXECUTABLE_NAME} "-lGL")
endif()
