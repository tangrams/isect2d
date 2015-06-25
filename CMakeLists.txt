cmake_minimum_required(VERSION 2.8)
project(isect2d)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++0x -stdlib=libc++ -g -O0")

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

list(APPEND GLFW_LDFLAGS
    "-framework OpenGL"
    "-framework Cocoa"
    "-framework IOKit"
    "-framework CoreFoundation"
    "-framework CoreVideo")

add_subdirectory(${PROJECT_SOURCE_DIR}/tests/box2d/)
include_directories(${PROJECT_SOURCE_DIR}/tests/box2d/)
include_directories(${PROJECT_SOURCE_DIR}/tests/)
include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(${EXECUTABLE_NAME} tests/b2test.cpp include/isect2d.h)

target_link_libraries(${EXECUTABLE_NAME} Box2D ${GLFW_LDFLAGS})

