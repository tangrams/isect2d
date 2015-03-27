cmake_minimum_required(VERSION 2.8)
project(sec2d)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++0x -stdlib=libc++ -g -O0")

add_executable(sec2d.out main.cpp src/sec2d.h)
