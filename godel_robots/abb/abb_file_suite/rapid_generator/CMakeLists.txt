cmake_minimum_required(VERSION 2.8.3)
project(rapid_generator)

add_definitions(-std=c++11)
include_directories(include)
add_library(rapid_generator src/rapid_emitter.cpp)
