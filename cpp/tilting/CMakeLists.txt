cmake_minimum_required(VERSION 3.10.2)

project(nlsoft)
set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_FLAGS "-g")

add_subdirectory(src)
add_subdirectory(include)
include_directories(include)

file (GLOB SOURCES src/activations.cpp src/nn4mc.cpp src/helpers.cpp
                        src/gru.cpp src/swirl_target.cpp src/dense.cpp
                        src/matrix.cpp src/helpers.cpp)
file (GLOB HEADERS  include/activations.h
                                include/nn4mc.h include/helpers.hpp include/gru.h include/swirl_target.hpp
                                        include/dense.hpp include/matrix.hpp include/helpers.hpp)

add_library(nlsoftlib SHARED ${SOURCES} ${HEADERS})

add_executable(${PROJECT_NAME} test/arduino_test_tilting.cpp
                src/activations.cpp src/nn4mc.cpp src/gru.cpp src/helpers.cpp
                src/swirl_target.cpp src/dense.cpp src/matrix.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC include)

