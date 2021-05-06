cmake_minimum_required (VERSION 2.8.11)
project (NLSOFT)

add_library(test cxx_arduino_test/arduino_test_cxx.cpp)
add_library(nn_util neural_network_utilities.cpp)
add_library(matrix matrix.cpp)
add_library(figure_eight figure_eight_target.cpp)
add_library(nn4mc nn4mc.cpp)
add_library(activation activations.cpp)
add_library(sign signals.cpp)
add_library(dense dense.cpp)

set (CMAKE_CXX_FLAGS "-g")
set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_CXX_EXTENSIONS OFF)

file (GLOB CORE_LIB *.cpp)

target_include_directories(test PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(test_arduino cxx_arduino_test/arduino_test_cxx.cpp dense.cpp activations.cpp matrix.cpp neural_network_utilities.cpp figure_eight_target.cpp nn4mc.cpp signals.cpp)

target_link_libraries(test_arduino LINK_PUBLIC matrix test nn_util figure_eight sign activation dense nn4mc)
