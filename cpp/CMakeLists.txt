cmake_minimum_required (VERSION 2.8.11)
project (NLSOFT)

add_library (test test.cpp)
add_library (nn_util utilities/neural_network_utilities.cpp)
add_library (matrix utilities/matrix.cpp)
add_library (nn4mc sys_id/nn4mc.cpp)
add_library (dense sys_id/dense.cpp)
add_library (activation sys_id/activations.cpp)

SET (CMAKE_CXX_FLAGS "-g")
SET (CMAKE_CXX_STANDARD 11)
SET (CMAKE_CXX_STANDARD_REQUIRED ON)
SET (CMAKE_CXX_EXTENSIONS OFF)

INCLUDE_DIRECTORIES (utilities sys_id)

file (GLOB CORE_LIB *.cpp ../sys_id/*.cpp ../utilities/*.cpp)

target_include_directories (test PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} sys_id utilities)

add_executable (test_transpose test.cpp sys_id/dense.cpp sys_id/activations.cpp sys_id/nn4mc.cpp) 

target_link_libraries (test_transpose PUBLIC matrix test nn_util activation dense nn4mc)
