cmake_minimum_required(VERSION 3.10.2)

project(nlsoft)

add_subdirectory(src)
add_subdirectory(include)

file (GLOB SOURCES src/*.cpp)
file (GLOB HEADERS include/*)

add_library(nlsoftlib ${SOURCES} ${HEADERS})

foreach( testsourcefile get_filename_component(NAME ${SOURCES} ABSOLUTE )
    string( REPLACE ".cpp" "" testname ${testsourcefile} )
    MESSAGE(${testname})
    add_executable(${testname} ${testsourcefile} )
    target_link_libraries( ${testname} nlsoft ))
endforeach( testsourcefile ${SOURCES} )

add_executable(${PROJECT_NAME} test/arduino_test_tilting.cpp)


