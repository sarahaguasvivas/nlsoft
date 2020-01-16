# CMakeLists files in this project can
# refer to the root source directory of the project as ${HELLO_SOURCE_DIR} and
# to the root binary directory of the project as ${HELLO_BINARY_DIR}.
cmake_minimum_required (VERSION 2.8.11)

project (block_controller)

OPTION (GPC_USE_VRPN "Using vrpn library" OFF)

# Recurse into the "Hello" and "Demo" subdirectories. This does not actually
# cause another cmake executable to run. The same process will walk through
# the project's entire directory structure.
add_subdirectory (lib)

include_directories (src)
include_directories (data)
include_directories (include)

file (GLOB CORE_LIB src/*.cpp src/controller/*.cpp src/dynamixels/*.cpp src/optitrack/*.cpp src/signals/*.cpp)

#add_library (gpc_core SHARED ${CORE_LIB})

# Add executable called "helloDemo" that is built from the source files
# "demo.cxx" and "demo_b.cxx". The extensions are automatically found.
add_executable (controller_test test/test.cpp)

# Link the executable to the Hello library. Since the Hello library has
# public include directories we will use those link directories when building
# helloDemo
#target_link_libraries (controller_test gpc_core)
target_link_libraries (controller_test vrpn)
