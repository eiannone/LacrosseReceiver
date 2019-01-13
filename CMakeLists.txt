cmake_minimum_required(VERSION 3.10)  # CMake version check
project(LacrosseReceiver)             # Create project "LacrosseReceiver"
set(CMAKE_CXX_STANDARD 11)            # Enable c++11 standard

add_definitions(-DDEBUG=1)
include_directories(lib/Timings2Measure)
set(SOURCE_FILES test/debug_Timings2Measure.cpp lib/Timings2Measure/Timings2Measure.cpp)
add_executable(LacrosseReceiver ${SOURCE_FILES})  # Add executable target with source files listed in SOURCE_FILES variable

configure_file(test/desktop/test_Timings2Measure.dat ./ COPYONLY)
