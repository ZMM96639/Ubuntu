cmake_minimum_required(VERSION 3.0)
project(HighGUI)
include_directories(include)

add_subdirectory(lib)

add_executable(HighGUI main.cpp)

target_link_libraries(HighGUI lib)

