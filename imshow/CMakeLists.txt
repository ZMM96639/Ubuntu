CMAKE_MINIMUM_REQUIRED(VERSION 3.0)
PROJECT(test)

find_package(OpenCV REQUIRED)

add_executable(test inshow.cpp)

target_link_directories(test ${OpenCV_LIBS})