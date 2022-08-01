cmake_minimum_required(VERSION 3.0)

project(CapPixel)

include_directories(include)

find_package(OpenCV 4.5.0 REQUIRED)

aux_source_directory(src SRC)

add_library(${PROJECT_NAME} ${SRC})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

add_executable(test Examples/test.cpp)
target_link_libraries(test ${PROJECT_NAME})
