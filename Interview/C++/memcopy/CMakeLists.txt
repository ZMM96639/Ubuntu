cmake_minimum_required(VERSION 3.0)

project(memcopy)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

include_directories(include)

aux_source_directory(src SRC)
add_library(${PROJECT_NAME} ${SRC})

add_executable(test Examples/main.cpp)
target_link_libraries(test ${PROJECT_NAME})