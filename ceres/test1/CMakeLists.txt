cmake_minimum_required(VERSION 3.0)

project(test)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
link_directories(${EIGEN3_LIBRARIE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${Ceres_INCLUDE_DIRS})
link_directories(${Ceres_LIBRARY_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include)

# aux_source_directory(src SRC)

# add_library(${PROJECT_NAME} ${SRC})
# target_link_libraries(${PROJECT_NAME} ${Ceres_LIBS})

add_executable(main Examples/main.cpp)
target_link_libraries(main ${CERES_LIBRARIES})

add_executable(main2 Examples/main2.cpp)
target_link_libraries(main2 ${CERES_LIBRARIES})

add_executable(main3 Examples/main3.cpp)
target_link_libraries(main3 ${CERES_LIBRARIES})
