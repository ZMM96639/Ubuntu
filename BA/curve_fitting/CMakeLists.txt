cmake_minimum_required(VERSION 3.0)

project(curveFitting)

find_package(OpenCV 4.5.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
link_directories(${EIGEN3_LIBRARY_DIRS})

find_package(Sophus REQUIRED)
include_directories(${SOPHUS_INCLUDE_DIRS})
link_directories(${SOPHUS_LIBRARY_DIRS})

include_directories(include Thirdparty/g2o)

aux_source_directory(src SRC)

add_library(${PROJECT_NAME} ${SRC})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${SOPHUS_LIBRARIES}
                      ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so)

add_executable(curve Examples/curvefitting.cpp)
target_link_libraries(curve ${PROJECT_NAME})
