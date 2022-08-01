cmake_minimum_required(VERSION 3.0)
project(solvePnP)

find_package(OpenCV 4.5.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

find_package(Sophus REQUIRED)
include_directories(${SOPHUS_INCLUDE_DIRS})
link_directories(${SOPHUS_LIBRARY_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
link_libraries(${EIGEN3_LIBRARY_DIRS})

include_directories(include  Thirdparty/g2o)
aux_source_directory(src SRC)

add_library(${PROJECT_NAME} ${SRC})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS}
                                      ${Sophus_LIBRARIES}
                                      ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
                                      fmt)


# add_executable(cameraCalib Examples/monocular.cpp)
# target_link_libraries(cameraCalib ${OpenCV_LIBS})

add_executable(pose_estimation Examples/pose_estimation.cpp)
target_link_libraries(pose_estimation ${PROJECT_NAME})

# add_executable(pose_estimation_3d2d Examples/pose_estimation_3d2d.cpp)
# target_link_libraries(pose_estimation_3d2d ${PROJECT_NAME})