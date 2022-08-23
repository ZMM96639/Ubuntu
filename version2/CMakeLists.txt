cmake_minimum_required(VERSION 2.8)

project(poseErr)

set(CMAKE_CXX_STANDARO 14)
set(CMAKE_CXX_STANDARO_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Debug")

set(BUILD_PLUGIN_TILEDB ON)


find_package(PDAL REQUIRED CONFIG)
include_directories(${PDAL_INCLUDE_DIRS})
link_directories(${PDAL_LIBRARY_DIRS})
add_definitions(${PDAL_DEFINITIONS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
link_directories(${EIGEN3_LIBRARY_DIRS})
add_definitions(${EIGEN3_DEFINITION})

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PPROJECT_SOURCE_DIR}/lib)
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PPROJECT_SOURCE_DIR}/bin)


include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/third_party/include)

add_library(pdal_plugin_filter_pose1topose2 SHARED src/Pose1Topose2.cpp third_party/src/pose.cpp)
target_link_libraries(pdal_plugin_filter_pose1topose2 ${PDAL_LIBRARIES})

add_library(pdal_plugin_filter_pcdprojecttoimages SHARED src/PcdprojectToImages.cpp third_party/src/pose.cpp)
target_link_libraries(pdal_plugin_filter_pcdprojecttoimages ${PDAL_LIBRARIES} ${OpenCV_LIBS})

add_library(pdal_plugin_filter_drawpointpaironimage SHARED src/DrawpointpairOnImage.cpp third_party/src/pose.cpp)
target_link_libraries(pdal_plugin_filter_drawpointpaironimage  ${OpenCV_LIBS} ${PDAL_LIBRARIES})




