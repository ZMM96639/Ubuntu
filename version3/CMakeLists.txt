cmake_minimum_required(VERSION 3.0)
project(pdal_dilu)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Debug")

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
add_definitions(${EIGEN3_DEFINITIONS})

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${PROJECT_SOURCE_DIR}/third_party/include)


set(EULER ${PROJECT_SOURCE_DIR}/src/EulerConvert.cpp
          ${PROJECT_SOURCE_DIR}/third_party/src/pose.cpp)

add_library(pdal_plugin_filter_eulerconvert SHARED ${EULER})
target_include_directories(pdal_plugin_filter_eulerconvert PRIVATE ${PDAL_INCLUDE_DIRS})
target_link_directories(pdal_plugin_filter_eulerconvert PRIVATE ${PDAL_LIBRARIES})


set(GEO ${PROJECT_SOURCE_DIR}/src/GeoReference.cpp
        ${PROJECT_SOURCE_DIR}/src/isometry3d_interpolation_buffer
        ${PROJECT_SOURCE_DIR}/src/ogr_conversions.cpp
        ${PROJECT_SOURCE_DIR}/third_party/src/pose.cpp)

add_library(pdal_plugin_filter_georeference SHARED ${GEO})
target_include_directories(pdal_plugin_filter_georeference PRIVATE ${PDAL_INCLUDE_DIRS})
target_link_libraries(pdal_plugin_filter_georeference PRIVATE ${PDAL_LIBRARIES} gdal) 


set(POSE ${PROJECT_SOURCE_DIR}/src/Pose6DTransformation.cpp
         ${PROJECT_SOURCE_DIR}/third_party/src/pose.cpp)

add_library(pdal_plugin_filter_pose6dtransformation SHARED ${POSE})
target_include_directories(pdal_plugin_filter_pose6dtransformation PRIVATE ${PDAL_INCLUDE_DIRS})
target_link_libraries(pdal_plugin_filter_pose6dtransformation PRIVATE ${PDAL_LIBRARIES})


set(PRO ${PROJECT_SOURCE_DIR}/src/ProjectToImages.cpp
        ${PROJECT_SOURCE_DIR}/third_party/src/pose.cpp)

add_library(pdal_plugin_filter_projecttoimages SHARED ${PRO})
target_include_directories(pdal_plugin_filter_projecttoimages PRIVATE ${PDAL_INCLUDE_DIRS})
target_link_libraries(pdal_plugin_filter_projecttoimages PRIVATE ${PDAL_LIBRARIES} ${OpenCV_LIBRARIES}) 
