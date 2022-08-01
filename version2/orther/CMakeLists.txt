cmake_minimum_required(VERSION 2.8.12)
project(pdal_dilu)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")


#-------------------------linux platform-------------------------
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    find_package(PDAL 2.3.0  REQUIRED CONFIG)
    include_directories(${PDAL_INCLUDE_DIRS})
    link_directories(${PDAL_LIBRARY_DIRS})
    add_definitions(${PDAL_DEFINITIONS})


    add_library(pdal_plugin_filter_unixgpstimeconvert SHARED UnixGpsTimeConvert.cpp )
    target_include_directories(pdal_plugin_filter_unixgpstimeconvert PRIVATE ${PDAL_INCLUDE_DIRS})
    target_link_libraries(pdal_plugin_filter_unixgpstimeconvert PRIVATE ${PDAL_LIBRARIES})


    include_directories(./GeoReference/include ./third_party/include /usr/include/eigen3)
    add_library(pdal_plugin_filter_georeference SHARED ./GeoReference/GeoReference.cpp ./GeoReference/src/isometry3d_interpolation_buffer ./GeoReference/src/ogr_conversions.cpp ./third_party/src/pose.cpp)
    target_include_directories(pdal_plugin_filter_georeference PRIVATE ${PDAL_INCLUDE_DIRS})
    target_link_libraries(pdal_plugin_filter_georeference PRIVATE ${PDAL_LIBRARIES} gdal ) 


    add_library(pdal_plugin_filter_clustercutpursuit SHARED cutpursuit/ClusterCutPursuit.cpp)
    include_directories(cutpursuit/parallel-cut-pursuit-master/include)
    target_include_directories(pdal_plugin_filter_clustercutpursuit PRIVATE ${PDAL_INCLUDE_DIRS}  )
    target_link_libraries(pdal_plugin_filter_clustercutpursuit PRIVATE ${PDAL_LIBRARIES} libcut_pursuit.so)


    add_library(pdal_plugin_filter_localheight SHARED LocalHeight.cpp)
    target_include_directories(pdal_plugin_filter_localheight PRIVATE ${PDAL_INCLUDE_DIRS})
    target_link_libraries(pdal_plugin_filter_localheight PRIVATE ${PDAL_LIBRARIES})


    add_library(pdal_plugin_filter_mergeuniqueid SHARED MergeUniqueId.cpp )
    target_include_directories(pdal_plugin_filter_mergeuniqueid PRIVATE ${PDAL_INCLUDE_DIRS})
    target_link_libraries(pdal_plugin_filter_mergeuniqueid PRIVATE ${PDAL_LIBRARIES})



    add_library(pdal_plugin_filter_disandheighttotarget SHARED DisAndHeightToTarget.cpp )
    target_include_directories(pdal_plugin_filter_disandheighttotarget PRIVATE ${PDAL_INCLUDE_DIRS})
    target_link_libraries(pdal_plugin_filter_disandheighttotarget PRIVATE ${PDAL_LIBRARIES})



    add_library(pdal_plugin_filter_ferryfromother SHARED FerryFromOther.cpp )
    target_include_directories(pdal_plugin_filter_ferryfromother PRIVATE ${PDAL_INCLUDE_DIRS})
    target_link_libraries(pdal_plugin_filter_ferryfromother PRIVATE ${PDAL_LIBRARIES})


    add_library(pdal_plugin_filter_clustervoteclassification SHARED   ClusterVoteClassification.cpp)
    target_include_directories(pdal_plugin_filter_clustervoteclassification PRIVATE ${PDAL_INCLUDE_DIRS})
    target_link_libraries(pdal_plugin_filter_clustervoteclassification PRIVATE ${PDAL_LIBRARIES})


    include_directories(./third_party/include)
    add_library(pdal_plugin_filter_poseconvert SHARED   PoseConvert.cpp ./third_party/src/pose.cpp)
    target_include_directories(pdal_plugin_filter_poseconvert PRIVATE ${PDAL_INCLUDE_DIRS})
    target_link_libraries(pdal_plugin_filter_poseconvert PRIVATE ${PDAL_LIBRARIES})



    include_directories(./third_party/include)
    add_library(pdal_plugin_filter_eulerconvert SHARED   EulerConvert.cpp ./third_party/src/pose.cpp)
    target_include_directories(pdal_plugin_filter_eulerconvert PRIVATE ${PDAL_INCLUDE_DIRS})
    target_link_libraries(pdal_plugin_filter_eulerconvert PRIVATE ${PDAL_LIBRARIES})



    include_directories(./third_party/include)
    add_library(pdal_plugin_filter_pose6dtransformation SHARED   Pose6DTransformation.cpp ./third_party/src/pose.cpp)
    target_include_directories(pdal_plugin_filter_pose6dtransformation PRIVATE ${PDAL_INCLUDE_DIRS})
    target_link_libraries(pdal_plugin_filter_pose6dtransformation PRIVATE ${PDAL_LIBRARIES})


    include_directories(./GeoReference/include ./third_party/include /usr/include/eigen3)
    add_library(pdal_plugin_filter_lookuppose SHARED ./LookUpPose.cpp ./GeoReference/src/isometry3d_interpolation_buffer ./GeoReference/src/ogr_conversions.cpp ./third_party/src/pose.cpp)
    target_include_directories(pdal_plugin_filter_lookuppose PRIVATE ${PDAL_INCLUDE_DIRS})
    target_link_libraries(pdal_plugin_filter_lookuppose PRIVATE ${PDAL_LIBRARIES} gdal ) 





    add_library(pdal_plugin_filter_reprojectiontoother SHARED ./ReProjectionToOther.cpp )
    target_include_directories(pdal_plugin_filter_reprojectiontoother PRIVATE ${PDAL_INCLUDE_DIRS})
    target_link_libraries(pdal_plugin_filter_reprojectiontoother PRIVATE ${PDAL_LIBRARIES} ) 



    find_package(OpenCV  REQUIRED )
    include_directories(${OpenCV_INCLUDE_DIRS})
    link_directories(${OpenCV_LIBRARY_DIRS})
    add_definitions(${OpenCV_DEFINITIONS})
    include_directories( ./third_party/include /usr/include/eigen3)
    add_library(pdal_plugin_filter_colorizebyimages SHARED ./ColorizeByImages.cpp ./third_party/src/pose.cpp )
    target_include_directories(pdal_plugin_filter_colorizebyimages PRIVATE ${PDAL_INCLUDE_DIRS})
    target_link_libraries(pdal_plugin_filter_colorizebyimages PRIVATE ${PDAL_LIBRARIES} ${OpenCV_LIBRARIES} ) 


    add_library(pdal_plugin_filter_classifybyclusterfeatures SHARED   ClassifyByClusterFeatures.cpp ) 
    target_include_directories(pdal_plugin_filter_classifybyclusterfeatures PRIVATE ${PDAL_INCLUDE_DIRS})
    target_link_libraries(pdal_plugin_filter_classifybyclusterfeatures PRIVATE ${PDAL_LIBRARIES})



    find_package(OpenCV  REQUIRED )
    include_directories(${OpenCV_INCLUDE_DIRS})
    link_directories(${OpenCV_LIBRARY_DIRS})
    add_definitions(${OpenCV_DEFINITIONS})
    include_directories( ./third_party/include /usr/include/eigen3)
    add_library(pdal_plugin_filter_projecttoimages SHARED ./ProjectToImages.cpp ./third_party/src/pose.cpp )
    target_include_directories(pdal_plugin_filter_projecttoimages PRIVATE ${PDAL_INCLUDE_DIRS})
    target_link_libraries(pdal_plugin_filter_projecttoimages PRIVATE ${PDAL_LIBRARIES} ${OpenCV_LIBRARIES} ) 


#-------------------------window platform-------------------------
elseif(CMAKE_SYSTEM_NAME MATCHES "Windows")
    add_definitions(-D_USE_MATH_DEFINES)
    add_definitions(-D_WIN32_LEAN_AND_MEAN)
    add_definitions(-DPDAL_DLL_EXPORT)
    add_definitions(-DPDAL_DLL_EXPORT=1)


    set (BUILD_PLUGIN_TILEDB ON)
    # No symbol 'PF_initPlugin'

    set(PDAL_DIR D:/miniconda3/envs/myenv/Library/lib/cmake/PDAL)
    find_package(PDAL 2.3.0  CONFIG)
    include_directories(${PDAL_INCLUDE_DIRS})
    link_directories(${PDAL_LIBRARY_DIRS})
    add_definitions(${PDAL_DEFINITIONS})

    find_package(OpenMP)
    if (OPENMP_FOUND)
        set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    endif()


    add_library(libpdal_plugin_filter_unixgpstimeconvert SHARED UnixGpsTimeConvert.cpp )
    target_include_directories(libpdal_plugin_filter_unixgpstimeconvert PRIVATE ${PDAL_INCLUDE_DIRS})
    target_link_libraries(libpdal_plugin_filter_unixgpstimeconvert PRIVATE ${PDAL_LIBRARIES} ${PDAL_ADD_PLUGIN_LINK_WITH} ws2_32.lib)


    include_directories(./GeoReference/include ./third_party/include /usr/include/eigen3)
    add_library(libpdal_plugin_filter_georeference SHARED ./GeoReference/GeoReference.cpp ./GeoReference/src/isometry3d_interpolation_buffer ./GeoReference/src/ogr_conversions.cpp ./third_party/src/pose.cpp)
    target_include_directories(libpdal_plugin_filter_georeference PRIVATE ${PDAL_INCLUDE_DIRS})
    target_link_libraries(libpdal_plugin_filter_georeference PRIVATE ${PDAL_LIBRARIES} gdal_i.lib  ws2_32.lib ) 


    add_library(libpdal_plugin_filter_clustercutpursuit SHARED cutpursuit/ClusterCutPursuit.cpp)
    include_directories(cutpursuit/parallel-cut-pursuit-master/include)
    target_include_directories(libpdal_plugin_filter_clustercutpursuit PRIVATE ${PDAL_INCLUDE_DIRS}  )
    target_link_libraries(libpdal_plugin_filter_clustercutpursuit PRIVATE ${PDAL_LIBRARIES} cut_pursuit.dll  ws2_32.lib)


    add_library(libpdal_plugin_filter_localheight SHARED LocalHeight.cpp)
    target_include_directories(libpdal_plugin_filter_localheight PRIVATE ${PDAL_INCLUDE_DIRS})
    target_link_libraries(libpdal_plugin_filter_localheight PRIVATE ${PDAL_LIBRARIES}  ws2_32.lib)


    add_library(libpdal_plugin_filter_mostfrequentvalue SHARED MostFrequentValue.cpp)
    target_include_directories(libpdal_plugin_filter_mostfrequentvalue PRIVATE ${PDAL_INCLUDE_DIRS})
    target_link_libraries(libpdal_plugin_filter_mostfrequentvalue PRIVATE ${PDAL_LIBRARIES}  ws2_32.lib)


    add_library(libpdal_plugin_filter_mergeuniqueid SHARED MergeUniqueId.cpp )
    target_include_directories(libpdal_plugin_filter_mergeuniqueid PRIVATE ${PDAL_INCLUDE_DIRS})
    target_link_libraries(libpdal_plugin_filter_mergeuniqueid PRIVATE ${PDAL_LIBRARIES}  ws2_32.lib)



    add_library(libpdal_plugin_filter_disandheighttotarget SHARED DisAndHeightToTarget.cpp )
    target_include_directories(libpdal_plugin_filter_disandheighttotarget PRIVATE ${PDAL_INCLUDE_DIRS})
    target_link_libraries(libpdal_plugin_filter_disandheighttotarget PRIVATE ${PDAL_LIBRARIES}  ws2_32.lib)



    add_library(libpdal_plugin_filter_ferryfromother SHARED FerryFromOther.cpp )
    target_include_directories(libpdal_plugin_filter_ferryfromother PRIVATE ${PDAL_INCLUDE_DIRS})
    target_link_libraries(libpdal_plugin_filter_ferryfromother PRIVATE ${PDAL_LIBRARIES}  ws2_32.lib)


    add_library(libpdal_plugin_filter_clustervoteclassification SHARED   ClusterVoteClassification.cpp)
    target_include_directories(libpdal_plugin_filter_clustervoteclassification PRIVATE ${PDAL_INCLUDE_DIRS})
    target_link_libraries(libpdal_plugin_filter_clustervoteclassification PRIVATE ${PDAL_LIBRARIES}  ws2_32.lib)


    include_directories(./third_party/include)
    add_library(libpdal_plugin_filter_poseconvert SHARED   PoseConvert.cpp ./third_party/src/pose.cpp)
    target_include_directories(libpdal_plugin_filter_poseconvert PRIVATE ${PDAL_INCLUDE_DIRS})
    target_link_libraries(libpdal_plugin_filter_poseconvert PRIVATE ${PDAL_LIBRARIES}  gdal_i.lib  ws2_32.lib)



    include_directories(./third_party/include)
    add_library(libpdal_plugin_filter_eulerconvert SHARED   EulerConvert.cpp ./third_party/src/pose.cpp)
    target_include_directories(libpdal_plugin_filter_eulerconvert PRIVATE ${PDAL_INCLUDE_DIRS})
    target_link_libraries(libpdal_plugin_filter_eulerconvert PRIVATE ${PDAL_LIBRARIES} gdal_i.lib  ws2_32.lib)



    include_directories(./third_party/include)
    add_library(libpdal_plugin_filter_pose6dtransformation SHARED   Pose6DTransformation.cpp ./third_party/src/pose.cpp)
    target_include_directories(libpdal_plugin_filter_pose6dtransformation PRIVATE ${PDAL_INCLUDE_DIRS})
    target_link_libraries(libpdal_plugin_filter_pose6dtransformation PRIVATE ${PDAL_LIBRARIES} gdal_i.lib  ws2_32.lib)


    include_directories(./GeoReference/include ./third_party/include /usr/include/eigen3)
    add_library(libpdal_plugin_filter_lookuppose SHARED ./LookUpPose.cpp ./GeoReference/src/isometry3d_interpolation_buffer ./GeoReference/src/ogr_conversions.cpp ./third_party/src/pose.cpp)
    target_include_directories(libpdal_plugin_filter_lookuppose PRIVATE ${PDAL_INCLUDE_DIRS})
    target_link_libraries(libpdal_plugin_filter_lookuppose PRIVATE ${PDAL_LIBRARIES} gdal_i.lib  ws2_32.lib) 



    add_library(libpdal_plugin_filter_reprojectiontoother SHARED ./ReProjectionToOther.cpp )
    target_include_directories(libpdal_plugin_filter_reprojectiontoother PRIVATE ${PDAL_INCLUDE_DIRS})
    target_link_libraries(libpdal_plugin_filter_reprojectiontoother PRIVATE ${PDAL_LIBRARIES}  ws2_32.lib) 


    include_directories(G:/PDAL-2.3.0/filters/private/expr)
    add_library(libpdal_plugin_filter_classifybyclusterfeatures SHARED   ClassifyByClusterFeatures.cpp G:/PDAL-2.3.0/filters/private/expr/AssignParser.cpp  G:/PDAL-2.3.0/filters/private/expr/AssignStatement.cpp G:/PDAL-2.3.0/filters/private/expr/BaseParser.cpp G:/PDAL-2.3.0/filters/private/expr/ConditionalExpression.cpp G:/PDAL-2.3.0/filters/private/expr/ConditionalParser.cpp G:/PDAL-2.3.0/filters/private/expr/Expression.cpp G:/PDAL-2.3.0/filters/private/expr/IdentExpression.cpp G:/PDAL-2.3.0/filters/private/expr/Lexer.cpp G:/PDAL-2.3.0/filters/private/expr/MathExpression.cpp  G:/PDAL-2.3.0/filters/private/expr/MathParser.cpp) 
    target_include_directories(libpdal_plugin_filter_classifybyclusterfeatures PRIVATE ${PDAL_INCLUDE_DIRS})
    target_link_libraries(libpdal_plugin_filter_classifybyclusterfeatures PRIVATE ${PDAL_LIBRARIES}  gdal_i.lib  ws2_32.lib)

else()
    message(STATUS "Cur System: ${CMAKE_SYSTEM_NAME}")
endif()
