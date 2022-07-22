find_package(OpenCV REQUIRED)

aux_source_directory(. DIR)

add_library(lib ${DIR})

target_link_libraries(lib ${OpenCV_LIBS})