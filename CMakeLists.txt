cmake_minimum_required(VERSION 3.5)
project(cozmo_cpp)

find_package(aruco REQUIRED)
find_package(OpenCV REQUIRED)
set (CMAKE_CXX_STANDARD 11)

add_executable(extract_poses_from_video extract_poses_from_video.cpp pcdwriter.cpp)
target_include_directories(extract_poses_from_video PRIVATE ${aruco_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_compile_options(extract_poses_from_video PRIVATE -fpermissive)
target_link_libraries(extract_poses_from_video aruco opencv_imgcodecs opencv_core opencv_aruco)

add_executable(my_test_markermap my_test_markermap.cpp pcdwriter.cpp)
target_include_directories(my_test_markermap PRIVATE ${aruco_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(my_test_markermap aruco opencv_imgcodecs opencv_core opencv_aruco)