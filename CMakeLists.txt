cmake_minimum_required(VERSION 3.0.0)
project(DenseReconstruction VERSION 0.1.0)

file(GLOB_RECURSE ALL_SRCS "${CMAKE_SOURCE_DIR}/src/*.cpp")

set(OpenCV_DIR D:/CLib/opencv/build/x64/vc15/lib)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(Eigen3_DIR D:/CLib/eigen3/include/eigen3)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(DenseReconstruction main.cpp ${ALL_SRCS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${EIGEN3_LIBRARIES})