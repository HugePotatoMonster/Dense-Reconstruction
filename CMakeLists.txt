cmake_minimum_required(VERSION 3.10)
project(demo VERSION 1.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
file(GLOB_RECURSE SOURCE_1 "${CMAKE_SOURCE_DIR}/src/*.cpp") 
file(GLOB_RECURSE SOURCE_2 "${CMAKE_SOURCE_DIR}/src/*.cu") 


set(OpenCV_DIR C:/Users/huang/Desktop/Tools/OCV/opencv/build)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

cuda_add_executable(demo main.cpp ${SOURCE_1} ${SOURCE_2})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
