cmake_minimum_required(VERSION 3.10)
project(demo VERSION 1.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
file(GLOB_RECURSE SOURCE_1 "${CMAKE_SOURCE_DIR}/src/*.cpp") 

set(OpenCV_DIR C:/Users/huang/Desktop/Tools/OCV/opencv/build)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(demo main.cpp ${SOURCE_1} "src/StereoMapping/Helper/smCostHelper.cpp" "include/Common/Utility/cmMarchingCubesDef.h")
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})