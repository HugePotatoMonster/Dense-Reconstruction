cmake_minimum_required(VERSION 3.10)
project(demo VERSION 1.0)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
file(GLOB_RECURSE SOURCE_1 "${CMAKE_SOURCE_DIR}/src/*.c") 
file(GLOB_RECURSE SOURCE_2 "${CMAKE_SOURCE_DIR}/src/*.cpp") 
file(GLOB_RECURSE SOURCE_3 "${CMAKE_SOURCE_DIR}/src/*.cu") 

# OpenCV & CUDA
set(OpenCV_DIR C:/Users/huang/Desktop/Tools/OCV/opencv/build)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# OpenGL & GLFW & GLAD
set(GLFW_LIBRARIES_1 C:/WR/2/glfw-3.3.7.bin.WIN64/lib-vc2019/glfw3.lib)
set(GLFW_LIBRARIES_2 C:/WR/2/glfw-3.3.7.bin.WIN64/lib-vc2019/glfw3_mt.lib)
set(GLFW_LIBRARIES_3 C:/WR/2/glfw-3.3.7.bin.WIN64/lib-vc2019/glfw3dll.lib)
set(GLFW_INCLUDE_DIRS C:/WR/2/glfw-3.3.7.bin.WIN64/include)
include_directories(${GLFW_INCLUDE_DIRS})
set(GLAD_INCLUDE_DIRS C:/WR/Dense-Reconstruction/lib/glad/include)
include_directories(${GLAD_INCLUDE_DIRS})

cuda_add_executable(demo main.cpp ${SOURCE_1} ${SOURCE_2} ${SOURCE_3})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${GLFW_LIBRARIES_1}  ${GLFW_LIBRARIES_2}  ${GLFW_LIBRARIES_3})
