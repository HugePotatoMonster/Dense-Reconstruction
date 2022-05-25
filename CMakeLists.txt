cmake_minimum_required(VERSION 3.10)
project(demo VERSION 1.0)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
file(GLOB_RECURSE SOURCE_1 "${CMAKE_SOURCE_DIR}/src/*.c") 
file(GLOB_RECURSE SOURCE_2 "${CMAKE_SOURCE_DIR}/src/*.cpp") 
file(GLOB_RECURSE SOURCE_3 "${CMAKE_SOURCE_DIR}/src/*.cu") 
file(GLOB_RECURSE SOURCE_4 "D:/CLib/glad/src/*.c") 

# OpenCV & CUDA
set(OpenCV_DIR D:/CLib/opencv/build/x64/vc15/lib)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# OpenGL & GLFW & GLAD
set(GLFW_LIBRARIES_1 D:/CLib/glfw-3.3.7.bin.WIN64/lib-vc2019/glfw3.lib)
set(GLFW_LIBRARIES_2 D:/CLib/glfw-3.3.7.bin.WIN64/lib-vc2019/glfw3_mt.lib)
set(GLFW_LIBRARIES_3 D:/CLib/glfw-3.3.7.bin.WIN64/lib-vc2019/glfw3dll.lib)
set(GLFW_INCLUDE_DIRS D:/CLib/glfw-3.3.7.bin.WIN64/include)
include_directories(${GLFW_INCLUDE_DIRS})
set(GLAD_INCLUDE_DIRS D:/CLib/glad/include)
include_directories(${GLAD_INCLUDE_DIRS})

# GLM
set(GLM_INCLUDE_DIRS D:/CLib/glm-0.9.9.8/glm)
include_directories(${GLM_INCLUDE_DIRS})

# Done
cuda_add_executable(demo main.cpp ${SOURCE_1} ${SOURCE_2} ${SOURCE_3} ${SOURCE_4})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${GLFW_LIBRARIES_1}  ${GLFW_LIBRARIES_2}  ${GLFW_LIBRARIES_3})
