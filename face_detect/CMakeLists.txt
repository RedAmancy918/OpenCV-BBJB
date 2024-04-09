cmake_minimum_required(VERSION 3.1)  
project(FaceDetection)               

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

find_package(OpenCV REQUIRED)  
include_directories(${OpenCV_INCLUDE_DIRS})  

add_executable(FaceDetect test.cpp)  

target_link_libraries(FaceDetect ${OpenCV_LIBS})  

