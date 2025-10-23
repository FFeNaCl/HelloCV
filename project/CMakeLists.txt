cmake_minimum_required(VERSION 3.0)

project(Demo)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(Demo1 main.cpp)

target_link_libraries(Demo1 ${OpenCV_LIBS})