cmake_minimum_required(VERSION 3.0)

project(Demn)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(Demo main.cpp)

target_link_libraries(Demo ${OpenCV_LIBS})

add_executable(Demo1 main1.cpp)

target_link_libraries(Demo1 ${OpenCV_LIBS})

add_executable(Demo2 main2.cpp)

target_link_libraries(Demo2 ${OpenCV_LIBS})

add_executable(Demo3 main3.cpp)

target_link_libraries(Demo3 ${OpenCV_LIBS})

add_executable(Demo4 main4.cpp)

target_link_libraries(Demo4 ${OpenCV_LIBS})