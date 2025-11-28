cmake_minimum_required(VERSION 3.8)
project(referee_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

############################################
## Message & Service generation
############################################

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Object.msg"
  "srv/HitArmor.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

############################################
## Executables
############################################

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

# Server node
add_executable(hit_arror_server src/hit_arror_server.cpp)
ament_target_dependencies(hit_arror_server
  rclcpp
  std_msgs
  geometry_msgs
  rosidl_default_runtime
)
target_link_libraries(hit_arror_server
  ${OpenCV_LIBRARIES}
)

# Simple test client
add_executable(hit_arror_client src/hit_arror_client.cpp)
ament_target_dependencies(hit_arror_client
  rclcpp
  geometry_msgs
  rosidl_default_runtime
)

# Fake vision publisher using Object.msg
add_executable(fake_object_publisher src/fake_object_publisher.cpp)
ament_target_dependencies(fake_object_publisher
  rclcpp
  geometry_msgs
  std_msgs
  rosidl_default_runtime
)

install(TARGETS
  hit_arror_server
  hit_arror_client
  fake_object_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
