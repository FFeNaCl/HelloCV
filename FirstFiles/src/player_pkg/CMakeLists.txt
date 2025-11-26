cmake_minimum_required(VERSION 3.8...3.10)
project(player_pkg)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED) 
find_package(OpenCV REQUIRED)
find_package(pose_msg REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(referee_pkg REQUIRED)
find_package(sensor_msgs REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wno-deprecated-declarations)
endif()
#Circle   11111111111111111111111111111111111
add_executable(CircleNode src/circle.cpp)
ament_target_dependencies(CircleNode
  rclcpp
  std_msgs
  OpenCV 
  geometry_msgs
  cv_bridge
  pose_msg
  referee_pkg
)
# 链接OpenCV库
target_link_libraries(CircleNode ${OpenCV_LIBS})

# 为Vision_test添加消息类型支持（新方法）
target_link_libraries(CircleNode ${typesupport_target})

# 安装Vision_test
install(TARGETS CircleNode
  DESTINATION lib/${PROJECT_NAME}
)

#Rectangle   11111111111111111111111111111111111
add_executable(VisionNode src/vision_node.cpp)
ament_target_dependencies(VisionNode
  rclcpp
  std_msgs
  OpenCV 
  geometry_msgs
  cv_bridge
  pose_msg
  referee_pkg
)
# 链接OpenCV库
target_link_libraries(VisionNode ${OpenCV_LIBS})

# 为Vision_test添加消息类型支持（新方法）
target_link_libraries(VisionNode ${typesupport_target})

# 安装Vision_test
install(TARGETS VisionNode
  DESTINATION lib/${PROJECT_NAME}
)

#Armor   11111111111111111111111111111111111
add_executable(ArmorNode src/armor.cpp)
ament_target_dependencies(ArmorNode
  rclcpp
  std_msgs
  OpenCV 
  geometry_msgs
  cv_bridge
  pose_msg
  referee_pkg
)
# 链接OpenCV库
target_link_libraries(ArmorNode ${OpenCV_LIBS})

# 为Vision_test添加消息类型支持（新方法）
target_link_libraries(ArmorNode ${typesupport_target})

# 安装Vision_test
install(TARGETS ArmorNode
  DESTINATION lib/${PROJECT_NAME}
)

#Test   11111111111111111111111111111111111
add_executable(TestNode src/test.cpp)
ament_target_dependencies(TestNode
  rclcpp
  std_msgs
  OpenCV 
  geometry_msgs
  cv_bridge
  pose_msg
  referee_pkg
)
# 链接OpenCV库
target_link_libraries(TestNode ${OpenCV_LIBS})

# 为Vision_test添加消息类型支持（新方法）
target_link_libraries(TestNode ${typesupport_target})

# 安装Vision_test
install(TARGETS TestNode
  DESTINATION lib/${PROJECT_NAME}
)


#Steptwo   11111111111111111111111111111111111
add_executable(SteptwoNode src/steptwo.cpp)
ament_target_dependencies(SteptwoNode
  rclcpp
  std_msgs
  OpenCV 
  geometry_msgs
  cv_bridge
  pose_msg
  referee_pkg
)
# 链接OpenCV库
target_link_libraries(SteptwoNode ${OpenCV_LIBS})

# 为Vision_test添加消息类型支持（新方法）
target_link_libraries(SteptwoNode ${typesupport_target})

# 安装Vision_test
install(TARGETS SteptwoNode
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# 包含OpenCV头文件
include_directories(${OpenCV_INCLUDE_DIRS})


ament_package()
