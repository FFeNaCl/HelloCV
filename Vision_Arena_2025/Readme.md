ros2 launch referee_pkg referee_pkg_launch.xml TeamName:="TEAMENAME" StageSelect:=0 ModeSelect:=0

source install/setup.bash

ros2 launch camera_sim_pkg camera.launch.py

ros2 launch target_model_pkg target_action.launch.py model:=src/target_model_pkg/urdf/rectangle/rectangle.sdf model_name:=rectangle
ros2 launch target_model_pkg target_action.launch.py model:=src/target_model_pkg/urdf/sphere/sphere.sdf model_name:=sphere
ros2 launch target_model_pkg target_action.launch.py model:=src/target_model_pkg/urdf/armor/armor_1.sdf model_name:=armor_1

ros2 topic pub /type std_msgs/msg/Int32 "{data: 1}"

colcon build

ros2 run player_pkg VisionNode

ros2 run player_pkg SteptwoNode

