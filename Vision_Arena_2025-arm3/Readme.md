先启动裁判系统
ros2 launch referee_pkg referee_pkg_launch.xml TeamName:="TEAM1"
环境变量，这个每一个终端都需要运行一次
source install/setup.bash
启动摄像头,这样gazebo就会有摄像头了
ros2 launch camera_sim_pkg camera.launch.py
这些是模型
ros2 launch target_model_pkg target_action.launch.py model:=src/target_model_pkg/urdf/rectangle/rectangle.sdf model_name:=rect
ros2 launch target_model_pkg target_action.launch.py model:=src/target_model_pkg/urdf/sphere/sphere.sdf model_name:=sphere
ros2 launch target_model_pkg target_action.launch.py model:=src/target_model_pkg/urdf/armor/armor_1.sdf model_name:=armor_red_1
这可以让它们运动
ros2 topic pub /type std_msgs/msg/Int32 "{data: 1}"
改位置的话，可以直接从gazebo里面改，当然用指令也行
ros2 topic pub /pose geometry_msgs/msg/Pose "{position: {x: 0.0, y: 1.0, z: 0.0}}"
这个是编译包，因为里面只有一个功能包所以不用使用select
colcon build
这个是启动节点，这两个节点分别启动


ros2 run player_pkg TestNode
ros2 run player_pkg SteptwoNode
ros2 run player_pkg VisionNode


如果用launch
ros2 launch player_pkg vision.launch
