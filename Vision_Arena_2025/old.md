# 一、启动方法
建议在非比赛阶段不使用docker进行工作
## 裁判系统
如有需要自行更改，下面仅作为示例
```
ros2 launch referee_pkg referee_pkg_launch.xml TeamName:="TEAMENAME" StageSelect:=0 ModeSelect:=0
```
备赛期间选择起始阶段0，恒定模式0
## gazebo

运行摄像头仿真
```
ros2 launch camera_sim_pkg camera.launch.py
```
**比赛时不需要以下操作，仅运行摄像头仿真**
运行目标仿真
```
ros2 launch target_model_pkg target_action.launch.py
```

参数有：
model 模型文件（路径src/target_model_pkg/urdf/）

model_name 模型名字（不能生成名字一样的模型）<font style="color:#DF2A3F;">**识别要定义为[话题与服务消息说明](doc/Topic.md)下对应的模型名字**</font>(例如例程中使用的模型名字为sphere，这里的model_name也要写sphere)（**命名不同无法识别**）

# 从tar文件中读取镜像 其名成为 vision-vrena-2025:v0.1.2
docker load -i Vision-Vrena-2025.tar

# 运行Dockerfile文件将选手的文件以及裁判系统文件移入以构造一个新的镜像其名称为vision-vrena-2025:v0.1.3
docker build -t vision-vrena-2025:v0.1.3 .

# 运行docker-compose.yml文件以镜像构造三个用dockernetwork链接可互相通信的容器
docker-compose up

关闭容器
docker-compose down 
#备赛期间可以不使用docker-compose方式运行容器，直接运行单个容器，建立网络链接。