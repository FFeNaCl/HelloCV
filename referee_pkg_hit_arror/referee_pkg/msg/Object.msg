# Object.msg (选手发布的物体信息基础类型)
string target_type                  # 目标类型，如 "armor_1", "circle_red"
geometry_msgs/Point[] corners       # 四个角点的像素坐标 （左下起始，逆时针方向）
