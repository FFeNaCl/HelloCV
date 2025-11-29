## launch文件参数说明及使用方法
### referee_pkg_launch.xml
```cpp
<arg name="TeamName" default="TeamA" description = "队伍名称"/>
<arg name="StageSelect" default="0" description = "起始阶段选择 : 0,1,2,3,4,5"/>
<arg name="ModeSelect" default="0" description = "模式选择 {0: 恒定模式 , 1 : 顺序模式}"/>
```

<font style="color:#DF2A3F;">特别注意:备赛期间选择起始阶段0，恒定模式0 </font>，正式比赛运行时有所不同

```cpp
ros2 launch referee_pkg referee_pkg_launch.xml \
    TeamName:="TEAMENAME" \

```