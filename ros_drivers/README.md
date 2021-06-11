### 力士乐定位软件ROS相关驱动使用说明

1. 北阳ROS Drivers
2.  雷达数据中转到Locator的ROS包

#### 关于北阳ROS驱动如何将数据传递到Locator
1. 安装好ROS
2. 将文件hokuyo ros driver、simple_ros_driver拷贝到`catkin_ws`的src目录下
3. 打开`catkin_ws`目录（名字可能不一样，不同的ROS版本有区别），打开Terminal,运行： `catkin_make`
4. 设置源：`source ~/catkin_ws/devel/setup.bash`
5. 运行hokuyo驱动：`rosrun urg_node urg_node _ip_address:=192.168.0.10`
6. 再次设置msg2tcp的源： `source ~/catkin_ws/devel/setup.bash`
7. 运行msg2tcp的ros包: `roslaunch msg2tcp send_laser.launch`
8. 登录并打开Locator，点击Recording就能正确使用软件


#### 参考文章
ros kinetic +hokuyo 源码安装 https://blog.csdn.net/hongliang2009/article/details/73302986
