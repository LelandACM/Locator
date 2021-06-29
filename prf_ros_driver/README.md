### 力士乐定位软件倍加福R2000 ROS相关驱动使用说明
`目前力士乐定位软件内嵌支持倍加福雷达,可以直接运行，但一些老版本型号的倍加福不支持内嵌，需要使用ROS中转,请知晓。`
1. 倍加福ROS Driver(pepperl_fuchs_r2000)
2. 雷达数据中转到Locator的ROS包(simple_ros_driver)
#### 关于倍加福ROS驱动如何将数据传递到Locator
1. 安装好ROS
2. 将文件pepperl_fuchs_r2000、simple_ros_driver拷贝到catkin_ws的src目录下
3. 打开catkin_ws目录（名字可能不一样，不同的ROS版本有区别），打开Terminal,运行： catkin_make
4. 设置源：source ~/catkin_ws/devel/setup.bash
5. 运行倍加福驱动：rosrun pepperl_fuchs_r2000 r2000.launch
6. 监测是否运行成功： rostopic echo /scan
7. 再次设置msg2tcp的源： source ~/catkin_ws/devel/setup.bash
8. 由于倍加福的节点名叫r2000_node ,需要修改simple_ros_driver里映射的node名称为：`/r2000_node/scan`，hokuyo使用默认的`/san`
```
  <launch>
        <node name="laser2tcp_node" pkg="msg2tcp" type="laser2tcp_node" output="screen">
        <param name="port" type="int" value="9090" />
        <param name="laser_topic" type="string" value="/r2000_node/scan"/> 
        </node>
    </launch>
```
9. 运行msg2tcp的ros包: roslaunch msg2tcp send_laser.launch
10. 登录并打开Locator，点击Recording就能正确使用软件
