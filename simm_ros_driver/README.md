### 星秒传感器ROS驱动对接力士乐定位软件使用步骤

1. 将ROS_simple_driver 、 pavo_ros.zip 拷贝到`ROS`的src目录下(记得解压），例如：本人电脑是：`catkin_ws/src`
2. 在Terminal工具运行以下命令（按顺序）：
   * source devel/setup.sh
   * roslaunch pavo_ros pavo_scan_view.launch
   * source devel/setup.sh
   * roslaunch msg2tcp send_laser.launch
3. 打开力士乐定位软件`LocalizationGUI`(本人电脑目录为：/home/lelandacm/LLS/installation/)
4. 设置传感器类型和传感器的IP地址,星秒的为： type: simple ,  laserAddress: 自定义
