## 星秒传感器驱动

1. 创建对象，传入主机的ip和端口（如果不传入主机地址，则为被动模式）
```
pavo::pavo_driver *driver = new pavo::pavo_driver("192.168.0.11", 2368);
```
2. 打开驱动，传入传感器的ip和端口
```
bool isRunning = driver->pavo_open(hostName, 2368);
```
3. 设置传感器参数
```
driver->enable_motor(true);    //开启电机运转
driver->set_motor_speed(25);   //电机速度为25HZ
driver->set_merge_coef(1);     //数据颗粒度（详情看文档）
```
4. 获取传感器数据
```
std::vector<pavo_response_scan_t> vec;  //数据的结构体
bool isOK = driver->get_scanned_data(vec, 0);  //第二个参数为等待时间，如果为0，则为永久等待
```

5. 其他
* 如果要运行多线程程序，这命令规则为：g++ -pthread  -o simm_driver main.cpp
* 运行此程序需要先安装boost库
