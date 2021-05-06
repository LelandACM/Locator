## 兴颂传感器驱动

1. 创建雷达驱动
```
string server_address = "192.168.1.89";
int port = 8080;  //雷达端口默认是8080，不可更改
hins::ConnectionAddress laser_conn_info(server_address, port);
hins::XingSongDriverHdr driver_hdr = std::make_shared<hins::XingSongDriver>(laser_conn_info);
```
2. 获取雷达数据
```
hins::ScanData data = driver_hdr->GetFullScan();
```
3. 将数据构建成BinaryInterfaceDummyLaserData类
```
auto sensor = laserData.getWaitConstantScan(ranges, false, indensities, length);
server.write(&sensor, sizeof(sensor));
```

4. 其他
* 运行此程序需要先安装boost库
* 请使用makefile工具编译此程序
* 运行程序命令： cd build/ ,然后运行 ./xs_driver
