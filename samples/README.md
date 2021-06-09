### 自启动定位、跨楼层、设置初始值(Samples)

```
  // 1.init 初始化（docker地址、用户名、密码）
  Bosch bosch("172.17.0.1", "admin", "123");

  //2.sessionLogin 登录客户端
  bosch.sessionLogin(8080);

  //3.开启自动定位
  //先关掉当前定位，再打开
  bosch.clientLocalizationSwitch(false);
  sleep(2);
  bosch.clientLocalizationSwitch(true);
  sleep(2);
  
  //4.设置当前地图(原理为修改config里面的属性，自己可以编程修改传感器ip、地址等一系列参数），切换楼层用此语句
  bosch.configSet_string("application.localization.activeMapName","地图名称");
  
  //5.设置初始位置(x,y,yaw)
  bosch.clientSetSeed(1, 1, 0.1);
```

### 注意事项
1. 依赖 json.hpp 和 libcurl (sudo apt install curl)
2. 编译代码为 g++ bosch.cpp -lcurl
3. 使用倒入证书功能licensingFeatureSet 时证书与cpp文件在同一路径。
