## 版本变化1.4

### 一、config 配置：
1. 由于添加双雷达，主雷达的参数值被修改:
```
– ClientLaserMask.maxRange 修改为 ClientLaserMask.laser.maxRange；
– ClientLaserMask.minRange 修改为 ClientLaserMask.laser.minRange；
– ClientLaserMask.maxRangeLines 修改为 ClientLaserMask.laser.maxRangeLines；
– ClientLaserMask.minRangeLines 修改为 ClientLaserMask.laser.minRangeLines；
– ClientSensor.laserType 修改为 ClientSensor.laser.type；
– ClientSensor.laserAddress 修改为 ClientSensor.laser.address；
– ClientSensor.laserClientAddress 修改为 ClientSensor.laser.clientAddress；
– ClientSensor.mirrorLaserScans 修改为 ClientSensor.laser.mirrorLaserScans；
— ClientSensor.laserTransformOdometry 修改为 ClientSensor.vehicleTransformOdometry；
— ClientSensor.laserErrorModelTransformOdometry 修改为  ClientSensor.vehicleErrorModelTransformOdometry；
— ClientSensor.laserTransformImu 修改为 ClientSensor.vehicleTransformImu；
— ClientSensor.laserErrorModelTransformImu 修改为 ClientSensor.vehicleErrorModelTransformImu；
```    
### 二、功能添加添加：
1. 添加了双雷达参数：
```
– ClientLaserMask.enableLaser2 ；
– ClientLaserMask.laser2.maxRange； 
– ClientLaserMask.laser2.minRange ；
– ClientLaserMask.laser2.maxRangeLines； 
– ClientLaserMask.laser2.minRangeLines；
– ClientSensor.laser2.type；
– ClientSensor.laser2.address； 
– ClientSensor.laser2.clientAddress； 
– ClientSensor.laser2.mirrorLaserScans； 
– ClientSensor.laser.useIntensities；
– ClientSensor.laser2.useIntensities；
```
2. 添加雷达强度
```
– ClientSensor.laser.laserUseIntensities；
– ClientSensor.laser2.laserUseIntensities；
```
3. 添加了修正雷达坐标
```
– ClientSensor.laser.vehicleTransformLaser.x；
– ClientSensor.laser.vehicleTransformLaser.y；
– ClientSensor.laser.vehicleTransformLaser.z；
– ClientSensor.laser.vehicleTransformLaser.roll；
– ClientSensor.laser.vehicleTransformLaser.pitch；
– ClientSensor.laser.vehicleTransformLaser.yaw；
– ClientSensor.laser2.vehicleTransformLaser.x；
– ClientSensor.laser2.vehicleTransformLaser.y；
– ClientSensor.laser2.vehicleTransformLaser.z；
– ClientSensor.laser2.vehicleTransformLaser.roll；
– ClientSensor.laser2.vehicleTransformLaser.pitch；
– ClientSensor.laser2.vehicleTransformLaser.yaw；
```

### 三、新增API
1. 添加aboutModulesComponent，返回值0，1，2，3，4，对应：出错，client，server，client_support， servver_support模块；
2. 添加configDefaultList，返回默认config值;
3. config参数名称修改，参数添加；
4. 添加sessionList，获得session信息；
5. 添加sessionRevokeId，获得session的revokeId;
6. 添加sessionRevoke，根据revokeId，撤销session；
7. 添加sessionRevokeAll，撤销所有session;
8. 添加clientExpandMapEnable，开始拓展地图；
9. clientExpandMapDisable，停止拓展地图；
10. clientExpandMapDeleteOverwriteZones，删除EditZone；
11. clientExpandMapListOverwriteZones，查询EditZone列表；
12. clientExpandMapGetPriorMapPointCloud，获得先前地图的点云图；
