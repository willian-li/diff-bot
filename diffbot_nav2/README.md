# diffbot_nav2

## launch：启动文件

### location.launch.py

- 读取建好的地图
- amcl定位

### navigation_launch.py

- 控制器
- 路径规划
- 速度平滑
- 成本地图

## params:参数配置

- 控制器插件：MPPIController

  注：*RotationShimController*和*DWBLocalPlanner*结合使用效果也可以

- 局部成本地图:["obstacle_layer","inflation_layer"] 【雷达扫描，膨胀】

- 全局成本地图: ["static_layer", "obstacle_layer","inflation_layer"] 【静态地图，雷达，扫描，膨胀】

  注：
  1.未使用*voxel_layer*或*spatio_temporal_voxel_layer*(读取点云数据构建成本地图)，使用效果不佳

  2.未使用插件*filters: ["keepout_filter"]*(导航禁区)，使用后会导致控制器犹豫不决，具体表现机器人行走时左右摆头或停止不前

## maps:离线地图

## test:测试

- 导航禁区的测试