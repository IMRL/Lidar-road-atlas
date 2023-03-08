This code is an implementation of our work "Lidar Road-Atlas: An Efficient Map Representation for General 3d Urban Environment", published at Journal of Field Robotics, 2023.
[Reference]: Banghe Wu, Chengzhong Xu, and Hui Kong, Lidar Road-Atlas: An Efficient Map Representation for General 3d Urban Environment, Journal of Field Robotics, 2023


----------------------------------------------------------------------------------------------------------------------------------------------------------

## Road-Atlas 相关代码

- RoadAtlas-ros: 建图程序本体，ROS版本
    - chunkmap: 基础地图表示库
    - chunkmap_msg: 地图库交互消息格式定义
    - chunkmap_rviz: 地图库的rviz显示插件
    - LidarAtlas: 建图程序本体
        - ground_detect: 局部地图生成
        - map_builder: 地图构建
        - lidarRoadDetect: 单帧可通行区域检测
    - LeGO-LOAM: 已适配的LeGO-LOAM，(iris回环, 实验标准版本)
    - SC-LeGO-LOAM-gd: 已适配的LeGO-LOAM，(scancontext回环, 代码干净一点)
- gd-noros: 建图程序，无ROS的版本
- common-localization-framework: 新的定位程序，写了个简单的框架。只实现图像的定位算法，雷达的移植过去没测试。
- gd-localization: 旧的定位程序，代码未清理。
- magnum-chunkmap-viz: chunkmap地图查看器

## 项目依赖

> C++17

对于全部程序: OpenCV, PCL, Eigen, msgpack

- RoadAtlas-ros: ROS, yaml-cpp
- common-localization-framework: fmt, range-v3
- magnum: OpenGL, SDL2

## 编译注意事项

> cmake记得加`-DCMAKE_BUILD_TYPE=Release`

- RoadAtlas-ros:
    - LiDAR模型是定义在代码里的，chunkmap的障碍编码参数也是
    - save_path不为空的话，会尝试保存地图文件到指定路径
- gd-noros, common-localization-framework, gd-localization:
    - 注意修改参数和数据路径
- magnum-chunkmap-viz:

## 基本使用

- RoadAtlas-ros:
    - 启动一个LeGO-LOAM
    - `roslaunch LidarAtlas run.launch save_path:="......"`

# reference of code

- chunkmap & road-atlas(gd) 注释在gd-noros里
