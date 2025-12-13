# ROS2导航规划系统
一个基于 ROS2 的导航与规划系统，用于参加 RoboMaster 机器人竞赛。

## 环境
- Ubuntu-22.04
- ros humble
- Mid-360
- USB2CAN

按照下面的步骤进行配置：
todo

## 快速开始



## 使用指南





## 目录
```
nav_ros2_ws
|- README.md
|- src
    |- simulate_pkgs
        |- virtual_lidar_driver_pkg  // 模拟雷达驱动包
        |- virtual_robot_sim_pkg     // 模拟机器人包
    |- slam_pkgs
        |- livox_ros_driver2         // livox雷达驱动
        |- fast_lio|point_lio|liosam // SLAM 包
        |- map_trans_pkg              // 地图转换包    
    |- nav2_bringup_pkg                   // 导航系统启动包
    |- can_serial_pkg                 // 数据打包成CAN数据包以及上下行通信
    |- rm_related_pkgs 
        |- robot_description_pkg        // 机器人描述包
        |- decision_pkg               // 决策包
```

## TF 树
### 仿真部分
- world -> base_link : virtual_robot_sim_ndoe 发布
- base_link -> robot_yaw -> robot_lidar :  robot_state_publisher发布，由 URDF 文件描述

### SLAM + 导航部分
- map -> odom : relocalization_node 发布
- odom -> body : SLAM 节点 point_lio 发布
- body -> body_2d : tf_tran_node 发布

## 话题结构
### 仿真部分
- /livox/imu <- virtual_lidar_driver_node
- /livox/lidar_pc2 <- virtual_lidar_driver_node
- /livox/lidar <- pc2_to_livoxcloud <- /livox/lidar_pc2
- /cmd_vel -> virtual_robot_sim_node

### SLAM 部分
- /livox/imu -> point_lio_mapping
- /livox/lidar -> point_lio_mapping
- /cloud_registered <- point_lio_mapping
- /cloud_registered -> relocalization_node
- relocalization_node -> /submap /map_to_odom /cur_scan_in_map /localization
- /map <- pcd_publisher
- /map -> relocalization_node
- /initial_pose <- rviz
- /initial_pose -> relositalization_node
- /cloud_registered -> relositalization_node
- /cloud_registered -> pointcloud2_to_laserscan
- /laserscan <- pointcloud2_to_laserscan

### 导航部分
- /map_2d <- map_publisher
- /map_2d -> navigation2
- /laserscan -> navigation2
- /global_costmap <- navigation2
- /local_costmap <- navigation2
- /global_plan <- navigation2
- /local_plan <- navigation2
- /navigation_goal <- rviz
- /navigation_goal -> navigation2
- /cmd_vel <- navigation2

### 串口部分
- /cmd_vel -> middle_codding_node
- /msg_to_can <- middle_codding_node
- /msg_to_can -> can_serial_node

## 组件
### virtual_lidar_driver_pkg [Done]
用于模拟雷达驱动的行为，给 slam 部分的代码提供数据。

包含几个个脚本，分别是：
- map_generator.py：地图生成节点，可以利用 png 地图和 yaml 文件来生成 pcd 和 pgm 地图文件
- virtual_lidar_driver_node：模拟雷达数据驱动节点，将模拟数据发布为点云数据，并发布为CAN数据包
- pc2_to_livoxcloud ： 把点云从 pointcloud2 转换成 livox 专用的格式

包含启动文件：
- virtual_lidar_driver.launch.py


### virtual_robot_sim_pkg [Done]
用于模拟机器人在地图中的移动行为。

包含一个节点 virtual_robot_sim_node 用于：
- 订阅 /map 来确定移动能否成功
- 订阅 /cmd_vel 然后在地图中移动
- 发布 /camera_init 到 /body 的 TF

包含启动文件：
- virtual_robot_sim.launch.py
- simulation.launch.py : 启动包括 robot_urdf.launch.py 、 virtual_lidar_driver_pkg 和 virtual_robot_sim_pkg

### map_trans_pkg [Done]
用于在不同地图格式之间提供转换服务。
- relocalization_node : 重定位，根据先验点云和slam点云做匹配，得到 map -> odom 的 tf 转换
- pointcloud2_to_laserscan : 将3d激光雷达的点云数据转换成2d激光雷达数据  [Done]
- pointcloud2_to_laserscan_elv : 将3d激光雷达的点云数据转换成2d激光雷达数据，不过采用高程图的方式来处理斜坡
- map_publisher_node : 读取并发布 pgm 地图
- pcd_publisher_node : 读取并发布 pcd 地图
[Done]

包含启动文件：
- pc2ls.launch.py
- pc2ls_elv.launch.py
- relocalization.launch.py
- test_relocalization.launch.py
  - 启动 virtual_lidar_driver_pkg / virtual_lidar.launch.py
  - 启动 map_trans_pkg / pcd_publisher.py
  - 启动 map_trans_pkg / relocalization.launch.py
  - 利用已有地图和地图切片来对于重定位模块进行测试


### can_serial_pkg
用于做 CAN 通信相关操作。

- can_serial_node
- middle_codding_ndoe

### robot_description_pkg  [Done]
用于保存机器人的描述文件，例如机器人的 TF 结构描述(URDF)

包含launch文件用于发布机器人的 TF
- robot_urdf.launch.py




### decision_pkg
包含状态机用于选择要前往的坐标点。


### nav2_bringup_pkg
- nav2.launch.py : 真实世界版本导航启动接口 -> 带slam部分，带 livox_ros_driver2，不带 sim 部分
- sim_nav2.launch.py : 仿真版本导航启动接口 -> 带slam部分，带 sim 部分，不带 livox_ros_driver2
- simple_sim_nav2.launch.py : 简单仿真版本导航启动接口 -> 不带slam部分 ， 使用amcl来定位 ， 再加上 nav2