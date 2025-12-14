# ROS2导航规划系统
一个基于 ROS2 的导航与规划系统，用于参加 RoboMaster 机器人竞赛。

## 环境
- Ubuntu-22.04
- ros humble
- Mid-360 (硬件)
- USB2CAN (硬件)

## 快速开始
### 下载和编译项目
1. 下载项目 `git clone https://github.com/CeasarSmj/nav2_ros_ws.git`
2. 安装依赖项 `bash dependency_install.bash`
3. 编译项目 `bash compile.bash`，可能会出现 fast_lio 中有 std_err ，可以无视，再运行一次编辑即可


### 调整雷达配置
1. 前往 Livox 官网下载 LivoxViewer2 ， 然后给雷达上电连接电脑
2. 设置电脑对雷达的 IPv4 连接的 ip 为手动设置，设置 ip : 192.168.1.123, mask : 255.255.255.0
3. 用 LivoxViewer2 查看雷达的 ip
4. 修改 livox_ros_driver2/config/MiD360_config.json中的雷达ip为查看的ip，host_ip 为 192.168.1.123

### 扫图
1. 再运行一次 `bash compile.bash`
2. 加载环境`. install/setup.bash`
3. 启动slam`ros2 launch map_trans_toolkit slam.launch.py`
4. 控制 雷达/机器人 移动建图，完成后打开 rqt , 在菜单栏中选择 servive/service_caller ，call 一下 /map_save 即可
5. 把保存在工作空间目录下的 test.pcd 移动到 r2n_bringup_pkg/pcds 目录下
6. 运行 map_trans_pkg/scripts/pcd2pgm_simplify.py

### 运行规划
1. 再运行步一次 `bash compile.bash`
2. 加载环境`. install/setup.bash`
3. 运行 `ros2 launch r2n_bringup_pkg nav2.launch.py`
4. 在 rviz 中给定目标点，查看是否给出了路径
5. 查看 /cmd_vel 中是否有数据，`ros2 topic echo /cmd_vel`

### CAN 通信部分适配
1. 调整 can_serial_pkg/include/data_encoder.h 中的结构体以及编码函数为自己的版本，同时下位机也要遵守同样的通信规范
2. 插入 usb转can 设备，并查看端口号 `ls /dev/ttyUSB*`
3. 调整运行时的端口号，前往 r2n_bringup_pkg/launch/nav2.launch.py 修改 serial_port
4. 根据运行规划部分的启动方式启动，给定目标位置，查看机器人是否实现了移动

## 目录
```
nav_ros2_ws
|- README.md
|- src
    |- slam_pkgs
        |- livox_ros_driver2         // livox雷达驱动
        |- fast_lio // SLAM 包
        |- map_trans_pkg              // 地图转换包    
    |- nav2_bringup_pkg                   // 导航系统启动包
    |- can_serial_pkg                 // 数据打包成CAN数据包以及上下行通信
    |- rm_related_pkgs 
        |- robot_description_pkg        // 机器人描述包
        |- decision_pkg               // 决策包
```

## TF 树
map -> camera_init : map_trans_pkg / relocalize_node
camera_init -> body : fast_lio / mapping
body -> body_2d : map_trans_pkg / pointcloud2_to_laserscan


## 工作流
### slam
1. 雷达驱动 livox_ros_driver2 给出 /livox/lidar 和 /livox/imu
2. fast_lio mapping 持续进行 slam 并发布地图到 /Laser_map 
3. 通过 服务 /save_map 来触发地图的保存

### 导航
1. 雷达驱动 livox_ros_driver2 给出 /livox/lidar 和 /livox/imu
2. fast_lio mapping 持续进行 slam 并发里程计信息，发布 camera_init->body，发布扫描信息 /cloud_registered_in_body
3. relocalize_node 加载先验点云 test.pcd， 并和实时扫描到的进行匹配，并发布 map->camera_init
4. poiintcloud2_to_laserscan 监听 map->body，并发布 body->body_2d，同时订阅 /cloud_registered_in_body 发布 /scan ,也就是2D扫描数据
5. map_publisher_node 发布先验地图 test.pgm 到 /map
6. navigation2 部分订阅先验地图 /map， 订阅实时扫描 /scan ， 持续从 tf 树中监听 map->odom->body_2d。其中navigation2中的amcl和localization已经关闭，使用自己搭建的来实现
7. navigation2 部分监听目标位置话题 /goal_pose , 并进行规划后发布 costmap、plan 等等，最终发布 /cmd_vel
8. middle_coding_node 订阅 /cmd_vel，对它进行编码和打包后发送到 /msg_to_can
9. can_serial_node 订阅 /msg_to_can，然后发布到对应的 usb 转 can 设备中