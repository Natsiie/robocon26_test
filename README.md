# ROS2 机器人导航与感知系统（个人二次开发版）

基于 ROS2 Humble 的机器人建图、全局重定位、Nav2 导航与视觉-激光感知系统，硬件平台为 Livox Mid-360 与 USB-CAN 底盘。

> **关于本仓库**
>
> 本项目基于 [CeasarSmj/nav_ros2_ws](https://gitee.com/CeasarSmj/nav_ros2_ws) 二次开发。原项目提供了 FAST-LIO2、Nav2、地图转换与 CAN 通信的基础框架；本仓库在此基础上补充了视觉-激光融合目标定位、点云聚类跟踪、九宫格状态识别、任务决策，以及全局重定位节点的线程安全优化。
>
> 原仓库未附带 LICENSE。如需公开分发，请先与原作者确认；FAST-LIO2、FAST-LIVO2、livox_ros_driver2、FAST-Calib 等第三方组件版权归各自作者所有。

## 1. 功能模块

| 模块 | 说明 |
| --- | --- |
| 3D 建图 | Livox Mid-360 + FAST-LIO2，输出 PCD 点云地图与里程计 |
| 地图转换 | PCD 按高度切片生成 2D 占据栅格（PGM + YAML） |
| 全局重定位 | Open3D 多尺度 ICP，先验 PCD 与实时扫描匹配，发布 `map → camera_init` |
| 3D → 2D | 点云重力对齐、高度滤波后投影为 `/scan`，接入 2D Nav2 |
| 路径规划 | Nav2：SMAC Hybrid-A* 全局规划 + DWB 局部控制 |
| 底盘通信 | `/cmd_vel` 编码为串口/CAN 帧，支持断线重连 |
| 视觉感知 | YOLOv8 检测、LiDAR-相机投影融合、目标 3D 定位 |
| 点云感知 | DBSCAN 聚类 + 轻量跟踪器，输出稳定目标 ID 与包围盒 |
| 九宫格识别 | RANSAC 平面拟合 + 3×3 动态网格，输出每格占用与类别 |
| 任务决策 | 台阶目标匹配、任务数组生成、到达观测点触发识别快照 |

## 2. 环境与依赖

- Ubuntu 22.04
- ROS2 Humble
- Livox Mid-360
- USB-CAN（`/dev/ttyUSB*`）

系统与 ROS 依赖：

```bash
bash dependency_install.bash
```

Python 侧还需要：

```bash
pip3 install open3d ultralytics opencv-python numpy scipy scikit-learn pyserial
```

> `cv_bridge`、`sensor_msgs_py` 随 ROS2 安装；`ultralytics` 仅感知模块需要。

## 3. 编译

```bash
bash compile.bash
source install/setup.bash
```

如果 FAST-LIO 首次编译出现 `std_err`，重新执行一次 `bash compile.bash` 即可。

## 4. 目录结构

```
nav_ros2_ws_test
├── README.md
├── compile.bash                 # colcon 编译
├── dependency_install.bash      # 系统 / ROS 依赖安装
├── slam.bash                    # 建图启动
├── nav.bash                     # 导航启动
├── src
│   ├── slam_pkgs
│   │   ├── FAST_LIO_ROS2        # FAST-LIO2 建图与里程计
│   │   ├── livox_ros_driver2    # Livox Mid-360 驱动
│   │   └── map_trans_pkg        # 重定位、点云转 scan、地图发布、感知脚本
│   ├── r2n_bringup_pkg          # Nav2 启动、参数、RViz、地图
│   ├── can_serial_pkg           # /cmd_vel → 串口/CAN
│   └── rm_related_pkgs          # 机器人描述等
└── src/slam_pkgs/map_trans_pkg/scripts
    ├── yolo_detector.py         # YOLOv8 检测节点
    ├── cube_locator.py          # 视觉-激光融合 3D 定位
    ├── dbscan_clustering.py     # 点云聚类与跟踪
    ├── grid_status_manager.py   # 九宫格状态识别
    ├── StepManage.py            # 台阶任务管理
    ├── Step_monitor.py          # 观测点触发
    └── ...
```

## 5. 快速开始

### 5.1 雷达配置

1. 使用 LivoxViewer2 查看 Mid-360 的 IP。
2. 配置主机网卡 IP（示例：`192.168.1.50`，掩码 `255.255.255.0`）。
3. 修改 `src/slam_pkgs/livox_ros_driver2/config/MID360_config.json`：

```json
{
  "lidar_configs": [
    { "ip": "<雷达 IP>" }
  ],
  "host_net_info": {
    "cmd_data_ip": "<主机 IP>",
    "push_msg_ip": "<主机 IP>",
    "point_data_ip": "<主机 IP>",
    "imu_data_ip": "<主机 IP>",
    "log_data_ip": "<主机 IP>"
  }
}
```

### 5.2 建图与地图转换

```bash
source install/setup.bash
ros2 launch fast_lio slam.launch.py
# 或
bash slam.bash
```

控制机器人/雷达移动建图，完成后调用保存服务：

```bash
ros2 service call /save_map std_srvs/srv/Trigger
```

将生成的 `test.pcd` 放到 `src/r2n_bringup_pkg/pcds/test.pcd`，然后生成 2D 地图：

```bash
python3 src/slam_pkgs/map_trans_pkg/scripts/pcd2pgm_simplify.py
```

脚本会生成 `src/r2n_bringup_pkg/maps/test.pgm` 与 `test.yaml`。

> `PCD`、`PGM`、`YAML` 地图文件不纳入版本管理，需要按上述流程本地生成。

### 5.3 独立重定位

```bash
ros2 launch map_trans_pkg relocalization.launch.py
```

该 launch 启动 Livox 驱动、FAST-LIO、`relocalization_node` 与 RViz。重定位节点参数在 launch 中配置：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `global_map_path` | `<pcds>/test.pcd` | 先验 PCD 地图 |
| `map_voxel_size` | 0.3 | 地图降采样体素 |
| `scan_voxel_size` | 0.1 | 实时扫描降采样体素 |
| `localization_freq` | 1.0 | 重定位频率（Hz） |
| `localization_threshold` | 0.8 | ICP fitness 阈值 |
| `fov_far` | 30.0 | 参与匹配的最远距离（m） |
| `tf_pub_freq` | 50.0 | TF 发布频率（Hz） |

### 5.4 导航

```bash
source install/setup.bash
ros2 launch r2n_bringup_pkg nav.launch.py
# 或
bash nav.bash
```

启动后在 RViz 中设置 `2D Goal Pose`，检查：

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /scan
```

`nav.launch.py` 会依次启动：Livox 驱动、FAST-LIO、重定位、Nav2、点云转 scan、地图发布、CAN 节点与 RViz。

### 5.5 感知与任务

YOLOv8 检测（权重需自行提供）：

```bash
ros2 run map_trans_pkg yolo_detector.py --ros-args \
  -p model_path:=/path/to/v5.pt \
  -p image_topic:=/image_raw \
  -p bbox_topic:=/yolo_bboxes \
  -p device:=cpu
```

视觉-激光融合定位（建议订阅去畸变点云）：

```bash
ros2 run map_trans_pkg cube_locator.py --ros-args \
  -r /livox/lidar:=/cloud_registered
```

输出话题：`/detected_cube_pose`，格式为 `[class_id, x, y, z, confidence]`。

点云聚类与跟踪：

```bash
ros2 run map_trans_pkg dbscan_clustering.py
```

输出话题：`/pointcloud_clusters`（MarkerArray）、`/filtered_pointcloud`。

九宫格状态识别：

```bash
ros2 run map_trans_pkg grid_status_manager.py --ros-args \
  -r /yolo_bboxes_front:=/yolo_bboxes
```

输出话题：`/grid_item_status`（9 维 `Int32MultiArray`）。

任务决策：

```bash
ros2 run map_trans_pkg StepManage.py
ros2 run map_trans_pkg Step_monitor.py
```

输出话题：`/final_step_mission`（12 维任务数组）。

### 5.6 CAN 底盘

1. 根据下位机协议修改 `src/can_serial_pkg/include/data_encoder.h`。
2. 插入 USB-CAN，确认端口：`ls /dev/ttyUSB*`。
3. 修改 `src/r2n_bringup_pkg/launch/nav.launch.py` 中的 `serial_port` 与 `baud_rate`。
4. 启动导航后检查 `ros2 topic echo /msg_to_can` 与 `/msg_from_can`。

通信帧格式、量化范围见 `src/can_serial_pkg/include/can_coder.hpp` 与 `data_encoder.h`。

## 6. TF 树与话题流

```
map ──(relocalization_node)──> camera_init ──(FAST-LIO)──> body ──(pointcloud2_to_laserscan)──> body_2d
```

主要话题：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/livox/lidar` | `PointCloud2` | 原始雷达点云 |
| `/livox/imu` | `Imu` | 雷达内置 IMU |
| `/cloud_registered` | `PointCloud2` | FAST-LIO 去畸变点云 |
| `/Odometry` | `Odometry` | FAST-LIO 里程计 |
| `/map_to_odom` | `Odometry` | 重定位修正量 |
| `/localization` | `Odometry` | `map` 系定位 |
| `/scan` | `LaserScan` | 2D 导航扫描 |
| `/map` | `OccupancyGrid` | 先验 2D 地图 |
| `/cmd_vel` | `Twist` | Nav2 速度指令 |
| `/msg_to_can` | `can_msgs/Frame` | 编码后的 CAN 帧 |
| `/image_raw` | `Image` | 相机图像 |
| `/yolo_bboxes` | `Float32MultiArray` | `[cx, cy, w, h, conf, cls]` |
| `/detected_cube_pose` | `Float32MultiArray` | 目标 3D 坐标 |
| `/pointcloud_clusters` | `MarkerArray` | 聚类/跟踪可视化 |
| `/grid_item_status` | `Int32MultiArray` | 九宫格状态 |
| `/final_step_mission` | `Float32MultiArray` | 台阶任务数组 |

## 7. 不包含的内容

以下文件因体积或版权原因不纳入仓库，需要本地准备：

- `src/r2n_bringup_pkg/pcds/*.pcd`：建图生成的先验点云
- `src/r2n_bringup_pkg/maps/*.pgm` / `*.yaml`：由 PCD 转换生成
- `rosbag2_*`：调试用 rosbag
- YOLO 权重（`*.pt` / `*.onnx` / `*.engine`）
- `FAST-LIVO2`、`FAST-Calib-ROS2`、`rpg_vikit`：可选第三方源码

## 8. 二次开发内容

相对原仓库，本版本主要补充/修改了：

- `map_trans_pkg/src/relocalization.cpp`：重定位节点的线程安全重构，使用细粒度锁与 `shared_ptr` 局部快照，将 ICP 计算移出临界区。
- `map_trans_pkg/scripts/yolo_detector.py`：YOLOv8 检测节点，支持模型路径、图像/输出话题、置信度、输入尺寸与设备参数化。
- `map_trans_pkg/scripts/cube_locator.py`：LiDAR-相机投影融合，bbox 内点云中值 + 滑动平均输出 3D 坐标。
- `map_trans_pkg/scripts/dbscan_clustering.py`：DBSCAN 聚类 + 贪心匹配/EMA/生命周期跟踪。
- `map_trans_pkg/scripts/grid_status_manager.py`：RANSAC 平面拟合 + 3×3 网格占用识别。
- `map_trans_pkg/scripts/StepManage.py` / `Step_monitor.py`：台阶任务匹配与观测点触发。
- `map_trans_pkg/launch/relocalization.launch.py`：独立重定位启动文件。
- `map_trans_pkg/CMakeLists.txt`：新增感知脚本与 `planb_node` 的安装配置。

## 9. 致谢

- 基础框架：[CeasarSmj/nav_ros2_ws](https://gitee.com/CeasarSmj/nav_ros2_ws)
- FAST-LIO2：HKU-MARS
- Livox ROS Driver 2：Livox-SDK
- Nav2：ROS Navigation2 社区
- YOLOv8：Ultralytics
