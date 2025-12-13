# 激光雷达仿真节点

## 简述
用于通过从 [ pcd 获取点云数据 + 从 tf 获取位置数据 ] 的方式做点云提取和imu数据生成，从而模拟激光雷达数据。

后续这个包将会用于测试 slam 算法

基于 ros humble 实现。

## 目录结构
```
virtual_lidar_driver_pkg/
├── config/
│   └── virtual_lidar_config.yaml
├── include/
│   ├── point_cloud_simulator.hpp
│   └── imu_data_generate.hpp
├── launch/
│   └── virtual_lidar.launch.py
├── scripts/
│   └── map_generator.py
├── src/
│   ├── pc2_to_livoxcloud.cpp
│   ├── point_cloud_simulator.cpp
│   ├── imu_data_generate.cpp
│   └── virtual_lidar_driver_node.cpp
├── package.xml
├── CMakeLists.txt
├── README.md
```

## 代码功能说明
### map_generator.py [已完成]
用于从png图像生成pcd和pgm地图。
png图像需要满足特定的颜色对应特定高度的要求，然后脚本将会读取像素并解析空间布局，然后生成点云和pgm地图。
其中 pcd 文件将会被用于模拟生成激光雷达数据。

### pc2_to_livoxcloud.cpp  [已完成]
用于把 pointcloud2 格式转化成 livox_ros_driver2 里用的格式，从而模拟 mid360。
需要依赖 livox_ros_driver2

### virtual_lidar_driver_node.cpp
激光雷达仿真的核心节点，为节点的入口，
负责从参数服务器获取参数，以将点云处理、imu数据生成 部分统一调用封装，接入 ros 的通信系统。

为了程序的实现更加整洁，所有 tf获取 、通信、线程启动相关部分都放在这个代码中，其它模块需要则以参数的形式传入。

需要设置计时器用于,并再回调函数中发布数据：
- 点云数据生成 ： 获取雷达相对世界的位姿，然后利用 point_cloud_simulator 模块得到点云，同时利用这个tf更新imu模拟器得到imu数据，然后发布点云和imu数据
- IMU数据生成 ： 获取雷达相对世界的位姿，然后利用 imu_simulator 更新imu数据，然后发布

注意，为了真实性，发布的点云和IMU数据所在的坐标系都应该是 lidar_frame_id
其中 world_frame_id 到 lidar_frame_id 的变换会由外部的节点模拟移动并进行发布，这里只需要订阅即可。


还有一点需要特殊说明的是，由于需要模拟雷达行为，所以 imu 数据发布需要在两个地方进行：
1. imu_timer_callback ： 帧率为 imu_update_rate 的计时器回调，用于高频更新以得到相对精确的 imu 数据。
2. lidar_timer_callback ： 帧率为 lidar_update_rate 的计时器回调，用于和激光雷达数据一同发布，实现和雷达事件同步。
这样的话imu发布帧率回略高于 imu_update_rate ，但是为了时间同步，这是可以接受的。
同时为了时间同步，IMU 数据和 lidar 数据 的时间戳应当为 监听的 tf 的时间戳。

这个类自身需要的参数（从参数服务器获取的）包括：
  - world_frame_id [string]
  - lidar_frame_id [string]
  - imu_topic [string]
  - lidar_topic [string]
  - imu_update_rate [double]
  - lidar_update_rate [double]



### point_cloud_simulator.cpp/hpp
包含一个类负责进行点云处理，能够实现的处理为：
- 加载 PCD 文件，需要输入 pcd_path ：
  0. 支持加载 XYZ 和 XYZI 两种，但是加载完后要统一为 XYZI 格式。得到全局点云记为 world_pc，为了方便查询可以使其为有序表，例如KD-tree等等。
- 随机噪声预先生成
  0. 初始化阶段中，如果 points_noise_stderr > 0，则计算 noise_num = total_points_in_world_pc * (2/3 * pi * fov.dist_max^3)/world_pc_bound_rect_V * 0.2 ，然后生成 noise_num 个 中心为 0 ， xyz方向标准差为 points_noise_stderr 的高斯噪声矢量 noise_vec
- 获取 FOV 内的点，需要输入为 lidar_pose ：
  1. 根据 fov 在球面上做点采样,采样的点包括 dist=0 的点，以及 yaw 与 pitch 从最大到最小值插值的点（由于fov可能为360，所以只取最大最小会出问题）。得到采样点在世界系中的坐标 (x,y,z), 放到数组 sample_points 中。
  2. 统计 sample_points 中 xyz 各自的最大最小值，得到一个bound_rect 。
  3. 根据 bound_rect 从 world_pc 中 快速提取出这个立方体空间内的点，记为 fov_rect_pc 。
  4. 计算 fov_rect_pc 中的点到 lidar_pose 的距离，剔除距离超过 fov.dist_max 的点，得到 fov_pc_in_dist。
  5. 计算 world_tf 到 lidar_pose 的旋转矩阵 R_mat 和平移 t_vec , 把 fov_pc_in_dist 的点转换到 lidar_pose 的坐标系下，记为 fov_pc_in_lidar_tf 。
  6. 把 fov_pc_in_lidar_tf 中的点坐标转化为球坐标系 (x,y,z) -> (yaw,pitch,dist) ，记为 fov_pc_in_lidar_tf_sphere 。
  7. 根据 fov 的角度边界来筛选 fov_pc_in_lidar_tf_sphere 中的点，得到 fov_pc_sphere 
  8. fov_pc_sphere 转为 (x,y,z) 得到 fov_pc_in_lidar_tf_xyz
  9. 根据参数 lidar_downsample_voxel_size 来对于 fov_pc_in_lidar_tf_xyz 进行降采样，得到 fov_pc 
  10. 如果参数 points_noise_stderr>0 ，则给 fov_pc 按批次(批次为 noise_num )加入预生成的噪声 noise_vec ,如果有余数点则截取 noise_vec 中对应长度的部分加上去, 得到更新的 fov_pc 
  11. 更新 noise_vec ， 方法为删除第一项噪声矢量，然后在最后再生成一个噪声矢量，从而保证即使机器人静止也会每次扫描到的点云有不同的噪声。
  12. 返回 fov_pc 作为返回值
- 进阶处理，遮挡处理渲染再采样，需要输入 fov_pc：
  1. 根据 参数 pixel_per_deg 和 fov中的pitch和yaw范围来计算并生成一个像素矩阵，记为 pixel_mat_of_dist
  2. 把 fov_pc 转化为柱坐标系 (x,y,z) -> (yaw,z,dist) , 记为 fov_pc_cylinder
  3. 把 fov_pc_cylinder 按照 dist 升序排列，记为 fov_pc_cylinder_sorted
  4. 把 fov_pc_cylinder_sorted 按照从近到远的顺序渲染到 pixel_mat_of_dist 中，如果渲染时对应像素已经有点了则跳过，从而实现遮挡效果
  5. 为了防止稀疏点云影响效果，给 pixel_mat_of_dist 过一次 min_pool ，使得远被近点覆盖，记为 pixel_mat_of_dist_min_pool
  6. 从 pixel_mat_of_dist_min_pool 反向映射像素到柱坐标系 (yaw,z,dist) 得到 resample_pc_cylinder
  7. 把 resample_pc_cylinder 转回 (x,y,z) 格式得到 resample_pc_in_fov
  8. 其中 resample_pc_in_fov 为最终返回值


这个类需要的参数（从参数服务器获取的）包括：
  - pcd_path [string]
  - advanced_mode [bool]
  - fov_description [float] -> [dist_min, dist_max, yaw_min, yaw_max, pitch_min, pitch_max]
  - pixel_per_deg [int]
  - downsample [bool]
  - lidar_downsample_voxel_size [float]
  - points_noise_stderr [float]


### imu_data_generate.cpp/hpp
这个类主要功能是预测和更新对应节点的位置、速度、加速度数据并进行imu数据生成。

其中它的更新接口为 update_imu_data(world_to_lidar_pose)。
目前仿真环境只能通过跟踪雷达的位姿轨迹而无法直接获取加速度，所以输入为雷达在世界系的位姿。
也就是这里需要通过位姿的变化去反向推算出 imu 的姿态、加速度、角速度和角加速度。

更新的方式为改进的 EKF ，以位姿数据为观测量，位姿+速度+角速度+加速度+角加速度 为状态量，根据刚体运动模型对于更新矩阵进行建模。建模假设为减速与减加速度运动，也就是速度、角速度、加速度和角加速度项在更新矩阵中带一个很小的衰减系数（例如 0.98）。另外在建模过程中需要考虑到科里奥利力等在符合运动中的各个参数之间的影响。

由于实际仿真中机器人只会在 x,y 平面内运动，所以可以再加一个 3Dof 模型。不过由于这个模型比 6dof 情形参量更少，所以运动建模必须要更加精细。

虽然这个算法中imu数据的生成和真实imu有差距，但是显然这种基于预测的模型本身就会有噪声，所以如果紧紧是用于测试 SLAM 算法的话应该是够用的。

需要的参数包括：
  - with_3dof_model [bool]
  - acc_cov [double]
  - gyro_cov [double]
  - gravity_vector [double] (注意，注入重力矢量时注意坐标系在world_frame还是lidar_frame)

### virtual_lidar.launch.py
launch 文件，用于启动 virtual_lidar_driver_node 和 pc2_to_livoxcloud 这两个节点。其中它们的参数从 virtual_lidar_params.yaml 文件中读取。