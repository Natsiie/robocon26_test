cmake_minimum_required(VERSION 3.8)
project(map_trans_pkg)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 禁用PCL的预编译头以减少警告
set(PCL_NO_PRECOMPILE ON CACHE BOOL "Disable PCL precompiled headers")
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ROS2核心
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# ROS2消息
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# TF2
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)  # <--- [修复] 补上了这个关键依赖

# PCL相关
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(pcl_conversions REQUIRED)
find_package(laser_geometry REQUIRED)

# 其他
find_package(OpenCV REQUIRED)
find_package(Open3D REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables from the package directory
install(DIRECTORY
  ${PROJECT_NAME}/
  DESTINATION lib/${PROJECT_NAME}
  PATTERN "*.py" PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

# Install scripts
install(DIRECTORY
  scripts/
  DESTINATION lib/${PROJECT_NAME}
  PATTERN "*.py" PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# pointcloud2_to_laserscan
add_executable(pointcloud2_to_laserscan src/pointcloud2_to_laserscan.cpp)
ament_target_dependencies(pointcloud2_to_laserscan
  rclcpp
  sensor_msgs
  pcl_conversions
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
)
target_link_libraries(pointcloud2_to_laserscan
  ${PCL_LIBRARIES}
  laser_geometry::laser_geometry
)
install(TARGETS pointcloud2_to_laserscan
  DESTINATION lib/${PROJECT_NAME}
)

# pointcloud2laserscan_elv
add_executable(pointcloud2laserscan_elv src/pointcloud2_to_laserscan_elv.cpp)
ament_target_dependencies(pointcloud2laserscan_elv
  rclcpp
  sensor_msgs
  nav_msgs
  pcl_conversions
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
)
target_link_libraries(pointcloud2laserscan_elv
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)
install(TARGETS pointcloud2laserscan_elv
  DESTINATION lib/${PROJECT_NAME}
)

# relocalization_node
add_executable(relocalization_node src/relocalization.cpp)
ament_target_dependencies(relocalization_node
  rclcpp
  sensor_msgs
  nav_msgs
  pcl_conversions
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  tf2_eigen  # <--- [修复] 补上了这个关键依赖
)
target_include_directories(relocalization_node PRIVATE
  include
  ${Open3D_INCLUDE_DIRS}
)
target_link_libraries(relocalization_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${Open3D_LIBRARIES}
)
install(TARGETS relocalization_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装Python可执行节点
install(PROGRAMS
  map_trans_pkg/map_publisher_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME map_publisher_node
)

# 安装 Python 脚本到 lib/map_trans_pkg 目录下，使其成为可执行节点
install(PROGRAMS
  scripts/pose_to_serial.py
  scripts/transform_fusion.py
  scripts/global_localization.py  # <--- [新增] 必须加上这一行
  scripts/publish_initial_pose.py
  scripts/dbscan_clustering.py
  scripts/yolo_detector.py       # <--- 添加这一行
  scripts/cube_locator.py
  scripts/grid_status_manager.py
  scripts/StepManage.py
  scripts/Step_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  map_trans_pkg/pcd_publisher_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pcd_publisher_node
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# planb_node (PointCloudFilter)
add_executable(planb_node src/planb.cpp)
ament_target_dependencies(planb_node
  rclcpp
  sensor_msgs
  std_msgs
  pcl_conversions
)
target_link_libraries(planb_node
  ${PCL_LIBRARIES}
)
# 注意：你的 planb.cpp 里没有直接 include OpenCV 或 Open3D，
# 所以这里只需要链接 PCL 库即可。如果未来加了相关代码，记得在这里补上。

install(TARGETS planb_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
