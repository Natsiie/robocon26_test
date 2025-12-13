cmake_minimum_required(VERSION 3.8)
project(virtual_lidar_driver_pkg)

# 忽略 Catkin 特定的变量，因为这是 Ament 包
if(DEFINED CATKIN_INSTALL_INTO_PREFIX_ROOT)
  set(CATKIN_INSTALL_INTO_PREFIX_ROOT ${CATKIN_INSTALL_INTO_PREFIX_ROOT} CACHE INTERNAL "Catkin variable - unused in Ament")
endif()

if(DEFINED CATKIN_SYMLINK_INSTALL)
  set(CATKIN_SYMLINK_INSTALL ${CATKIN_SYMLINK_INSTALL} CACHE INTERNAL "Catkin variable - unused in Ament")
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置CMP0074策略以避免警告
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(OpenMP)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# 添加所有源文件到可执行文件
add_executable(virtual_lidar_driver_node 
  src/virtual_lidar_driver_node.cpp
  src/point_cloud_simulator.cpp
  src/imu_data_generate.cpp)

# 添加新的可执行文件
add_executable(pc2_to_livoxcloud_node
  src/pc2_to_livoxcloud.cpp)

# 为虚拟激光雷达节点添加依赖
ament_target_dependencies(virtual_lidar_driver_node
  rclcpp
  sensor_msgs
  tf2     
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  pcl_conversions
  ament_index_cpp
  livox_ros_driver2
)

# 为新可执行文件添加依赖
ament_target_dependencies(pc2_to_livoxcloud_node
  rclcpp
  sensor_msgs
  livox_ros_driver2)

target_link_libraries(virtual_lidar_driver_node
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
  tf2::tf2  
  ${tf2_geometry_msgs_LIBRARIES}
)

# 安装
install(TARGETS virtual_lidar_driver_node
        pc2_to_livoxcloud_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch scripts/
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.py" PERMISSIONS OWNER_EXECUTE OWNER_READ OWNER_WRITE
                              GROUP_EXECUTE GROUP_READ
                              WORLD_EXECUTE WORLD_READ
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
ament_package()