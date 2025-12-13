cmake_minimum_required(VERSION 3.8)
project(virtual_robot_sim_pkg)

# 检查编译器支持C++14
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# 安装Python模块
ament_python_install_package(${PROJECT_NAME})

# 安装Python可执行节点
install(PROGRAMS
  virtual_robot_sim_pkg/virtual_robot_sim_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME virtual_robot_sim_node
)

# 安装启动文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装 rivz 配置
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# 导出依赖
ament_export_dependencies(rclpy)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2_ros)

# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()