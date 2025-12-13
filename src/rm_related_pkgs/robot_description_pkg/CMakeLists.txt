cmake_minimum_required(VERSION 3.8)
project(robot_description_pkg)

# 检查编译器支持C++14
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)

# 安装启动文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装URDF文件
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# 安装RViz配置文件（如果存在）
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
  OPTIONAL
)

# 安装配置文件目录
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  OPTIONAL
)

# 导出依赖
ament_package()