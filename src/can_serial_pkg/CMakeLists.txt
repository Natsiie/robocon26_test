cmake_minimum_required(VERSION 3.8)
project(can_serial_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)

# 查找 libserialport（系统库）
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIALPORT REQUIRED libserialport)

add_executable(can_serial_node src/can_serial_node.cpp)

target_include_directories(can_serial_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LIBSERIALPORT_INCLUDE_DIRS})

target_link_libraries(can_serial_node
  ${LIBSERIALPORT_LIBRARIES})

ament_target_dependencies(can_serial_node
  rclcpp
  can_msgs)

install(TARGETS
  can_serial_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()