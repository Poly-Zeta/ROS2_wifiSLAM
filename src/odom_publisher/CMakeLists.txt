cmake_minimum_required(VERSION 3.8)
project(odom_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  include
)

# 既存ノード
add_executable(odom_publisher_main
  src/odom_publisher_main.cpp
  src/odom_publisher_node.cpp
)
ament_target_dependencies(odom_publisher_main
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
)

# 新規ノード static_tf_from_yaml
add_executable(static_tf_from_yaml
  src/static_tf_from_yaml_node.cpp
  src/static_tf_from_yaml_main.cpp
)
ament_target_dependencies(static_tf_from_yaml
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
)

# インストール設定
install(TARGETS
  odom_publisher_main
  static_tf_from_yaml
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()