cmake_minimum_required(VERSION 3.8)
project(line_follower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# OpenCV は ament_target_dependencies に入れずCMake の find_package で明示
find_package(OpenCV REQUIRED)

# ノードクラスをライブラリ化
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/line_follower_node.cpp
)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ROS系だけを ament_target_dependencies に載せる
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
)

# OpenCV は link と include を明示
target_link_libraries(${PROJECT_NAME}_node
  ${OpenCV_LIBRARIES}
)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

# main 実行ファイル
ament_auto_add_executable(${PROJECT_NAME}_main
  src/line_follower_main.cpp
)

target_link_libraries(${PROJECT_NAME}_main
  ${PROJECT_NAME}_node
)

ament_target_dependencies(${PROJECT_NAME}_main
  rclcpp
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()

