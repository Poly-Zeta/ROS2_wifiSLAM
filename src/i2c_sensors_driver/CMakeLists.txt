cmake_minimum_required(VERSION 3.8)
project(i2c_sensors_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(rosidl_default_generators REQUIRED)

ament_auto_find_build_dependencies()

# メッセージ生成
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Esp32TimeSync.msg"
  DEPENDENCIES std_msgs
)

# ノードクラスをライブラリ化
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/i2c_sensors_driver_node.cpp
  include/i2c_sensors_driver/i2c_sensors_driver.hpp
)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp std_msgs sensor_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(${PROJECT_NAME}_node
  ${cpp_typesupport_target}
)

# rosidl_target_interfaces(${PROJECT_NAME}_node
#   ${PROJECT_NAME} "rosidl_typesupport_cpp"
# )

# main 実行ファイル
ament_auto_add_executable(${PROJECT_NAME}_main
  src/i2c_sensors_driver_main.cpp
)
target_link_libraries(${PROJECT_NAME}_main ${PROJECT_NAME}_node)
ament_target_dependencies(${PROJECT_NAME}_main
  rclcpp
)
# rosidl_get_typesupport_target(cpp_typesupport_target
#   ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(${PROJECT_NAME}_main
  ${cpp_typesupport_target}
)

# rosidl_target_interfaces(${PROJECT_NAME}_main
#   ${PROJECT_NAME} "rosidl_typesupport_cpp"
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_auto_package()

# cmake_minimum_required(VERSION 3.8)
# project(i2c_sensors_driver)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find_package(ament_cmake_auto REQUIRED)
# find_package(rosidl_default_generators REQUIRED)

# ament_auto_find_build_dependencies()

# # メッセージ生成
# rosidl_generate_interfaces(${PROJECT_NAME}_interfaces
#   "msg/Esp32TimeSync.msg"
#   DEPENDENCIES std_msgs
# )

# # 実行ファイル（衝突回避のため別名）
# ament_auto_add_executable(${PROJECT_NAME}_node
#   src/i2c_sensors_driver_node.cpp
#   src/i2c_sensors_driver_main.cpp
# )

# # ライブラリ
# ament_auto_add_library(${PROJECT_NAME}_lib SHARED
#   src/i2c_sensors_driver_node.cpp
#   include/i2c_sensors_driver/i2c_sensors_driver.hpp
# )

# # 依存関係
# ament_target_dependencies(${PROJECT_NAME}_node
#   rclcpp std_msgs sensor_msgs
# )

# # メッセージ型サポートをリンク
# rosidl_target_interfaces(${PROJECT_NAME}_node
#   ${PROJECT_NAME}_interfaces "rosidl_typesupport_cpp")
# rosidl_target_interfaces(${PROJECT_NAME}_lib
#   ${PROJECT_NAME}_interfaces "rosidl_typesupport_cpp")

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_export_dependencies(rosidl_default_runtime)
# ament_auto_package()

# # cmake_minimum_required(VERSION 3.8)
# # project(i2c_sensors_driver)

# # if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
# #   add_compile_options(-Wall -Wextra -Wpedantic)
# # endif()

# # # find_package(rclcpp REQUIRED)
# # # find_package(std_msgs REQUIRED)
# # # find_package(sensor_msgs REQUIRED)
# # find_package(ament_cmake_auto REQUIRED)
# # find_package(rosidl_default_generators REQUIRED)
# # # uncomment the following section in order to fill in
# # # further dependencies manually.
# # # find_package(<dependency> REQUIRED)
# # ament_auto_find_build_dependencies()

# # rosidl_generate_interfaces(${PROJECT_NAME}
# #   "msg/Esp32TimeSync.msg"
# #   DEPENDENCIES std_msgs
# # )

# # ament_auto_add_executable(i2c_sensors_driver
# #   src/i2c_sensors_driver_node.cpp
# #   src/i2c_sensors_driver_main.cpp
# # )

# # ament_auto_add_library(i2c_sensors_driver_node SHARED
# #   src/i2c_sensors_driver_node.cpp
# #   src/i2c_sensors_driver_main.cpp
# #   include/i2c_sensors_driver/i2c_sensors_driver.hpp
# # )

# # ament_target_dependencies(i2c_sensors_driver)# rclcpp std_msgs sensor_msgs)

# # rosidl_target_interfaces(i2c_sensors_driver
# #   ${PROJECT_NAME} "rosidl_typesupport_cpp")
# # rosidl_target_interfaces(i2c_sensors_driver_node
# #   ${PROJECT_NAME} "rosidl_typesupport_cpp")

# # if(BUILD_TESTING)
# #   find_package(ament_lint_auto REQUIRED)
# #   ament_lint_auto_find_test_dependencies()
# # endif()

# # ament_export_dependencies(rosidl_default_runtime)
# # ament_auto_package()
