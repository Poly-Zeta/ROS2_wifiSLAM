cmake_minimum_required(VERSION 3.8)
project(imu_odom_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/imu_odom_publisher_node.cpp
  include/imu_odom_publisher/imu_odom_publisher.hpp
)

ament_auto_add_executable(${PROJECT_NAME}_main
  src/imu_odom_publisher_main.cpp
)
target_link_libraries(${PROJECT_NAME}_main ${PROJECT_NAME}_node)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()