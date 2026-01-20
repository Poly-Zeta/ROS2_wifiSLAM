cmake_minimum_required(VERSION 3.8)
project(uart_wifi_analyzer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

include_directories(include)

# Node library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/uart_wifi_analyzer_node.cpp
  include/uart_wifi_analyzer/uart_wifi_analyzer.hpp
)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs)

# Main executable
ament_auto_add_executable(${PROJECT_NAME}_main
  src/uart_wifi_analyzer_main.cpp
)
target_link_libraries(${PROJECT_NAME}_main ${PROJECT_NAME}_node)
ament_target_dependencies(${PROJECT_NAME}_main rclcpp std_msgs)

# Install launch
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()