cmake_minimum_required(VERSION 3.8)
project(image_binarizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Node library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/image_binarizer_node.cpp
)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp sensor_msgs cv_bridge OpenCV
)

# Main executable
ament_auto_add_executable(${PROJECT_NAME}_main
  src/image_binarizer_main.cpp
)
target_link_libraries(${PROJECT_NAME}_main ${PROJECT_NAME}_node)
ament_target_dependencies(${PROJECT_NAME}_main rclcpp)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
