cmake_minimum_required(VERSION 3.8)
project(rmw_stress_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

# Image subscriber composable node
add_library(image_subscriber_component SHARED
  src/image_subscriber_node.cpp
)

target_include_directories(image_subscriber_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(image_subscriber_component
  rclcpp
  rclcpp_components
  sensor_msgs
)

rclcpp_components_register_nodes(image_subscriber_component "stress_test::ImageSubscriberNode")

# Install
install(TARGETS image_subscriber_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
