cmake_minimum_required(VERSION 3.8)
project(odrive_can_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ros2_socketcan REQUIRED)

add_library(odrive_hardware_interface SHARED src/odrive_hardware_interface.cpp src/odrive_axis.cpp src/odrive_can.cpp)

target_include_directories(odrive_hardware_interface PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

ament_target_dependencies(
  odrive_hardware_interface
  SYSTEM
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  ros2_socketcan)

pluginlib_export_plugin_description_file(hardware_interface odrive_can_driver.xml)

install(TARGETS odrive_hardware_interface DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
