cmake_minimum_required(VERSION 3.8)
project(arha_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(angles REQUIRED)

add_library(arha_controllers SHARED
  src/joint_impedance_controller.cpp
)
target_include_directories(arha_controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(arha_controllers
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  realtime_tools::realtime_tools
  angles::angles
)

# Export the controller plugin
pluginlib_export_plugin_description_file(controller_interface arha_controllers_plugins.xml)

install(
  TARGETS arha_controllers
  EXPORT export_arha_controllers
  DESTINATION lib
)

ament_export_libraries(arha_controllers)
ament_export_targets(export_arha_controllers)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_lint_auto_find_test_dependencies_FAST_RESOLVE ON)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
