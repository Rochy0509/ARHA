cmake_minimum_required(VERSION 3.8)
project(arha_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(arha_description REQUIRED)
find_package(robotiq_hande_description REQUIRED)
find_package(realsense2_description REQUIRED)

# Gazebo Classic dependencies
find_package(gazebo_ros REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(gazebo_plugins REQUIRED)

# ROS2 core
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)

# Controllers
find_package(controller_manager REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(position_controllers REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(OpenCV REQUIRED)

install(
  DIRECTORY urdf config worlds rviz models
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

install(
  PROGRAMS scripts/aruco_pick_and_place.py
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(aruco_pick_and_place_node src/aruco_pick_and_place_node.cpp)
target_include_directories(aruco_pick_and_place_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(aruco_pick_and_place_node
  rclcpp
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  shape_msgs
  cv_bridge
  tf2_ros
  tf2_geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  OpenCV
)
target_link_libraries(aruco_pick_and_place_node
  ${OpenCV_LIBRARIES}
)
install(
  TARGETS aruco_pick_and_place_node
  DESTINATION lib/${PROJECT_NAME}
)

# ArUco detector node (simple chest camera detection)
add_executable(aruco_detector_node src/aruco_detector_node.cpp)
target_include_directories(aruco_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(aruco_detector_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  OpenCV
)
target_link_libraries(aruco_detector_node
  ${OpenCV_LIBRARIES}
)
install(
  TARGETS aruco_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

# Arm workspace test node
add_executable(arm_workspace_test src/arm_workspace_test.cpp)
target_include_directories(arm_workspace_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(arm_workspace_test PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  arm_workspace_test
  moveit_ros_planning_interface
  rclcpp
)
install(
  TARGETS arm_workspace_test
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
