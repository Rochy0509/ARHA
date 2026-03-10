cmake_minimum_required(VERSION 3.22)
project(arha_moveit2)

find_package(ament_cmake REQUIRED)
find_package(arha_description REQUIRED)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts")
  install(
    PROGRAMS
      scripts/static_left_joint_state_publisher.py
      scripts/zero_right_arm_motors.py
      scripts/right_arm_five_positions.py
      scripts/play_poses.py
    DESTINATION lib/${PROJECT_NAME})
endif()

find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(play_poses src/play_poses.cpp)
ament_target_dependencies(play_poses rclcpp moveit_ros_planning_interface)

add_executable(play_poses_pilz src/play_poses_pilz.cpp)
ament_target_dependencies(play_poses_pilz rclcpp rclcpp_action moveit_ros_planning_interface moveit_msgs)

install(TARGETS play_poses play_poses_pilz
  DESTINATION lib/${PROJECT_NAME}
)
