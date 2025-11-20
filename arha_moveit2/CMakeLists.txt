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
    PROGRAMS scripts/static_left_joint_state_publisher.py scripts/zero_right_arm_motors.py
    DESTINATION lib/${PROJECT_NAME})
endif()
