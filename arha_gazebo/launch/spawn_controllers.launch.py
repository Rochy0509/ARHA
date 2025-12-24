from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file to spawn ros2_control controllers for ARHA robot.
    Controllers are spawned in sequence to ensure proper initialization.
    """
    
    # Spawn joint_state_broadcaster first
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen",
    )
    
    # Spawn left_arm_controller after joint_state_broadcaster
    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_arm_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen",
    )
    
    # Spawn right_arm_controller after left_arm_controller
    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_arm_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen",
    )
    
    # Spawn left_gripper_controller
    left_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_gripper_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen",
    )
    
    # Spawn right_gripper_controller
    right_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_gripper_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen",
    )
    
    # Delay spawning to ensure Gazebo is ready
    delayed_joint_state_broadcaster = TimerAction(
        period=2.0,
        actions=[joint_state_broadcaster_spawner]
    )
    
    # Chain controller spawning
    delayed_left_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[left_arm_controller_spawner],
        )
    )
    
    delayed_right_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=left_arm_controller_spawner,
            on_exit=[right_arm_controller_spawner],
        )
    )
    
    delayed_left_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=right_arm_controller_spawner,
            on_exit=[left_gripper_controller_spawner],
        )
    )
    
    delayed_right_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=left_gripper_controller_spawner,
            on_exit=[right_gripper_controller_spawner],
        )
    )
    
    return LaunchDescription([
        delayed_joint_state_broadcaster,
        delayed_left_arm_controller,
        delayed_right_arm_controller,
        delayed_left_gripper_controller,
        delayed_right_gripper_controller,
    ])