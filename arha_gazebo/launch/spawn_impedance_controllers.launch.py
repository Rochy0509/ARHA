from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file to spawn arha_controllers/JointImpedanceController for ARHA robot in Gazebo.
    """
    
    # Spawn left_arm_impedance_controller
    left_arm_impedance_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_arm_impedance_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen",
    )
    
    # Spawn right_arm_impedance_controller
    right_arm_impedance_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_arm_impedance_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen",
    )
    
    # Delay spawning to ensure Gazebo is ready
    delayed_left_arm_impedance = TimerAction(
        period=2.0,
        actions=[left_arm_impedance_spawner]
    )
    
    # Chain controller spawning
    delayed_right_arm_impedance = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=left_arm_impedance_spawner,
            on_exit=[right_arm_impedance_spawner],
        )
    )
    
    return LaunchDescription([
        delayed_left_arm_impedance,
        delayed_right_arm_impedance,
    ])
