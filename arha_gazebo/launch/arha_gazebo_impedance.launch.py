import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    arha_gazebo_share = get_package_share_directory('arha_gazebo')

    # 1. Start the base Gazebo simulation
    base_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(arha_gazebo_share, 'launch', 'arha_gazebo.launch.py')
        )
    )
    
    # 2. Wait for simulation to settle then spawn impedance controllers
    # Note: This will run alongside the default arm controllers. 
    # The user should deactivate the default ones manually or via scripts.
    spawn_impedance = TimerAction(
        period=15.0, # Wait longer than the default spawners (which stop at 8.5s)
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(arha_gazebo_share, 'launch', 'spawn_impedance_controllers.launch.py')
                )
            )
        ]
    )
    
    return LaunchDescription([
        base_sim,
        spawn_impedance,
    ])

