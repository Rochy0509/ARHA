from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get package paths
    arha_gazebo_path = get_package_share_directory('arha_gazebo')
    xacro_file = os.path.join(arha_gazebo_path, 'urdf', 'arha_with_grippers.urdf.xacro')
    rviz_config = os.path.join(arha_gazebo_path, 'rviz', 'visual_rviz.rviz')
    
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    
    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )
    
    # RViz with saved config
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        rviz
    ])