from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Include the base Gazebo + MoveIt launch
    arha_gazebo_share = get_package_share_directory('arha_gazebo')
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(arha_gazebo_share, 'launch', 'arha_gazebo_moveit.launch.py')
        )
    )
    
    # Workspace test node
    test_node = Node(
        package='arha_gazebo',
        executable='arm_workspace_test',
        output='screen',
    )
    
    return LaunchDescription([
        base_launch,
        test_node,
    ])
