import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    arha_gazebo_dir = get_package_share_directory('arha_gazebo')

    xacro_file = os.path.join(arha_gazebo_dir, 'urdf', 'arha_with_grippers.urdf.xacro')
    moveit_config = (
        MoveItConfigsBuilder("arha_description", package_name="arha_moveit2")
        .robot_description(file_path=xacro_file)
        .to_moveit_configs()
    )

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz2 alongside MoveIt',
    )

    gazebo_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(arha_gazebo_dir, 'launch', 'arha_gazebo_moveit.launch.py')
        ),
        launch_arguments={
            'launch_rviz': LaunchConfiguration('launch_rviz'),
        }.items(),
    )

    # Pick and place node - Octomap is configured in arha_gazebo_moveit.launch.py
    pick_and_place_node = Node(
        package='arha_gazebo',
        executable='aruco_pick_and_place_node',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([
        launch_rviz_arg,
        gazebo_moveit_launch,
        pick_and_place_node,
    ])
