import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    arha_gazebo_dir = get_package_share_directory('arha_gazebo')

    # Launch arguments
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz2 for visualization',
    )

    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value='0.035',
        description='Size of ArUco markers in meters',
    )

    # Include Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(arha_gazebo_dir, 'launch', 'arha_gazebo.launch.py')
        ),
        launch_arguments={
            'launch_rviz': LaunchConfiguration('launch_rviz'),
        }.items(),
    )

    # ArUco detector node
    aruco_detector_node = Node(
        package='arha_gazebo',
        executable='aruco_detector_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'marker_size': LaunchConfiguration('marker_size'),
                'camera_topic': '/camera_chest/camera_chest/image_raw',
                'camera_info_topic': '/camera_chest/camera_chest/camera_info',
                'depth_topic': '/camera_chest/depth/points',
                'target_frame': 'base_link',
                'publish_debug_images': False,
            }
        ],
    )

    return LaunchDescription([
        launch_rviz_arg,
        marker_size_arg,
        gazebo_launch,
        aruco_detector_node,
    ])
