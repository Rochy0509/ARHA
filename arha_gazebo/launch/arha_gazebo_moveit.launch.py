import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    # Get package directories
    arha_gazebo_dir = get_package_share_directory('arha_gazebo')
    arha_moveit2_dir = get_package_share_directory('arha_moveit2')

    # Use the same xacro file as Gazebo
    xacro_file = os.path.join(arha_gazebo_dir, 'urdf', 'arha_with_grippers.urdf.xacro')

    # Build MoveIt configs with the Gazebo xacro file and sensors
    moveit_config = (
        MoveItConfigsBuilder("arha_description", package_name="arha_moveit2")
        .robot_description(file_path=xacro_file)
        .sensors_3d(file_path="config/sensors_3d.yaml")
        .to_moveit_configs()
    )

    # Launch Gazebo with robot
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(arha_gazebo_dir, 'launch', 'arha_gazebo.launch.py')
        ]),
    )

    # MoveIt move_group node with Octomap
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True},
            # Enable Octomap capability
            {'capabilities': 'move_group/MoveGroupCartesianPathService '
                            'move_group/MoveGroupExecuteTrajectoryAction '
                            'move_group/MoveGroupKinematicsService '
                            'move_group/MoveGroupMoveAction '
                            'move_group/MoveGroupPlanService '
                            'move_group/MoveGroupQueryPlannersService '
                            'move_group/MoveGroupStateValidationService '
                            'move_group/MoveGroupGetPlanningSceneService '
                            'move_group/ApplyPlanningSceneService '
                            'move_group/ClearOctomapService'},
            # Occupancy map parameters
            {'octomap_frame': 'base_link'},
            {'octomap_resolution': 0.02},
            {'max_range': 3.0},
        ],
    )
    
    # RViz
    launch_rviz = LaunchConfiguration('launch_rviz')
    rviz_config_file = os.path.join(arha_moveit2_dir, 'config', 'moveit.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': True},
        ],
        condition=IfCondition(launch_rviz),
    )

    nodes_to_start = [
        gazebo_launch,
        move_group_node,
        rviz_node,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Launch RViz with MoveIt configuration',
        )
    )
    
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )