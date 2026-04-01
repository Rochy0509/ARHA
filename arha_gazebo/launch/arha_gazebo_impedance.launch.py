import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    arha_gazebo_share = get_package_share_directory('arha_gazebo')

    # Launch Arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(arha_gazebo_share, 'worlds', 'arha_manipulation.world'),
        description='Path to Gazebo world file'
    )
    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='0.0', description='Robot spawn x')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='0.0', description='Robot spawn y')
    spawn_z_arg = DeclareLaunchArgument('spawn_z', default_value='0.0', description='Robot spawn z')
    spawn_roll_arg = DeclareLaunchArgument('spawn_roll', default_value='0.0', description='Robot spawn roll')
    spawn_pitch_arg = DeclareLaunchArgument('spawn_pitch', default_value='0.0', description='Robot spawn pitch')
    spawn_yaw_arg = DeclareLaunchArgument('spawn_yaw', default_value='0.0', description='Robot spawn yaw')

    # 1. Start the base Gazebo simulation
    base_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(arha_gazebo_share, 'launch', 'arha_gazebo.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_z': LaunchConfiguration('spawn_z'),
            'spawn_roll': LaunchConfiguration('spawn_roll'),
            'spawn_pitch': LaunchConfiguration('spawn_pitch'),
            'spawn_yaw': LaunchConfiguration('spawn_yaw'),
        }.items()
    )
    
    return LaunchDescription([
        world_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        spawn_roll_arg,
        spawn_pitch_arg,
        spawn_yaw_arg,
        base_sim,
    ])

