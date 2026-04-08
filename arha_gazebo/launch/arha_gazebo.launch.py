import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction, SetEnvironmentVariable, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # Get package paths
    arha_gazebo_path = get_package_share_directory('arha_gazebo')
    gazebo_ros_path = get_package_share_directory('gazebo_ros')
    arha_description_share = get_package_share_directory('arha_description')
    realsense_description_share = get_package_share_directory('realsense2_description')
    pincopen_share = get_package_share_directory('pincopen')
    head_share = get_package_share_directory('head')

    # Launch Arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(arha_gazebo_path, 'worlds', 'arha_manipulation.world'),
        description='Path to Gazebo world file'
    )
    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='0.0', description='Robot spawn x')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='0.0', description='Robot spawn y')
    spawn_z_arg = DeclareLaunchArgument('spawn_z', default_value='0.0', description='Robot spawn z')
    spawn_roll_arg = DeclareLaunchArgument('spawn_roll', default_value='0.0', description='Robot spawn roll')
    spawn_pitch_arg = DeclareLaunchArgument('spawn_pitch', default_value='0.0', description='Robot spawn pitch')
    spawn_yaw_arg = DeclareLaunchArgument('spawn_yaw', default_value='0.0', description='Robot spawn yaw')

    # Paths to files
    world_file = LaunchConfiguration('world')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_roll = LaunchConfiguration('spawn_roll')
    spawn_pitch = LaunchConfiguration('spawn_pitch')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    xacro_file = os.path.join(arha_gazebo_path, 'urdf', 'arha_with_grippers.urdf.xacro')
    models_path = os.path.join(arha_gazebo_path, 'models')

    model_path_list = [models_path]
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH')
    if existing_model_path:
        model_path_list.append(existing_model_path)
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=':'.join(model_path_list)
    )

    # Robot description - resolve package:// URIs to absolute paths for Gazebo Classic
    # Using python processing for better error visibility and reliability
    import xacro
    import re
    doc = xacro.process_file(xacro_file)
    robot_description_raw = doc.toxml()

    # Minify XML to bypass the strict 64KB Foxy/Humble ROS2 `--param` parser limit
    robot_description_min = re.sub(r'>\s+<', '><', robot_description_raw)
    robot_description_min = re.sub(r'<!--.*?-->', '', robot_description_min, flags=re.DOTALL)

    # Apply mesh replacements (prepend file:// so parser accepts them as absolute paths)
    robot_description_processed = robot_description_min.replace('package://arha_description/', 'file://' + arha_description_share + '/')
    robot_description_processed = robot_description_processed.replace('package://realsense2_description/', 'file://' + realsense_description_share + '/')
    robot_description_processed = robot_description_processed.replace('package://pincopen/', 'file://' + pincopen_share + '/')
    robot_description_processed = robot_description_processed.replace('package://head/', 'file://' + head_share + '/')

    robot_description = {'robot_description': robot_description_processed}

    # Robot state publisher with use_sim_time
    # Note: robot_description is set on the node namespace only to avoid conflicts
    # with gazebo_ros2_control which cannot handle XML content as parameter overrides
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot_state_publisher',
        parameters=[
            robot_description,
            {'use_sim_time': True}
        ],
        output='screen',
        remappings=[
            ('/robot_state_publisher/robot_description', '/robot_description')
        ]
    )

    # Start Gazebo Classic server
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_path, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'false',
            'pause': 'false'
        }.items()
    )

    # Start Gazebo Classic client
    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_path, 'launch', 'gzclient.launch.py')
        )
    )

    # Spawn robot (delayed to let Gazebo start)
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'arha',
                    '-topic', '/robot_description',
                    '-x', spawn_x,
                    '-y', spawn_y,
                    '-z', spawn_z,
                    '-R', spawn_roll,
                    '-P', spawn_pitch,
                    '-Y', spawn_yaw
                ],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )

    # Controller config file path
    controller_config = os.path.join(arha_gazebo_path, 'config', 'gazebo_controllers.yaml')

    # Load joint state broadcaster first (5 seconds after spawn)
    load_joint_state_controller = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )

    # Load arm controllers (7 seconds)
    load_left_arm_controller = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['left_arm_controller'],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )

    load_right_arm_controller = TimerAction(
        period=7.5,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['right_arm_controller'],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )

    load_left_gripper_controller = TimerAction(
        period=8.5,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'left_gripper_controller',
                    '--controller-manager', '/controller_manager',
                    '--controller-manager-timeout', '120'
                ],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )

    load_right_gripper_controller = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'right_gripper_controller',
                    '--controller-manager', '/controller_manager',
                    '--controller-manager-timeout', '120'
                ],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )


    return LaunchDescription([
        world_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        spawn_roll_arg,
        spawn_pitch_arg,
        spawn_yaw_arg,
        gazebo_model_path,
        robot_state_publisher,
        start_gazebo_server,
        start_gazebo_client,
        spawn_robot,
        load_joint_state_controller,
        load_left_arm_controller,
        load_right_arm_controller,
        load_left_gripper_controller,
        load_right_gripper_controller,
    ])
