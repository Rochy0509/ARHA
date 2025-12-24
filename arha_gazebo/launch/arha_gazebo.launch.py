import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction, SetEnvironmentVariable, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # Get package paths
    arha_gazebo_path = get_package_share_directory('arha_gazebo')
    gazebo_ros_path = get_package_share_directory('gazebo_ros')
    arha_description_share = get_package_share_directory('arha_description')
    robotiq_description_share = get_package_share_directory('robotiq_hande_description')
    realsense_description_share = get_package_share_directory('realsense2_description')

    # Paths to files
    world_file = os.path.join(arha_gazebo_path, 'worlds', 'arha_training.world')
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

    # Robot description - strip XML header/comments and resolve package:// URIs to absolute paths
    # See: https://github.com/ros-controls/gazebo_ros2_control/issues/295
    mesh_replacements = (
        f"sed -e 's#package://arha_description/#{arha_description_share}/#g' "
        f"-e 's#package://robotiq_hande_description/#{robotiq_description_share}/#g' "
        f"-e 's#package://realsense2_description/#{realsense_description_share}/#g'"
    )
    robot_description = ParameterValue(
        Command([f'bash -c "xacro {xacro_file} | '
                 f'perl -0pe \'s/<\\?xml.*?\\?>\\s*//; s/<!--.*?-->//sg; s/\\A\\s+//\' | '
                 f'{mesh_replacements}"']),
        value_type=str
    )

    # Robot state publisher with use_sim_time
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True}
        ],
        output='screen'
    )

    # Start Gazebo Classic server
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_path, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'true',
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
                    '-x', '0',
                    '-y', '0',
                    '-z', '0.02'
                ],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )

    # Load joint state broadcaster first (5 seconds after spawn)
    load_joint_state_controller = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
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

    # Load gripper controllers (8 seconds)
    load_left_gripper_controller = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['left_gripper_controller'],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )

    load_right_gripper_controller = TimerAction(
        period=8.5,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['right_gripper_controller'],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
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
