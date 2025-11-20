import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    declared_arguments = []

    # Declare arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value="moveit.rviz",
            description="RViz configuration file",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Use fake hardware (mock) or real hardware interface",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )


def launch_setup(context, *args, **kwargs):
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    # Get package directories
    moveit_config_pkg = get_package_share_directory("arha_moveit2")

    # Build MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("arha_description", package_name="arha_moveit2")
        .robot_description(
            file_path=os.path.join(
                moveit_config_pkg, "config", "arha_description_right_arm.urdf.xacro"
            ),
            mappings={"use_fake_hardware": use_fake_hardware.perform(context)}
        )
        .robot_description_semantic(
            file_path=os.path.join(moveit_config_pkg, "config", "arha_description.srdf")
        )
        .trajectory_execution(
            file_path=os.path.join(
                moveit_config_pkg, "config", "moveit_controllers_right_arm.yaml"
            )
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Extra MoveIt trajectory execution tolerances for real hardware lag
    trajectory_execution_params = os.path.join(
        moveit_config_pkg, "config", "trajectory_execution_right_arm.yaml"
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # RViz
    rviz_base = os.path.join(moveit_config_pkg, "config")
    rviz_full_config = os.path.join(rviz_base, rviz_config.perform(context))

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
        condition=IfCondition(use_rviz),
    )

    # Static TF for virtual joint
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "world_link"],
    )

    # ros2_control using FakeSystem (mock hardware)
    ros2_controllers_path = os.path.join(
        moveit_config_pkg,
        "config",
        "ros2_controllers_right_arm.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    # Publish static joint states for the unused left arm so MoveIt sees a complete robot state
    left_arm_static_js = Node(
        package="arha_moveit2",
        executable="static_left_joint_state_publisher.py",
        name="left_arm_static_joint_state_publisher",
        output="log",
        parameters=[{"rate": 10.0}],
    )

    # Load controllers
    load_controllers = []
    for controller in ["right_arm_controller", "joint_state_broadcaster"]:
        load_controllers += [
            ExecuteProcess(
                cmd=[
                    "ros2 run controller_manager spawner {}".format(controller)
                ],
                shell=True,
                output="screen",
            )
        ]

    # Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"publish_robot_description_semantic": True},
            trajectory_execution_params,
        ],
    )

    # Delay starting move_group slightly to let hardware/controllers settle
    delayed_move_group = TimerAction(period=2.0, actions=[move_group_node])

    # Zero motors before bringing up controllers/move_group
    zero_script_path = os.path.join(
        get_package_prefix("arha_moveit2"),
        "lib",
        "arha_moveit2",
        "zero_right_arm_motors.py",
    )
    zero_motors = ExecuteProcess(
        cmd=[zero_script_path],
        name="zero_right_arm_motors",
        output="screen",
    )

    # Start the rest after zeroing completes
    start_after_zero = RegisterEventHandler(
        OnProcessExit(
            target_action=zero_motors,
            on_exit=[
                robot_state_publisher,
                rviz_node,
                static_tf,
                ros2_control_node,
                left_arm_static_js,
                delayed_move_group,
                *load_controllers,
            ],
        )
    )

    nodes_to_start = [zero_motors, start_after_zero]

    return nodes_to_start
