from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("arha_description", package_name="arha_moveit2").to_moveit_configs()
    base_launch = generate_spawn_controllers_launch(moveit_config)

    from launch_ros.actions import Node

    left_arm_impedance_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_arm_impedance_controller",
            "--inactive",
        ],
    )

    right_arm_impedance_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_arm_impedance_controller",
            "--inactive",
        ],
    )

    # We need to extract the LaunchDescription from generate_spawn_controllers_launch 
    # to add our inactive controllers to it
    from launch import LaunchDescription
    
    # generate_spawn_controllers_launch returns a LaunchDescription
    base_launch.add_action(left_arm_impedance_spawner)
    base_launch.add_action(right_arm_impedance_spawner)

    return base_launch
