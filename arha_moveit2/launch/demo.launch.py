import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Use fake hardware (mock) or real hardware interface",
        ),
        DeclareLaunchArgument(
            "zero_on_startup",
            default_value="false",
            description="True if the hardware interface should zero the encoders upon initialization",
        )
    ]
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )


def launch_setup(context, *args, **kwargs):
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    zero_on_startup = LaunchConfiguration("zero_on_startup")
    moveit_config_pkg = get_package_share_directory("arha_moveit2")

    moveit_config = (
        MoveItConfigsBuilder("arha_description", package_name="arha_moveit2")
        .robot_description(
            file_path=os.path.join(
                moveit_config_pkg, "config", "arha_description.urdf.xacro"
            ),
            mappings={"use_fake_hardware": use_fake_hardware.perform(context),
                      "zero_on_startup": zero_on_startup.perform(context)}
        )
        .robot_description_semantic(
            file_path=os.path.join(moveit_config_pkg, "config", "arha_description.srdf")
        )
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # We must explicitly add the Pilz sequence capability
    moveit_config.move_group_capabilities = {
        "capabilities": "pilz_industrial_motion_planner/MoveGroupSequenceAction pilz_industrial_motion_planner/MoveGroupSequenceService",
        "disable_capabilities": ""
    }
            
    return generate_demo_launch(moveit_config).entities
