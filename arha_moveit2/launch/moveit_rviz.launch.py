from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    builder = MoveItConfigsBuilder("arha_description", package_name="arha_moveit2").planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
    moveit_config = builder.to_moveit_configs()
    return generate_moveit_rviz_launch(moveit_config)
