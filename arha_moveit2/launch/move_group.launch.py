from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    builder = MoveItConfigsBuilder("arha_description", package_name="arha_moveit2").planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
    moveit_config = builder.to_moveit_configs()
    
    # We must explicitly add the Pilz sequence capability
    moveit_config.move_group_capabilities = {
        "capabilities": "pilz_industrial_motion_planner/MoveGroupSequenceAction pilz_industrial_motion_planner/MoveGroupSequenceService",
        "disable_capabilities": ""
    }
    
    return generate_move_group_launch(moveit_config)
