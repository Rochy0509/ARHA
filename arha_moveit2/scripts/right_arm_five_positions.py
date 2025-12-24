#!/usr/bin/env python3
"""Send the right arm through five MoveIt joint-space positions.

Update TARGET_WAYPOINTS before running; angles are in radians and follow JOINT_ORDER.
"""

from __future__ import annotations

import sys
from typing import Dict, List

import rclpy
from rclpy.node import Node

try:
    from moveit_commander import MoveGroupCommander
except ImportError as exc:  # pragma: no cover - runtime dependency
    MoveGroupCommander = None
    MOVEIT_IMPORT_ERROR = exc
else:
    MOVEIT_IMPORT_ERROR = None

# ---------- Edit these to define your five target poses ----------
GROUP_NAME = "right_arm"
JOINT_ORDER = [
    "ShoulderRU_joint",
    "ShoulderRD_joint",
    "ElbowRU_joint",
    "ElbowRD_joint",
    "WristRU_joint",
    "WristRD_joint",
]

# Each entry is a list of six joint angles (radians) matching JOINT_ORDER above.
TARGET_WAYPOINTS: List[List[float]] = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Position 1
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Position 2
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Position 3
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Position 4
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Position 5
]
# -----------------------------------------------------------------

VELOCITY_SCALE = 0.2
ACCELERATION_SCALE = 0.2


class FivePositionRunner(Node):
    def __init__(self):
        super().__init__("right_arm_five_position_runner")
        self.group = MoveGroupCommander(GROUP_NAME)
        self.group.set_max_velocity_scaling_factor(VELOCITY_SCALE)
        self.group.set_max_acceleration_scaling_factor(ACCELERATION_SCALE)
        self.targets = self._format_targets()

    def _format_targets(self) -> List[Dict[str, float]]:
        targets: List[Dict[str, float]] = []
        for idx, waypoint in enumerate(TARGET_WAYPOINTS, start=1):
            if len(waypoint) != len(JOINT_ORDER):
                self.get_logger().error(
                    f"Waypoint {idx} has {len(waypoint)} values; expected {len(JOINT_ORDER)} (see JOINT_ORDER)."
                )
                continue
            targets.append({joint: float(value) for joint, value in zip(JOINT_ORDER, waypoint)})
        return targets

    def _extract_trajectory(self, plan):
        if hasattr(plan, "joint_trajectory"):
            return plan
        if isinstance(plan, (list, tuple)):
            for item in plan:
                if hasattr(item, "joint_trajectory"):
                    return item
        return None

    def run_sequence(self) -> bool:
        if not self.targets:
            self.get_logger().error("No valid targets loaded; edit TARGET_WAYPOINTS at the top of the file.")
            return False

        for idx, target in enumerate(self.targets, start=1):
            self.get_logger().info(f"Planning waypoint {idx}/{len(self.targets)}")
            self.group.set_start_state_to_current_state()
            self.group.set_joint_value_target(target)

            plan_result = self.group.plan()
            trajectory = self._extract_trajectory(plan_result)

            if not trajectory or len(trajectory.joint_trajectory.points) == 0:
                self.get_logger().error(f"Planning failed for waypoint {idx}")
                return False

            self.get_logger().info(f"Executing waypoint {idx}/{len(self.targets)}")
            success = self.group.execute(trajectory, wait=True)
            self.group.stop()
            self.group.clear_pose_targets()

            if not success:
                self.get_logger().error(f"Execution failed for waypoint {idx}")
                return False

        self.get_logger().info("Completed all waypoints.")
        return True


def main():
    if MOVEIT_IMPORT_ERROR is not None:
        print(
            f"moveit_commander is not available: {MOVEIT_IMPORT_ERROR}. "
            "Source your workspace and install missing MoveIt dependencies.",
            file=sys.stderr,
        )
        return 1

    rclpy.init()
    node = FivePositionRunner()
    success = node.run_sequence()
    node.destroy_node()
    rclpy.shutdown()
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
