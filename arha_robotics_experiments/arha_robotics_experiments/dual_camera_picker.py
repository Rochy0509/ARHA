#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import yaml
import math
import os
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

try:
    from moveit_commander import MoveGroupCommander, PlanningSceneInterface
except ImportError:
    MoveGroupCommander = PlanningSceneInterface = None

class DualCameraPicker(Node):
    def __init__(self):
        super().__init__("dual_camera_picker")

        self.get_logger().info("Initializing Dual Camera Picker...")

        # Load Config
        pkg_share = get_package_share_directory("arha_robotics_experiments")
        yaml_path = os.path.join(pkg_share, "config", "picking_objects.yaml")
        with open(yaml_path, 'r') as f:
            self.config = yaml.safe_load(f)
        
        self.bot_config = self.config.get("robot_dynamics", {})
        self.objects = self.config.get("objects", {})
        self.obstacles = self.config.get("obstacles", {})

        # Setup MoveGroup and Scene
        self.right_arm = MoveGroupCommander("right_arm")
        self.scene = PlanningSceneInterface(synchronous=True)
        self.right_arm.set_max_velocity_scaling_factor(float(self.bot_config.get("velocity_scaling", 0.3)))
        self.right_arm.set_max_acceleration_scaling_factor(float(self.bot_config.get("acceleration_scaling", 0.3)))
        self.right_arm.set_planner_id(self.bot_config.get("planner_id", "RRTstar"))

        self.right_gripper_pub = self.create_publisher(
            JointTrajectory, "/right_gripper_controller/joint_trajectory", 5
        )

        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Wait 2 seconds for TF tree to populate before starting picking sequence
        self.timer = self.create_timer(2.0, self.execute_sequence)
        
        # Background timer that runs every 3 seconds to update the shelf boundaries if they are seen!
        self.obstacle_timer = self.create_timer(3.0, self.sync_obstacles)
        self.sequence_started = False
        self.registered_obstacles = set()

    def sync_obstacles(self):
        """ Continuously scan the TF tree for defined obstacle anchor tags and inject them into MoveIt """
        for obs_name, params in self.obstacles.items():
            if obs_name in self.registered_obstacles:
                continue # Already locked in place
            
            tag_frame = f"tag36h11:{params['tag_id']}"
            offset = params.get("offset", [0.0, 0.0, 0.0])
            size_dims = params.get("size", [0.1, 0.1, 0.1])
            
            obs_pose = self.get_tag_pose(tag_frame, x_offset=offset[0], y_offset=offset[1], z_offset=offset[2])
            if obs_pose:
                self.get_logger().info(f"Spotted anchor {tag_frame}! Spawning collision box '{obs_name}' into MoveIt...")
                self.scene.add_box(obs_name, obs_pose, size=(size_dims[0], size_dims[1], size_dims[2]))
                self.registered_obstacles.add(obs_name)

    def execute_sequence(self):
        if self.sequence_started:
            return
        self.sequence_started = True

        target_obj_name = "alcohol_bottle"
        target = self.objects[target_obj_name]
        tag_id = target["tag_id"]
        # Standard apriltag_ros framing: tag36h11:0 or tag_0 depending on your node
        tag_frame = f"tag36h11:{tag_id}"

        self.get_logger().info(f"Looking for '{target_obj_name}' tag frame: {tag_frame}")

        # STAGE 1: Macro Approach (Head Camera dominates TF typically)
        macro_pose = self.get_tag_pose(tag_frame, z_offset=-target["approach_distance"])
        if not macro_pose:
            self.get_logger().error(f"Could not find {tag_frame} in TF tree. Is the AprilTag detector running?")
            self.sequence_started = False
            return
        
        self.get_logger().info("Executing Macro Approach...")
        self.right_arm.set_pose_target(macro_pose)
        success = self.right_arm.go(wait=True)
        self.right_arm.stop()
        self.right_arm.clear_pose_targets()

        if not success:
            self.get_logger().error("Macro Approach trajectory failed.")
            return

        # Let the arm settle and let wrist camera lock onto the tag
        self.get_logger().info("Macro reached. Settling wrist camera over tag...")
        time.sleep(2.0)

        # STAGE 2: Micro Grasp (Wrist Camera dominates TF locally)
        self.get_logger().info("Executing Micro Grasp (Cartesian)...")
        grasp_offset = target.get("grasp_offset", [0.0, 0.0, 0.0])
        grasp_pose = self.get_tag_pose(tag_frame, x_offset=grasp_offset[0], y_offset=grasp_offset[1], z_offset=grasp_offset[2])
        
        if not grasp_pose:
            self.get_logger().error("Lost track of tag during micro approach!")
            return

        (plan, fraction) = self.right_arm.compute_cartesian_path([grasp_pose.pose], 0.01, 0.0)
        if fraction < 0.9:
            self.get_logger().error(f"Cartesian trajectory incomplete (fraction: {fraction}). Aborting.")
            return
        
        self.right_arm.execute(plan, wait=True)

        # GRASP
        self.get_logger().info("Actuating Gripper...")
        self.command_gripper(close=True)
        time.sleep(2.0)

        # RETREAT (Cartesian lift up)
        self.get_logger().info("Retreating from drop zone...")
        retreat_pose = macro_pose
        (plan_ret, fraction_ret) = self.right_arm.compute_cartesian_path([retreat_pose.pose], 0.01, 0.0)
        self.right_arm.execute(plan_ret, wait=True)
        self.get_logger().info("Sequence complete!")

    def get_tag_pose(self, tag_frame, x_offset=0.0, y_offset=0.0, z_offset=0.0):
        try:
            trans = self.tf_buffer.lookup_transform("base_link", tag_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=5.0))
            
            # Construct a pose relative to the tag itself
            pose = PoseStamped()
            pose.header.frame_id = tag_frame
            pose.pose.position.x = float(x_offset)
            pose.pose.position.y = float(y_offset)
            pose.pose.position.z = float(z_offset)
            pose.pose.orientation.w = 1.0

            # Transform into base_link world frame
            base_pose = do_transform_pose(pose, trans)
            
            # Force downward-facing orientation for the gripper right above the object natively
            base_pose.pose.orientation.x = 0.0
            base_pose.pose.orientation.y = 1.0 / math.sqrt(2)
            base_pose.pose.orientation.z = 0.0
            base_pose.pose.orientation.w = 1.0 / math.sqrt(2)

            return base_pose
        except Exception as e:
            self.get_logger().warn(f"TF loop exception: {e}")
            return None

    def command_gripper(self, close: bool):
        msg = JointTrajectory()
        msg.joint_names = ["right_left_finger_joint", "right_right_finger_joint"]
        point = JointTrajectoryPoint()
        # Tuning bounds: Pincopen usually ranges 0 to max_stroke. Check limits for open/close defaults.
        point.positions = [0.0 if close else 0.04, 0.0 if close else 0.04]
        point.time_from_start = rclpy.duration.Duration(seconds=1.5).to_msg()
        msg.points.append(point)
        self.right_gripper_pub.publish(msg)

def main():
    if not MoveGroupCommander:
        print("MoveIt Commander completely missing. Please ensure moveit dependencies are installed.")
        return

    rclpy.init()
    node = DualCameraPicker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.right_arm.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
