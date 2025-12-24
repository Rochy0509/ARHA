#!/usr/bin/env python3
"""
Aruco-based pick and place demo for the ARHA robot in Gazebo Classic.

This node listens to the wrist RGB cameras to detect ArUco markers that sit on
top of the cubes, estimates their pose relative to each wrist, transforms the
poses into the base frame, and queues pick requests.  The left arm is used when
the cube is closer to negative X and the right arm otherwise.  After grasping,
the cube is moved on top of the large target marker that sits on the table.

The chest depth camera point cloud is processed to build a coarse collision
representation that is injected into MoveIt’s planning scene so that the arm
does not collide with unknown obstacles.  Wrist depth clouds can also be used
to add extra obstacles near the gripper when approaching the cube.

The script is intentionally conservative: it only attempts one cube at a time
and falls back to idle if path planning fails.  It is meant to serve as a
starting point for more advanced autonomy pipelines.
"""

import math
import sys
import threading
from collections import deque
from copy import deepcopy
from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_geometry_msgs import do_transform_pose
import tf2_ros
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

try:
    from moveit_commander import (
        MoveGroupCommander,
        PlanningSceneInterface,
        RobotCommander,
    )
except ImportError as exc:
    MOVEIT_AVAILABLE = False
    MoveGroupCommander = PlanningSceneInterface = RobotCommander = None
    MOVEIT_IMPORT_ERROR = exc
else:
    MOVEIT_AVAILABLE = True
    MOVEIT_IMPORT_ERROR = None


ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()


@dataclass
class CameraModel:
    frame_id: str = ""
    matrix: np.ndarray = np.zeros((3, 3), dtype=np.float64)
    dist: np.ndarray = np.zeros((5,), dtype=np.float64)
    ready: bool = False


@dataclass
class TargetCube:
    marker_id: int
    pose: PoseStamped
    last_seen: Time


class ArucoPickAndPlace(Node):
    def __init__(self):
        super().__init__("aruco_pick_and_place")
        self.bridge = CvBridge()

        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface(synchronous=True)
        self.left_arm = MoveGroupCommander("left_arm")
        self.right_arm = MoveGroupCommander("right_arm")

        self.left_arm.set_max_velocity_scaling_factor(0.3)
        self.left_arm.set_max_acceleration_scaling_factor(0.3)
        self.right_arm.set_max_velocity_scaling_factor(0.3)
        self.right_arm.set_max_acceleration_scaling_factor(0.3)

        self.left_gripper_pub = self.create_publisher(
            JointTrajectory, "/left_gripper_controller/joint_trajectory", 5
        )
        self.right_gripper_pub = self.create_publisher(
            JointTrajectory, "/right_gripper_controller/joint_trajectory", 5
        )

        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.left_camera_model = CameraModel()
        self.right_camera_model = CameraModel()
        self.left_camera_link = "camera_left_wrist_color_optical_frame"
        self.right_camera_link = "camera_right_wrist_color_optical_frame"

        self.target_queue: deque[TargetCube] = deque(maxlen=10)
        self.queue_lock = threading.Lock()
        self.active_task = None

        self.drop_pose = PoseStamped()
        self.drop_pose.header.frame_id = "base_link"
        self.drop_pose.pose.position.x = -0.18
        self.drop_pose.pose.position.y = -0.45
        self.drop_pose.pose.position.z = 0.42
        self.drop_pose.pose.orientation.w = 1.0

        # Subscribers
        self.create_subscription(
            CameraInfo,
            "/camera_left_wrist/color/camera_info",
            self._left_info_cb,
            10,
        )
        self.create_subscription(
            CameraInfo,
            "/camera_right_wrist/color/camera_info",
            self._right_info_cb,
            10,
        )
        self.create_subscription(
            Image,
            "/camera_left_wrist/color/image_raw",
            self._left_image_cb,
            10,
        )
        self.create_subscription(
            Image,
            "/camera_right_wrist/color/image_raw",
            self._right_image_cb,
            10,
        )
        self.create_subscription(
            PointCloud2,
            "/camera_chest/depth/points",
            self._chest_cloud_cb,
            3,
        )
        self.left_wrist_cloud_sub = self.create_subscription(
            PointCloud2,
            "/camera_left_wrist/depth/points",
            self._left_wrist_cloud_cb,
            3,
        )
        self.right_wrist_cloud_sub = self.create_subscription(
            PointCloud2,
            "/camera_right_wrist/depth/points",
            self._right_wrist_cloud_cb,
            3,
        )

        self.timer = self.create_timer(1.0, self._process_queue)

    # ------------------------------------------------------------------
    # Camera callbacks
    # ------------------------------------------------------------------
    def _left_info_cb(self, msg: CameraInfo):
        self.left_camera_model.matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.left_camera_model.dist = np.array(msg.d, dtype=np.float64)
        self.left_camera_model.frame_id = msg.header.frame_id
        self.left_camera_model.ready = True

    def _right_info_cb(self, msg: CameraInfo):
        self.right_camera_model.matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.right_camera_model.dist = np.array(msg.d, dtype=np.float64)
        self.right_camera_model.frame_id = msg.header.frame_id
        self.right_camera_model.ready = True

    def _left_image_cb(self, msg: Image):
        if not self.left_camera_model.ready:
            return
        self._detect_markers(
            msg,
            self.left_camera_model,
            self.left_camera_link,
            marker_size=0.035,
        )

    def _right_image_cb(self, msg: Image):
        if not self.right_camera_model.ready:
            return
        self._detect_markers(
            msg,
            self.right_camera_model,
            self.right_camera_link,
            marker_size=0.035,
        )

    # ------------------------------------------------------------------
    # Point cloud processing
    # ------------------------------------------------------------------
    def _chest_cloud_cb(self, msg: PointCloud2):
        points = []
        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = p
            if -0.3 < x < 0.5 and -0.7 < y < 0.1 and z > 0.2:
                points.append((x, y, z))
        if not points:
            self.scene.remove_world_object("chest_obstacle")
            return
        arr = np.array(points)
        min_xyz = arr.min(axis=0)
        max_xyz = arr.max(axis=0)
        size = max_xyz - min_xyz
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = float((max_xyz[0] + min_xyz[0]) / 2.0)
        pose.pose.position.y = float((max_xyz[1] + min_xyz[1]) / 2.0)
        pose.pose.position.z = float((max_xyz[2] + min_xyz[2]) / 2.0)
        pose.pose.orientation.w = 1.0

        self.scene.remove_world_object("chest_obstacle")
        self.scene.add_box(
            "chest_obstacle",
            size=tuple(size + np.array([0.02, 0.02, 0.02])),
            pose=pose,
        )

    def _left_wrist_cloud_cb(self, msg: PointCloud2):
        self._update_local_obstacle(msg, "left_wrist_obstacle")

    def _right_wrist_cloud_cb(self, msg: PointCloud2):
        self._update_local_obstacle(msg, "right_wrist_obstacle")

    def _update_local_obstacle(self, msg: PointCloud2, object_id: str):
        points = []
        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = p
            if 0.05 < z < 0.3:
                points.append((x, y, z))
        if not points:
            self.scene.remove_world_object(object_id)
            return
        arr = np.array(points)
        min_xyz = arr.min(axis=0)
        max_xyz = arr.max(axis=0)
        pose = PoseStamped()
        pose.header.frame_id = msg.header.frame_id
        pose.pose.position.x = float((max_xyz[0] + min_xyz[0]) / 2.0)
        pose.pose.position.y = float((max_xyz[1] + min_xyz[1]) / 2.0)
        pose.pose.position.z = float((max_xyz[2] + min_xyz[2]) / 2.0)
        pose.pose.orientation.w = 1.0
        size = tuple((max_xyz - min_xyz) + np.array([0.01, 0.01, 0.01]))
        try:
            base_pose = do_transform_pose(
                pose,
                self.tf_buffer.lookup_transform(
                    "base_link", pose.header.frame_id, rclpy.time.Time()
                ),
            )
            self.scene.remove_world_object(object_id)
            self.scene.add_box(object_id, size=size, pose=base_pose)
        except tf2_ros.TransformException:
            self.get_logger().warn(f"TF lookup failed for {object_id}")

    # ------------------------------------------------------------------
    # ArUco detection
    # ------------------------------------------------------------------
    def _detect_markers(self, img_msg: Image, cam: CameraModel, camera_link: str, marker_size: float):
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMS)
        if ids is None:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                "base_link", camera_link, img_msg.header.stamp
            )
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(f"TF lookup failed: {exc}")
            return

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, marker_size, cam.matrix, cam.dist
        )

        for marker_id, rvec, tvec in zip(ids.flatten(), rvecs, tvecs):
            pose_cam = PoseStamped()
            pose_cam.header.frame_id = camera_link
            pose_cam.header.stamp = img_msg.header.stamp
            pose_cam.pose.position.x = float(tvec[0][0])
            pose_cam.pose.position.y = float(tvec[0][1])
            pose_cam.pose.position.z = float(tvec[0][2])
            quat = cv2.Rodrigues(rvec)[0]
            pose_cam.pose.orientation = self._rotation_matrix_to_quaternion(quat)

            pose_base = do_transform_pose(pose_cam, transform)
            pose_base.pose.position.z -= 0.015  # compensate for tag height

            with self.queue_lock:
                self.target_queue.append(
                    TargetCube(marker_id=int(marker_id), pose=pose_base, last_seen=img_msg.header.stamp)
                )

    @staticmethod
    def _rotation_matrix_to_quaternion(rot):
        trace = np.trace(rot)
        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            w = 0.25 * s
            x = (rot[2][1] - rot[1][2]) / s
            y = (rot[0][2] - rot[2][0]) / s
            z = (rot[1][0] - rot[0][1]) / s
        else:
            # Fallback to ensure valid quaternion
            qw = math.sqrt(1.0 + rot[0][0] - rot[1][1] - rot[2][2]) / 2.0
            qx = (rot[0][1] + rot[1][0]) / (4.0 * qw)
            qy = (rot[0][2] + rot[2][0]) / (4.0 * qw)
            qz = (rot[1][2] + rot[2][1]) / (4.0 * qw)
            w, x, y, z = qw, qx, qy, qz
        pose_quat = PoseStamped().pose.orientation
        pose_quat.w = float(w)
        pose_quat.x = float(x)
        pose_quat.y = float(y)
        pose_quat.z = float(z)
        return pose_quat

    # ------------------------------------------------------------------
    # Task processing
    # ------------------------------------------------------------------
    def _process_queue(self):
        if self.active_task is not None:
            return
        with self.queue_lock:
            if not self.target_queue:
                return
            target = self.target_queue.popleft()
        self.active_task = target
        success = self._execute_pick_and_place(target)
        if not success:
            self.get_logger().warn("Pick and place failed, re-queueing target")
            with self.queue_lock:
                if len(self.target_queue) < self.target_queue.maxlen:
                    self.target_queue.append(target)
        self.active_task = None

    def _execute_pick_and_place(self, target: TargetCube) -> bool:
        arm_group = self.left_arm if target.pose.pose.position.y > -0.45 else self.right_arm
        gripper_pub = self.left_gripper_pub if arm_group is self.left_arm else self.right_gripper_pub
        arm_name = "left" if arm_group is self.left_arm else "right"
        self.get_logger().info(f"Using {arm_name} arm for marker {target.marker_id}")

        pregrasp = deepcopy(target.pose)
        pregrasp.pose.position.z += 0.12
        self._orient_pose_downward(pregrasp.pose)

        grasp_pose = deepcopy(target.pose)
        grasp_pose.pose.position.z += 0.03
        self._orient_pose_downward(grasp_pose.pose)

        retreat_pose = deepcopy(pregrasp)
        retreat_pose.pose.position.z += 0.05

        if not self._plan_and_execute(arm_group, pregrasp):
            return False
        if not self._plan_cartesian(arm_group, [grasp_pose]):
            return False
        self._command_gripper(gripper_pub, close=True)
        if not self._plan_cartesian(arm_group, [retreat_pose]):
            return False

        drop_pre = deepcopy(self.drop_pose)
        drop_pre.pose.position.z += 0.08
        drop_pose = deepcopy(self.drop_pose)
        drop_pose.pose.position.z += 0.03

        if not self._plan_and_execute(arm_group, drop_pre):
            return False
        if not self._plan_cartesian(arm_group, [drop_pose]):
            return False
        self._command_gripper(gripper_pub, close=False)

        if not self._plan_cartesian(arm_group, [drop_pre]):
            return False
        return True

    # ------------------------------------------------------------------
    # Motion helpers
    # ------------------------------------------------------------------
    def _orient_pose_downward(self, pose):
        pose.orientation.x = 0.0
        pose.orientation.y = 1.0 / math.sqrt(2)
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0 / math.sqrt(2)

    def _plan_and_execute(self, move_group: MoveGroupCommander, pose: PoseStamped) -> bool:
        pose.header.stamp = self.get_clock().now().to_msg()
        move_group.set_pose_target(pose)
        plan = move_group.plan()
        if not plan or len(plan.joint_trajectory.points) == 0:
            self.get_logger().warn("Planning failed")
            return False
        return move_group.execute(plan, wait=True)

    def _plan_cartesian(self, move_group: MoveGroupCommander, waypoints):
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        if fraction < 0.9:
            self.get_logger().warn(f"Cartesian path incomplete ({fraction:.2f})")
            return False
        return move_group.execute(plan, wait=True)

    def _command_gripper(self, pub: JointTrajectory, close: bool):
        msg = JointTrajectory()
        msg.joint_names = [
            f"{'left' if pub is self.left_gripper_pub else 'right'}_robotiq_hande_left_finger_joint",
            f"{'left' if pub is self.left_gripper_pub else 'right'}_robotiq_hande_right_finger_joint",
        ]
        point = JointTrajectoryPoint()
        point.positions = [0.005 if close else 0.025, 0.005 if close else 0.025]
        point.time_from_start = rclpy.duration.Duration(seconds=1.5).to_msg()
        msg.points.append(point)
        pub.publish(msg)


def main():
    if not MOVEIT_AVAILABLE:
        print(
            "[aruco_pick_and_place] moveit_commander is not available "
            f"(ImportError: {MOVEIT_IMPORT_ERROR}).\n"
            "Please install MoveIt (e.g. 'sudo apt install ros-humble-moveit' "
            "and any missing python deps like python3-pykdl) and source your workspace.",
            file=sys.stderr,
        )
        return 1

    rclpy.init()
    node = ArucoPickAndPlace()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
