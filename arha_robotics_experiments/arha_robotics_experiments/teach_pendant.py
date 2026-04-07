import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import SwitchController
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient

import tf2_ros
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import RobotState

class TeachPendantNode(Node):
    def __init__(self):
        super().__init__('teach_pendant_node')
        
        self.current_joint_dict = {}
        
        # Subscribe to joint states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
            
        # Service client for controller switching
        self.switch_ctrl_client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        
        # Action clients for trajectory execution
        self.left_action_client = ActionClient(self, FollowJointTrajectory, '/left_arm_controller/follow_joint_trajectory')
        self.right_action_client = ActionClient(self, FollowJointTrajectory, '/right_arm_controller/follow_joint_trajectory')

        # Gripper actions
        self.left_gripper_client = ActionClient(self, FollowJointTrajectory, '/left_gripper_controller/follow_joint_trajectory')
        self.right_gripper_client = ActionClient(self, FollowJointTrajectory, '/right_gripper_controller/follow_joint_trajectory')

        # Cartesian Path Service
        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        
        # TF2 listener for recording Cartesian poses
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.time.Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def joint_state_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.current_joint_dict[name] = pos
            
    def switch_controllers(self, start_controllers, stop_controllers):
        if not self.switch_ctrl_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('switch_controller service not available')
            return False
            
        req = SwitchController.Request()
        req.start_controllers = start_controllers
        req.stop_controllers = stop_controllers
        req.strictness = SwitchController.Request.BEST_EFFORT
        
        future = self.switch_ctrl_client.call_async(req)
        
        # Wait for the future to complete (called from UI thread, so safe to block slightly)
        while not future.done():
            time.sleep(0.01)
            
        return future.result().ok


class TeachPendantApp:
    def __init__(self, root, ros_node):
        self.root = root
        self.node = ros_node
        self.root.title("ARHA Teach Pendant")
        self.root.geometry("400x600")
        
        # Data storage
        self.waypoints = [] # stores Joint Waypoints
        self.cartesian_left = [] # stores Pose Waypoints for Left relative to world_link
        self.cartesian_right = [] # stores Pose Waypoints for Right relative to world_link
        
        # UI Layout
        style = ttk.Style()
        style.configure('TButton', font=('Helvetica', 12), padding=10)
        
        # Status Label
        self.status_var = tk.StringVar()
        self.status_var.set("Status: Ready")
        status_label = tk.Label(root, textvariable=self.status_var, font=('Helvetica', 10, 'bold'), fg="blue")
        status_label.pack(pady=10)
        
        # Mode Frame
        mode_frame = ttk.LabelFrame(root, text="Control Mode")
        mode_frame.pack(fill="x", padx=20, pady=10)
        
        ttk.Button(mode_frame, text="Enable Gravity Comp (Limp)", command=self.enable_gravity_comp).pack(fill="x", padx=10, pady=5)
        ttk.Button(mode_frame, text="Enable Position Control (Rigid)", command=self.enable_position_ctrl).pack(fill="x", padx=10, pady=5)
        
        # Gripper Control Frame
        gripper_frame = ttk.LabelFrame(root, text="Gripper Control (Live)")
        gripper_frame.pack(fill="x", padx=20, pady=5)
        
        left_grip_frame = tk.Frame(gripper_frame)
        left_grip_frame.pack(fill="x", padx=10, pady=2)
        tk.Label(left_grip_frame, text="Left Gripper:").pack(side="left")
        ttk.Button(left_grip_frame, text="Open", command=lambda: self.command_gripper('left', -0.57)).pack(side="left", padx=5)
        ttk.Button(left_grip_frame, text="Close", command=lambda: self.command_gripper('left', 0.0)).pack(side="left", padx=5)

        right_grip_frame = tk.Frame(gripper_frame)
        right_grip_frame.pack(fill="x", padx=10, pady=2)
        tk.Label(right_grip_frame, text="Right Gripper:").pack(side="left")
        ttk.Button(right_grip_frame, text="Open", command=lambda: self.command_gripper('right', -0.57)).pack(side="left", padx=5)
        ttk.Button(right_grip_frame, text="Close", command=lambda: self.command_gripper('right', 0.0)).pack(side="left", padx=5)
        
        # Waypoint Frame
        wp_frame = ttk.LabelFrame(root, text="Waypoints")
        wp_frame.pack(fill="x", padx=20, pady=10)
        
        self.wp_count_var = tk.StringVar()
        self.wp_count_var.set("Saved Waypoints: 0")
        tk.Label(wp_frame, textvariable=self.wp_count_var).pack(pady=5)
        
        ttk.Button(wp_frame, text="📥 Record Current Pose", command=self.save_waypoint).pack(fill="x", padx=10, pady=5)
        ttk.Button(wp_frame, text="🗑️ Clear Waypoints", command=self.clear_waypoints).pack(fill="x", padx=10, pady=5)
        
        # Execution Frame
        exec_frame = ttk.LabelFrame(root, text="Execution")
        exec_frame.pack(fill="x", padx=20, pady=10)
        
        # Speed Slider
        speed_frame = tk.Frame(exec_frame)
        speed_frame.pack(fill="x", padx=10, pady=5)
        tk.Label(speed_frame, text="Time per waypoint (s):").pack(side="left")
        self.time_slider = tk.Scale(speed_frame, from_=0.5, to=10.0, resolution=0.5, orient="horizontal")
        self.time_slider.set(3.0) # Default to 3.0s (slower, more accurate)
        self.time_slider.pack(side="right", fill="x", expand=True)

        self.linear_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(exec_frame, text="Use Linear Cartesian Path (MoveIt2)", variable=self.linear_var).pack(fill="x", padx=10, pady=2)

        self.iso_vel_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(exec_frame, text="Apply 10% ISO Velocity Scaling (Bypasses Time Slider)", variable=self.iso_vel_var).pack(fill="x", padx=10, pady=2)

        ttk.Button(exec_frame, text="▶️ Play Waypoints sequence", command=self.play_waypoints).pack(fill="x", padx=10, pady=5)
        ttk.Button(exec_frame, text="🏠 Send Arms Home", command=self.send_home).pack(fill="x", padx=10, pady=5)

    def update_status(self, msg):
        self.status_var.set(f"Status: {msg}")
        self.root.update_idletasks()

    def enable_gravity_comp(self):
        self.update_status("Switching to Gravity Comp...")
        # controller manager will handle missing servers via BEST_EFFORT
        stop_ctrls = ['left_arm_controller', 'right_arm_controller']
            
        success = self.node.switch_controllers(
            start_controllers=[],
            stop_controllers=stop_ctrls
        )
        if success:
            self.update_status("Mode: Gravity Comp")
        else:
            self.update_status("Error: Failed to switch to Gravity Comp")

    def enable_position_ctrl(self):
        self.update_status("Switching to Position Control...")
        
        start_ctrls = ['left_arm_controller', 'right_arm_controller']

        success = self.node.switch_controllers(
            start_controllers=start_ctrls,
            stop_controllers=[]
        )
        if success:
            self.update_status("Mode: Position Control")
        else:
            self.update_status("Error: Failed to switch to Position Control")

    def command_gripper(self, side, position):
        goal = FollowJointTrajectory.Goal()
        
        # Point 0: Current position (start)
        pt0 = JointTrajectoryPoint()
        curr_pos = self.node.current_joint_dict.get(f"{side}_cam_joint", 0.0)
        pt0.positions = [curr_pos]
        pt0.velocities = [0.0]
        pt0.accelerations = [0.0]
        pt0.time_from_start.sec = 0
        pt0.time_from_start.nanosec = 0
        
        # Point 1: Target position
        pt1 = JointTrajectoryPoint()
        pt1.positions = [position]
        pt1.velocities = [0.0]
        pt1.accelerations = [0.0]
        pt1.time_from_start.sec = 1
        pt1.time_from_start.nanosec = 0
        
        goal.trajectory.points = [pt0, pt1]
        
        if side == 'left':
            goal.trajectory.joint_names = ['left_cam_joint']
            if self.node.left_gripper_client.wait_for_server(timeout_sec=1.0):
                self.node.left_gripper_client.send_goal_async(goal)
            else:
                self.update_status("Error: Left gripper server not ready")
        else:
            goal.trajectory.joint_names = ['right_cam_joint']
            if self.node.right_gripper_client.wait_for_server(timeout_sec=1.0):
                self.node.right_gripper_client.send_goal_async(goal)
            else:
                self.update_status("Error: Right gripper server not ready")

    def _create_gripper_trajectory(self, waypoints, side, total_time):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [f"{side}_cam_joint"]
        
        if len(waypoints) == 0:
            return goal
            
        time_per_wp = total_time / len(waypoints)
        
        for i, wp in enumerate(waypoints):
            # If not found, assume 0.0 (Open)
            val = wp.get(f"{side}_cam_joint", 0.0) 
            pt = JointTrajectoryPoint()
            pt.positions = [val]
            pt.velocities = [0.0]
            pt.accelerations = [0.0]
            
            pt_time = (i + 1) * time_per_wp
            pt.time_from_start.sec = int(pt_time)
            pt.time_from_start.nanosec = int((pt_time % 1) * 1e9)
            goal.trajectory.points.append(pt)
            
        return goal

    def save_waypoint(self):
        if not self.node.current_joint_dict:
            messagebox.showwarning("Warning", "No joint states received yet. Is the robot running?")
            return
            
        # 1. Copy current physical joint state
        wp = dict(self.node.current_joint_dict)
        self.waypoints.append(wp)
        
        # 2. Record true Cartesian poses for Linear Interpolation Feature
        try:
            # We look up transform from root to TCP
            tl = self.node.tf_buffer.lookup_transform('world_link', 'WristLD_link', rclpy.time.Time())
            tr = self.node.tf_buffer.lookup_transform('world_link', 'WristRD_link', rclpy.time.Time())
            
            p_left = Pose()
            p_left.position.x = tl.transform.translation.x
            p_left.position.y = tl.transform.translation.y
            p_left.position.z = tl.transform.translation.z
            p_left.orientation = tl.transform.rotation
            self.cartesian_left.append(p_left)
            
            p_right = Pose()
            p_right.position.x = tr.transform.translation.x
            p_right.position.y = tr.transform.translation.y
            p_right.position.z = tr.transform.translation.z
            p_right.orientation = tr.transform.rotation
            self.cartesian_right.append(p_right)
            
        except Exception as e:
            self.node.get_logger().warn(f"Could not record Cartesian TF: {e}")
        
        self.wp_count_var.set(f"Saved Waypoints: {len(self.waypoints)}")
        self.update_status(f"Saved Waypoint #{len(self.waypoints)}")

    def clear_waypoints(self):
        self.waypoints = []
        self.cartesian_left = []
        self.cartesian_right = []
        self.wp_count_var.set("Saved Waypoints: 0")
        self.update_status("Cleared all waypoints")

    def _extract_limb_positions(self, wp, limb_prefix):
        # The joints are strictly named according to URDF (e.g. ShoulderLU_joint vs ShoulderRU_joint)
        joint_names = []
        positions = []
        
        if limb_prefix == 'left':
            expected = ['ShoulderLU_joint', 'ShoulderLD_joint', 'ElbowLU_joint', 'ElbowLD_joint', 'WristLU_joint', 'WristLD_joint']
        else:
            expected = ['ShoulderRU_joint', 'ShoulderRD_joint', 'ElbowRU_joint', 'ElbowRD_joint', 'WristRU_joint', 'WristRD_joint']
            
        for name in expected:
            if name in wp:
                joint_names.append(name)
                positions.append(wp[name])
                
        return joint_names, positions

    def _call_cartesian_path(self, group_name, link_name, poses):
        if not self.node.cartesian_client.wait_for_service(timeout_sec=2.0):
            return None
            
        req = GetCartesianPath.Request()
        req.header.frame_id = 'world_link'
        req.group_name = group_name
        req.link_name = link_name
        req.waypoints = poses
        req.max_step = 0.01  # 1 cm resolution
        req.jump_threshold = 0.0 # Disabled
        req.avoid_collisions = False
        
        # Apply strict 10% velocity scaling limit if selected
        if hasattr(self, 'iso_vel_var') and self.iso_vel_var.get():
            req.max_velocity_scaling_factor = 0.1
            req.max_acceleration_scaling_factor = 0.1
        else:
            req.max_velocity_scaling_factor = 1.0
            req.max_acceleration_scaling_factor = 1.0
        
        # Populate RobotState with current joints
        rs = RobotState()
        js = JointState()
        js.name = list(self.node.current_joint_dict.keys())
        js.position = list(self.node.current_joint_dict.values())
        rs.joint_state = js
        req.start_state = rs
        
        future = self.node.cartesian_client.call_async(req)
        while not future.done():
            time.sleep(0.01)
            
        return future.result()

    def send_trajectory(self, waypoints):
        self.update_status("Sending trajectory...")
        
        has_left = self.node.left_action_client.wait_for_server(timeout_sec=0.1)
        has_right = self.node.right_action_client.wait_for_server(timeout_sec=0.1)
        
        if not has_left and not has_right:
            self.update_status("Error: No arm action servers ready!")
            return

        time_per_wp = float(self.time_slider.get())
        
        if self.linear_var.get() and len(self.cartesian_left) == len(waypoints):
            # -------- LINEAR CARTESIAN PATH --------
            self.update_status("Computing Cartesian path...")
            
            if has_left:
                res_left = self._call_cartesian_path('left_arm', 'WristLD_link', self.cartesian_left)
                if not res_left or res_left.fraction < 0.9:
                    self.update_status(f"Error: L Path failed! ({res_left.fraction:.2f})")
                    return
                left_goal = FollowJointTrajectory.Goal()
                left_goal.trajectory = res_left.solution.joint_trajectory

            if has_right:
                res_right = self._call_cartesian_path('right_arm', 'WristRD_link', self.cartesian_right)
                if not res_right or res_right.fraction < 0.9:
                    self.update_status(f"Error: R Path failed! ({res_right.fraction:.2f})")
                    return
                right_goal = FollowJointTrajectory.Goal()
                right_goal.trajectory = res_right.solution.joint_trajectory
            # ONLY RE-TIME MANUALLY IF WE ARE NOT RUNNING THE NATIVE ISO 10% SCALING LIMIT TEST
            if not (hasattr(self, 'iso_vel_var') and self.iso_vel_var.get()):
                total_time = time_per_wp * len(waypoints)
                
                goals_to_process = []
                if has_left: goals_to_process.append(left_goal)
                if has_right: goals_to_process.append(right_goal)
                
                for goal in goals_to_process:
                    num_points = len(goal.trajectory.points)
                    if num_points == 0:
                        continue
                        
                    # Calculate cumulative arc length for smooth velocity parametrization
                    distances = [0.0]
                    for j in range(1, num_points):
                        prev_p = goal.trajectory.points[j-1].positions
                        curr_p = goal.trajectory.points[j].positions
                        dist = sum((c - p)**2 for c, p in zip(curr_p, prev_p)) ** 0.5
                        distances.append(distances[-1] + dist)
                        
                    total_dist = distances[-1]
                    
                    # Assign time strictly proportional to distance
                    for j, pt in enumerate(goal.trajectory.points):
                        if total_dist <= 0.0001:
                            fraction = (j + 1) / num_points # Fallback
                        else:
                            fraction = distances[j] / total_dist
                            
                        pt_time = total_time * fraction
                        
                        if j > 0 and pt_time <= goal.trajectory.points[j-1].time_from_start.sec + goal.trajectory.points[j-1].time_from_start.nanosec * 1e-9:
                             pt_time = goal.trajectory.points[j-1].time_from_start.sec + goal.trajectory.points[j-1].time_from_start.nanosec * 1e-9 + 0.001
                             
                        pt.time_from_start.sec = int(pt_time)
                        pt.time_from_start.nanosec = int((pt_time % 1) * 1e9)
                        pt.velocities = []
                        pt.accelerations = []
            else:
                # We are directly executing MoveIt's ISO-limited time profile!
                # Update total_time variable so the GUI thread waits properly
                if has_right and len(right_goal.trajectory.points) > 0:
                    last_pt = right_goal.trajectory.points[-1]
                    total_time = last_pt.time_from_start.sec + last_pt.time_from_start.nanosec * 1e-9
                elif has_left and len(left_goal.trajectory.points) > 0:
                    last_pt = left_goal.trajectory.points[-1]
                    total_time = last_pt.time_from_start.sec + last_pt.time_from_start.nanosec * 1e-9
            
        else:
            # -------- JOINT INTERPOLATED PATH --------
            left_goal = FollowJointTrajectory.Goal()
            left_goal.trajectory.joint_names, _ = self._extract_limb_positions(waypoints[0], 'left')
            
            right_goal = FollowJointTrajectory.Goal()
            right_goal.trajectory.joint_names, _ = self._extract_limb_positions(waypoints[0], 'right')
            
            for i, wp in enumerate(waypoints):
                _, left_pos = self._extract_limb_positions(wp, 'left')
                left_pt = JointTrajectoryPoint()
                left_pt.positions = left_pos
                left_pt.time_from_start.sec = int(time_per_wp * (i + 1))
                left_pt.time_from_start.nanosec = int((time_per_wp * (i + 1) % 1) * 1e9)
                left_goal.trajectory.points.append(left_pt)
                
                _, right_pos = self._extract_limb_positions(wp, 'right')
                right_pt = JointTrajectoryPoint()
                right_pt.positions = right_pos
                right_pt.time_from_start.sec = int(time_per_wp * (i + 1))
                right_pt.time_from_start.nanosec = int((time_per_wp * (i + 1) % 1) * 1e9)
                right_goal.trajectory.points.append(right_pt)
                
            total_time = time_per_wp * len(waypoints)

        # Send async goals
        if has_left:
            self.node.left_action_client.send_goal_async(left_goal)
            if self.node.left_gripper_client.wait_for_server(timeout_sec=0.1):
                left_grip_goal = self._create_gripper_trajectory(waypoints, 'left', total_time)
                self.node.left_gripper_client.send_goal_async(left_grip_goal)
                
        if has_right:
            self.node.right_action_client.send_goal_async(right_goal)
            if self.node.right_gripper_client.wait_for_server(timeout_sec=0.1):
                right_grip_goal = self._create_gripper_trajectory(waypoints, 'right', total_time)
                self.node.right_gripper_client.send_goal_async(right_grip_goal)
        
        self.update_status(f"Playing... (ETA: {total_time:.1f}s)")
        
        def reset_status():
            time.sleep(total_time + 0.5)
            self.update_status("Playback Complete")
            
        threading.Thread(target=reset_status, daemon=True).start()

    def play_waypoints(self):
        if not self.waypoints:
            messagebox.showwarning("Warning", "No waypoints saved to play!")
            return
            
        # Ensure position control is active before playing
        self.enable_position_ctrl()
        self.send_trajectory(self.waypoints)

    def send_home(self):
        # Create a single perfectly 0.0 waypoint for all expected joints
        home_wp = {
            'ShoulderLU_joint': 0.0, 'ShoulderLD_joint': 0.0, 'ElbowLU_joint': 0.0, 'ElbowLD_joint': 0.0, 'WristLU_joint': 0.0, 'WristLD_joint': 0.0,
            'ShoulderRU_joint': 0.0, 'ShoulderRD_joint': 0.0, 'ElbowRU_joint': 0.0, 'ElbowRD_joint': 0.0, 'WristRU_joint': 0.0, 'WristRD_joint': 0.0
        }
        
        # If current joint dict exists, grab it as the start point to ensure smooth interpolation
        if self.node.current_joint_dict:
            start_wp = dict(self.node.current_joint_dict)
            self.enable_position_ctrl()
            self.send_trajectory([start_wp, home_wp])
        else:
            self.enable_position_ctrl()
            self.send_trajectory([home_wp])


def main(args=None):
    rclpy.init(args=args)
    
    ros_node = TeachPendantNode()
    
    # Spin ROS 2 Node in a background daemon thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()
    
    # Run Tkinter on the main thread
    root = tk.Tk()
    app = TeachPendantApp(root, ros_node)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
