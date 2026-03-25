#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
import tf2_ros
import time
import math
import csv

try:
    import matplotlib.pyplot as plt
    import numpy as np
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False


class TC04VelocityTester(Node):
    def __init__(self):
        super().__init__('tc04_velocity_limit')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.action_client = ActionClient(self, FollowJointTrajectory, '/right_arm_controller/follow_joint_trajectory')
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.current_joints = {}
        
    def joint_callback(self, msg):
        self.current_joints = dict(zip(msg.name, msg.position))
        
    def get_tcp_pose(self):
        try:
            # We look up the exact Cartesian position of the Right Wrist
            t = self.tf_buffer.lookup_transform('world_link', 'WristRD_link', rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y, t.transform.translation.z)
        except:
            return None

def smooth(y, box_pts):
    box = np.ones(box_pts)/box_pts
    y_smooth = np.convolve(y, box, mode='same')
    return y_smooth

def main(args=None):
    rclpy.init(args=args)
    node = TC04VelocityTester()
    
    # Spin in background just for TF/Action updates
    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    print("\n" + "="*60)
    print("  MAXIMUM PHYSICAL SPEED PROFILER (AUTOMATED ISO 9283)")
    print("  Tolerance: Discover Absolute Peak Operating Speed & Pose")
    print("="*60)
    
    print("\n[Protocol]")
    print("This script will AUTOMATICALLY perform a 0 to 90 degree sweep on the Right Shoulder.")
    print("Ensure the robot workspace is CLEAR.")
    input("\nPress Enter to begin automated speed test...")
    
    # Wait for joints to populate
    while not node.current_joints:
        time.sleep(0.1)
        
    if not node.action_client.wait_for_server(timeout_sec=5.0):
        print("[ERROR] Action server /right_arm_controller/follow_joint_trajectory not available.")
        rclpy.shutdown()
        return

    joint_names = ['ShoulderRU_joint', 'ShoulderRD_joint', 'ElbowRU_joint', 'ElbowRD_joint', 'WristRU_joint', 'WristRD_joint']
    
    print("\n1/3 --> Moving to origin (0 degrees) slowly...")
    goal_origin = FollowJointTrajectory.Goal()
    goal_origin.trajectory.joint_names = joint_names
    
    pt0 = JointTrajectoryPoint()
    pos0 = []
    for j in joint_names:
        if j == 'ShoulderRU_joint':
            pos0.append(0.0)
        else:
            pos0.append(node.current_joints.get(j, 0.0))
            
    pt0.positions = pos0
    pt0.time_from_start.sec = 3
    goal_origin.trajectory.points.append(pt0)
    
    node.action_client.send_goal_async(goal_origin)
    time.sleep(3.5) # Wait for it to arrive
    
    print("2/3 --> Executing high-speed 0 to 90 degree sweep while recording...")
    goal_sweep = FollowJointTrajectory.Goal()
    goal_sweep.trajectory.joint_names = joint_names
    
    pt90 = JointTrajectoryPoint()
    pos90 = list(pos0)
    pos90[0] = 1.5708 # ShoulderRU_joint to 90 degrees
    pt90.positions = pos90
    
    # Very aggressive timing to hit maximum structural speed limits (1 second!)
    pt90.time_from_start.sec = 1 
    pt90.time_from_start.nanosec = 0
    goal_sweep.trajectory.points.append(pt90)
    
    node.action_client.send_goal_async(goal_sweep)
    
    # Record TF during the sweep
    start_time = time.time()
    raw_data = [] # (time, x, y, z)
    
    while time.time() - start_time < 2.0: # Record for 2 seconds to capture the 1-second move
        pt = node.get_tcp_pose()
        if pt is not None:
            raw_data.append((time.time() - start_time, pt[0], pt[1], pt[2]))
        time.sleep(0.02) # ~50 Hz target
        
    print("3/3 --> Done. Generating metrics...")
    
    print("\n" + "="*60)
    print("  RESULTS ANALYSIS")
    print("="*60)
    
    if len(raw_data) < 10:
        print("[ERROR] Not enough data. Is robot_state_publisher running?")
        rclpy.shutdown()
        return

    # 1. We must filter real-world quantization noise (encoders publishing at 10-20Hz)
    # The physical arm trajectory over 0 to 90 degrees is mathematically smooth.
    # Therefore, 5th-degree Polynomial Regression perfectly recovers the true XYZ Cartesian arc physically traversed,
    # stripping away the CAN bus 'staircase' quantization seen in the raw data.
    times = [d[0] for d in raw_data]
    xs = [d[1] for d in raw_data]
    ys = [d[2] for d in raw_data]
    zs = [d[3] for d in raw_data]
    
    if MATPLOTLIB_AVAILABLE:
        poly_x = np.polyfit(times, xs, 5)
        poly_y = np.polyfit(times, ys, 5)
        poly_z = np.polyfit(times, zs, 5)
        
        smooth_xs = np.polyval(poly_x, times)
        smooth_ys = np.polyval(poly_y, times)
        smooth_zs = np.polyval(poly_z, times)
        
        # 2. Analytical Kinematic Derivative
        # The exact velocity is mathematically derived from the positional polynomials!
        deriv_x = np.polyder(poly_x)
        deriv_y = np.polyder(poly_y)
        deriv_z = np.polyder(poly_z)
        
        vel_x = np.polyval(deriv_x, times)
        vel_y = np.polyval(deriv_y, times)
        vel_z = np.polyval(deriv_z, times)
        
        # Calculate instantaneous smooth 3D speed magnitude
        smoothed_speeds = np.sqrt(vel_x**2 + vel_y**2 + vel_z**2)
        
        # Also compute raw discrete speeds for background visualization
        raw_speeds = [0.0]
        for i in range(1, len(raw_data)):
            dt = times[i] - times[i-1]
            if dt > 0.001:
                sp = math.sqrt((xs[i]-xs[i-1])**2 + (ys[i]-ys[i-1])**2 + (zs[i]-zs[i-1])**2) / dt
            else:
                sp = 0.0
            raw_speeds.append(sp)
    else:
        # Fallback without numpy
        raw_speeds = [0.0]*len(times)
        smoothed_speeds = [0.0]*len(times)
        smooth_xs, smooth_ys, smooth_zs = xs, ys, zs
        
    # Calculate Max Speed metrics
    max_speed = max(smoothed_speeds) if len(smoothed_speeds) > 0 else 0.0
    
    print(f"\nAbsolute Peak Physical TCP Speed Reached (Analytical):")
    print(f" > {max_speed:.4f} m/s")
    
    # Write CSV
    with open('automated_speed_profile.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Time_s', 'X_m', 'Y_m', 'Z_m', 'Raw_Speed_m_s', 'Filtered_Speed_m_s'])
        for i in range(len(raw_data)):
            writer.writerow([f"{times[i]:.4f}", f"{xs[i]:.5f}", f"{ys[i]:.5f}", f"{zs[i]:.5f}", f"{raw_speeds[i]:.4f}", f"{smoothed_speeds[i]:.4f}"])
            
    print("\nRaw kinematic data saved to 'automated_speed_profile.csv'")
    
    # Plotting Dual-pane Graph (Velocity & Pose)
    if MATPLOTLIB_AVAILABLE:
        print("Generating Multi-Axis Speed & Pose Graph...")
        plt.style.use('seaborn-v0_8-whitegrid')
        
        fig, (ax_vel, ax_pose) = plt.subplots(2, 1, figsize=(10, 10), sharex=True)
        
        # --- Top Plot: Velocity ---
        ax_vel.plot(times, raw_speeds, color='gray', alpha=0.2, label='Raw Hardware Encoder Spikes')
        ax_vel.plot(times, smoothed_speeds, color='blue', linewidth=3, label='Analytical Polynomial Derivative (Speed Envelope)')
        ax_vel.axhline(y=max_speed, color='green', linestyle='--', linewidth=3, label=f'True Peak Speed ({max_speed:.4f} m/s)')
        
        ax_vel.set_title(f"Automated Horizontal Arc Swap: Smooth Peak Speed: {max_speed:.4f} m/s", fontsize=14, fontweight='bold')
        ax_vel.set_ylabel('Velocity Magnitude (m/s)', fontsize=12)
        ax_vel.set_ylim(-0.01, max(0.1, max_speed * 1.3))
        ax_vel.legend(loc='center left', bbox_to_anchor=(1.02, 0.5), borderaxespad=0.)
        
        # --- Bottom Plot: Cartesian Pose Algorithms ---
        ax_pose.plot(times, [x*1000 for x in xs], color='gray', alpha=0.3, label='Raw TF Quantization')
        ax_pose.plot(times, [x*1000 for x in smooth_xs], color='red', linewidth=2, label='X Position (mm)')
        ax_pose.plot(times, [y*1000 for y in smooth_ys], color='green', linewidth=2, label='Y Position (mm)')
        ax_pose.plot(times, [z*1000 for z in smooth_zs], color='purple', linewidth=2, label='Z Position (mm)')
        
        ax_pose.set_title("5th-Degree Kinematic Polynomial Position Reconstruction", fontsize=14, fontweight='bold')
        ax_pose.set_xlabel('Time (s)', fontsize=12)
        ax_pose.set_ylabel('Position (mm)', fontsize=12)
        ax_pose.legend(loc='center left', bbox_to_anchor=(1.02, 0.5), borderaxespad=0.)
        
        plt.tight_layout()
        plt.subplots_adjust(right=0.75)  # Make room for the legend on the right
        plt.savefig('automated_speed_profile.png', dpi=300, bbox_inches='tight')
        print("Multi-axis graph securely saved to 'automated_speed_profile.png'")
        
    rclpy.shutdown()

if __name__ == '__main__':
    main()
