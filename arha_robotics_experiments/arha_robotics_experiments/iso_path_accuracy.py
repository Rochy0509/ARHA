#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
import math
import time
import csv
import threading

try:
    import matplotlib.pyplot as plt
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False


class ISOPathAccuracyLogger(Node):
    def __init__(self, arm_name):
        super().__init__('iso_path_accuracy_logger')
        self.arm_name = arm_name
        self.topic_name = f'/{arm_name}_controller/controller_state'
        
        self.subscription = self.create_subscription(
            JointTrajectoryControllerState,
            self.topic_name,
            self.state_callback,
            10
        )
        self.lock = threading.Lock()
        self.is_recording = False
        
        self.timestamps = []
        self.errors_deg = []
        self.speeds_deg_s = []
        self.accels_deg_s2 = []
        
        self.start_time = None
        self.get_logger().info(f"ISO Logger initialized. Subscribed to {self.topic_name}")

    def state_callback(self, msg):
        with self.lock:
            if not self.is_recording:
                return
                
            if self.start_time is None:
                self.start_time = time.time()
                
            t = time.time() - self.start_time
            
            # Find the max absolute error, speed, and accel across all 6 joints for this timestep
            # ISO path accuracy often looks at the worst-case deviation
            max_err = 0.0
            max_vel = 0.0
            max_acc = 0.0
            
            for i in range(len(msg.joint_names)):
                err = abs(math.degrees(msg.error.positions[i]))
                if err > max_err: max_err = err
                
                if len(msg.reference.velocities) > i:
                    vel = abs(math.degrees(msg.reference.velocities[i]))
                    if vel > max_vel: max_vel = vel
                    
                if len(msg.reference.accelerations) > i:
                    acc = abs(math.degrees(msg.reference.accelerations[i]))
                    if acc > max_acc: max_acc = acc
                    
            self.timestamps.append(t)
            self.errors_deg.append(max_err)
            self.speeds_deg_s.append(max_vel)
            self.accels_deg_s2.append(max_acc)


def main(args=None):
    rclpy.init(args=args)
    
    print("\n--- ISO 9283 Path Accuracy Reporter ---")
    print("Which arm are you testing?")
    print("  [1] Right Arm")
    print("  [2] Left Arm")
    choice = input("Choice: ").strip()
    arm = "right_arm" if choice != '2' else "left_arm"
    
    node = ISOPathAccuracyLogger(arm)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print(f"\nWaiting to record {arm}...")
    print("1. Go to your Teach Pendant and prepare your Waypoints.")
    input("2. Press ENTER here to START recording, then immediately hit 'Play Waypoints' in the GUI...")
    
    with node.lock:
        node.is_recording = True
        
    print("\nRecording... (Press ENTER to STOP recording when the trajectory finishes)")
    input()
    
    with node.lock:
        node.is_recording = False
        
    print(f"\nRecording stopped. Captured {len(node.timestamps)} datapoints.")
    
    if len(node.timestamps) < 2:
        print("Not enough data captured. Exiting.")
        rclpy.shutdown()
        return

    # Generate CSV Output
    csv_file = f"iso9283_report_{arm}.csv"
    with open(csv_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Time_s', 'Max_Joint_Error_deg', 'Max_Joint_Speed_deg_s', 'Max_Joint_Accel_deg_s2'])
        for t, e, v, a in zip(node.timestamps, node.errors_deg, node.speeds_deg_s, node.accels_deg_s2):
            writer.writerow([f"{t:.3f}", f"{e:.4f}", f"{v:.4f}", f"{a:.4f}"])
            
    print(f"Data saved to {csv_file}")

    # Generate Matplotlib Graphs
    if MATPLOTLIB_AVAILABLE:
        print("Generating ISO 9283 style graphs...")
        
        plt.style.use('seaborn-v0_8-whitegrid')
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
        
        # 1. Path Error
        ax1.plot(node.timestamps, node.errors_deg, color='#d62728', linewidth=1.5)
        ax1.set_title('Path Tracking Error', fontsize=12, fontweight='bold')
        ax1.set_ylabel('Max Joint Error (deg)', fontsize=10)
        ax1.set_ylim(bottom=0)
        ax1.grid(True, linestyle='--', alpha=0.7)
        
        # 2. Speed
        ax2.plot(node.timestamps, node.speeds_deg_s, color='#1f77b4', linewidth=1.5)
        ax2.set_title('Commanded Speed', fontsize=12, fontweight='bold')
        ax2.set_ylabel('Max Speed (deg/s)', fontsize=10)
        ax2.set_ylim(bottom=0)
        ax2.grid(True, linestyle='--', alpha=0.7)
        
        # 3. Acceleration
        ax3.plot(node.timestamps, node.accels_deg_s2, color='#2ca02c', linewidth=1.5)
        ax3.set_title('Commanded Acceleration', fontsize=12, fontweight='bold')
        ax3.set_xlabel('Time (seconds)', fontsize=10)
        ax3.set_ylabel('Max Acceleration (deg/s²)', fontsize=10)
        ax3.grid(True, linestyle='--', alpha=0.7)
        
        plt.tight_layout()
        img_name = f'iso9283_report_{arm}.png'
        plt.savefig(img_name, dpi=300, bbox_inches='tight')
        print(f"Graph saved as '{img_name}'! You can include this in your report.")
        
        # Optional: Show the interactive window if in a desktop environment
        try:
            plt.show()
        except:
            pass
    else:
        print("\nNotice: 'matplotlib' is not installed, so graphs were not generated.")
        print("To generate graphs automatically, run: pip install matplotlib")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
