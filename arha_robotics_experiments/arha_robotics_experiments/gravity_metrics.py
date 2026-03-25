#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time
import csv
import threading

try:
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    import matplotlib.lines as mlines
    import numpy as np
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False

from control_msgs.msg import JointTrajectoryControllerState

class GravityMetricsLogger(Node):
    def __init__(self):
        super().__init__('gravity_metrics_logger')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.state_sub = self.create_subscription(
            JointTrajectoryControllerState,
            '/right_arm_controller/controller_state',
            self.controller_state_callback,
            10
        )
        self.latest_msg = None
        self.last_controller_msg_time = 0.0
        self.lock = threading.Lock()
        self.get_logger().info("Gravity Metrics Logger initialized.")

    def joint_state_callback(self, msg):
        with self.lock:
            self.latest_msg = msg

    def controller_state_callback(self, msg):
        with self.lock:
            self.last_controller_msg_time = time.time()

    def get_latest_state(self):
        with self.lock:
            return self.latest_msg, self.last_controller_msg_time


# Only validate the Right Arm — Left Arm is still being tuned.
RIGHT_ARM_JOINTS = {
    'ShoulderRU_joint', 'ShoulderRD_joint',
    'ElbowRU_joint',    'ElbowRD_joint',
    'WristRU_joint',    'WristRD_joint',
}

def calculate_rms(values, mean):
    if not values:
        return 0.0
    sum_sq_diff = sum((x - mean) ** 2 for x in values)
    return math.sqrt(sum_sq_diff / len(values))


def main(args=None):
    rclpy.init(args=args)
    node = GravityMetricsLogger()
    
    # Spin in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("\n--- ARHA Gravity Compensation Metrics Logger ---")
    print("Waiting for /joint_states...\n")
    
    while True:
        msg_tup = node.get_latest_state()
        if msg_tup[0] is not None:
            break
        time.sleep(0.5)

    try:
        while True:
            print("\nSelect a test to run for your report:")
            print("  [1] Backdrivability Drift Test (5s, < 1.0 deg)")
            print("  [2] Torque Residual Test (5s, RMS variance)")
            print("  [3] TC-03 Gravity Hold Safety Test (30s × 3 configs, ES-07, < 2.0 deg)")
            print("  [4] Controller Transition Handover Graph (Blue -> Red overlap)")
            print("  [5] Exit")
            
            choice = input("Enter choice: ").strip()

            if choice == '1':
                print("\n>>> Drift Test: Press Enter, then RELEASE the arm. It will measure drift over 5 seconds.")
                input("Press Enter to begin...")
                
                start_msg, _ = node.get_latest_state()
                start_positions = dict(zip(start_msg.name, start_msg.position))
                
                print("Recording for 5 seconds... Do not touch the arm.")
                time.sleep(5.0)
                
                end_msg, _ = node.get_latest_state()
                end_positions = dict(zip(end_msg.name, end_msg.position))
                
                print("\n--- Drift Results (< 1.0 deg is passing) ---")
                
                with open('drift_metrics.csv', 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(['Joint_Name', 'Start_Pos_deg', 'End_Pos_deg', 'Absolute_Drift_deg', 'Passed'])
                    
                    for name in start_positions:
                        if name not in RIGHT_ARM_JOINTS:
                            continue
                        if name in end_positions:
                            start_deg = math.degrees(start_positions[name])
                            end_deg = math.degrees(end_positions[name])
                            drift = abs(end_deg - start_deg)
                            passed = "PASS" if drift < 1.0 else "FAIL"
                            
                            print(f"{name:20}: {drift:.3f} deg ({passed})")
                            writer.writerow([name, f"{start_deg:.3f}", f"{end_deg:.3f}", f"{drift:.3f}", passed])
                            
                print("\nSaved full results to 'drift_metrics.csv'")

            elif choice == '2':
                print("\n>>> Torque Residual Test: Leave the arm static.")
                input("Press Enter to begin 5-second sampling at ~10Hz...")
                
                print("Sampling...")
                samples = []
                for _ in range(50): # 50 samples at 10Hz = 5 seconds
                    msg, _ = node.get_latest_state()
                    if msg:
                        samples.append(dict(zip(msg.name, msg.effort)))
                    time.sleep(0.1)

                print("\n--- Torque Residual Results ---")
                print("Note: In gravity mode, commanded effort matches gravity torque.")
                print("The RMS variance measures how much the motor hunts/vibrates around the hold value.")
                
                with open('torque_metrics.csv', 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(['Joint_Name', 'Mean_Holding_Current_Amps', 'RMS_Variance_Amps'])
                    
                    # Assume all samples have the same keys
                    if samples:
                        keys = [k for k in samples[0].keys() if k in RIGHT_ARM_JOINTS]
                        for key in keys:
                            efforts = [s[key] for s in samples if key in s]
                            if efforts:
                                mean_eff = sum(efforts) / len(efforts)
                                rms_eff = calculate_rms(efforts, mean_eff)
                                
                                print(f"{key:20}: Mean = {mean_eff:6.3f} A, RMS Variance = {rms_eff:6.3f} A")
                                writer.writerow([key, f"{mean_eff:.4f}", f"{rms_eff:.4f}"])
                                
                print("\nSaved full results to 'torque_metrics.csv'")

            elif choice == '3':
                # ── TC-03 GRAVITY COMPENSATION POSITION HOLD ──────────────────────────
                HOLD_SECS   = 30       # seconds per configuration
                SAMPLE_HZ   = 10       # samples per second
                PASS_DEG    = 2.0      # pass/fail threshold (ISO ES-07)
                NUM_CONFIGS = 3
                ALL_PASS    = True

                print("\n" + "=" * 60)
                print("  TC-03: Gravity Compensation Position Hold (ES-07)")
                print(f"  Protocol: {NUM_CONFIGS} configs × {HOLD_SECS}s @ {SAMPLE_HZ} Hz")
                print(f"  Pass criterion: max drift < {PASS_DEG}° per config")
                print("=" * 60)

                config_names = ["Low / Home", "Mid Reach", "Extended Reach"]
                csv_rows = []

                for cfg_idx, cfg_name in enumerate(config_names, start=1):
                    print(f"\n[Config {cfg_idx}/{NUM_CONFIGS}] → {cfg_name}")
                    print("   1. Move the arm to this configuration.")
                    print("   2. Enable Gravity Comp in the Teach Pendant.")
                    input("   3. Release the arm and press Enter to begin 30-second recording...")

                    # ── Capture initial position ─────────────────────────────────
                    start_msg, _ = node.get_latest_state()
                    q0 = dict(zip(start_msg.name, start_msg.position))

                    # ── Continuous sampling ──────────────────────────────────────
                    samples = []
                    print(f"   Recording for {HOLD_SECS}s ... do not touch the arm.")
                    for tick in range(HOLD_SECS * SAMPLE_HZ):
                        msg, _ = node.get_latest_state()
                        if msg:
                            samples.append(dict(zip(msg.name, msg.position)))
                        time.sleep(1.0 / SAMPLE_HZ)
                        if (tick + 1) % (SAMPLE_HZ * 5) == 0:
                            elapsed = (tick + 1) // SAMPLE_HZ
                            print(f"   ... {elapsed}s / {HOLD_SECS}s")

                    # ── Compute max drift across ALL samples, not just final ─────
                    per_joint_max_drift = {}  # joint -> max absolute drift in deg
                    for s in samples:
                        for jname, q_t in s.items():
                            if jname not in RIGHT_ARM_JOINTS:
                                continue
                            if jname in q0:
                                drift_deg = abs(math.degrees(q_t - q0[jname]))
                                if jname not in per_joint_max_drift or drift_deg > per_joint_max_drift[jname]:
                                    per_joint_max_drift[jname] = drift_deg

                    worst_joint = max(per_joint_max_drift, key=per_joint_max_drift.get)
                    worst_drift = per_joint_max_drift[worst_joint]
                    cfg_pass = worst_drift < PASS_DEG
                    if not cfg_pass:
                        ALL_PASS = False

                    result_str = "PASS ✓" if cfg_pass else "FAIL ✗"
                    print(f"\n   ── Config {cfg_idx} ({cfg_name}) Results ──")
                    for jn, d in sorted(per_joint_max_drift.items()):
                        flag = "" if d < PASS_DEG else " ← EXCEEDED"
                        print(f"   {jn:22}: max drift = {d:6.3f} °{flag}")
                    print(f"   Worst joint: {worst_joint} ({worst_drift:.3f}°) → {result_str}")

                    csv_rows.append({
                        'Config_Index':    cfg_idx,
                        'Config_Name':     cfg_name,
                        'Worst_Joint':     worst_joint,
                        'Max_Drift_deg':   f"{worst_drift:.4f}",
                        'Pass_Criterion_deg': PASS_DEG,
                        'PASS_FAIL':       'PASS' if cfg_pass else 'FAIL',
                    })

                # ── Summary ──────────────────────────────────────────────────────
                print("\n" + "=" * 60)
                print("  TC-03 OVERALL RESULT:", "PASS ✓" if ALL_PASS else "FAIL ✗")
                print(f"  ES-07 compliance: max drift < {PASS_DEG}° over {HOLD_SECS}s across all configs")
                print("=" * 60)

                # ── Write CSV ─────────────────────────────────────────────────────
                csv_file = 'tc03_gravity_hold.csv'
                with open(csv_file, 'w', newline='') as f:
                    writer = csv.DictWriter(f, fieldnames=csv_rows[0].keys())
                    writer.writeheader()
                    writer.writerows(csv_rows)
                print(f"\n  Detailed results saved to '{csv_file}'")

            elif choice == '4':
                if not MATPLOTLIB_AVAILABLE:
                    print("\n[ERROR] Matplotlib is required to generate this graph. Please install it:")
                    print("pip install matplotlib numpy")
                    continue
                    
                print("\n" + "=" * 60)
                print("  TC-04: Controller Transition Handover Graph")
                print("=" * 60)
                print("We will record exactly 15 seconds. Get ready!")
                print("1. Set up a movement in the Teach Pendant (e.g. playing waypoints).")
                print("2. While it's moving, or right when it finishes, click 'Enable Gravity Comp'.")
                input("\nPress Enter to begin 15-second recording...")
                
                print("\n[REC] Recording... Please perform your controller switch!")
                start_time = time.time()
                
                # Arrays to store plotting data
                timestamps = []
                joint_data = {j: [] for j in RIGHT_ARM_JOINTS}
                modes = [] # "pos" or "grav"
                
                while time.time() - start_time < 15.0:
                    msg, last_ctrl_time = node.get_latest_state()
                    if msg:
                        t = time.time() - start_time
                        timestamps.append(t)
                        
                        pos_dict = dict(zip(msg.name, msg.position))
                        for j in RIGHT_ARM_JOINTS:
                            if j in pos_dict:
                                joint_data[j].append(math.degrees(pos_dict[j]))
                            else:
                                joint_data[j].append(0.0) # Fallback
                                
                        # Controller states are published at 50Hz when active. 
                        # If we haven't received a state packet in 0.2s, the safety boundary switched to Limp.
                        if time.time() - last_ctrl_time < 0.2:
                            modes.append("pos")
                        else:
                            modes.append("grav")
                    
                    time.sleep(0.05) # ~20Hz
                    
                print(f"[STOP] Captured {len(timestamps)} frames. Generating dynamic handover graph...")
                
                plt.style.use('seaborn-v0_8-whitegrid')
                # Create a 2x3 grid of subplots for the 6 right arm joints
                fig, axs = plt.subplots(3, 2, figsize=(12, 10), sharex=True)
                fig.suptitle(f"Controller Handover: Position Control -> Gravity Compensation", fontsize=14, fontweight='bold')
                
                axs = axs.flatten()
                
                ordered_joints = ['ShoulderRU_joint', 'ShoulderRD_joint', 'ElbowRU_joint', 'ElbowRD_joint', 'WristRU_joint', 'WristRD_joint']
                
                for idx, j_name in enumerate(ordered_joints):
                    ax = axs[idx]
                    ax.set_title(j_name)
                    
                    x = np.array(timestamps)
                    y = np.array(joint_data[j_name])
                    
                    # Create line segments mapping from t to t+1
                    points = np.array([x, y]).T.reshape(-1, 1, 2)
                    segments = np.concatenate([points[:-1], points[1:]], axis=1)
                    
                    # Color the segment based on the active mode
                    colors = []
                    for i in range(len(modes)-1):
                        colors.append('#1f77b4' if modes[i] == 'pos' else '#d62728')
                            
                    lc = LineCollection(segments, colors=colors, linewidths=2.0, alpha=0.9)
                    ax.add_collection(lc)
                    
                    # Compute reasonable scaling
                    if len(y[y != 0.0]) > 0:
                        y_min, y_max = y.min(), y.max()
                        padding = max(abs(y_max - y_min) * 0.2, 5.0)
                        ax.set_ylim(y_min - padding, y_max + padding)
                        ax.set_xlim(0, 15)
                    
                    ax.set_ylabel('Position (deg)')
                    if idx >= 4:
                        ax.set_xlabel('Time (s)')
                
                # Legend Overlay
                blue_line = mlines.Line2D([], [], color='#1f77b4', linewidth=3, label='Position Control (Rigid)')
                red_line = mlines.Line2D([], [], color='#d62728', linewidth=3, label='Gravity Comp (Limp)')
                fig.legend(handles=[blue_line, red_line], loc='upper right', bbox_to_anchor=(0.95, 0.95), framealpha=1.0)
                
                plt.tight_layout(rect=[0, 0.03, 1, 0.96])
                plt.savefig('handover_graph.png', dpi=300)
                print("SUCCESS: Graph natively saved to 'handover_graph.png' for your report!")

            elif choice == '5':
                break
            else:
                print("Invalid choice.")
                
    except KeyboardInterrupt:
        print("\nExiting.")
        
    rclpy.shutdown()

if __name__ == '__main__':
    main()
