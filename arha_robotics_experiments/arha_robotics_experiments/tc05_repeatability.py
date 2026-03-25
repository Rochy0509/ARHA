#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
import time
import math
import csv

try:
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    import numpy as np
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False


class TC05RepeatabilityTester(Node):
    def __init__(self):
        super().__init__('tc05_repeatability')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
    def get_tcp_pose(self):
        try:
            # We look up the exact Cartesian position of the Right Wrist
            t = self.tf_buffer.lookup_transform('world_link', 'WristRD_link', rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y, t.transform.translation.z)
        except Exception as e:
            return None


def main(args=None):
    rclpy.init(args=args)
    node = TC05RepeatabilityTester()
    
    # Spin in background just for TF updates
    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    print("\n" + "="*60)
    print("  TC-05 ISO 9283 POSE REPEATABILITY VALIDATOR")
    print("  Tolerance: <= 1.0 mm Maximum Euclidean Spread")
    print("="*60)
    
    print("\n[Protocol]")
    print("1. Teach a 'Home' shrinking configuration in your Teach Pendant.")
    print("2. Teach a 'Target' mid-workspace pose.")
    print("3. Loop 10 times: Drive Home -> Drive Target")
    print("4. We will record the exact physical TCP XYZ position at Target 10 times in a row.")
    
    trials = 10
    recorded_points = []
    
    for i in range(1, trials + 1):
        print(f"\n--- Trial {i}/{trials} ---")
        input("Drive robot to Home, then drive to Target. Press Enter when settled at Target...")
        
        # Give it a second to physically settle
        time.sleep(1.0)
        
        pt = node.get_tcp_pose()
        if pt is None:
            print("[ERROR] Could not read TF for WristRD_link. Ensure robot state publisher is running.")
            return
            
        print(f"Recorded TCP (m): {pt[0]:.5f}, {pt[1]:.5f}, {pt[2]:.5f}")
        recorded_points.append(pt)
        
    print("\n" + "="*60)
    print("  RESULTS ANALYSIS")
    print("="*60)
    
    # Convert to mm
    pts_mm = [(p[0]*1000, p[1]*1000, p[2]*1000) for p in recorded_points]
    
    # Compute Centroid
    cx = sum(p[0] for p in pts_mm) / trials
    cy = sum(p[1] for p in pts_mm) / trials
    cz = sum(p[2] for p in pts_mm) / trials
    
    centroid = (cx, cy, cz)
    print(f"\nCentroid (mm):  X: {cx:.3f},  Y: {cy:.3f},  Z: {cz:.3f}")
    
    # Compute Maximum Pairwise Euclidean Spread (Diameter of the point cloud)
    max_spread = 0.0
    p1_worst, p2_worst = None, None
    
    for i in range(trials):
        for j in range(i+1, trials):
            dist = math.sqrt((pts_mm[i][0] - pts_mm[j][0])**2 + 
                             (pts_mm[i][1] - pts_mm[j][1])**2 + 
                             (pts_mm[i][2] - pts_mm[j][2])**2)
            if dist > max_spread:
                max_spread = dist
                p1_worst = i
                p2_worst = j
                
    passed = max_spread <= 1.0
    
    print(f"\nMaximum TCP Spread (Worst-case distance between ANY two trials):")
    print(f" > {max_spread:.4f} mm")
    print(f"\nTC-05 PASS/FAIL CRITERION (<= 1.0 mm):")
    print(f" > {'PASS ✓' if passed else 'FAIL ✗'}")
    
    # Save CSV
    with open('tc05_repeatability.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Trial', 'X_mm', 'Y_mm', 'Z_mm', 'Dist_To_Centroid_mm'])
        for i, p in enumerate(pts_mm):
            dist_to_c = math.sqrt((p[0]-cx)**2 + (p[1]-cy)**2 + (p[2]-cz)**2)
            writer.writerow([i+1, f"{p[0]:.4f}", f"{p[1]:.4f}", f"{p[2]:.4f}", f"{dist_to_c:.4f}"])
            
    print("\nRaw data saved to 'tc05_repeatability.csv'")
    
    # Plotting
    if MATPLOTLIB_AVAILABLE:
        print("Generating 3D validation scatter plot...")
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        xs = [p[0] for p in pts_mm]
        ys = [p[1] for p in pts_mm]
        zs = [p[2] for p in pts_mm]
        
        ax.scatter(xs, ys, zs, c='b', marker='o', s=50, label='Trial TCP Positions')
        ax.scatter([cx], [cy], [cz], c='r', marker='x', s=100, label='Centroid')
        
        # Force equal aspect ratio plotting
        max_range = np.array([max(xs)-min(xs), max(ys)-min(ys), max(zs)-min(zs)]).max() / 2.0
        # Give it a 0.5mm baseline so the 1mm sphere actually fits
        if max_range < 0.6: max_range = 0.6
        
        # Attempt to draw a 1mm total diameter sphere (r=0.5mm) around centroid to prove it visually
        u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
        r = 0.5 
        sphere_x = cx + r * np.cos(u) * np.sin(v)
        sphere_y = cy + r * np.sin(u) * np.sin(v)
        sphere_z = cz + r * np.cos(v)
        ax.plot_surface(sphere_x, sphere_y, sphere_z, color='g', alpha=0.15, edgecolor='none')
        
        # Explicitly label the green sphere in the legend
        import matplotlib.patches as mpatches
        sphere_proxy = mpatches.Patch(color='g', alpha=0.3, label='1mm Spread Tolerance Boundary')
        handles, labels = ax.get_legend_handles_labels()
        handles.append(sphere_proxy)
        
        ax.set_title(f"TC-05 ISO 9283 Pose Repeatability\nMax Spread: {max_spread:.4f} mm ({'PASS' if passed else 'FAIL'})", fontweight='bold')
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')
        
        ax.set_xlim(cx - max_range, cx + max_range)
        ax.set_ylim(cy - max_range, cy + max_range)
        ax.set_zlim(cz - max_range, cz + max_range)
        
        plt.legend(handles=handles, loc='upper left')
        plt.tight_layout()
        plt.savefig('tc05_repeatability_plot.png', dpi=300)
        print("Validation graph saved to 'tc05_repeatability_plot.png'")
        
    rclpy.shutdown()

if __name__ == '__main__':
    main()
