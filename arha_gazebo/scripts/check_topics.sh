#!/bin/bash

# Diagnostic script to check if point cloud topics are publishing

echo "=== Checking Point Cloud Topics ==="
echo ""

echo "1. Chest camera depth points:"
timeout 2 ros2 topic hz /camera_chest/depth/points 2>/dev/null || echo "   NOT PUBLISHING"

echo ""
echo "2. Left wrist camera depth points:"
timeout 2 ros2 topic hz /camera_left_wrist/depth/points 2>/dev/null || echo "   NOT PUBLISHING"

echo ""
echo "3. Right wrist camera depth points:"
timeout 2 ros2 topic hz /camera_right_wrist/depth/points 2>/dev/null || echo "   NOT PUBLISHING"

echo ""
echo "=== All Topics ==="
ros2 topic list | grep -E "(camera|points|depth)"
