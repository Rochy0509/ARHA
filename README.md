# Autonomous Retirement Home Assistant (ARHA)
> [!NOTE]  
> Under development currently

This is the official repository for ARHA. A capstone project developed by students at Ontario Tech University, with the primary goal of creating an autonomous robot to assist in retirement homes by automating repetitive tasks that staff must perform.

## Installation

> [!IMPORTANT]
> This repository cannot be cloned directly as a workspace. The packages must be copied into your ROS2 workspace.
```bash
# Create a ROS2 workspace if you don't have one
mkdir -p ~/ARHA_ws/src
cd ~/ARHA_ws/src

# Clone the repository into src
git clone https://github.com/Rochy0509/ARHA.git

# Copy packages to workspace src (if needed, adjust based on repo structure)
# Or clone directly into src as shown above

# Build the workspace
cd ~/ARHA_ws
colcon build

# Source the workspace
source install/setup.bash
```

## Current Progress

### Robot Description & Motion Planning
- **arha_description**: Contains the complete URDF model for ARHA's dual-arm humanoid upper body with proper joint definitions and kinematic chains
- **arha_moveit2**: Fully configured MoveIt2 package ready for motion planning and trajectory execution
  - Includes planning groups for both left and right arms (6-DOF each)
  - Integrated with ros2_control for simulated trajectory execution
  - Configured with OMPL planning pipeline and RRTConnect planner

### How to Launch
```bash
# Source your ROS2 workspace
cd ~/ARHA_ws
source install/setup.bash

# Launch MoveIt2 with RViz for motion planning
ros2 launch arha_moveit2 demo.launch.py
```

In RViz, you can:
- Plan and execute trajectories for either arm using the MotionPlanning panel
- Use predefined poses: `left_arm_home`, `left_ready`, `right_arm_home`, `right_ready`
- Interactively move the end effectors and plan to goal states

## Current Tasks
 - Development of the finger sensor code
 
## To Be Created
 - Improvement of previous robotic arm code to be bimanual with robust control system using ROS2 Humble
 - Incorporation of Lerobot_ros for RL training
 - Data acquisition mechanism interface (human to robot)
 - Development of robot base navigation and task scheduling

## Current Software Team
 - Kenneth Martinez (ROS2, MoveIt2, Lerobot_ros)
 - Riley Workman (Finger sensor and support)
 - Khalil Balde (Robot Base motor control, Nav2, Sensors)
