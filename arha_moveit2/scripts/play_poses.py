#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import time

class PoseSequencer(Node):
    def __init__(self):
        super().__init__('pose_sequencer')
        
        self.left_client = ActionClient(self, FollowJointTrajectory, '/left_arm_controller/follow_joint_trajectory')
        self.right_client = ActionClient(self, FollowJointTrajectory, '/right_arm_controller/follow_joint_trajectory')

        self.get_logger().info('Waiting for action servers (make sure MoveIt/hardware is launched!)...')
        self.left_client.wait_for_server()
        self.right_client.wait_for_server()
        self.get_logger().info('Action servers found!')

        self.left_joints = [
            'ShoulderLU_joint', 'ShoulderLD_joint', 'ElbowLU_joint', 
            'ElbowLD_joint', 'WristLU_joint', 'WristLD_joint'
        ]
        
        self.right_joints = [
            'ShoulderRU_joint', 'ShoulderRD_joint', 'ElbowRU_joint', 
            'ElbowRD_joint', 'WristRU_joint', 'WristRD_joint'
        ]
        
        # Poses extracted from the user's data
        self.poses = [
            { # Pose 1
                'left':  [-1.5706,  0.0, -0.0001, -1.5706, -0.0001,  0.0],
                'right': [ 1.5706,  0.0,  0.0,     1.5704,  0.0,     0.0]
            },
            { # Pose 2
                'left':  [-1.5704,  0.0,  0.0,     0.0,     0.0,     0.0],
                'right': [ 0.0,     0.0,  0.0,     0.0,     0.0,     0.0]
            },
            { # Pose 3
                'left':  [-1.5706,  0.0,  0.0,     0.0,     1.5707,  0.0],
                'right': [ 1.5636,  0.0,  0.0380,  0.0,    -1.5704,  0.0]
            },
            { # Pose 4 (Confirmed different from pose 3)
                'left':  [-1.0335,  0.0,  1.5645, -1.5704, -1.1306,  0.0],
                'right': [ 1.5704,  0.0, -1.5704,  1.5706, -1.5702, -0.0078]
            },
            { # Pose 5
                'left':  [-1.5704,  0.0,  0.0075, -1.5706, -1.5704,  0.0],
                'right': [ 1.5704,  0.0,  0.0054,  0.0,    -1.5706, -0.0078]
            },
            { # Pose 6
                'left':  [ 0.0055,  0.3654,  1.5666, -1.5706, -1.5704,  0.0],
                'right': [ 0.0055, -0.3652, -1.5706,  1.5704,  0.0,    -0.0080]
            }
        ]

    def send_goal(self, client, joint_names, positions, duration_sec):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = duration_sec
        point.time_from_start.nanosec = 0
        
        goal_msg.trajectory.points.append(point)
        
        arm_name = 'Left' if 'L' in joint_names[0] else 'Right'
        self.get_logger().info(f'  -> Sending trajectory goal for {arm_name} arm...')
        future = client.send_goal_async(goal_msg)
        return future

    def play_all_poses(self):
        duration = 1 # 3 seconds per movement
        
        for idx, pose in enumerate(self.poses):
            self.get_logger().info(f'\n--- Playing Pose {idx + 1} ---')
            
            # Send right arm first as requested
            self.send_goal(self.right_client, self.right_joints, pose['right'], duration)
            
            # Stagger delay: allow right arm to begin moving out of the way before the left arm starts
            time.sleep(1.0) 
            
            # Send left arm
            self.send_goal(self.left_client, self.left_joints, pose['left'], duration)
            
            # Wait for both arms to complete their trajectory (remaining duration + safety buffer)
            time.sleep(duration)

        self.get_logger().info('\nAll poses finished successfully!')

def main(args=None):
    rclpy.init(args=args)
    node = PoseSequencer()
    
    try:
        node.play_all_poses()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
