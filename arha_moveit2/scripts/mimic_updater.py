#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class MimicUpdater(Node):
    def __init__(self):
        super().__init__('mimic_updater')
        self.sub = self.create_subscription(JointState, '/joint_states', self.js_cb, 10)
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Pincopen gripper mimic rules from URDF
        self.rules = {
            'left_Revolute_2_0': ('left_cam_joint', -1.0),
            'left_Revolute_5_0': ('left_cam_joint', -1.0),
            'left_Revolute_3_0': ('left_cam_joint', 1.0),
            'left_Revolute_7_0': ('left_cam_joint', 1.0),
            'left_Revolute_6_0': ('left_cam_joint', -1.0),
            'left_Revolute_4_0': ('left_cam_joint', 1.0),
            
            'right_Revolute_2_0': ('right_cam_joint', -1.0),
            'right_Revolute_5_0': ('right_cam_joint', -1.0),
            'right_Revolute_3_0': ('right_cam_joint', 1.0),
            'right_Revolute_7_0': ('right_cam_joint', 1.0),
            'right_Revolute_6_0': ('right_cam_joint', -1.0),
            'right_Revolute_4_0': ('right_cam_joint', 1.0),
        }

    def js_cb(self, msg: JointState):
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        
        has_cam = False
        for i, name in enumerate(msg.name):
            if name in ['left_cam_joint', 'right_cam_joint']:
                has_cam = True
                
            # For any joint that drives a mimic, generate the mimic state
            for mimic_name, (parent_name, mult) in self.rules.items():
                if name == parent_name:
                    out.name.append(mimic_name)
                    out.position.append(msg.position[i] * mult)
                    
        # Only publish if we generated mimic joints
        if has_cam and out.name:
            self.pub.publish(out)

def main():
    rclpy.init()
    node = MimicUpdater()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
