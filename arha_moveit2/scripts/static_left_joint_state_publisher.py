#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class StaticLeftJointStatePublisher(Node):
    def __init__(self):
        super().__init__("static_left_joint_state_publisher")

        self.joint_names = [
            "ShoulderLU_joint",
            "ShoulderLD_joint",
            "ElbowLU_joint",
            "ElbowLD_joint",
            "WristLU_joint",
            "WristLD_joint",
        ]

        self.declare_parameter("rate", 10.0)
        self.declare_parameter("positions", [0.0] * len(self.joint_names))

        rate_hz = float(self.get_parameter("rate").value)
        self.positions = list(self.get_parameter("positions").value)
        if len(self.positions) != len(self.joint_names):
            self.get_logger().warn(
                "positions length (%d) does not match joint count (%d); "
                "truncating/expanding with zeros.",
                len(self.positions),
                len(self.joint_names),
            )
            padded = [0.0] * len(self.joint_names)
            for i, val in enumerate(self.positions[: len(self.joint_names)]):
                padded[i] = float(val)
            self.positions = padded

        self.publisher_ = self.create_publisher(JointState, "joint_states", 10)
        period = 1.0 / rate_hz if rate_hz > 0.0 else 0.1
        self.timer = self.create_timer(period, self.publish_joint_states)

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.positions
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = []
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StaticLeftJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
