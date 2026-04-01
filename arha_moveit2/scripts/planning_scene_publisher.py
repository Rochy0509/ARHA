#!/usr/bin/env python3
"""
planning_scene_publisher.py

Publishes known static obstacles (e.g. the manipulation table) into the
MoveIt2 planning scene at startup. This gives the OMPL planner hard
collision geometry rather than relying solely on slow Octomap updates.

Run after move_group is ready (use a TimerAction or delay in launch).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header


class PlanningScenePublisher(Node):
    def __init__(self):
        super().__init__('planning_scene_publisher')

        # Latch the planning scene topic so late subscribers receive it
        self.publisher = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            qos_profile=10
        )

        # Give move_group a moment to subscribe after startup
        self.timer = self.create_timer(1.0, self.publish_scene)
        self.published = False
        self.get_logger().info('Planning scene publisher ready, waiting 1s...')

    def publish_scene(self):
        if self.published:
            return
        self.published = True
        self.timer.cancel()

        scene = PlanningScene()
        scene.is_diff = True  # merge into existing scene, not replace

        # Table — matches arha_manipulation.world:
        #   pose:  x=0.18, y=-0.50, z=0.20, yaw=pi/2
        #   size:  0.6 x 1.0 x 0.4 m  (top surface at z=0.40)
        table = CollisionObject()
        table.header = Header()
        table.header.frame_id = 'world'
        table.id = 'manipulation_table'

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.4, 0.4, 0.4]
        table.primitives.append(box)

        pose = Pose()
        pose.position = Point(x=0.18, y=-0.50, z=0.20)
        # yaw = pi/2  → quaternion (0, 0, sin(pi/4), cos(pi/4))
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.7071068, w=0.7071068)
        table.primitive_poses.append(pose)

        table.operation = CollisionObject.ADD
        scene.world.collision_objects.append(table)

        # Floor — thin plane so planner doesn't go underground
        floor = CollisionObject()
        floor.header = Header()
        floor.header.frame_id = 'world'
        floor.id = 'floor'

        floor_box = SolidPrimitive()
        floor_box.type = SolidPrimitive.BOX
        floor_box.dimensions = [4.0, 4.0, 0.02]
        floor.primitives.append(floor_box)

        floor_pose = Pose()
        floor_pose.position = Point(x=0.0, y=0.0, z=-0.05)
        floor_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        floor.primitive_poses.append(floor_pose)

        floor.operation = CollisionObject.ADD
        scene.world.collision_objects.append(floor)

        self.publisher.publish(scene)
        self.get_logger().info(
            'Published static planning scene: table + floor collision objects'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PlanningScenePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
