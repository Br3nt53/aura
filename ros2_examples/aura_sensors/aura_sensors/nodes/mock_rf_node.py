#!/usr/bin/env python3
"""
Example ROS 2 node: Physics-inspired RF presence hints.
- Path loss ~ 1/(d^2 + 1)
- Simple "wall" attenuation heuristic when x>0
Publish to: /aura/rf_presence_hint (example topic)
"""
import math
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

# Replace with your real message type if you have one
from std_msgs.msg import Float32MultiArray as PresenceHintMsg

class MockRfNode(Node):
    def __init__(self):
        super().__init__('mock_rf_node')
        self.sub = self.create_subscription(Pose, '/mock_target/pose', self.on_pose, 10)
        self.pub = self.create_publisher(PresenceHintMsg, '/aura/rf_presence_hint', 10)
        self.sensor_pos = np.array([0.0, 0.0, 1.0])
        self.wall_atten = 0.3
        self.min_conf = 0.1
        self.get_logger().info("Mock RF node (physics-based) started. Subscribing to /mock_target/pose")

    def on_pose(self, msg: Pose):
        target = np.array([msg.position.x, msg.position.y, msg.position.z if msg.position.z else 1.0])
        d = np.linalg.norm(target - self.sensor_pos)
        conf = 1.0 / (d*d + 1.0)
        if target[0] > 0:  # behind "wall" heuristic
            conf *= self.wall_atten
        if conf < self.min_conf:
            return
        out = PresenceHintMsg()
        # Pack: [confidence, x, y, z]
        out.data = [float(conf), float(target[0]), float(target[1]), float(target[2])]
        self.pub.publish(out)

def main():
    rclpy.init()
    n = MockRfNode()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
