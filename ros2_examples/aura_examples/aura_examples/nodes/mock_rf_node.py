#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray  # [confidence, x, y, z]

class MockRfNode(Node):
    def __init__(self):
        super().__init__('mock_rf_node')
        self.sub = self.create_subscription(Pose, '/mock_target/pose', self.on_pose, 10)
        self.pub = self.create_publisher(Float32MultiArray, '/aura/rf/presence_hint', 10)
        self.sensor = np.array([0.0, 0.0, 1.0], dtype=float)
        self.wall_atten = 0.3
        self.min_conf = 0.1
        self.get_logger().info("Mock RF node up. Sub: /mock_target/pose Pub: /aura/rf/presence_hint")

    def on_pose(self, msg: Pose):
        target = np.array([msg.position.x, msg.position.y, msg.position.z or 1.0], dtype=float)
        d = np.linalg.norm(target - self.sensor)
        conf = 1.0 / (d*d + 1.0)
        if target[0] > 0:
            conf *= self.wall_atten
        if conf < self.min_conf:
            return
        out = Float32MultiArray()
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
