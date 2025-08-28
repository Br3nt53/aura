#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from pynput import keyboard

class TeleopTargetNode(Node):
    def __init__(self):
        super().__init__('teleop_target_node')
        self.pub = self.create_publisher(Pose, '/mock_target/pose', 10)
        self.pos = np.array([0.0, -2.0, 1.0], dtype=float)
        self.vel = 2.0
        self.dt = 0.05
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()
        self.create_timer(self.dt, self.tick)
        self.get_logger().info("Teleop Target: WASD to move")

    def on_press(self, key):
        try:
            if key.char == 'w': self.pos[0] += self.vel * self.dt
            elif key.char == 's': self.pos[0] -= self.vel * self.dt
            elif key.char == 'a': self.pos[1] += self.vel * self.dt
            elif key.char == 'd': self.pos[1] -= self.vel * self.dt
        except AttributeError:
            pass

    def tick(self):
        msg = Pose()
        msg.position.x, msg.position.y, msg.position.z = map(float, self.pos)
        msg.orientation.w = 1.0
        self.pub.publish(msg)

def main():
    rclpy.init()
    n = TeleopTargetNode()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
