#!/usr/bin/env python3
"""
Keyboard teleop for a moving target.
Publishes geometry_msgs/Pose on /mock_target/pose

Requires: pip install pynput
Controls: WASD to move in x/y plane.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from pynput import keyboard


class TeleopTargetNode(Node):
    def __init__(self) -> None:
        super().__init__("teleop_target_node")
        self.pub = self.create_publisher(Pose, "/mock_target/pose", 10)

        # Position: start slightly offset
        self.pos = np.array([0.0, -2.0, 1.0])
        self.vel = 2.0  # m/s
        self.dt = 0.05  # timestep

        # Start keyboard listener in background
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

        # Publish loop
        self.create_timer(self.dt, self.tick)

        self.get_logger().info("Teleop Target Started. Use WASD keys to move.")

    def on_press(self, key) -> None:
        """Handle WASD keypresses to move the mock target."""
        try:
            if key.char == "w":
                self.pos[0] += self.vel * self.dt
            elif key.char == "s":
                self.pos[0] -= self.vel * self.dt
            elif key.char == "a":
                self.pos[1] += self.vel * self.dt
            elif key.char == "d":
                self.pos[1] -= self.vel * self.dt
        except AttributeError:
            # Non-character keys (shift, ctrl, etc.)
            pass

    def tick(self) -> None:
        """Publish the current position as a Pose message."""
        msg = Pose()
        msg.position.x = float(self.pos[0])
        msg.position.y = float(self.pos[1])
        msg.position.z = float(self.pos[2])
        msg.orientation.w = 1.0
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = TeleopTargetNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
