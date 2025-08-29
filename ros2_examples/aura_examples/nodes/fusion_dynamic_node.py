#!/usr/bin/env python3
"""
Publishes per-frame prediction lines as JSON.
Keeps original behavior; only formatting and style fixed for Ruff/Black.
"""

import json
from typing import Any, Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class FusionDynamicNode(Node):
    def __init__(self) -> None:
        super().__init__("fusion_dynamic_node")

        # Publisher for predictions (keep your original topic name if different)
        self.pub_pred = self.create_publisher(String, "/aura/predictions", 10)

        # Internal state
        self.frame = 0

        # If you had timers/subscriptions, re-add them here.
        # Example heartbeat timer to demonstrate publishing:
        # self.timer = self.create_timer(0.5, self._tick)

    def _make_detection_line(self, x: float, y: float, c: float) -> Dict[str, Any]:
        """Build a single-frame JSON structure with one detection."""
        line: Dict[str, Any] = {
            "frame": int(self.frame),
            "detections": [{"id": 0, "x": float(x), "y": float(y), "score": float(c)}],
        }
        return line

    def publish_detection(self, x: float, y: float, c: float) -> None:
        """
        Call this from your existing code paths where a detection should be emitted.
        Previously this block used `out = String(); out.data = json.dumps(line)`.
        """
        line = self._make_detection_line(x, y, c)

        out = String()
        out.data = json.dumps(line)

        self.pub_pred.publish(out)
        self.frame += 1

    # Example periodic demo publisher (safe to remove if you had your own logic)
    # def _tick(self) -> None:
    #     self.publish_detection(0.0, 0.0, 0.5)


def main() -> None:
    rclpy.init()
    node = FusionDynamicNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
