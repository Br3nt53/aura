#!/usr/bin/env python3
import json
import time
from typing import Dict, List, Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from ..trackers.legacy_tracker import LegacyTracker


class FusionTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("fusion_tracker_node")

        # In a real application, these would come from ROS 2 parameters
        params: Dict[str, Any] = {
            "assoc_gate_m": 1.5,
            "track_decay_sec": 2.5,
            "alpha": 0.3,
        }
        self.tracker = LegacyTracker(params)
        self.subs: Dict[str, rclpy.subscription.Subscription] = {}

        self.pub = self.create_publisher(String, "/aura/tracks", 10)
        self.create_timer(1.0, self.refresh_topics)
        self.frame = 0

    def refresh_topics(self) -> None:
        """Subscribe to any presence hint topics discovered at runtime."""
        for name, types in self.get_topic_names_and_types():
            if not name.endswith("/presence_hint"):
                continue
            if name in self.subs:
                continue

            self.get_logger().info(f"[fusion] subscribing to {name}")
            self.subs[name] = self.create_subscription(
                Float32MultiArray, name, self.on_hint, 20
            )

    def on_hint(self, msg: Float32MultiArray) -> None:
        data = list(map(float, msg.data))
        dets: List[Dict[str, float]] = []

        triples = [data[i : i + 3] for i in range(0, len(data), 3)]
        for trip in triples:
            if len(trip) < 3:
                continue
            x, y, c = trip
            dets.append({"x": float(x), "y": float(y), "score": float(c)})

        # Delegate all tracking logic to the tracker object
        tracks = self.tracker.update(dets, time.monotonic())

        # Publish the results
        if tracks:
            line = {"frame": self.frame, "detections": tracks}
            out = String()
            out.data = json.dumps(line)
            self.pub.publish(out)
            self.frame += 1


def main() -> None:
    rclpy.init()
    node = FusionTrackerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
