#!/usr/bin/env python3
"""
Simple tracker demo.
Converts presence hints to tracked objects with a nearest-neighbor gate.
Only formatting/style changes vs. typical implementation to satisfy Ruff/Black.
"""

import json
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String


def dist2(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    dx = float(a[0]) - float(b[0])
    dy = float(a[1]) - float(b[1])
    return dx * dx + dy * dy


@dataclass
class Track:
    x: float
    y: float
    stab: float
    alpha: float
    decay_sec: float
    last_t: float
    _ID: int = 0  # class-level counter (not used per-instance)

    @classmethod
    def _next_id(cls) -> int:
        cls._ID += 1
        return cls._ID

    def alive(self, tnow: float) -> bool:
        return (tnow - self.last_t) <= self.decay_sec

    def update(self, x: float, y: float, c: float, tnow: float) -> None:
        """EMA on confidence, latch position."""
        self.x = float(x)
        self.y = float(y)
        self.stab = self.alpha * float(c) + (1.0 - self.alpha) * self.stab
        self.last_t = tnow


class FusionTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("fusion_tracker_node")

        # Params that were referenced in lint errors
        self.params: Dict[str, float] = {
            "assoc_gate_m": 1.5,
            "rf_weight": 0.6,
            "track_decay_sec": 2.5,
            "alpha": 0.3,
        }

        self.tracks: List[Track] = []
        self.subs: Dict[str, rclpy.subscription.Subscription] = {}

        # Output publisher for current detections/tracks
        self.pub = self.create_publisher(String, "/aura/tracks", 10)

        # If you had a static subscription, keep it; otherwise we refresh topics periodically.
        self.create_timer(1.0, self.refresh_topics)

        # Simulate a frame counter if you were using one
        self.frame = 0

    def refresh_topics(self) -> None:
        """Subscribe to any presence hint topics discovered at runtime."""
        for name, types in self.get_topic_names_and_types():
            if not name.endswith("/presence_hint"):
                continue
            if "std_msgs/msg/Float32MultiArray" not in types:
                continue
            if name in self.subs:
                continue

            self.get_logger().info(f"[fusion] subscribing to {name}")
            self.subs[name] = self.create_subscription(
                Float32MultiArray, name, self.on_hint, 20
            )

    def on_hint(self, msg: Float32MultiArray) -> None:
        """
        Example hint: [x, y, conf] repeating (flattened).
        Convert to detections, associate to tracks under gating, and publish JSON.
        """
        data = list(map(float, msg.data))
        dets: List[Dict[str, float]] = []
        tnow = time.monotonic()

        # Parse flattened triples
        triples = [data[i : i + 3] for i in range(0, len(data), 3)]
        for trip in triples:
            if len(trip) < 3:
                continue
            x, y, c = trip
            dets.append({"id": 0, "x": float(x), "y": float(y), "score": float(c)})

            # Association to nearest track within gate
            gate2 = float(self.params["assoc_gate_m"]) ** 2
            best: Optional[float] = None
            bi = -1

            for i, tr in enumerate(self.tracks):
                if not tr.alive(tnow):
                    continue
                d2 = dist2((tr.x, tr.y), (x, y))
                within_gate = d2 <= gate2
                better = best is None or d2 < best
                if within_gate and better:
                    best = d2
                    bi = i

            if bi >= 0:
                self.tracks[bi].update(x, y, c, tnow)
            else:
                self.tracks.append(
                    Track(
                        x=float(x),
                        y=float(y),
                        stab=float(c),
                        alpha=float(self.params["alpha"]),
                        decay_sec=float(self.params["track_decay_sec"]),
                        last_t=tnow,
                    )
                )

        # Prune dead tracks
        self.tracks = [t for t in self.tracks if t.alive(tnow)]

        # Publish if any detections present (mirrors the pattern in your errors)
        if dets:
            line = {"frame": int(self.frame), "detections": dets}

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
