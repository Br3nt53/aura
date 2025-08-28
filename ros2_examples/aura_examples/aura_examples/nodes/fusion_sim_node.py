#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from std_msgs.msg import Int32

class FusionSimNode(Node):
    """
    Minimal fusion simulator:
    - Subscribes to RF presence hints: Float32MultiArray [confidence, x, y, z]
    - Subscribes to frame counter (optional): /aura/frame (Int32), defaults to internal counter
    - Publishes predictions JSONL lines on /aura/predictions_jsonl
      {"frame": N, "detections": [{"id": 0, "x": X, "y": Y, "score": C}]}
    """
    def __init__(self):
        super().__init__('fusion_sim_node')
        self.sub_hint = self.create_subscription(Float32MultiArray, '/aura/rf_presence_hint', self.on_hint, 10)
        self.sub_frame = self.create_subscription(Int32, '/aura/frame', self.on_frame, 10)
        self.pub_pred = self.create_publisher(String, '/aura/predictions_jsonl', 10)
        self.frame = 0

    def on_frame(self, msg: Int32):
        self.frame = int(msg.data)

    def on_hint(self, msg: Float32MultiArray):
        try:
            c, x, y, z = msg.data[:4]
        except Exception:
            return
        line = {"frame": int(self.frame), "detections": [{"id": 0, "x": float(x), "y": float(y), "score": float(c)}]}
        out = String()
        out.data = __import__('json').dumps(line)
        self.pub_pred.publish(out)
        self.frame += 1  # if no external frame, this increments

def main():
    rclpy.init()
    n = FusionSimNode()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
