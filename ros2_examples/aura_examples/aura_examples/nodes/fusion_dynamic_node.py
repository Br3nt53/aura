#!/usr/bin/env python3
import json, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import Float32MultiArray, String

class FusionDynamicNode(Node):
    """
    Dynamically discovers topics of type std_msgs/Float32MultiArray whose name ends with '/presence_hint'
    and subscribes to them. Each message is interpreted as [confidence, x, y, z].
    Outputs predictions JSONL on /aura/predictions_jsonl.
    """
    def __init__(self):
        super().__init__('fusion_dynamic_node')
        self.pub_pred = self.create_publisher(String, '/aura/predictions_jsonl', 20)
        self.subs = {}  # topic -> Subscription
        self.frame = 0
        self.qos = QoSProfile(depth=10)
        self.create_timer(1.0, self.refresh_topics)

    def refresh_topics(self):
        topics = self.get_topic_names_and_types()
        for name, types in topics:
            if not name.endswith('/presence_hint'):
                continue
            if 'std_msgs/msg/Float32MultiArray' not in types:
                continue
            if name in self.subs:
                continue
            self.get_logger().info(f"[fusion] subscribing to {name}")
            sub = self.create_subscription(Float32MultiArray, name, self.on_hint, 10)
            self.subs[name] = sub

    def on_hint(self, msg: Float32MultiArray):
        try:
            c, x, y, z = msg.data[:4]
        except Exception:
            return
        line = {"frame": int(self.frame), "detections": [{"id": 0, "x": float(x), "y": float(y), "score": float(c)}]}
        out = String(); out.data = json.dumps(line)
        self.pub_pred.publish(out)
        self.frame += 1

def main():
    rclpy.init()
    n = FusionDynamicNode()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
