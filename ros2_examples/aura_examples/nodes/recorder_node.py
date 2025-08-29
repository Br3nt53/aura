#!/usr/bin/env python3
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RecorderNode(Node):
    def __init__(self):
        super().__init__("recorder_node")
        self.declare_parameter("workdir", "out/tmp")
        workdir = self.get_parameter("workdir").get_parameter_value().string_value
        self.path = Path(workdir) / "pred.jsonl"
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.f = self.path.open("w")
        self.get_logger().info(f"Recording predictions to {self.path}")
        self.sub = self.create_subscription(
            String, "/aura/predictions_jsonl", self.on_pred, 50
        )

    def on_pred(self, msg: String):
        self.f.write(msg.data.strip() + "\n")
        self.f.flush()

    def destroy_node(self):
        try:
            if hasattr(self, "f"):
                self.f.close()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    n = RecorderNode()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
