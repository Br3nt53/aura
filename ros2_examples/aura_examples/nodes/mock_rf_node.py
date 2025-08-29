import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from aura_interfaces.msg import AuraRfHint, AuraRfHintList
import math
import random


class MockRfNode(Node):
    def __init__(self):
        super().__init__("mock_rf_node")
        self.subscription = self.create_subscription(
            PoseStamped, "/aura/ground_truth", self.gt_callback, 10
        )
        self.publisher_ = self.create_publisher(
            AuraRfHintList, "/aura/rf_presence_hint", 10
        )

        # Declare parameters for more realistic simulation
        self.declare_parameter("sensor_range", 10.0)
        self.declare_parameter("wall_attenuation", 0.5)
        self.declare_parameter("detection_probability", 1.0)  # 1.0 = 100% chance
        self.declare_parameter(
            "positional_noise_stddev", 0.0
        )  # Standard deviation in meters

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.latest_gt = {}

    def gt_callback(self, msg):
        self.latest_gt[msg.header.frame_id] = msg.pose.position

    def timer_callback(self):
        sensor_range = (
            self.get_parameter("sensor_range").get_parameter_value().double_value
        )
        wall_attenuation = (
            self.get_parameter("wall_attenuation").get_parameter_value().double_value
        )
        detection_probability = (
            self.get_parameter("detection_probability")
            .get_parameter_value()
            .double_value
        )
        noise_stddev = (
            self.get_parameter("positional_noise_stddev")
            .get_parameter_value()
            .double_value
        )

        msg_list = AuraRfHintList()
        msg_list.header.stamp = self.get_clock().now().to_msg()

        for target_id, pos in self.latest_gt.items():
            # 1. Check for detection probability
            if random.random() > detection_probability:
                continue  # Target was not detected in this frame

            d = math.sqrt(pos.x**2 + pos.y**2)
            if d < sensor_range:
                hint = AuraRfHint()

                # 2. Add positional noise
                hint.x = pos.x + random.gauss(0, noise_stddev)
                hint.y = pos.y + random.gauss(0, noise_stddev)

                # Simplified signal strength model
                signal = 1.0 / (d**2 + 1.0)

                # Simple wall model: if target is in a different "room" (quadrant)
                if pos.x * 0 < 0 or pos.y * 0 < 0:
                    signal *= wall_attenuation

                hint.signal = signal
                msg_list.hints.append(hint)

        self.publisher_.publish(msg_list)


def main(args=None):
    rclpy.init(args=args)
    node = MockRfNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
