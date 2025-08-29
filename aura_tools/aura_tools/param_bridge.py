#!/usr/bin/env python3
"""
Param Bridge: subscribes to /aura/param_set (std_msgs/String) with JSON payloads like:
  {"node": "/aura_fusion", "name": "assoc_gate_m", "value": 1.2}

It forwards them to the target node's SetParameters service.
"""

import json
from typing import Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


class ParamBridge(Node):
    def __init__(self) -> None:
        super().__init__("param_bridge")

        # Subscribes to JSON commands to set parameters on a target node.
        self.sub = self.create_subscription(String, "/aura/param_set", self.on_msg, 10)

        # Cache of service clients keyed by target node name.
        self.clients: Dict[str, rclpy.client.Client] = {}

    def on_msg(self, msg: String) -> None:
        """Handle a JSON command and call SetParameters on the target node."""
        try:
            payload = json.loads(msg.data)
            node = payload.get("node", "/aura_fusion")  # default target node
            name = payload["name"]  # parameter name, e.g. "assoc_gate_m"
            value = payload["value"]  # bool/int/float/str
        except Exception as e:  # noqa: BLE001 (keep broad here to log any JSON error)
            self.get_logger().error(f"bad JSON on /aura/param_set: {e}")
            return

        # Get or create a client for the node's SetParameters service.
        cli = self.clients.get(node)
        if cli is None:
            service_name = f"{node}/set_parameters"
            cli = self.create_client(SetParameters, service_name)
            self.clients[node] = cli

        # Ensure the service is available.
        if not cli.service_is_ready():
            if not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().warning(f"SetParameters not ready for {node}")
                return

        # Build ParameterValue from Python type.
        pv = ParameterValue()
        if isinstance(value, float):
            pv.type = ParameterType.PARAMETER_DOUBLE
            pv.double_value = float(value)
        elif isinstance(value, bool):
            pv.type = ParameterType.PARAMETER_BOOL
            pv.bool_value = bool(value)
        elif isinstance(value, int):
            pv.type = ParameterType.PARAMETER_INTEGER
            pv.integer_value = int(value)
        else:
            # Fall back to string; this also handles None or unsupported types.
            pv.type = ParameterType.PARAMETER_STRING
            pv.string_value = str(value)

        # Prepare and send the request.
        req = SetParameters.Request()
        req.parameters = [Parameter(name=name, value=pv)]
        future = cli.call_async(req)

        def _done(_: rclpy.task.Future) -> None:
            self.get_logger().info(f"set {node}:{name}={value}")

        future.add_done_callback(_done)


def main() -> None:
    rclpy.init()
    node = ParamBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
