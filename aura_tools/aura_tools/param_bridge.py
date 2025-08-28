#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

class ParamBridge(Node):
    def __init__(self):
        super().__init__('param_bridge')
        self.sub = self.create_subscription(String, '/aura/param_set', self.on_msg, 10)
        self.clients = {}  # service clients per node

    def on_msg(self, msg: String):
        try:
            p = json.loads(msg.data)
            node = p.get("node","/aura_fusion")           # target node name
            name = p["name"]                               # e.g. "assoc_gate_m"
            value = p["value"]                             # float/int/bool
        except Exception as e:
            self.get_logger().error(f"bad JSON: {e}")
            return

        cli = self.clients.get(node)
        if cli is None:
            srv = f"{node}/set_parameters"
            cli = self.create_client(SetParameters, srv)
            self.clients[node] = cli

        if not cli.service_is_ready():
            if not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f"no service for {node}")
                return

        pv = ParameterValue()
        if isinstance(value, float):
            pv.type = ParameterType.PARAMETER_DOUBLE; pv.double_value = float(value)
        elif isinstance(value, bool):
            pv.type = ParameterType.PARAMETER_BOOL; pv.bool_value = bool(value)
        elif isinstance(value, int):
            pv.type = ParameterType.PARAMETER_INTEGER; pv.integer_value = int(value)
        else:
            pv.type = ParameterType.PARAMETER_STRING; pv.string_value = str(value)

        req = SetParameters.Request()
        req.parameters = [Parameter(name=name, value=pv)]
        fut = cli.call_async(req)
        def done(_):
            self.get_logger().info(f"set {node}:{name}={value}")
        fut.add_done_callback(done)

def main():
    rclpy.init()
    n = ParamBridge()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
