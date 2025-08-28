#!/usr/bin/env python3
import rclpy, yaml, math
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32
from pathlib import Path

class ScenarioPlayer(Node):
    def __init__(self):
        super().__init__('scenario_player')
        self.declare_parameter('scenario', 'scenarios/crossing_targets.yaml')
        self.declare_parameter('loop', False)
        scenario_path = self.get_parameter('scenario').get_parameter_value().string_value
        self.loop = self.get_parameter('loop').get_parameter_value().bool_value
        self.scenario_path = scenario_path
        data = yaml.safe_load(open(scenario_path, 'r'))
        self.dt = float(data.get('sim', {}).get('dt_sec', 0.05))
        self.duration = float(data.get('sim', {}).get('duration_sec', 10.0))
        self.nframes = max(1, int(round(self.duration / self.dt)))
        self.targets = data.get('targets', [])
        self.pub_pose = self.create_publisher(Pose, '/mock_target/pose', 10)
        self.pub_frame = self.create_publisher(Int32, '/aura/frame', 10)
        self.frame = 0
        self.timer = self.create_timer(self.dt, self.tick)
        self.get_logger().info(f"ScenarioPlayer loaded {scenario_path} for {self.nframes} frames at dt={self.dt}s")

    def tick(self):
        if self.frame >= self.nframes:
            if self.loop:
                self.frame = 0
            else:
                self.get_logger().info("Scenario complete; shutting down ScenarioPlayer.")
                self.destroy_node()
                rclpy.shutdown()
                return
        # Multi-target: publish each as a Pose; consumer can choose one or more
        # Here we publish one Pose per frame (first target) for simplicity.
        # You can extend to PoseArray if desired.
        msg = Pose()
        objs = self._objects_at(self.frame)
        if objs:
            msg.position.x = float(objs[0]['x'])
            msg.position.y = float(objs[0]['y'])
            msg.position.z = 1.0
        msg.orientation.w = 1.0
        self.pub_pose.publish(msg)
        self.pub_frame.publish(Int32(data=int(self.frame)))
        self.frame += 1

    def _objects_at(self, fi: int):
        tsec = fi * self.dt
        objs = []
        for t in self.targets:
            sx, sy = t['start']
            ex, ey = t['end']
            speed = float(t.get('speed_mps', 0.0))
            dist = math.hypot(ex - sx, ey - sy)
            if speed > 0.0 and dist > 1e-6:
                total_time = dist / speed
                frac = min(1.0, tsec / max(1e-9, total_time))
            else:
                frac = min(1.0, tsec / max(1e-9, self.duration))
            x = sx + (ex - sx) * frac
            y = sy + (ey - sy) * frac
            objs.append({'id': int(t['id']), 'x': x, 'y': y})
        return objs

def main():
    rclpy.init()
    n = ScenarioPlayer()
    rclpy.spin(n)

if __name__ == "__main__":
    main()
