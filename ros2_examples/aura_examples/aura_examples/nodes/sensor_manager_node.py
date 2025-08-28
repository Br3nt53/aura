#!/usr/bin/env python3
import time, os, fnmatch, subprocess, threading, signal, yaml
from pathlib import Path

import rclpy
from rclpy.node import Node

class SensorManager(Node):
    """
    Watches for device files (from udev) and launches the appropriate driver command per mapping.
    If a device disappears, the driver process is terminated.
    """
    def __init__(self):
        super().__init__('sensor_manager')
        self.declare_parameter('config', 'hardware/config/sensors.yaml')
        self.declare_parameter('scan_interval_sec', 1.0)
        cfg_path = self.get_parameter('config').get_parameter_value().string_value
        self.scan_interval = float(self.get_parameter('scan_interval_sec').get_parameter_value().double_value)
        self.config = yaml.safe_load(open(cfg_path, 'r')) if os.path.exists(cfg_path) else {"mappings": {}}
        self.get_logger().info(f"SensorManager using config: {cfg_path}")
        self.mappings = self.config.get('mappings', {})
        self.processes = {}  # dev_path -> Popen
        self.timer = self.create_timer(self.scan_interval, self.scan)

    def scan(self):
        # For each mapping pattern, check globbed devices
        active = set()
        for pattern, cmd_templ in self.mappings.items():
            for dev in fnmatch.filter(os.listdir('/dev'), os.path.basename(pattern)):
                dev_path = f"/dev/{dev}"
                active.add(dev_path)
                if dev_path not in self.processes:
                    cmd = cmd_templ.format(dev=dev_path)
                    self.get_logger().info(f"[sensor] found {dev_path} → launching: {cmd}")
                    proc = subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid)
                    self.processes[dev_path] = proc
        # Cleanup for removed devices
        for dev_path, proc in list(self.processes.items()):
            if dev_path not in active:
                self.get_logger().warn(f"[sensor] gone {dev_path} → stopping driver")
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                    proc.wait(timeout=3)
                except Exception:
                    proc.kill()
                self.processes.pop(dev_path, None)

def main():
    rclpy.init()
    node = SensorManager()
    rclpy.spin(node)
    for p in list(node.processes.values()):
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGINT)
        except Exception:
            pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
