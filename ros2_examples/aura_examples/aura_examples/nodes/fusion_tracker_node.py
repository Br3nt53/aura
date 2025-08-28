#!/usr/bin/env python3
import json, math, time, yaml, os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

def dist2(a,b): return (a[0]-b[0])**2 + (a[1]-b[1])**2

class Track:
    def __init__(self, x, y, c, tnow, alpha, decay_sec):
        self.x = float(x); self.y = float(y)
        self.stab = c  # stability proxy (EMA of confidence)
        self.alpha = alpha
        self.last = tnow
        self.decay_sec = decay_sec
        self.id = Track._next_id()
    _ID = 0
    @classmethod
    def _next_id(cls):
        cls._ID += 1; return cls._ID

    def update(self, x, y, c, tnow):
        self.x = 0.7*self.x + 0.3*float(x)
        self.y = 0.7*self.y + 0.3*float(y)
        self.stab = (1.0 - self.alpha)*self.stab + self.alpha*float(c)
        self.last = tnow

    def alive(self, tnow):
        return (tnow - self.last) <= self.decay_sec

class FusionTrackerNode(Node):
    """
    Fuses presence hints with gating + stability weighting.
    Publishes predictions JSONL lines; score = incoming_conf * (0.5 + 0.5*stability)
    Uses golden params if config/golden_params.yaml exists.
    """
    def __init__(self):
        super().__init__('fusion_tracker_node')
        # defaults
        params = {
            "rf_weight": 0.6,
            "track_decay_sec": 1.5,
            "assoc_gate_m": 1.5,
            "birth_stability_min": 0.35,
            "delete_stability_min": 0.15,
            "stability_alpha": 0.2
        }
        cfg = os.path.join(os.path.dirname(__file__), "..","..","..","..","config","golden_params.yaml")
        cfg = os.path.abspath(cfg)
        if os.path.exists(cfg):
            try:
                loaded = yaml.safe_load(open(cfg))
                params.update({k: float(v) for k,v in loaded.items()})
                self.get_logger().info(f"Loaded golden params from {cfg}")
            except Exception as e:
                self.get_logger().warn(f"Failed to load golden params: {e}")
        self.params = params

        self.pub = self.create_publisher(String, '/aura/predictions_jsonl', 50)
        self.subs = {}
        self.tracks = []
        self.frame = 0
        self.create_timer(1.0, self.refresh_topics)
        self.create_timer(0.05, self.tick)

    def refresh_topics(self):
        for name, types in self.get_topic_names_and_types():
            if not name.endswith('/presence_hint'): continue
            if 'std_msgs/msg/Float32MultiArray' not in types: continue
            if name in self.subs: continue
            self.get_logger().info(f"[fusion] subscribing to {name}")
            self.subs[name] = self.create_subscription(Float32MultiArray, name, self.on_hint, 20)

    def on_hint(self, msg: Float32MultiArray):
        try:
            c, x, y, z = msg.data[:4]
        except Exception:
            return
        tnow = self.get_clock().now().nanoseconds / 1e9
        # associate to nearest track under gate
        gate2 = self.params["assoc_gate_m"]**2
        best = None; bi = -1
        for i,tr in enumerate(self.tracks):
            if not tr.alive(tnow): continue
            d2 = dist2((tr.x,tr.y), (x,y))
            if d2 <= gate2 and (best is None or d2 < best): best = d2; bi = i
        if bi >= 0:
            self.tracks[bi].update(x,y,c,tnow)
        else:
            # birth if initial stability is strong enough
            if c >= self.params["birth_stability_min"]:
                self.tracks.append(Track(x,y,c,tnow,self.params["stability_alpha"], self.params["track_decay_sec"]))

    def tick(self):
        # prune & emit
        tnow = self.get_clock().now().nanoseconds / 1e9
        alive = []
        dets = []
        for tr in self.tracks:
            if not tr.alive(tnow):
                if tr.stab >= self.params["delete_stability_min"]:
                    # keep a bit longer
                    pass
                else:
                    continue
            alive.append(tr)
            score = max(0.0, min(1.0, 0.5 + 0.5*tr.stab))
            dets.append({"id": tr.id, "x": tr.x, "y": tr.y, "score": score})
        self.tracks = alive
        if dets:
            line = {"frame": int(self.frame), "detections": dets}
            out = String(); out.data = json.dumps(line)
            self.pub.publish(out)
            self.frame += 1

def main():
    rclpy.init()
    n = FusionTrackerNode()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
