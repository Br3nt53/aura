from dataclasses import dataclass
from typing import Dict, List, Tuple, Any

from .tracker_interface import BaseTracker


def dist2(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    dx, dy = a[0] - b[0], a[1] - b[1]
    return dx * dx + dy * dy


@dataclass
class Track:
    x: float
    y: float
    stab: float
    alpha: float
    decay_sec: float
    last_t: float
    track_id: int

    def alive(self, tnow: float) -> bool:
        return (tnow - self.last_t) <= self.decay_sec

    def update(self, x: float, y: float, c: float, tnow: float):
        self.x, self.y = x, y
        self.stab = self.alpha * c + (1.0 - self.alpha) * self.stab
        self.last_t = tnow


class LegacyTracker(BaseTracker):
    def __init__(self, params: Dict[str, Any]):
        self.params = params
        self.tracks: List[Track] = []
        self.next_track_id = 0

    def update(
        self, detections: List[Dict[str, Any]], timestamp_sec: float
    ) -> List[Dict[str, Any]]:
        gate2 = self.params.get("assoc_gate_m", 1.5) ** 2

        for det in detections:
            x, y, c = det["x"], det["y"], det["score"]
            best_dist, best_idx = float("inf"), -1

            for i, tr in enumerate(self.tracks):
                if not tr.alive(timestamp_sec):
                    continue
                d2 = dist2((tr.x, tr.y), (x, y))
                if d2 < gate2 and d2 < best_dist:
                    best_dist, best_idx = d2, i

            if best_idx != -1:
                self.tracks[best_idx].update(x, y, c, timestamp_sec)
            else:
                self.tracks.append(
                    Track(
                        x=x,
                        y=y,
                        stab=c,
                        alpha=self.params.get("alpha", 0.3),
                        decay_sec=self.params.get("track_decay_sec", 2.5),
                        last_t=timestamp_sec,
                        track_id=self.next_track_id,
                    )
                )
                self.next_track_id += 1

        self.tracks = [t for t in self.tracks if t.alive(timestamp_sec)]

        return [
            {"id": t.track_id, "x": t.x, "y": t.y, "score": t.stab} for t in self.tracks
        ]
