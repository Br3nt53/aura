from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional, Any, Set
import json
import numpy as np
import os
from collections import defaultdict, Counter

try:
    from scipy.optimize import linear_sum_assignment

    _HAVE_SCIPY = True
except Exception:
    _HAVE_SCIPY = False

BBox = Tuple[float, float, float, float]
Det = Dict[str, object]


@dataclass
class EvalParams:
    rf_weight: float = 0.0
    wall_bonus: float = 0.0
    track_decay_sec: float = 1.0
    iou_threshold: float = 0.5
    fps: float = 10.0
    # "strict": fragment if gap > decay_frames; "inclusive": if gap >= decay_frames
    fragment_gap_mode: str = "strict"


def _hungarian(cost: np.ndarray):
    if cost.size == 0:
        return np.array([], dtype=int), np.array([], dtype=int)
    if _HAVE_SCIPY:
        r, c = linear_sum_assignment(cost)
        return r.astype(int), c.astype(int)
    # greedy fallback
    r_sel, c_sel = [], []
    used_r, used_c = set(), set()
    for ri in np.argsort(cost.min(axis=1) if cost.size else []):
        ci = int(cost[ri].argmin())
        if ri in used_r or ci in used_c:
            continue
        used_r.add(int(ri))
        used_c.add(int(ci))
        r_sel.append(int(ri))
        c_sel.append(int(ci))
    return np.array(r_sel, dtype=int), np.array(c_sel, dtype=int)


def _bbox_iou(a: BBox, b: BBox) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    aa = (ax2 - ax1) * (ay2 - ay1)
    ba = (bx2 - bx1) * (by2 - by1)
    den = aa + ba - inter
    return float(inter / den) if den > 0 else 0.0


class MOTEvaluator:
    def __init__(self, params: Optional[EvalParams] = None):
        self.params = params or EvalParams()
        self._skipped_counts: Dict[str, int] = {}

    def evaluate(
        self,
        pred_file: str,
        gt_file: str,
        *,
        scenario_path=None,
        workdir=None,
        rf_weight=None,
        wall_bonus=None,
        track_decay_sec=None,
    ) -> Dict[str, object]:
        if rf_weight is not None:
            self.params.rf_weight = float(rf_weight)
        if wall_bonus is not None:
            self.params.wall_bonus = float(wall_bonus)
        if track_decay_sec is not None:
            self.params.track_decay_sec = float(track_decay_sec)

        gt_by_frame = defaultdict(list)
        pr_by_frame = defaultdict(list)

        for d in self._read_jsonl(gt_file):
            gt_by_frame[int(d["frame"])].append(d)
        for d in self._read_jsonl(pred_file):
            pr_by_frame[int(d["frame"])].append(d)

        frames = sorted(set(gt_by_frame.keys()) | set(pr_by_frame.keys()))
        if not frames:
            # no data at all
            return {
                "mota": 1.0,
                "motp": 0.0,
                "auc": 0.0,
                "precision": 0.0,
                "recall": 0.0,
                "f1_score": 0.0,
                "id_switches": 0,
                "fragments": 0,
                "track_purity": 0.0,
                "track_completeness": 0.0,
                "id_fragmentation_rate": 0.0,
                "params": {
                    "rf_weight": self.params.rf_weight,
                    "wall_bonus": self.params.wall_bonus,
                    "track_decay_sec": self.params.track_decay_sec,
                    "iou_threshold": self.params.iou_threshold,
                    "fps": self.params.fps,
                    "fragment_gap_mode": self.params.fragment_gap_mode,
                },
                "meta": {
                    "scenario": scenario_path,
                    "workdir": workdir,
                    "data_quality": {"skipped_lines": self._skipped_counts},
                },
            }

        # stats
        total_gt = 0
        sum_fp = sum_fn = sum_idsw = 0
        matched_ious: List[float] = []
        tp = fp = fn = 0

        # continuity
        fps = float(getattr(self.params, "fps", 10.0))
        decay_frames = max(1, int(round(self.params.track_decay_sec * fps)))
        last_seen_frame: Dict[object, int] = {}
        last_pred_for_gt: Dict[object, object] = {}
        gt_fragments = Counter()
        pred_to_gt_counts: Dict[object, Counter] = defaultdict(Counter)
        gt_ever_tracked = set()
        all_pred_tracks: Set[Any] = set()
        use_bbox = False
        # quick peek: if any GT has bbox keys, we assume bbox/IoU mode
        for fr in frames:
            for g in gt_by_frame.get(fr, []):
                if all(k in g for k in ("x1", "y1", "x2", "y2")):
                    use_bbox = True
                    break
            if use_bbox:
                break

        for t in frames:
            gts = gt_by_frame.get(t, [])
            prs = pr_by_frame.get(t, [])
            total_gt += len(gts)
            all_pred_tracks.update(d.get("id") for d in prs)

            # build cost matrix (lower is better)
            if use_bbox:
                cost = np.zeros((len(gts), len(prs)), dtype=float)
                for i, g in enumerate(gts):
                    gb = (
                        float(g["x1"]),
                        float(g["y1"]),
                        float(g["x2"]),
                        float(g["y2"]),
                    )
                    for j, p in enumerate(prs):
                        if all(k in p for k in ("x1", "y1", "x2", "y2")):
                            pb = (
                                float(p["x1"]),
                                float(p["y1"]),
                                float(p["x2"]),
                                float(p["y2"]),
                            )
                            iou = _bbox_iou(gb, pb)
                        else:
                            # fallback to point IoU-ish (distance clamp)
                            dx = float(p.get("x", 0.0)) - float(g.get("x", 0.0))
                            dy = float(p.get("y", 0.0)) - float(g.get("y", 0.0))
                            d2 = dx * dx + dy * dy
                            iou = max(0.0, 1.0 - min(1.0, d2))
                        cost[i, j] = 1.0 - iou
            else:
                cost = np.ones((len(gts), len(prs)), dtype=float)
                for i, g in enumerate(gts):
                    gx, gy = float(g.get("x", 0.0)), float(g.get("y", 0.0))
                    for j, p in enumerate(prs):
                        dx = float(p.get("x", 0.0)) - gx
                        dy = float(p.get("y", 0.0)) - gy
                        d2 = dx * dx + dy * dy
                        if d2 <= 1.0:  # within threshold
                            cost[i, j] = d2
                        else:
                            cost[i, j] = 1e6

            if len(gts) and len(prs):
                rows, cols = _hungarian(cost)
            else:
                rows, cols = np.array([], dtype=int), np.array([], dtype=int)
            m_pairs = list(zip(rows.tolist(), cols.tolist()))
            matched: List[Tuple[object, object, float]] = []

            for ri, ci in m_pairs:
                if cost[ri, ci] >= 1e6:
                    continue
                gid = gts[ri].get("id")
                pid = prs[ci].get("id")
                # similarity as IoU-ish
                sim = 1.0 - float(cost[ri, ci])
                matched.append((gid, pid, sim))
                matched_ious.append(sim)
            # count TPs/FPs/FNs
            tp += len(matched)
            fn += len(gts) - len(matched)
            fp += len(prs) - len(matched)
            sum_fn += len(gts) - len(matched)
            sum_fp += len(prs) - len(matched)

            # ID switches + decay-based fragments
            for gt_id, pred_id, _sim in matched:
                if gt_id in last_seen_frame:
                    gap = t - last_seen_frame[gt_id]
                    if self.params.fragment_gap_mode == "inclusive":
                        is_fragment = gap >= decay_frames
                    else:
                        is_fragment = gap > decay_frames
                    if is_fragment:
                        gt_fragments[gt_id] += 1
                prev_pid = last_pred_for_gt.get(gt_id)
                if (
                    prev_pid is not None
                    and prev_pid != pred_id
                    and (t - last_seen_frame.get(gt_id, t)) <= decay_frames
                ):
                    sum_idsw += 1
                last_seen_frame[gt_id] = t
                last_pred_for_gt[gt_id] = pred_id
                pred_to_gt_counts[pred_id][gt_id] += 1
                gt_ever_tracked.add(gt_id)

        precision = float(tp) / float(max(1, tp + fp))
        recall = float(tp) / float(max(1, tp + fn))
        f1 = (
            (2 * precision * recall / (precision + recall))
            if (precision + recall) > 0
            else 0.0
        )
        # Simplified AUC placeholder (area under P-R envelope via trapezoid on single point)
        auc = precision * recall

        # MOTA
        if total_gt == 0:
            mota = 1.0 if (sum_fp + sum_fn + sum_idsw) == 0 else 0.0
        else:
            mota = 1.0 - float(sum_fn + sum_fp + sum_idsw) / float(total_gt)
        motp = float(np.mean(matched_ious)) if matched_ious else 0.0

        # track stats
        purities = []
        for pid, ctr in pred_to_gt_counts.items():
            tot = sum(ctr.values())
            if tot > 0:
                purities.append(max(ctr.values()) / tot)
        track_purity = float(np.mean(purities)) if purities else 0.0
        track_completeness = float(len(pred_to_gt_counts)) / float(
            max(1, len(all_pred_tracks))
        )
        fragments = int(sum(gt_fragments.values()))

        gt_to_pred_ids = defaultdict(set)
        for pid, ctr in pred_to_gt_counts.items():
            for gid in ctr.keys():
                gt_to_pred_ids[gid].add(pid)
        split_pairs = sum(max(0, len(pids) - 1) for pids in gt_to_pred_ids.values())
        id_fragmentation_rate = float(split_pairs) / float(max(1, len(gt_ever_tracked)))

        meta = {"scenario": scenario_path, "workdir": workdir}
        if getattr(self, "_skipped_counts", None) is not None:
            meta["data_quality"] = {"skipped_lines": self._skipped_counts}

        return {
            "mota": float(mota),
            "motp": float(motp),
            "auc": float(auc),
            "precision": float(precision),
            "recall": float(recall),
            "f1_score": float(f1),
            "id_switches": int(sum_idsw),
            "fragments": int(fragments),
            "track_purity": float(track_purity),
            "track_completeness": float(track_completeness),
            "id_fragmentation_rate": float(id_fragmentation_rate),
            "params": {
                "rf_weight": self.params.rf_weight,
                "wall_bonus": self.params.wall_bonus,
                "track_decay_sec": self.params.track_decay_sec,
                "iou_threshold": self.params.iou_threshold,
                "fps": self.params.fps,
                "fragment_gap_mode": self.params.fragment_gap_mode,
            },
            "meta": meta,
        }


def _read_jsonl(self, path: str):
    skipped = 0
    with open(path, "r", encoding="utf-8") as f:
        for ln, line in enumerate(f, 1):
            s = line.strip()
            if not s:
                continue
            try:
                yield json.loads(s)
            except json.JSONDecodeError as e:
                print(f"[WARN] {path}:{ln}: skipping malformed JSONL line: {e}")
                skipped += 1
                continue
    self._skipped_counts[os.path.basename(path)] = skipped
