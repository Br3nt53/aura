import json
from evaluation.mot_evaluator import MOTEvaluator, EvalParams


def _w(path, rows):
    path.write_text("\n".join(json.dumps(r) for r in rows))


def test_fragment_boundary_strict_vs_inclusive(tmp_path):
    # fps=10, decay_sec=1.0 -> decay_frames=10
    # GT present 0..10 (last seen at 10), then at 20 (gap=10), then at 21 (gap=1 post-resume)
    gt = [{"frame": i, "id": 1, "x": 0, "y": 0} for i in range(0, 11)]
    gt.append({"frame": 20, "id": 1, "x": 0, "y": 0})  # boundary gap = 10
    gt.append({"frame": 21, "id": 1, "x": 0, "y": 0})  # not a fragment in either mode
    pr = [{"frame": r["frame"], "id": "a", "x": 0, "y": 0, "conf": 1.0} for r in gt]
    gp, pp = tmp_path / "gt.jsonl", tmp_path / "pred.jsonl"
    _w(gp, gt)
    _w(pp, pr)

    strict = MOTEvaluator(
        EvalParams(track_decay_sec=1.0, fps=10.0, fragment_gap_mode="strict")
    ).evaluate(str(pp), str(gp))
    assert strict["fragments"] == 0  # 10-gap is not > 10

    inclusive = MOTEvaluator(
        EvalParams(track_decay_sec=1.0, fps=10.0, fragment_gap_mode="inclusive")
    ).evaluate(str(pp), str(gp))
    assert inclusive["fragments"] == 1  # 10-gap counts when gap >= decay_frames


def test_fragment_with_11_gap_in_strict(tmp_path):
    # Construct an 11-frame gap to ensure strict mode counts 1 fragment.
    # GT frames: 0..10, then 22 (gap=12 from last seen=10), then 23 (no new fragment)
    gt = [{"frame": i, "id": 1, "x": 0, "y": 0} for i in range(0, 11)]
    gt.append({"frame": 22, "id": 1, "x": 0, "y": 0})  # gap=12 (>10)
    gt.append({"frame": 23, "id": 1, "x": 0, "y": 0})
    pr = [{"frame": r["frame"], "id": "a", "x": 0, "y": 0, "conf": 1.0} for r in gt]
    gp, pp = tmp_path / "gt.jsonl", tmp_path / "pred.jsonl"
    _w(gp, gt)
    _w(pp, pr)

    strict = MOTEvaluator(
        EvalParams(track_decay_sec=1.0, fps=10.0, fragment_gap_mode="strict")
    ).evaluate(str(pp), str(gp))
    assert strict["fragments"] == 1

    inclusive = MOTEvaluator(
        EvalParams(track_decay_sec=1.0, fps=10.0, fragment_gap_mode="inclusive")
    ).evaluate(str(pp), str(gp))
    assert inclusive["fragments"] == 1
