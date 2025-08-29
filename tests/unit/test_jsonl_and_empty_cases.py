import json
from evaluation.mot_evaluator import MOTEvaluator, EvalParams


def _w(path, rows):
    path.write_text("\n".join(json.dumps(r) for r in rows))


def test_malformed_jsonl_lines_are_skipped(tmp_path):
    gt = [{"frame": 0, "id": 1, "x": 0, "y": 0}]
    pr_good = {"frame": 0, "id": "a", "x": 0, "y": 0, "conf": 1.0}
    gp, pp = tmp_path / "gt.jsonl", tmp_path / "pred.jsonl"
    _w(gp, gt)
    pp.write_text(
        json.dumps(pr_good) + "\n" + "{this is not json}\n" + json.dumps(pr_good) + "\n"
    )
    res = MOTEvaluator(EvalParams()).evaluate(str(pp), str(gp))
    assert "meta" in res and "data_quality" in res["meta"]
    skipped = res["meta"]["data_quality"]["skipped_lines"]
    assert any(k.endswith("pred.jsonl") and v >= 1 for k, v in skipped.items())


def test_empty_gt_nonempty_pred(tmp_path):
    gp, pp = tmp_path / "gt.jsonl", tmp_path / "pred.jsonl"
    gp.write_text("")
    pr = [{"frame": 0, "id": "a", "x": 0, "y": 0, "conf": 1.0}]
    _w(pp, pr)
    res = MOTEvaluator(EvalParams()).evaluate(str(pp), str(gp))
    assert res["mota"] == 0.0


def test_empty_pred_nonempty_gt(tmp_path):
    gp, pp = tmp_path / "gt.jsonl", tmp_path / "pred.jsonl"
    gt = [{"frame": 0, "id": 1, "x": 0, "y": 0}]
    _w(gp, gt)
    pp.write_text("")
    res = MOTEvaluator(EvalParams()).evaluate(str(pp), str(gp))
    assert res["precision"] == 0.0
    assert res["recall"] == 0.0
    assert res["mota"] <= 0.0
