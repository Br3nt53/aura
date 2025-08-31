import sys
from pathlib import Path
import re
import json
import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
RUN_SINGLE = REPO_ROOT / "tools" / "run_single.py"


def test_run_single_no_objective_typo():
    """Guard against metrics_dict.get(objective, ...) regression."""
    src = RUN_SINGLE.read_text(encoding="utf-8")
    bad = re.findall(r"metrics_dict\.get\(\s*objective\s*,", src)
    assert (
        not bad
    ), "Found bad pattern metrics_dict.get(objective, ...) in tools/run_single.py"


@pytest.mark.skipif(
    not (REPO_ROOT / "evaluation" / "mot_evaluator.py").exists(),
    reason="evaluation.mot_evaluator not present",
)
def test_evaluator_minimal_objective_or_mota(tmp_path):
    """Sanity run of MOTEvaluator (no ROS). Accept either objective or mota key."""
    sys.path.insert(0, str(REPO_ROOT))
    from evaluation.mot_evaluator import MOTEvaluator, EvalParams  # type: ignore

    gp = tmp_path / "gt.jsonl"
    pp = tmp_path / "pred.jsonl"

    gt = [{"frame": 0, "id": 1, "x": 0.0, "y": 0.0}]
    pr = [{"frame": 0, "id": "p1", "x": 0.0, "y": 0.0, "conf": 1.0}]

    gp.write_text("\n".join(json.dumps(r) for r in gt), encoding="utf-8")
    pp.write_text("\n".join(json.dumps(r) for r in pr), encoding="utf-8")

    res = MOTEvaluator(EvalParams()).evaluate(str(pp), str(gp))
    assert ("objective" in res) or (
        "mota" in res
    ), f"Missing objective/mota in {sorted(res.keys())}"
