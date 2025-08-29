import json, subprocess, pathlib
def test_run_single(tmp_path: pathlib.Path):
    out = tmp_path / "metrics.json"
    subprocess.run(
        ["python3", "tools/run_single.py",
         "--scenario", "scenarios/crossing_targets.yaml",
         "--out", str(out)],
        check=True,
    )
    m = json.loads(out.read_text())
    assert 0.0 <= m.get("auc", 0.0) <= 1.0
