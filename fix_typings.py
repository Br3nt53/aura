from pathlib import Path
import re

# ---- aura_tools/__init__.py: annotate __all__ ----
p = Path("aura_tools/aura_tools/__init__.py")
if p.exists():
    s = p.read_text(encoding="utf-8")
    if "__all__" in s:
        s = re.sub(
            r"^\s*__all__\s*=\s*(\[.*?\])\s*$",
            r"from typing import List\n__all__: list[str] = \1",
            s,
            flags=re.M,
        )
    else:
        s = "from typing import List\n__all__: list[str] = []\n" + s
    p.write_text(s, encoding="utf-8")

# ---- evaluation/mot_evaluator.py ----
p = Path("evaluation/mot_evaluator.py")
if p.exists():
    s = p.read_text(encoding="utf-8")
    if "from typing import" not in s:
        s = "from typing import Any, Dict, Set\n" + s
    s = re.sub(
        r"^\s*gt_fragments\s*=\s*\{\s*\}\s*$",
        "gt_fragments: Dict[Any, Any] = {}",
        s,
        flags=re.M,
    )
    s = re.sub(
        r"^\s*all_pred_tracks\s*=\s*set\(\s*\)\s*$",
        "all_pred_tracks: Set[Any] = set()",
        s,
        flags=re.M,
    )
    p.write_text(s, encoding="utf-8")

# ---- tests/test_teleop.py ----
p = Path("tests/test_teleop.py")
if p.exists():
    s = p.read_text(encoding="utf-8")
    if (
        "spec = importlib.util.spec_from_file_location" in s
        and "assert spec and spec.loader" not in s
    ):
        s = re.sub(
            r"(spec\s*=\s*importlib\.util\.spec_from_file_location\(.*\)\s*\n)",
            r"\1assert spec and spec.loader\n",
            s,
            count=1,
        )
    p.write_text(s, encoding="utf-8")

# ---- tools/*: yaml imports ----
for fname in ("tools/run_ros2_pipeline.py", "tools/make_gt_from_yaml.py"):
    p = Path(fname)
    if p.exists():
        s = p.read_text(encoding="utf-8")
        if "import yaml" in s and "# type: ignore" not in s:
            s = s.replace("import yaml", "import yaml  # type: ignore[import-untyped]")
            p.write_text(s, encoding="utf-8")

print("[ok] patched typings where needed")
