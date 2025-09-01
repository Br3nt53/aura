import importlib.util
from pathlib import Path
import types

import numpy as np
import pytest
from typing import Any, TYPE_CHECKING, cast

if TYPE_CHECKING:
    from importlib.machinery import ModuleSpec
else:
    ModuleSpec = Any  # type: ignore[assignment]

# Skip automatically if ROS 2 (rclpy) isn't available (e.g., on a plain CI runner)
rclpy = pytest.importorskip("rclpy")


def _load_module(module_path: Path):
    spec = importlib.util.spec_from_file_location(
        "teleop_target_node", str(module_path)
    )
    mod = importlib.util.module_from_spec(cast(ModuleSpec, spec))
    assert spec and spec.loader
    spec.loader.exec_module(mod)  # type: ignore[attr-defined]
    return mod


def test_teleop_moves_xy(tmp_path: Path):
    # Adjust the path if your file lives elsewhere
    mod_path = Path(
        "ros2_examples/aura_sensors/aura_sensors/nodes/teleop_target_node.py"
    )
    assert mod_path.exists(), f"Missing file: {mod_path}"
    mod = _load_module(mod_path)

    rclpy.init()
    node = mod.TeleopTargetNode()
    try:
        start = node.pos.copy()

        # Helper that returns an object with a `.char` attribute (avoid lambda for Ruff E731)
        def make_key(c: str):
            return types.SimpleNamespace(char=c)

        # Simulate 'w' (increase x) and 'a' (increase y)
        node.on_press(make_key("w"))
        node.on_press(make_key("a"))

        dx = node.vel * node.dt
        dy = node.vel * node.dt

        np.testing.assert_allclose(node.pos[0], start[0] + dx, rtol=0, atol=1e-9)
        np.testing.assert_allclose(node.pos[1], start[1] + dy, rtol=0, atol=1e-9)
        np.testing.assert_allclose(node.pos[2], start[2], rtol=0, atol=1e-9)
    finally:
        node.destroy_node()
        rclpy.shutdown()
