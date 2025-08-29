from __future__ import annotations
from typing import Dict, Tuple


class EvalPlugin:
    """
    Implement this interface in aura_eval_plugin.py (repo root) to integrate your real sim.
    Required method:
        run_sim_and_collect(self, scenario_path: str, params: Dict, workdir: str) -> Tuple[str, str]
    Returns (pred_jsonl_path, gt_jsonl_path)
    """

    def run_sim_and_collect(
        self, scenario_path: str, params: Dict, workdir: str
    ) -> Tuple[str, str]:
        raise NotImplementedError
