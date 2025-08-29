.DEFAULT_GOAL := eval-fast
.PHONY: setup eval-fast clean-fast

# Paths & defaults
PY := .venv/bin/python
SCENARIO ?= scenarios/crossing_targets.yaml
PARAMS   ?= configs/default.yaml
OUT      ?= out/tmp

setup:
	@command -v python3 >/dev/null || { echo "python3 not found"; exit 1; }
	@test -x .venv/bin/python || python3 -m venv .venv
	@$(PY) -m pip -q install --upgrade pip >/dev/null
	@$(PY) -m pip -q install pandas motmetrics pytest pyyaml >/dev/null || true
	@echo "[make] setup complete"

eval-fast: setup
	@echo "[make] Running AURA evaluator fast-path (SKIP_ROS=1, USE_AURA_EVALUATOR=1)"
	@USE_AURA_EVALUATOR=1 SKIP_ROS=1 $(PY) tools/run_single.py \
		--scenario $(SCENARIO) \
		--params   $(PARAMS) \
		--out-dir  $(OUT)
	@echo "[make] Done. Metrics at $(OUT)/metrics.json"
	@head -n 20 $(OUT)/metrics.json || true

clean-fast:
	@echo "[make] Cleaning fast eval outputs ($(OUT))"
	@rm -rf $(OUT) && mkdir -p $(OUT)
	@echo "[make] Clean complete."

# ==== Fast AURA eval without ROS (venv + deps) ====
PY := $(shell [ -x .venv/bin/python ] && echo .venv/bin/python || echo python3)
SCENARIO ?= scenarios/crossing_targets.yaml
PARAMS   ?= configs/default.yaml
OUT      ?= out/tmp

setup:
	@command -v python3 >/dev/null || { echo "python3 not found"; exit 1; }
	@test -x .venv/bin/python || python3 -m venv .venv
	@.venv/bin/python -m pip -q install --upgrade pip >/dev/null
	@.venv/bin/python -m pip -q install pandas motmetrics pytest pyyaml >/dev/null || true
	@echo "[make] setup complete"

# --- Synthetic sample (no downloads) ---
sample-setup:
	@echo "[make] Creating tiny sample MOT JSONL gt/pred…"
	@.venv/bin/python tools/create_sample_mot_jsonl.py

eval-sample: setup sample-setup
	@echo "[make] Evaluating sample with AURA evaluator (no ROS)…"
	@USE_AURA_EVALUATOR=1 SKIP_ROS=1 .venv/bin/python tools/run_single.py \
		--scenario $(SCENARIO) \
		--params   $(PARAMS) \
		--out-dir  out/sample-mot
	@echo "[make] Metrics at out/sample-mot/metrics.json"
	@head -n 30 out/sample-mot/metrics.json || true

clean-sample:
	@rm -rf out/sample-mot && echo "[make] Cleaned out/sample-mot"

# --- MOT17-style local directory (uses data/mot17-mini) ---
MOT_DIR ?= data/mot17-mini
eval-mot17: setup
	@echo "[make] Converting MOT -> JSONL …"
	@.venv/bin/python tools/convert_mot_to_jsonl.py --data-root $(MOT_DIR) --out-dir out/mot17-mini
	@echo "[make] Evaluating MOT17-mini with AURA evaluator (no ROS)…"
	@USE_AURA_EVALUATOR=1 SKIP_ROS=1 .venv/bin/python tools/run_single.py \
		--scenario $(SCENARIO) \
		--params   $(PARAMS) \
		--out-dir  out/mot17-mini
	@echo "[make] Metrics at out/mot17-mini/metrics.json"
	@head -n 30 out/mot17-mini/metrics.json || true

clean-mot17:
	@rm -rf out/mot17-mini && echo "[make] Cleaned out/mot17-mini"

# Keep your existing eval-fast/clean-fast if already present
