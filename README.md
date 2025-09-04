# AURA Experiments Runner Bundle

A comprehensive and self-contained toolkit for parameter sweeps, multi-target tracking stress tests, RF clutter injection, Unity-based HUD UX iteration, hardware integration, and full-system evaluation via ROS 2.

---

## 🚀 Quickstart

### Option A: Local Fast Evaluation (No ROS or Docker)

1.  **Install uv**
    ```bash
    curl -LsSf [https://astral.sh/uv/install.sh](https://astral.sh/uv/install.sh) | sh
    ```

2.  **Install Dependencies**
    ```bash
    uv pip install -r requirements.txt -r requirements-dev.txt
    ```

3.  **Run Fast Evaluation**
    This performs ground-truth generation, prediction synthesis, evaluation, and outputs metrics:
    ```bash
    make eval-fast
    ```
    Results are printed to the terminal and saved to `out/tmp/metrics.json`.

### Option B: Full System via Docker & ROS 2

1.  **Start the Full System**
    Build the Docker image and launch the container:
    ```bash
    ./start_aura.sh
    ```
2.  **Stop the System**
    ```bash
    ./stop_aura.sh
    ```

## ✨ Features & Components

### Experimentation & Evaluation Tools

* **Parameter Sweeps & Reports**: `tools/run_experiments.py` supports grid sweeps and generates standalone HTML heatmap reports.
* **Single Scenario Runner**: `tools/run_single.py` is the replaceable evaluation entry point for real simulation or evaluation workflows.
* **Scenario Definitions**: YAML files under `scenarios/` (e.g., `crossing_targets.yaml`, `rf_clutter.yaml`) define test environments.

### Unity Integration & Live Tuning

* **Unity HUD Support**: Unity scripts (`GhostOverlay.cs`, `ParamSender.cs`) in `unity/Assets/...` enable confidence-based visualization and parameter control.
* **ROS 2 Param Bridge**: The `aura_tools` package supports real-time parameter injection via `/aura/param_set`.

### ROS 2 Packages & Launch Infrastructure

* **ROS Examples Package (`aura_examples`)**: Includes simulated nodes (`scenario_player_node.py`, `mock_rf_node.py`, `fusion_sim_node.py`, `recorder_node.py`, `teleop_target_node.py`) and a launch pipeline (`bringup.launch.py`) for scenario → RF → fusion → record workflows.

    Run via:
    ```bash
    python3 tools/run_single.py \
      --scenario scenarios/crossing_targets.yaml \
      --rf_weight 0.6 --wall_bonus 0.4 --track_decay_sec 1.5 \
      --ros2_pipeline --workdir out/tmp
    ```

### Hardware Integration

* **Plug-and-Play Sensor Support**: Udev rules in `hardware/udev/` ensure stable device names (e.g. `/dev/aura_uwb0`).
* **Sensor Manager (`sensor_bringup.launch.py`)**: Auto-launches/stops driver nodes on device plug/unplug.
* **Fusion Dynamic Discovery**: Subscribes to any `/presence_hint` topics automatically.
* Example bridge launchers included under `hardware/drivers/`.

### Golden Configurations & Bound Derivation

* **Golden Parameters**: Stored in `config/golden_params.yaml`, auto-loaded by the fusion tracker.
* **Safe Bounds for Unity**: `unity/Assets/StreamingAssets/ParamBounds.json` enforces experimental slider limits via:
    ```bash
    python3 tools/freeze_golden.py
    python3 tools/derive_bounds.py
    ```

### Stability-Weighted Fusion

* `fusion_tracker_node.py` implements gating + exponential moving average (EMA) stability, using parameters like `assoc_gate_m`, `track_decay_sec`, `stability_alpha`, `birth_stability_min`, `delete_stability_min`, and `rf_weight`. Outputs predictions via `/aura/predictions_jsonl`.

### Adversarial & Regression Testing

* **Challenge Bags**: Generate adversarial test scenarios using:
    ```bash
    tools/make_challenge_bag.sh scenarios/crossing_targets.yaml out/challenge true
    ```
    Then evaluate via standard pipeline or export using ROS bag tools.

---
## 📂 Repository Layout

Of course. Here are the complete, updated files to finalize your migration to uv and standardize your project's environment.

1. scripts/docker_smoke.sh (Updated)

This script is now significantly simplified. It no longer creates a redundant virtual environment (.venv) inside the container. Instead, it directly uses the system's Python environment, where uv has already installed all necessary dependencies, making the smoke test faster and more efficient.

Bash
#!/usr/bin/env bash
# docker_smoke.sh — minimal functional smoke in Docker
# Builds local image and runs evaluator-only to assert output exists.
# Usage: ./scripts/docker_smoke.sh
set -euo pipefail
IFS=$'\n\t'
cd "$(dirname "$0")/.."

IMAGE="aura:ci"
echo "[stage] docker build → $IMAGE"
docker build -t "$IMAGE" .

echo "[stage] docker run smoke"
docker run --rm -v "$PWD":/work -w /work "$IMAGE" bash -lc "\
  set -e
  echo '[smoke] Running the single evaluation...'
  mkdir -p out/tmp
  if [ -f tools/run_single.py ]; then
    if [ -f scenarios/crossing_targets.yaml ]; then
      python tools/run_single.py --scenario scenarios/crossing_targets.yaml --out out/tmp/metrics.json
    else
      python tools/run_single.py --out out/tmp/metrics.json
    fi
    test -s out/tmp/metrics.json
  else
    echo 'tools/run_single.py not found'; exit 3
  fi"

echo "[ok] smoke passed; metrics present at out/tmp/metrics.json"
2. Makefile (Updated)

The Makefile has been modified to remove the creation of a local .venv. The setup target now directly uses uv to install dependencies, assuming uv is installed on the user's system. All other targets have been updated to call the system's python instead of a virtual environment's python.

Makefile
.DEFAULT_GOAL := eval-fast
.PHONY: setup eval-fast clean-fast eval-sample clean-sample eval-mot17 clean-mot17 eval-kitti clean-kitti help

# --- Configuration ---
# All targets will now use the system's default python interpreter.
PY := python

# --- Primary Targets ---
setup: ## Install all dependencies using uv
	@command -v uv >/dev/null || { echo "uv not found, please install it first. See https://astral.sh/uv"; exit 1; }
	@echo "[make] Installing dependencies with uv..."
	@uv pip install -r requirements.txt -r requirements-dev.txt
	@echo "[make] setup complete"

eval-fast: setup ## Run AURA evaluator fast-path (no ROS)
	@echo "[make] Running AURA evaluator fast-path (SKIP_ROS=1, USE_AURA_EVALUATOR=1)"
	@USE_AURA_EVALUATOR=1 SKIP_ROS=1 $(PY) tools/run_single.py \
		--scenario $(SCENARIO) \
		--params   $(PARAMS) \
		--out-dir  $(OUT)
	@echo "[make] Done. Metrics at $(OUT)/metrics.json"
	@head -n 20 $(OUT)/metrics.json || true

clean-fast: ## Remove out/tmp
	@echo "[make] Cleaning fast eval outputs ($(OUT))"
	@rm -rf $(OUT) && mkdir -p $(OUT)
	@echo "[make] Clean complete."

# --- Other Evaluation Targets ---
eval-sample: setup ## Generate tiny sample gt/pred and evaluate
	@echo "[make] Creating tiny sample MOT JSONL gt/pred…"
	@$(PY) tools/create_sample_mot_jsonl.py
	@echo "[make] Evaluating sample with AURA evaluator (no ROS)…"
	@USE_AURA_EVALUATOR=1 SKIP_ROS=1 $(PY) tools/run_single.py \
		--scenario $(SCENARIO) \
		--params   $(PARAMS) \
		--out-dir  out/sample-mot
	@echo "[make] Metrics at out/sample-mot/metrics.json"
	@head -n 20 out/sample-mot/metrics.json || true

clean-sample: ## Remove out/sample-mot
	@rm -rf out/sample-mot && echo "[make] Cleaned out/sample-mot"

eval-mot17: setup ## Convert MOT-style dir to JSONL and evaluate
	@echo "[make] Converting MOT -> JSONL …"
	@$(PY) tools/convert_mot_to_jsonl.py --data-root $(MOT_DIR) --out-dir out/mot17-mini
	@echo "[make] Evaluating MOT17-mini with AURA evaluator (no ROS)…"
	@USE_AURA_EVALUATOR=1 SKIP_ROS=1 $(PY) tools/run_single.py \
		--scenario $(SCENARIO) \
		--params   $(PARAMS) \
		--out-dir  out/mot17-mini
	@echo "[make] Metrics at out/mot17-mini/metrics.json"
	@head -n 30 out/mot17-mini/metrics.json || true

clean-mot17: ## Remove out/mot17-mini
	@rm -rf out/mot17-mini && echo "[make] Cleaned out/mot17-mini"

eval-kitti: setup ## Convert KITTI label file to JSONL and evaluate
	@echo "[make] Converting KITTI -> JSONL …"
	@$(PY) tools/convert_kitti_to_jsonl.py --labels $(KITTI_DIR)/label_02/0000.txt --out out/kitti-mini/gt.jsonl
	@echo "[make] Evaluating KITTI-mini with AURA evaluator (no ROS)…"
	@USE_AURA_EVALUATOR=1 SKIP_ROS=1 $(PY) tools/run_single.py \
		--scenario $(SCENARIO) \
		--params   $(PARAMS) \
		--out-dir  out/kitti-mini
	@echo "[make] Metrics at out/kitti-mini/metrics.json"
	@head -n 30 out/kitti-mini/metrics.json || true

clean-kitti: ## Remove out/kitti-mini
	@rm -rf out/kitti-mini && echo "[make] Cleaned out/kitti-mini"

help: ## Show this help
	@awk '/^[A-Za-z0-9_.-]+:.*##/ { split($$0,a,":"); tgt=a[1]; i = index($$0, "##"); desc = substr($$0, i+3); printf "%-16s %s\n", tgt, desc }' Makefile
3. README.md (Updated)

The README.md has been updated to reflect the new uv-based setup process. The instructions now guide users to install uv and use it to manage their dependencies, removing all references to make setup for creating a virtual environment.

Markdown
# AURA Experiments Runner Bundle

A comprehensive and self-contained toolkit for parameter sweeps, multi-target tracking stress tests, RF clutter injection, Unity-based HUD UX iteration, hardware integration, and full-system evaluation via ROS 2.

---

## 🚀 Quickstart

### Option A: Local Fast Evaluation (No ROS or Docker)

1.  **Install uv**
    ```bash
    curl -LsSf [https://astral.sh/uv/install.sh](https://astral.sh/uv/install.sh) | sh
    ```

2.  **Install Dependencies**
    ```bash
    uv pip install -r requirements.txt -r requirements-dev.txt
    ```

3.  **Run Fast Evaluation**
    This performs ground-truth generation, prediction synthesis, evaluation, and outputs metrics:
    ```bash
    make eval-fast
    ```
    Results are printed to the terminal and saved to `out/tmp/metrics.json`.

### Option B: Full System via Docker & ROS 2

1.  **Start the Full System**
    Build the Docker image and launch the container:
    ```bash
    ./start_aura.sh
    ```
2.  **Stop the System**
    ```bash
    ./stop_aura.sh
    ```

## ✨ Features & Components

### Experimentation & Evaluation Tools

* **Parameter Sweeps & Reports**: `tools/run_experiments.py` supports grid sweeps and generates standalone HTML heatmap reports.
* **Single Scenario Runner**: `tools/run_single.py` is the replaceable evaluation entry point for real simulation or evaluation workflows.
* **Scenario Definitions**: YAML files under `scenarios/` (e.g., `crossing_targets.yaml`, `rf_clutter.yaml`) define test environments.

### Unity Integration & Live Tuning

* **Unity HUD Support**: Unity scripts (`GhostOverlay.cs`, `ParamSender.cs`) in `unity/Assets/...` enable confidence-based visualization and parameter control.
* **ROS 2 Param Bridge**: The `aura_tools` package supports real-time parameter injection via `/aura/param_set`.

### ROS 2 Packages & Launch Infrastructure

* **ROS Examples Package (`aura_examples`)**: Includes simulated nodes (`scenario_player_node.py`, `mock_rf_node.py`, `fusion_sim_node.py`, `recorder_node.py`, `teleop_target_node.py`) and a launch pipeline (`bringup.launch.py`) for scenario → RF → fusion → record workflows.

    Run via:
    ```bash
    python3 tools/run_single.py \
      --scenario scenarios/crossing_targets.yaml \
      --rf_weight 0.6 --wall_bonus 0.4 --track_decay_sec 1.5 \
      --ros2_pipeline --workdir out/tmp
    ```

### Hardware Integration

* **Plug-and-Play Sensor Support**: Udev rules in `hardware/udev/` ensure stable device names (e.g. `/dev/aura_uwb0`).
* **Sensor Manager (`sensor_bringup.launch.py`)**: Auto-launches/stops driver nodes on device plug/unplug.
* **Fusion Dynamic Discovery**: Subscribes to any `/presence_hint` topics automatically.
* Example bridge launchers included under `hardware/drivers/`.

### Golden Configurations & Bound Derivation

* **Golden Parameters**: Stored in `config/golden_params.yaml`, auto-loaded by the fusion tracker.
* **Safe Bounds for Unity**: `unity/Assets/StreamingAssets/ParamBounds.json` enforces experimental slider limits via:
    ```bash
    python3 tools/freeze_golden.py
    python3 tools/derive_bounds.py
    ```

### Stability-Weighted Fusion

* `fusion_tracker_node.py` implements gating + exponential moving average (EMA) stability, using parameters like `assoc_gate_m`, `track_decay_sec`, `stability_alpha`, `birth_stability_min`, `delete_stability_min`, and `rf_weight`. Outputs predictions via `/aura/predictions_jsonl`.

### Adversarial & Regression Testing

* **Challenge Bags**: Generate adversarial test scenarios using:
    ```bash
    tools/make_challenge_bag.sh scenarios/crossing_targets.yaml out/challenge true
    ```
    Then evaluate via standard pipeline or export using ROS bag tools.

---
## 📂 Repository Layout
/
├── .github/
├── config/
│   └── golden_params.yaml
├── evaluation/
├── hardware/
│   ├── udev/
│   └── drivers/
├── ros2_examples/
├── scripts/
├── scenarios/
├── tests/
├── tools/
│   ├── run_experiments.py
│   ├── run_single.py
│   ├── optimize_experiments.py
│   ├── freeze_golden.py
│   ├── derive_bounds.py
│   └── make_challenge_bag.sh
├── unity/Assets/NeuromorphAR/Scripts/
│   ├── GhostOverlay.cs
│   └── ParamSender.cs
├── Dockerfile
├── docker-compose.yml
├── Makefile
├── start_aura.sh
├── stop_aura.sh
├── run_stress_tests.sh
├── onboarding.sh
├── cleanup.sh
├── requirements*.txt
└── README.md


---
## Summary of Capabilities
* Fast evaluation via `make eval-fast`
* Interactive sweeps & heatmap reporting
* Unity-based HUD visualization & live tuning
* Full ROS 2 pipeline simulation and evaluation
* Hardware integration with dynamic sensor support
* Golden configuration generation and bounds derivation for safe UI
* Stability-weighted fusion for enhanced tracking robustness
* Adversarial testing workflows using challenge bags

