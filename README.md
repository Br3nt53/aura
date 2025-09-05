AURA Multi-Target Tracking Toolkit


AURA is a comprehensive, self-contained toolkit for developing, testing, and evaluating multi-target tracking (MTT) algorithms. It provides a robust framework for parameter sweeps, full-system simulation with ROS 2, and rapid, Docker-free local evaluation.
✨ Key Features
Dual-Mode Operation — Fast, lightweight evaluations locally without ROS, or high-fidelity ROS 2 simulation pipeline.
ROS 2 Integration — A complete package (aura_examples) with nodes for scenario playback, simulated RF sensing, sensor fusion, and data recording.
Containerized Environment — Docker-based setup for consistent, reproducible development and testing.
Stress Testing — Scenarios that stress trackers under high clutter and dense targets.
Hardware & Unity Ready — Hooks for real-sensor integration and a Unity bridge for live parameter tuning.
CI/CD — GitHub Actions for automated linting, testing, and a Docker smoke test on each push.
🧰 System Requirements
Recommended
OS: Ubuntu 22.04 LTS (primary target for ROS 2 Humble)
Docker: 24.x or newer, with docker compose plugin
Python: 3.11 for local/no-Docker workflows
Notes
macOS and Windows users: local ROS 2 is non-trivial; prefer the Docker route for the full simulation.
Linux without Docker: ensure ROS 2 Humble is installed and sourced before launching the pipeline.
🚀 Quickstart
There are two primary ways to use AURA.
Option A — Docker (Recommended)
This path runs the complete ROS 2 simulation in a clean, reproducible environment.
# 1) Build the container image
docker build -t aura:latest .

# 2) Launch an interactive shell in the container with the repo mounted at /work
docker run --rm -it -v "$PWD":/work -w /work aura:latest bash

# 3) Inside the container: (one-time) install Python deps for local utilities
uv pip install -r requirements.txt -r requirements-dev.txt

# 4) Inside the container: build the ROS 2 workspace
cd ros2_ws
colcon build
source install/setup.bash
cd ..

# 5) Inside the container: run the ROS 2 simulation (edit the scenario/params as needed)
ros2 launch aura_examples bringup.launch.py scenario:=scenarios/rf_clutter.yaml

# 6) (Optional) Inside the container: run tests and lint
make test
make lint
Prefer Docker Compose?
If you maintain a docker-compose.yml, you can equivalently use:
docker compose build
docker compose up -d
docker compose exec aura bash
Then run the same steps as above inside the container shell.
If you previously used helper scripts like start_aura.sh, stop_aura.sh, or run_stress_tests.sh, they may simply wrap the compose commands above. If they are not present, use the direct docker/docker compose commands.
Option B — Local (No Docker)
Use this for quick, ROS-free evaluations and development.
# 1) Ensure Python 3.11 is available
python3 --version

# 2) Install uv and dependencies
curl -LsSf https://astral.sh/uv/install.sh | sh
uv pip install -r requirements.txt -r requirements-dev.txt

# 3) Run a fast, ROS-free evaluation (edit SCENARIO/PARAMS/OUT as needed)
make eval-fast SCENARIO=scenarios/rf_clutter.yaml PARAMS=scenarios/params.min.yaml OUT=out/fast-eval

# Metrics will be written to:
#   out/fast-eval/metrics.json
🧪 Running a Single Experiment (Entry Point)
The main entry point for evaluations is tools/run_single.py. It can generate ground truth from a scenario, synthesize predictions (non-ROS runs), and compute metrics.
python3 tools/run_single.py \
  --scenario scenarios/rf_clutter.yaml \
  --params   scenarios/params.min.yaml \
  --out-dir  out/my_experiment
🧵 Full ROS 2 Pipeline
# Source your ROS 2 environment (Docker users: the base image typically does this)
source /opt/ros/humble/setup.bash

# Build and source the AURA workspace
cd ros2_ws
colcon build
source install/setup.bash
cd ..

# Launch the simulation (edit the scenario path as desired)
ros2 launch aura_examples bringup.launch.py scenario:=scenarios/rf_clutter.yaml
🧯 Makefile Targets
make setup   # install Python deps (uv) and prepare local tools
make test    # run pytest-based unit/integration tests
make lint    # run ruff/black checks
make fmt     # auto-format with ruff/black
make eval-fast SCENARIO=... PARAMS=... OUT=...  # quick, ROS-free evaluation
📊 Metrics (What You Get in metrics.json)
AURA’s evaluator (evaluation/mot_evaluator.py) records standard multi-target tracking KPIs commonly used in MOT literature. Exact fields may vary by configuration, but typically include:
detections.total — Number of detections evaluated
tracks.total — Number of ground-truth tracks
tp / fp / fn — True/false positives and false negatives
id_switches — Count of identity switches
precision / recall / f1 — Detection-level rates
MOTA / MOTP — Multi-Object Tracking Accuracy and Precision
IDF1 — F1 over ID associations (identity preservation)
fragmentations — Track continuity breaks
Example metrics.json:
{
  "summary": {
    "detections": 1243,
    "tracks": 18,
    "tp": 1104,
    "fp": 139,
    "fn": 98,
    "id_switches": 7,
    "precision": 0.888,
    "recall": 0.918,
    "f1": 0.903,
    "MOTA": 0.862,
    "MOTP": 0.211,
    "IDF1": 0.879,
    "fragmentations": 5
  }
}
If your evaluator exposes additional metrics (e.g., HOTA, association AUC), they will appear as extra fields. See the evaluator source for authoritative definitions.
🔁 Reproducibility
To minimize drift across machines and time:
Fixed Seeds
Set an explicit seed for any random components (scenario sampling, noise, shuffles). Convention: read AURA_SEED from the environment with a sensible default.
export AURA_SEED=12345
Pinned Dependencies
Capture a constraints snapshot after a known-good run:
uv pip freeze > constraints.txt
Reinstall with constraints to reproduce:
uv pip install -r requirements.txt -r requirements-dev.txt -c constraints.txt
Golden Parameters
Keep a canonical baseline at config/golden_params.yaml.
Store frozen baselines in version control (e.g., out/baselines/<name>/metrics.json) and compare in CI to catch regressions.
Deterministic I/O
Write outputs under a run-scoped directory (out/<run_id>/...) and record: git commit, seed, scenario/params paths, and package versions.
🗂️ Repository Structure
.
├── .github/              # CI/CD workflows
├── config/               # Configuration (e.g., golden_params.yaml)
├── evaluation/           # Core evaluation logic (e.g., mot_evaluator.py)
├── hardware/             # Hardware integration (udev rules, configs)
├── ros2_examples/        # ROS 2 package(s) and launch files
├── ros2_ws/              # ROS 2 workspace
├── scenarios/            # YAML files for experiment conditions
├── tools/                # CLI scripts (e.g., run_single.py)
├── tests/                # Pytest unit/integration tests
├── unity/                # C# scripts for Unity integration
├── Dockerfile            # Main development/simulation container
├── docker-compose.yml    # Docker Compose configuration (optional)
└── Makefile              # Commands for setup, testing, evaluation
🧾 Scenario Schema (Authoring Your Own)
Scenarios are YAML files ingested by tools/run_single.py and the ROS nodes. The exact schema is defined by the loader/parsers in this repo, but a typical pattern looks like:
name: rf_clutter_demo
duration_sec: 30
frame_rate_hz: 20

sensor:
  type: rf_sim
  noise_std: 0.15
  clutter:
    rate_hz: 25
    spatial_extent: [ -50, 50, -50, 50 ]  # xmin, xmax, ymin, ymax

targets:
  - id: 1
    initial_state:
      position: [ -20, -10 ]
      velocity: [  1.2,  0.7 ]
  - id: 2
    initial_state:
      position: [  15,  18 ]
      velocity: [ -0.8, -1.1 ]

# Optional occlusions/obstacles for harder scenarios
occlusions:
  - rect: [ -5, 5, -3, 3 ]
    probability: 0.6
Tips
Keep units consistent (document them in comments).
Validate the scenario by running a short eval-fast and inspecting metrics.json plus any generated plots (if enabled).
🧪 Expected Results (Sanity Check)
For a tiny toy scenario with two crossing targets and moderate clutter, you should expect:
Non-zero FP due to clutter,
High recall if the simulated sensor is generous,
ID switches ≈ 0–few if crossings are brief and the fusion/tracker tuning is stable.
Use the out/.../metrics.json summary to spot obvious misconfigurations (e.g., all FNs, exploding FPs, or precision/recall ≈ 0).
🧩 Unity & Live Tuning
The unity/ directory contains C# scripts for HUD and live parameter control. At runtime, parameters can be piped into ROS to adjust tracker behavior. Ensure the Unity scene references the correct ROS topic names and matches your scenario rate.
🧷 Troubleshooting
No ROS graph / launch hangs — Ensure the ROS environment is sourced and that colcon build completed without errors.
eval-fast writes empty metrics — Confirm the scenario file is non-empty and fields match the expected schema; try a known good scenario first.
Drifting results between runs — Set AURA_SEED, pin dependencies with constraints.txt, and use config/golden_params.yaml.
Unity cannot tune params — Check topic names, message types, and ROS↔Unity bridge connectivity.
🤖 CI/CD
ci.yml — Python tests + style checks (ruff/black).
docker-smoke.yml — Builds the Docker image and performs a basic runtime smoke test.
(Optional) Add a regression check comparing current metrics.json to a golden baseline for designated scenarios.
🤝 Contributing
Please run:
make fmt && make lint && make test
before submitting a PR. Consider adding or updating:
A new scenario under scenarios/,
Unit tests in tests/,
Documentation examples and expected results.
