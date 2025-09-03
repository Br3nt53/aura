# AURA Experiments Runner Bundle

A comprehensive and self-contained toolkit for parameter sweeps, multi-target tracking stress tests, RF clutter injection, Unity-based HUD UX iteration, hardware integration, and full-system evaluation via ROS 2.

---

## 🚀 Quickstart

### Option A: Local Fast Evaluation (No ROS or Docker)

1. **Setup the environment**  
   Create a virtual environment and install dependencies:
   ```bash
   make setup
Run fast evaluation
This performs ground-truth generation, prediction synthesis, evaluation, and outputs metrics:
make eval-fast
Results are printed to the terminal and saved to out/tmp/metrics.json.
Option B: Full System via Docker & ROS 2
Start the full system
Build the Docker image and launch the container:
./start_aura.sh
Stop the system
./stop_aura.sh
Features & Components
Experimentation & Evaluation Tools
Parameter Sweeps & Reports
tools/run_experiments.py supports grid sweeps and generates standalone HTML heatmap reports.
Single Scenario Runner (Stub)
tools/run_single.py—replaceable evaluation entry point for real simulation or evaluation workflows.
Scenario Definitions
YAML files under scenarios/ (e.g., crossing_targets.yaml, rf_clutter.yaml) define test environments.
Unity Integration & Live Tuning
Unity HUD Support
Unity scripts (GhostOverlay.cs, ParamSender.cs) in unity/Assets/... enable confidence-based visualization and parameter control.
ROS 2 Param Bridge
The aura_tools package supports real-time parameter injection via /aura/param_set.
ROS 2 Packages & Launch Infrastructure
ROS Examples Package (aura_examples)
Includes simulated nodes (scenario_player_node.py, mock_rf_node.py, fusion_sim_node.py, recorder_node.py, teleop_target_node.py) and a launch pipeline (bringup.launch.py) for scenario → RF → fusion → record workflows.
Run via:
python3 tools/run_single.py \
  --scenario scenarios/crossing_targets.yaml \
  --rf_weight 0.6 --wall_bonus 0.4 --track_decay_sec 1.5 \
  --ros2_pipeline --workdir out/tmp
Hardware Integration
Plug-and-Play Sensor Support
Udev rules in hardware/udev/ ensure stable device names (e.g. /dev/aura_uwb0).
Sensor Manager (sensor_bringup.launch.py) auto-launches/stops driver nodes on device plug/unplug.
Fusion dynamic discovery subscribes to any /presence_hint topics automatically.
Example bridge launchers included under hardware/drivers/.
Golden Configurations & Bound Derivation
Golden Parameters
Stored in config/golden_params.yaml, auto-loaded by the fusion tracker.
Safe Bounds for Unity
unity/Assets/StreamingAssets/ParamBounds.json enforces experimental slider limits via:
python3 tools/freeze_golden.py
python3 tools/derive_bounds.py
Stability-Weighted Fusion
fusion_tracker_node.py implements gating + exponential moving average (EMA) stability, using parameters like assoc_gate_m, track_decay_sec, stability_alpha, birth_stability_min, delete_stability_min, and rf_weight. Outputs predictions via /aura/predictions_jsonl.
Adversarial & Regression Testing
Challenge Bags
Generate adversarial test scenarios using:
tools/make_challenge_bag.sh scenarios/crossing_targets.yaml out/challenge true
Then evaluate via standard pipeline or export using ROS bag tools.
Repository Layout
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
Summary of Capabilities
Fast evaluation via make eval-fast
Interactive sweeps & heatmap reporting
Unity-based HUD visualization & live tuning
Full ROS 2 pipeline simulation and evaluation
Hardware integration with dynamic sensor support
Golden configuration generation and bounds derivation for safe UI
Stability-weighted fusion for enhanced tracking robustness
Adversarial testing workflows using challenge bags
