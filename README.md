# aura_experiments_runner_bundle

A minimal, self-contained bundle to run parameter sweeps, stress-test multi-target tracking,
inject RF clutter, and iterate on HUD UX for the AURA project. It includes:

- **Experiment runner** (`tools/run_experiments.py`) with grid sweeps and an HTML heatmap report
- **Single-run stub** (`tools/run_single.py`) for you to replace with your actual simulator/evaluator
- **Scenarios** (`scenarios/*.yaml`) for crossing targets and RF clutter
- **Live-tuning ROS 2 bridge** (`aura_tools` package) to set parameters from Unity at runtime
- **Unity scripts** (`unity/Assets/NeuromorphAR/Scripts`) for confidence-based HUD visuals and slider → ROS param sender

> ⚠️ The `tools/run_single.py` evaluator is a stub that generates plausible-looking metrics. Replace it
with your real simulation/evaluation entry points (ROS bag playback, Unity loopback, etc.).

## Quickstart

```bash
# 0) (Optional) Create a venv and install project deps
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

# 1) Build & source your ROS 2 workspace (for the aura_tools package) or run the node directly
# Option A: run as a plain script
python3 aura_tools/aura_tools/param_bridge.py

# Option B (ROS 2 package):
# Build in a colcon workspace (see 'ROS 2 Package Install' below)
# then run:
ros2 run aura_tools param_bridge

# 2) Launch a scenario (if you have an emulation launch file in your stack)
# Example (replace with your actual launch file):
# ros2 launch aura_launch aura_emulation.launch.py scenario:=scenarios/crossing_targets.yaml

# 3) Run the sweep and open the report
python3 tools/run_experiments.py --config tools/experiments.sample.json
open out/experiments/fusion_sweep_v1/report.html   # macOS
# or
xdg-open out/experiments/fusion_sweep_v1/report.html
```

### Live Tuning from Unity

- Add `ParamSender` to a Unity GameObject and assign your slider's **OnValueChanged(float)** to call
  `ParamSender.SetDouble("assoc_gate_m", value)` (or any other parameter name your node exposes).
- Run the ROS 2 param bridge (`aura_tools/aura_tools/param_bridge.py`). It listens on `/aura/param_set`
  for JSON strings like `{ "node": "/aura_fusion", "name": "assoc_gate_m", "value": 1.2 }`.

### ROS 2 Package Install (Optional)

If you want the param bridge as a proper ROS 2 Python package:

```bash
# inside your colcon workspace: ~/ros2_ws/src/
cp -r /path/to/aura_experiments_runner_bundle/aura_tools ~/ros2_ws/src/

# build
cd ~/ros2_ws
colcon build
source install/setup.bash  # or setup.zsh

# run
ros2 run aura_tools param_bridge
```

### Repo Layout

```
aura_experiments_runner_bundle/
├─ tools/
│  ├─ experiments.sample.json
│  ├─ run_experiments.py
│  └─ run_single.py
├─ scenarios/
│  ├─ crossing_targets.yaml
│  └─ rf_clutter.yaml
├─ aura_tools/                # ROS 2 Python package (optional install)
│  ├─ package.xml
│  ├─ setup.py
│  ├─ setup.cfg
│  ├─ resource/
│  │  └─ aura_tools
│  └─ aura_tools/
│     ├─ __init__.py
│     └─ param_bridge.py
├─ unity/Assets/NeuromorphAR/Scripts/
│  ├─ GhostOverlay.cs
│  └─ ParamSender.cs
├─ requirements.txt
└─ .gitignore
```

## Notes

- **Primary metric**: `auc` in this bundle. Add your own metrics in `tools/run_single.py` and the runner will
  aggregate medians across repetitions.
- **Report**: `out/experiments/<name>/report.html` has a self-contained heatmap (no external CDN dependencies).
- **Unity HUD**: `GhostOverlay.cs` adds a non-linear confidence curve and a low-confidence flicker to make
  uncertain ghosts appear less solid.

### Don’t know what GT/Pred are? Start here

- **Ground Truth (GT)** is the “answer key.” This bundle now **auto-generates GT** from your scenario YAML:
  - When you run `tools/run_single.py` without `--gt`, it calls `tools/make_gt_from_yaml.py` and writes `gt.jsonl` for you.
- **Predictions** are what your system outputs each frame. If you just have a spreadsheet/CSV, convert it with:
  ```bash
  python3 tools/pred_from_csv.py --csv my_detections.csv --out out/tmp/pred.jsonl
  ```
  CSV columns: `frame,x,y,score,id` (score/id optional).

Then evaluate once with:
```bash
python3 tools/run_single.py   --scenario scenarios/crossing_targets.yaml   --rf_weight 0.6 --wall_bonus 0.4 --track_decay_sec 1.5   --out out/tmp/metrics.json   --pred out/tmp/pred.jsonl
```
The sweep runner will use the same mechanism automatically.

## Advanced Features

### 1) AI-Powered Experiment Optimization (Bayesian)
Run far fewer experiments and still find great params:
```bash
pip install -r requirements.txt  # includes scikit-optimize
python3 tools/optimize_experiments.py --scenario scenarios/crossing_targets.yaml --n_calls 25
# results in out/bo_runs/bo_summary.json and trials.jsonl
```

### 2) Physics-Based RF Sensor Model (ROS 2 example)
We provide a physics-inspired RF mock node you can drop into a ROS 2 workspace:
```
ros2_examples/aura_sensors/aura_sensors/nodes/mock_rf_node.py
```
It publishes a simple presence hint with confidence shaped by distance^2 path loss and a "wall" attenuation heuristic.
Wire its output into your fusion node or logger to make predictions more realistic.

### 3) Human-in-the-Loop Simulation (Keyboard Teleop)
Drive a target manually to create adversarial motion:
```
ros2_examples/aura_sensors/aura_sensors/nodes/teleop_target_node.py
```
Controls: **WASD** to move in the x/y plane; publishes to `/mock_target/pose` at 20 Hz. Requires `pip install pynput`.

> Note: The ROS nodes are provided as examples to copy into your ROS 2 package (or launch directly with `python3` in a ROS-sourced shell). If you want them packaged fully, I can scaffold a proper ament_python package too.

### ROS 2 Package (ament_python) — `aura_examples`

We’ve added a fully buildable ROS 2 package with nodes + launch:

```
ros2_examples/aura_examples/
  ├─ aura_examples/nodes/
  │  ├─ scenario_player_node.py   # plays a YAML scenario, publishes /mock_target/pose and /aura/frame
  │  ├─ mock_rf_node.py           # physics-based RF hint publisher /aura/rf_presence_hint
  │  ├─ fusion_sim_node.py        # converts hints to JSON prediction lines on /aura/predictions_jsonl
  │  ├─ recorder_node.py          # writes /aura/predictions_jsonl to <workdir>/pred.jsonl
  │  └─ teleop_target_node.py     # manual target control (WASD) → /mock_target/pose
  ├─ launch/bringup.launch.py     # scenario→rf→fusion→record pipeline
  ├─ package.xml, setup.py, setup.cfg, resource/aura_examples
```

**Install into a workspace:**
```bash
# In a ROS 2 sourced shell
mkdir -p ~/ros2_ws/src
cp -r ros2_examples/aura_examples ~/ros2_ws/src/
cd ~/ros2_ws && colcon build
source install/setup.bash  # or setup.zsh
```

**Run the full pipeline & record predictions:**
```bash
ros2 launch aura_examples bringup.launch.py scenario:=<ABS_PATH>/scenarios/crossing_targets.yaml workdir:=<ABS_PATH>/out/tmp
# writes <workdir>/pred.jsonl
```

**Or drive it from our runner:**
```bash
python3 tools/run_single.py   --scenario scenarios/crossing_targets.yaml   --rf_weight 0.6 --wall_bonus 0.4 --track_decay_sec 1.5   --out out/tmp/metrics.json   --ros2_pipeline --workdir out/tmp
```

This runs the ROS pipeline long enough to play the scenario, writes `pred.jsonl`, auto-generates `gt.jsonl` from YAML, and computes AUC/MOTA.


## Plug-and-Play Hardware Integration

This repo now includes the scaffolding to make hardware **plug-and-play**:

1) **udev rules** for stable device names  
   - Edit `hardware/udev/99-aura-sensors.rules` with your vendor/product IDs (via `lsusb`).  
   - Install with `sudo hardware/udev/install_udev.sh`.  
   - Your devices appear as `/dev/aura_uwb0`, `/dev/aura_thermal0`, etc.

2) **Sensor Manager** (dynamic driver launcher)  
   - Config: `hardware/config/sensors.yaml` maps device path patterns → launch commands.  
   - Run: `ros2 launch aura_examples sensor_bringup.launch.py config:=hardware/config/sensors.yaml`  
   - When `/dev/aura_*` appears, the manager launches the right driver; when unplugged, it stops it automatically.

3) **Dynamic Fusion** (auto topic discovery)  
   - Node: `aura_examples/nodes/fusion_dynamic_node.py` subscribes to any `std_msgs/Float32MultiArray` topic ending with `/presence_hint`.  
   - Our example RF mock now publishes to `/aura/rf/presence_hint`. Add your drivers to publish the same shape.

4) **Example driver launchers**  
   - `hardware/drivers/uwb_bridge.launch.py`, `hardware/drivers/thermal_bridge.launch.py` — replace with your real bridge nodes/params.

This architecture lets you plug a sensor via USB, get a stable device path via udev, auto-launch its bridge, and have fusion begin consuming its stream — **no manual reconfiguration**.


## Golden Config & Safe Bounds (data → defaults)

- **Golden parameters** live in `config/golden_params.yaml`. The fusion tracker auto-loads them on startup.
  - Generate from your latest optimizer/sweeps:
    ```bash
    python3 tools/freeze_golden.py
    ```

- **Safe slider bounds** for Unity live in `unity/Assets/StreamingAssets/ParamBounds.json`.
  - Derive from experiments (keeps runs within 95% of best AUC):
    ```bash
    python3 tools/derive_bounds.py
    ```
  - The in-sim `TuningGUI` reads these and constrains sliders accordingly.

## Stability-weighted Fusion

- New node: `fusion_tracker_node.py` (gating + stability EMA). Parameters:
  - `assoc_gate_m`, `track_decay_sec`, `stability_alpha`, `birth_stability_min`, `delete_stability_min`, `rf_weight`.
- Loads golden params by default; publishes `/aura/predictions_jsonl` for evaluation.

## Challenge Bags

- Create adversarial recordings to regression-test edge cases:
  ```bash
  # Use teleop or tricky scenario and record a bag
  tools/make_challenge_bag.sh scenarios/crossing_targets.yaml out/challenge true
  ```
  Then evaluate with your existing pipeline or export to JSONL using the rosbag exporter skeleton.
