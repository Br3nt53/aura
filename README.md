# AURA Multi-Target Tracking Toolkit

[![CI](https://github.com/br3nt53/aura/actions/workflows/ci.yml/badge.svg)](https://github.com/br3nt53/aura/actions/workflows/ci.yml)
[![Docker Smoke Test](https://github.com/br3nt53/aura/actions/workflows/docker-smoke.yml/badge.svg)](https://github.com/br3nt53/aura/actions/workflows/docker-smoke.yml)

**AURA** is a comprehensive, self-contained toolkit for developing, testing, and evaluating multi-target tracking algorithms. It provides a robust framework for parameter sweeps, full-system simulation with ROS 2, and rapid, Docker-free local evaluation.

## ✨ Key Features

* **Dual-Mode Operation:** Run fast, lightweight evaluations locally without ROS, or launch the full ROS 2 simulation pipeline for high-fidelity testing.
* **ROS 2 Integration:** Includes a complete ROS 2 package (`aura_examples`) with nodes for scenario playback, simulated RF sensing, sensor fusion, and data recording.
* **Containerized Environment:** A Docker-based setup ensures a consistent and reproducible environment for development and testing.
* **Stress Testing:** Comes with a suite of challenging scenarios to validate tracker performance under adverse conditions like high clutter and dense targets.
* **Hardware & Unity Ready:** Provides infrastructure for integrating real hardware sensors and a bridge for live parameter tuning from external GUIs like Unity.
* **CI/CD Integration:** Includes GitHub Actions workflows for automated linting, testing, and Docker smoke tests on every push.

## 🚀 Getting Started

There are two primary ways to use the AURA toolkit.

### Option A: Docker (Recommended)

This is the easiest and most reliable way to run the full simulation.

1.  **Start the System:** Build the Docker image and launch the container in the background.
    ```bash
    ./start_aura.sh
    ```

2.  **Run the Stress Test Suite:** Execute the full suite of tests inside the running container.
    ```bash
    docker compose exec aura ./run_stress_tests.sh
    ```
    Results will be saved to the `out/stress_tests` directory on your host machine.

3.  **Stop the System:** Shut down and remove the container.
    ```bash
    ./stop_aura.sh
    ```

### Option B: Local Environment Setup

This is for development and running evaluations without Docker. You must have **ROS 2 Humble** and **Python 3.11** installed.

1.  **Install uv**:
    ```bash
    curl -LsSf [https://astral.sh/uv/install.sh](https://astral.sh/uv/install.sh) | sh
    ```

2.  **Install Dependencies**:
    ```bash
    uv pip install -r requirements.txt -r requirements-dev.txt
    ```

3.  **Run a Fast Evaluation:** Execute a single scenario evaluation without starting the ROS 2 system. This is great for quick checks.
    ```bash
    # Ensure the scenario file has content first!
    make eval-fast SCENARIO=scenarios/rf_clutter.yaml PARAMS=scenarios/params.min.yaml OUT=out/fast-eval
    ```
    Metrics will be saved to `out/fast-eval/metrics.json`.

## ⚙️ Core Workflows

### Running a Single Experiment

The `tools/run_single.py` script is the main entry point for all evaluations. It can generate ground truth data from a scenario, synthesize predictions (for non-ROS runs), and calculate performance metrics.

```bash
python3 tools/run_single.py \
    --scenario scenarios/rf_clutter.yaml \
    --params scenarios/params.min.yaml \
    --out-dir out/my_experiment
Running the Full ROS 2 Pipeline

You can launch the entire ROS 2 simulation pipeline, which includes the scenario player, mock sensor, fusion tracker, and recorder nodes.

Bash
# Source your ROS 2 environment
source /opt/ros/humble/setup.bash

# Build and source the AURA workspace
cd ros2_ws
colcon build
source install/setup.bash
cd ..

# Launch the simulation
ros2 launch aura_examples bringup.launch.py
The Makefile provides several helpful commands for local development:

make setup: Installs all dependencies using uv.

make test: Runs the pytest unit and integration tests.

make lint: Runs ruff and black to check for code style issues.

make fmt: Automatically formats the code with ruff and black.

📂 Repository Structure
.
├── .github/              # CI/CD workflows
├── config/               # Configuration files (e.g., golden_params.yaml)
├── evaluation/           # Core evaluation logic (mot_evaluator.py)
├── hardware/             # Hardware integration (udev rules, configs)
├── ros2_examples/        # The primary ROS 2 package with nodes and launch files
├── ros2_ws/              # ROS 2 workspace directory
├── scenarios/            # YAML files defining experiment conditions
├── tools/                # Command-line scripts for running experiments
├── tests/                # Pytest unit and integration tests
├── unity/                # C# scripts for Unity integration
├── Dockerfile            # Defines the main development/simulation container
├── docker-compose.yml    # Docker Compose configuration
└── Makefile              # Commands for setup, testing, and evaluation