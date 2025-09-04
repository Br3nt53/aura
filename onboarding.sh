#!/bin/bash
set -e # Exit immediately if a command exits with a non-zero status.

echo "--- AURA Project Onboarding ---"

# Step 1: Install System Dependencies
echo "[1/4] Installing system dependencies..."
sudo apt-get update
sudo apt-get install -y ros-humble-desktop python3.11-venv git curl
source /opt/ros/humble/setup.bash

# Step 2: Set up Python Environment with uv
echo "[2/4] Setting up Python environment with uv..."
curl -LsSf https://astral.sh/uv/install.sh | sh
source "$HOME/.cargo/env"
uv pip install -r requirements.txt -r requirements-dev.txt

# Step 3: Set up and Build ROS2 Workspace
echo "[3/4] Setting up and building the ROS2 workspace..."
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
cd ..

# Step 4: Run a confirmation test
echo "[4/4] Running a final confirmation smoke test..."
source ros2_ws/install/setup.bash
ros2 launch aura_examples bringup.launch.py &
LAUNCH_PID=$!
sleep 15
kill $LAUNCH_PID

echo "--- Onboarding Complete! ---"
echo "The AURA simulation environment is ready. In a new terminal, run:"
echo "source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash"