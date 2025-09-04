#!/bin/bash
set -e # Exit immediately if a command exits with a non-zero status.

echo "--- AURA Project Onboarding ---"

# Step 1: Install System Dependencies
echo "[1/4] Installing system dependencies..."
sudo apt-get update
sudo apt-get install -y ros-humble-desktop python3-pip git
curl -LsSf https://astral.sh/uv/install.sh | sh
uv pip install --system --no-cache -r requirements.txt

# Step 2: Set up ROS2 Workspace
echo "[2/4] Setting up ROS2 workspace..."
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Step 3: Build the Workspace
echo "[3/4] Building the ROS2 workspace..."
source /opt/ros/humble/setup.bash
colcon build

# Step 4: Run a confirmation test
echo "[4/4] Running a final confirmation smoke test..."
source install/setup.bash
ros2 launch aura_examples bringup.launch.py &
LAUNCH_PID=$!
sleep 15
kill $LAUNCH_PID

echo "--- Onboarding Complete! ---"
echo "The AURA simulation environment is ready. Run 'source install/setup.bash' in a new terminal to get started."