#!/bin/bash
set -e

KEYSTORE_DIR="keystore"

if [ -d "$KEYSTORE_DIR" ]; then
  echo "Keystore directory '$KEYSTORE_DIR' already exists. Aborting."
  exit 1
fi

echo "Creating new SROS keystore in ./$KEYSTORE_DIR"
ros2 security create_keystore $KEYSTORE_DIR

# List all nodes from the setup.py file to generate keys for
NODES=(
  "fusion_tracker_node"
  "fusion_dynamic_node"
  "sensor_manager_node"
  "recorder_node"
  "scenario_player_node"
  "teleop_target_node"
  # Add any other nodes from your packages here
)

for node in "${NODES[@]}"; do
  echo "Generating key for node: $node"
  ros2 security create_key $KEYSTORE_DIR $node
done

echo "Keystore generation complete."
echo "Next, populate permissions.xml with the correct policies."
