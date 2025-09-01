#!/usr/bin/env python3
import argparse
import subprocess
import yaml  # type: ignore[import-untyped]
import tempfile
import os


def run_ros2_pipeline(scenario_path, params_path, out_pred_path, test_case=None):
    """Runs the full ROS 2 pipeline for a given scenario."""

    with open(scenario_path, "r") as f:
        scenario_data = yaml.safe_load(f)

    scenario_to_run = scenario_data
    if test_case:
        if "test_cases" in scenario_data and test_case in scenario_data["test_cases"]:
            scenario_to_run = scenario_data["test_cases"][test_case]
        else:
            raise ValueError(f"Test case '{test_case}' not found in {scenario_path}")

    with tempfile.NamedTemporaryFile(
        mode="w", delete=False, suffix=".yaml"
    ) as tmp_scenario:
        yaml.dump(scenario_to_run, tmp_scenario)
        tmp_scenario_path = tmp_scenario.name

    try:
        # **THE ACTUAL FIX**
        # We must explicitly source the ROS 2 environments in the same shell
        # that executes the launch command. This is the most robust method.
        ros_command = (
            f"source /opt/ros/humble/setup.bash && "
            f"source /aura_ws/ros2_ws/install/setup.bash && "
            f"ros2 launch aura_examples bringup.launch.py "
            f"scenario:={tmp_scenario_path} "
            f"params_file:={params_path} "
            f"pred_path:={out_pred_path} "
            "headless:=true"
        )

        # The command is wrapped in a bash shell to handle the 'source' commands.
        cmd = ["/bin/bash", "-c", ros_command]

        print(f"Executing ROS 2 command: {ros_command}")

        process = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
        )

        for line in iter(process.stdout.readline, ""):
            print(line, end="")

        process.wait()

        if process.returncode != 0:
            print("--- STDERR ---")
            print(process.stderr.read())
            raise subprocess.CalledProcessError(process.returncode, cmd)

    finally:
        os.remove(tmp_scenario_path)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run the AURA ROS 2 pipeline.")
    parser.add_argument(
        "--scenario", required=True, help="Path to the scenario YAML file."
    )
    parser.add_argument(
        "--params", required=True, help="Path to the parameters YAML file."
    )
    parser.add_argument(
        "--out-pred",
        required=True,
        help="Path to save the prediction output JSONL file.",
    )
    parser.add_argument(
        "--test-case",
        required=False,
        help="Specify a single test case to run from a scenario file.",
    )
    args = parser.parse_args()

    run_ros2_pipeline(args.scenario, args.params, args.out_pred, args.test_case)
