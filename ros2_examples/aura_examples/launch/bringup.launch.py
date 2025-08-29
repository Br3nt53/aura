from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    scenario = DeclareLaunchArgument("scenario", default_value="")
    workdir = DeclareLaunchArgument("workdir", default_value="out/tmp")
    record = DeclareLaunchArgument("record", default_value="false")

    # Call the script directly from the source tree inside the container
    script_path = "/work/ros2_ws/src/aura_examples/scripts/scenario_player"

    return LaunchDescription([
        scenario, workdir, record,
        ExecuteProcess(
            cmd=[
                script_path,
                "--scenario", LaunchConfiguration("scenario"),
                "--workdir", LaunchConfiguration("workdir"),
            ],
            output="screen",
        ),
    ])
