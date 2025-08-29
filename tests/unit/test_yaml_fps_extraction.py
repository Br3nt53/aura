from tools import run_single_adapter
import textwrap


def test_fps_from_yaml_sensor(tmp_path):
    yaml_path = tmp_path / "scenario.yaml"
    yaml_path.write_text(
        textwrap.dedent(
            """
    sensor:
      fps: 25
    """
        )
    )
    fps = run_single_adapter._derive_fps_from_env_and_yaml(str(yaml_path))
    assert fps == 25.0


def test_fps_from_yaml_camera(tmp_path):
    yaml_path = tmp_path / "scenario.yaml"
    yaml_path.write_text(
        textwrap.dedent(
            """
    camera:
      fps: 15
    """
        )
    )
    fps = run_single_adapter._derive_fps_from_env_and_yaml(str(yaml_path))
    assert fps == 15.0


def test_fps_from_yaml_top_level(tmp_path):
    yaml_path = tmp_path / "scenario.yaml"
    yaml_path.write_text("fps: 60\n")
    fps = run_single_adapter._derive_fps_from_env_and_yaml(str(yaml_path))
    assert fps == 60.0
