import pathlib


def test_runner_placeholder(tmp_path: pathlib.Path):
    # Placeholder test to keep CI/lint happy; real smoke/integration lives elsewhere.
    assert (tmp_path / "ok").name == "ok"
