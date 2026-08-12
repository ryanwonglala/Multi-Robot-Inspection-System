import argparse
from pathlib import Path
import sys


PACKAGE = Path(__file__).resolve().parents[1]
SCRIPTS = PACKAGE / "scripts"
sys.path.insert(0, str(SCRIPTS))

import run_return_home as home  # noqa: E402


def args(tmp_path: Path, **overrides) -> argparse.Namespace:
    values = {
        "world": PACKAGE / "config/world_model_real_v5.yaml",
        "map": PACKAGE / "maps/lab_arena_v5.yaml",
        "nav_params": PACKAGE / "config/nav2_real.yaml",
        "report_dir": tmp_path,
        "use_rviz": False,
        "check_only": True,
        "enable_motion": False,
        "base_feedback_timeout": 30.0,
        "min_voltage": 11.0,
        "nav_startup_timeout": 0.0,
        "nav_timeout": 0.0,
        "max_start_error_m": 0.20,
    }
    values.update(overrides)
    return argparse.Namespace(**values)


def test_uses_validated_stop_as_initial_pose_and_authored_home(tmp_path, monkeypatch):
    monkeypatch.setattr(home, "get_package_share_directory", lambda _: str(PACKAGE))
    run = home.ReturnHomeRun(args(tmp_path))
    assert run.initial_pose == home.Pose2D(1.473, -0.330, -1.654)
    assert run.home_pose == home.Pose2D(-0.003, 0.017, -0.054)
    assert run.home_xy_tolerance == 0.05
    assert run.home_yaw_tolerance == 0.08


def test_check_only_does_not_start_ros_or_motion(tmp_path, monkeypatch):
    monkeypatch.setattr(home, "get_package_share_directory", lambda _: str(PACKAGE))
    run = home.ReturnHomeRun(args(tmp_path))
    assert run.run() == 0
    assert run.report["outcome"] == "check_only_ok"
    assert run.nav_process is None
    assert run.bridge is None
