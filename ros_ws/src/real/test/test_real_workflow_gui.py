from pathlib import Path
import sys


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

import real_workflow_gui as gui  # noqa: E402


def test_chinese_and_english_catalogs_have_identical_keys():
    assert set(gui.TEXTS["zh"]) == set(gui.TEXTS["en"])
    assert gui.TEXTS["zh"]["language"] == "English"
    assert gui.TEXTS["en"]["language"] == "中文"
    assert "Home" in gui.TEXTS["en"]["return_home"]


def test_patrol_is_a_standalone_inspection_only_process():
    command = gui.patrol_command()
    assert command[:4] == ["ros2", "run", "real", "run_full_workflow.py"]
    assert "--inspection-only" in command
    assert "--enable-motion" in command
    assert "--use-rviz" in command


def test_docking_is_a_separate_unlimited_stage():
    command = gui.docking_command()
    assert command[:4] == ["ros2", "run", "real", "run_hybrid_docking.py"]
    assert "--start-near-vp3" in command
    assert command[command.index("--nav-timeout") + 1] == "0"
    assert command[command.index("--servo-timeout") + 1] == "0"


def test_localization_button_never_enables_motion():
    command = gui.localization_command()
    assert "--localize-only" in command
    assert "--enable-motion" not in command


def test_return_home_is_independent_motion_stage_with_rviz():
    command = gui.return_home_command()
    assert command[:4] == ["ros2", "run", "real", "run_return_home.py"]
    assert "--enable-motion" in command
    assert "--use-rviz" in command
    assert command[command.index("--nav-timeout") + 1] == "0"


def test_emergency_stop_publishes_only_zero_twist():
    assert gui.ZERO_COMMAND[-1] == (
        "{linear: {x: 0.0}, angular: {z: 0.0}}")


def test_auto_clear_runs_validated_loop_on_jetson():
    command = gui.auto_clear_command("10.28.166.198")
    assert command[0] == "ssh"
    assert "nvidia@10.28.166.198" in command
    remote = command[-1]
    assert "SOARM_PORT=/dev/ttyACM0" in remote
    assert "exec .venv/bin/python -u auto_clear.py" in remote
    assert "scripts/camera_server.py --index 0" in remote
    assert "09_grasp.py" not in remote
    assert "--step" not in remote


def test_auto_clear_stop_requests_clean_exit_then_has_torque_off_fallback():
    command = gui.auto_clear_stop_command("10.28.166.198")
    remote = command[-1]
    assert gui.AUTO_CLEAR_PROCESS_PATTERN in remote
    assert "pkill -INT" in remote
    assert "scripts/13_recover.py --free" in remote


def test_auto_clear_duplicate_guard_does_not_match_parent_shell_text():
    pattern = gui.AUTO_CLEAR_PROCESS_PATTERN
    import re
    assert re.search(pattern, ".venv/bin/python -u auto_clear.py")
    assert re.search(pattern, "python auto_clear.py --step")
    parent = (
        "bash -c pgrep -f 'auto_clear.py' && "
        "exec .venv/bin/python -u auto_clear.py")
    assert re.search(pattern, parent) is None


def test_rgb_load_gate_uses_confirmed_domain_and_launcher_without_arm_start():
    command = gui.load_gate_command("10.28.166.198")
    assert command[0] == "ssh"
    assert "-Y" in command
    remote = command[-1]
    assert "ROS_DOMAIN_ID=30" in remote
    assert "DISPLAY=:1" not in remote
    assert "/home/nvidia/run_turtlebot3_load_arm_gate.sh" in remote
    assert "--arm-shell-command ''" in remote
    assert "--arm-command-topic /gui/load_arm_gate/disabled" in remote
    assert "--hold-sec 9999" in remote


def test_rgb_load_gate_arrival_signal_matches_confirmed_ready_text():
    remote = gui.load_gate_arrival_command("10.28.166.198")[-1]
    assert "ROS_DOMAIN_ID=30" in remote
    assert "/turtlebot3/load_unload_arrived" in remote
    assert "Ready, waiting for recognition results" in remote


def test_rgb_load_gate_stop_pattern_does_not_match_parent_shell():
    import re
    pattern = gui.LOAD_GATE_PROCESS_PATTERN
    worker = "python3 /home/nvidia/turtlebot3_load_arm_gate.py --trigger-mode arrival"
    parent = "bash -c " + worker
    assert re.search(pattern, worker)
    assert re.search(pattern, parent) is None
