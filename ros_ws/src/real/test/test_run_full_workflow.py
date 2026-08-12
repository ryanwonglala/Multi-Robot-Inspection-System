from pathlib import Path
import sys

import pytest
import yaml


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

import run_full_workflow as workflow  # noqa: E402


def complete_inspection_report():
    samples = []
    for label in ("viewpoint_1", "viewpoint_2", "viewpoint_3"):
        samples.extend({"stop_label": label, "yaw_index": index}
                       for index in range(6))
    return {
        "status": "completed",
        "route": ["arena"],
        "run_dir": "/tmp/example",
        "summary": {"checked_count": 1},
        "anomalies": [{"area": "arena", "region": "vp2"}],
        "areas": [{
            "target_area": "arena",
            "status": "checked",
            "selected_stops": [
                {"label": "viewpoint_1"}, {"label": "viewpoint_2"},
                {"label": "viewpoint_3"}],
            "scan_samples": samples,
        }],
    }


def test_complete_three_viewpoint_inspection_opens_handoff_gate():
    ready, reason = workflow.inspection_ready_for_handoff(
        complete_inspection_report())
    assert ready is True
    assert reason == "inspection_complete_at_vp3"


@pytest.mark.parametrize("mutation,reason", [
    (lambda report: report.update(status="completed_with_failures"),
     "inspection_status_completed_with_failures"),
    (lambda report: report["areas"][0]["selected_stops"].pop(),
     "vp1_vp2_vp3_not_all_visited"),
    (lambda report: report["areas"][0]["scan_samples"].pop(),
     "incomplete_six_direction_scan"),
])
def test_incomplete_inspection_never_opens_handoff_gate(mutation, reason):
    report = complete_inspection_report()
    mutation(report)
    assert workflow.inspection_ready_for_handoff(report) == (False, reason)


def test_inspection_command_stays_at_vp3_instead_of_returning_home():
    command = workflow.build_inspection_command(
        Path("inspection.yaml"), Path("world.yaml"))
    assert "route:=arena" in command
    assert "return_home:=false" in command


def test_docking_command_receives_live_post_loading_pose():
    pose = workflow.Pose2D(1.224, -1.594, -1.041)
    command = workflow.build_docking_command(
        Path("/tmp/docking"), pose, 0.25)
    assert "--start-near-vp3" in command
    assert command[command.index("--initial-x") + 1] == "1.224000000"
    assert command[command.index("--initial-y") + 1] == "-1.594000000"
    assert command[command.index("--initial-yaw") + 1] == "-1.041000000"
    assert command[command.index("--base-feedback-timeout") + 1] == "30.0"
    assert command[command.index("--nav-startup-timeout") + 1] == "0.0"
    assert command[command.index("--nav-timeout") + 1] == "0.0"
    assert command[command.index("--servo-timeout") + 1] == "0.0"
    assert "--resume-from-a" not in command


def test_real_inspection_restores_vp3_first_scan_heading():
    config = yaml.safe_load((Path(__file__).resolve().parents[1] /
                             "config/inspection_real_v5.yaml").read_text())
    params = config["inspection_runner"]["ros__parameters"]
    assert params["restore_final_viewpoint_scan_yaw"] is True
    assert params["restore_final_viewpoint_scan_yaw_attempts"] == 2


@pytest.mark.parametrize("reason", [
    "base_feedback_timeout",
    "base_feedback_timeout:missing=battery,odom,scan,camera",
])
def test_base_discovery_timeout_is_safe_to_retry(reason):
    assert workflow.retryable_docking_reason(reason) is True


@pytest.mark.parametrize("reason", [
    None,
    "nav2_coarse_A_and_B_failed",
    "apriltag_terminal_failed:preflight_timeout",
    "backup_failed",
])
def test_motion_stage_failures_are_not_automatically_retried(reason):
    assert workflow.retryable_docking_reason(reason) is False


def test_newest_report_ignores_preexisting_rounds(tmp_path):
    old = tmp_path / "inspection_old" / "details.yaml"
    old.parent.mkdir()
    old.write_text(yaml.safe_dump({"status": "completed"}))
    before = {old}
    new = tmp_path / "inspection_new" / "details.yaml"
    new.parent.mkdir()
    new.write_text(yaml.safe_dump({"status": "completed"}))
    assert workflow.newest_new_report(tmp_path, before) == new


def test_motion_run_requires_explicit_home_assertion():
    with pytest.raises(SystemExit):
        workflow.parse_args(["--enable-motion"])
    args = workflow.parse_args(["--enable-motion", "--start-at-home"])
    assert args.enable_motion is True
    assert args.start_at_home is True
    assert args.service_only is False


def test_service_only_mode_is_available_for_noninteractive_runs():
    args = workflow.parse_args([
        "--enable-motion", "--start-at-home", "--service-only"])
    assert args.service_only is True


def test_inspection_only_mode_supports_two_command_workflow():
    args = workflow.parse_args([
        "--enable-motion", "--start-at-home", "--inspection-only"])
    assert args.inspection_only is True
    assert args.start_at_home is True


def test_check_only_never_enables_motion():
    args = workflow.parse_args(["--check-only"])
    assert args.check_only is True
    assert args.enable_motion is False
