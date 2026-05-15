#!/usr/bin/env python3
"""Self-test the guarded final stability artifact gates with synthetic data."""

from __future__ import annotations

import copy
import json
import re
import subprocess
import tempfile
from argparse import Namespace
from pathlib import Path
from typing import Any

import run_stability_replay


WORKSPACE = Path("/home/gxxl/slam_ws_v2")
PACKAGE_ROOT = WORKSPACE / "extra_ws" / "src" / "ORB_SLAM3_ROS2"
GENERATOR = PACKAGE_ROOT / "tools" / "generate_final_stability_artifacts.py"
SETTINGS = PACKAGE_ROOT / "config" / "Monocular-Inertial" / "rmoncam_h30.yaml"
QOS = PACKAGE_ROOT / "config" / "bag_qos_override.yaml"
CAMERA_FPS_RE = re.compile(
    r"^\s*Camera\.fps\s*:\s*([0-9]+(?:\.[0-9]+)?)\s*(?:#.*)?$"
)
_REPLAY_FIXTURE: dict[str, Any] | None = None


def read_camera_fps(settings_file: Path) -> float:
    for line in settings_file.read_text(encoding="utf-8").splitlines():
        match = CAMERA_FPS_RE.match(line)
        if match:
            return float(match.group(1))
    raise ValueError(f"settings file is missing Camera.fps: {settings_file}")


def replay_args() -> Namespace:
    return Namespace(
        sensor_type="imu-monocular",
        image_topic="/sync/image_raw",
        imu_topic="/sync/imu",
        settings_file=str(SETTINGS),
        cam_imu_time_offset="0.0",
        orb_time_offset_mode="zero_for_sync",
        debug_enable=True,
        debug_runtime_metrics=True,
        enable_video_recording=False,
        publish_pose_only_when_tracking_ok=True,
        min_pose_publish_inliers="15",
        min_pose_publish_tracked_mappoints="15",
        max_image_queue_size="2",
        max_imu_queue_size="500",
        drop_stale_images=True,
        max_image_age_ms="200.0",
        use_rviz=False,
        use_pangolin=False,
        rate="1.0",
        duration="0.0",
        start_offset="0.0",
        replay_topics="",
        play_delay="2.0",
        qos_profile_overrides_path=str(QOS),
        timeout_sec=210.0,
    )


def replay_fixture() -> dict[str, Any]:
    global _REPLAY_FIXTURE
    if _REPLAY_FIXTURE is None:
        args = replay_args()
        bag_path = run_stability_replay.read_latest_bag(None, None)
        preflight = run_stability_replay.preflight_replay(args, bag_path)
        command = run_stability_replay.launch_command(args, bag_path)
        _REPLAY_FIXTURE = {
            "bag_path": str(bag_path),
            "command": command,
            "preflight": preflight,
        }
    return copy.deepcopy(_REPLAY_FIXTURE)


def base_run(run_index: int = 1) -> dict[str, Any]:
    return {
        "run_index": run_index,
        "elapsed_wall_sec": 100.0 + run_index,
        "stability_pass": True,
        "returncode": 0,
        "errors": [],
        "command": replay_fixture()["command"],
        "imu_initialized": True,
        "required_v2_tags_present": True,
        "first_imu_initialized_time": 10.0,
        "ok_ratio_after_init": 0.95,
        "max_lost_streak_sec_after_init": 0.0,
        "reset_count_after_init": 0,
        "reset_delta_last_60_sec": 0,
        "final_keyframes": 25,
        "median_inliers_after_init": 120.0,
        "median_tracked_map_points_after_init": 130.0,
        "image_to_last_imu_ms": {"p50": 2.0, "p95": 5.0},
        "track_ms": {"p50": 20.0, "p95": 40.0},
        "processed_image_age_ms": {"p95": 50.0},
        "img_buf_size_max": 2,
        "dropped_img_count": 1,
        "orb_processed_fps_est": {"p50": 8.0},
        "pose_publish_blocked_count": 3,
        "invalid_pose_publish_count": 0,
        "runtime_error_count": 0,
        "controlled_backlog_pass": True,
    }


def base_aggregate() -> dict[str, Any]:
    fixture = replay_fixture()
    preflight = fixture["preflight"]
    preflight["settings_camera_fps"] = read_camera_fps(SETTINGS)
    return {
        "label": "synthetic_final_gate_selftest",
        "bag_path": fixture["bag_path"],
        "run_count": 3,
        "pass_count": 3,
        "all_passed": True,
        "preflight": preflight,
        "runs": [base_run(index) for index in range(1, 4)],
    }


def run_generator(
    root: Path,
    name: str,
    aggregate: dict[str, Any],
) -> subprocess.CompletedProcess[str]:
    aggregate_path = root / f"{name}.json"
    aggregate_path.write_text(
        json.dumps(aggregate, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return subprocess.run(
        [
            "python3",
            str(GENERATOR),
            str(aggregate_path),
            "--output-dir",
            str(root / f"{name}_out"),
            "--kept-commit",
            "synthetic kept commit",
            "--excluded-hypothesis",
            "synthetic excluded hypothesis",
            "--risk",
            "synthetic risk",
        ],
        capture_output=True,
        text=True,
        check=False,
    )


def expect_success(root: Path, name: str, aggregate: dict[str, Any]) -> None:
    result = run_generator(root, name, aggregate)
    if result.returncode != 0:
        raise AssertionError(
            f"{name} should have passed, got {result.returncode}: {result.stderr}"
        )
    output_dir = root / f"{name}_out"
    required_outputs = (
        "best_config.yaml",
        "decision.md",
        "stability_report.md",
        "final_stability_report.md",
        "final_commands.md",
        "final_patch_summary.md",
    )
    missing = [
        filename
        for filename in required_outputs
        if not (output_dir / filename).exists()
    ]
    if missing:
        raise AssertionError(f"{name} did not write required outputs: {missing}")
    best_config = output_dir / "best_config.yaml"
    if best_config.read_text(encoding="utf-8") != SETTINGS.read_text(encoding="utf-8"):
        raise AssertionError(f"{name} best_config.yaml does not match settings")
    report = (output_dir / "final_stability_report.md").read_text(encoding="utf-8")
    for snippet in (
        "- image_topic_count:",
        "- image_topic_fps_span:",
        "- imu_topic_count:",
        "- imu_topic_fps_span:",
    ):
        if snippet not in report:
            raise AssertionError(f"{name} final report is missing {snippet}")


def expect_failure(
    root: Path,
    name: str,
    aggregate: dict[str, Any],
    expected_text: str,
) -> None:
    result = run_generator(root, name, aggregate)
    if result.returncode == 0:
        raise AssertionError(f"{name} should have failed")
    output = result.stdout + result.stderr
    if expected_text not in output:
        raise AssertionError(
            f"{name} failed for the wrong reason. Expected {expected_text!r}, got: {output}"
        )


def replace_command_arg(run: dict[str, Any], old: str, new: str) -> None:
    run["command"] = [new if item == old else item for item in run["command"]]


def remove_command_arg(run: dict[str, Any], value: str) -> None:
    run["command"] = [item for item in run["command"] if item != value]


def main() -> int:
    with tempfile.TemporaryDirectory(prefix="orb_final_gate_selftest_") as tmp:
        root = Path(tmp)
        positive = base_aggregate()
        expect_success(root, "positive", positive)

        aggregate_not_passed = base_aggregate()
        aggregate_not_passed["pass_count"] = 2
        aggregate_not_passed["all_passed"] = False
        expect_failure(
            root,
            "aggregate_not_passed",
            aggregate_not_passed,
            "aggregate is not fully passing",
        )

        run_count_mismatch = base_aggregate()
        run_count_mismatch["run_count"] = 4
        expect_failure(
            root,
            "run_count_mismatch",
            run_count_mismatch,
            "aggregate run_count does not match runs length",
        )

        nonzero_returncode = base_aggregate()
        nonzero_returncode["runs"][0]["returncode"] = 1
        expect_failure(
            root,
            "nonzero_returncode",
            nonzero_returncode,
            "run 1 has returncode=1",
        )

        mixed_command = base_aggregate()
        replace_command_arg(mixed_command["runs"][1], "rate:=1.0", "rate:=0.5")
        expect_failure(
            root,
            "mixed_command",
            mixed_command,
            "run 2 command differs from run 1 command",
        )

        run_index_mismatch = base_aggregate()
        run_index_mismatch["runs"][1]["run_index"] = 99
        expect_failure(
            root,
            "run_index_mismatch",
            run_index_mismatch,
            "run 2 has run_index=99, expected 2",
        )

        missing_elapsed = base_aggregate()
        del missing_elapsed["runs"][0]["elapsed_wall_sec"]
        expect_failure(
            root,
            "missing_elapsed",
            missing_elapsed,
            "run 1 has invalid elapsed_wall_sec=None",
        )

        missing_v2_tags = base_aggregate()
        missing_v2_tags["runs"][0]["required_v2_tags_present"] = False
        expect_failure(
            root,
            "missing_v2_tags",
            missing_v2_tags,
            "required_v2_tags_present=False",
        )

        missing_topics = base_aggregate()
        del missing_topics["preflight"]["replay_topics"]
        expect_failure(
            root,
            "missing_replay_topics",
            missing_topics,
            "preflight is missing replay_topics",
        )

        missing_imu_stats = base_aggregate()
        del missing_imu_stats["preflight"]["imu_topic_stats"]
        expect_failure(
            root,
            "missing_imu_topic_stats",
            missing_imu_stats,
            "preflight is missing imu_topic_stats",
        )

        empty_imu_stats = base_aggregate()
        empty_imu_stats["preflight"]["imu_topic_stats"]["count"] = 0
        expect_failure(
            root,
            "empty_imu_topic_stats",
            empty_imu_stats,
            "preflight IMU topic has no recorded messages",
        )

        wrong_bag = base_aggregate()
        wrong_bag["bag_path"] = "/tmp/not_the_latest_h30_bag"
        expect_failure(
            root,
            "wrong_bag_path",
            wrong_bag,
            "aggregate bag_path does not match current LATEST_BAG",
        )

        forbidden_topic = base_aggregate()
        forbidden_topic["preflight"]["replay_topics"] = [
            "/sync/image_raw",
            "/sync/imu",
            "/image_raw",
        ]
        expect_failure(
            root,
            "forbidden_replay_topic",
            forbidden_topic,
            "preflight replay topics include forbidden topics",
        )

        missing_pose_gate = base_aggregate()
        remove_command_arg(
            missing_pose_gate["runs"][0],
            "publish_pose_only_when_tracking_ok:=true",
        )
        expect_failure(
            root,
            "missing_pose_gate_arg",
            missing_pose_gate,
            "publish_pose_only_when_tracking_ok:=true",
        )

        wrong_pose_inlier_gate = base_aggregate()
        replace_command_arg(
            wrong_pose_inlier_gate["runs"][0],
            "min_pose_publish_inliers:=15",
            "min_pose_publish_inliers:=0",
        )
        expect_failure(
            root,
            "wrong_pose_inlier_gate",
            wrong_pose_inlier_gate,
            "min_pose_publish_inliers:=15",
        )

        missing_pose_mappoint_gate = base_aggregate()
        remove_command_arg(
            missing_pose_mappoint_gate["runs"][0],
            "min_pose_publish_tracked_mappoints:=15",
        )
        expect_failure(
            root,
            "missing_pose_mappoint_gate",
            missing_pose_mappoint_gate,
            "min_pose_publish_tracked_mappoints:=15",
        )

        rviz_enabled = base_aggregate()
        replace_command_arg(
            rviz_enabled["runs"][0],
            "use_rviz:=false",
            "use_rviz:=true",
        )
        expect_failure(root, "rviz_enabled", rviz_enabled, "use_rviz:=false")

        nonzero_start_offset = base_aggregate()
        nonzero_start_offset["preflight"]["start_offset_sec"] = 1.0
        expect_failure(
            root,
            "nonzero_start_offset",
            nonzero_start_offset,
            "preflight replay did not start at the beginning of the bag",
        )

        missing_start_offset_arg = base_aggregate()
        remove_command_arg(missing_start_offset_arg["runs"][0], "start_offset:=0.0")
        expect_failure(
            root,
            "missing_start_offset_arg",
            missing_start_offset_arg,
            "start_offset:=0.0",
        )

        runtime_error = base_aggregate()
        runtime_error["runs"][0]["runtime_error_count"] = 1
        expect_failure(root, "runtime_error", runtime_error, "runtime_error_count=1")

        invalid_pose = base_aggregate()
        invalid_pose["runs"][0]["invalid_pose_publish_count"] = 1
        expect_failure(
            root,
            "invalid_pose_publish",
            invalid_pose,
            "invalid_pose_publish_count=1",
        )

        reset_after_init = base_aggregate()
        reset_after_init["runs"][0]["reset_count_after_init"] = 1
        expect_failure(
            root,
            "reset_after_init",
            reset_after_init,
            "reset_count_after_init=1",
        )

        imu_window_p50_jump = base_aggregate()
        imu_window_p50_jump["runs"][0]["image_to_last_imu_ms"]["p50"] = 30.0
        expect_failure(
            root,
            "imu_window_p50_jump",
            imu_window_p50_jump,
            "image_to_last_imu_ms.p50=30.0",
        )

    print("final stability gate selftest passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
