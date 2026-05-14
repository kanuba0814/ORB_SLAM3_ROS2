#!/usr/bin/env python3
"""Self-test the guarded final stability artifact gates with synthetic data."""

from __future__ import annotations

import copy
import json
import re
import subprocess
import tempfile
from pathlib import Path
from typing import Any


WORKSPACE = Path("/home/gxxl/slam_ws_v2")
PACKAGE_ROOT = WORKSPACE / "extra_ws" / "src" / "ORB_SLAM3_ROS2"
GENERATOR = PACKAGE_ROOT / "tools" / "generate_final_stability_artifacts.py"
SETTINGS = PACKAGE_ROOT / "config" / "Monocular-Inertial" / "rmoncam_h30.yaml"
QOS = PACKAGE_ROOT / "config" / "bag_qos_override.yaml"
BAG_PATH = "/home/gxxl/slam_ws_v2/bags/goalDebug/h30_sync_raw_20260512_172136"
REPLAY_TOPICS = ["/sync/image_raw", "/sync/imu"]
CAMERA_FPS_RE = re.compile(
    r"^\s*Camera\.fps\s*:\s*([0-9]+(?:\.[0-9]+)?)\s*(?:#.*)?$"
)


def read_camera_fps(settings_file: Path) -> float:
    for line in settings_file.read_text(encoding="utf-8").splitlines():
        match = CAMERA_FPS_RE.match(line)
        if match:
            return float(match.group(1))
    raise ValueError(f"settings file is missing Camera.fps: {settings_file}")


def base_command() -> list[str]:
    return [
        "ros2",
        "launch",
        "orb_slam3_ros2",
        "h30_bag_replay.launch.py",
        f"bag_path:={BAG_PATH}",
        "play_bag:=true",
        "sensor_type:=imu-monocular",
        "image_topic:=/sync/image_raw",
        "imu_topic:=/sync/imu",
        f"settings_file:={SETTINGS}",
        "cam_imu_time_offset:=0.0",
        "orb_time_offset_mode:=zero_for_sync",
        "debug_enable:=true",
        "debug_runtime_metrics:=true",
        "enable_video_recording:=false",
        "publish_pose_only_when_tracking_ok:=true",
        "max_image_queue_size:=2",
        "max_imu_queue_size:=500",
        "drop_stale_images:=true",
        "max_image_age_ms:=200.0",
        "use_rviz:=false",
        "use_pangolin:=false",
        "rate:=1.0",
        "duration:=0.0",
        "replay_topics:=" + ",".join(REPLAY_TOPICS),
        "play_delay:=2.0",
        f"qos_profile_overrides_path:={QOS}",
    ]


def base_run() -> dict[str, Any]:
    return {
        "stability_pass": True,
        "returncode": 0,
        "errors": [],
        "command": base_command(),
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
    run = base_run()
    return {
        "label": "synthetic_final_gate_selftest",
        "bag_path": BAG_PATH,
        "run_count": 3,
        "pass_count": 3,
        "all_passed": True,
        "preflight": {
            "selected_topics": {"image": "/sync/image_raw", "imu": "/sync/imu"},
            "replay_topics": REPLAY_TOPICS,
            "image_topic_stats": {"count": 10, "fps_span": 8.0},
            "settings_file": str(SETTINGS),
            "qos_profile_overrides_path": str(QOS),
            "settings_camera_fps": read_camera_fps(SETTINGS),
            "required_timeout_sec": 120.0,
            "timeout_sec": 210.0,
            "bag_time_span_sec": 100.0,
            "replay_duration_sec": 100.0,
            "rate": 1.0,
            "play_delay_sec": 2.0,
        },
        "runs": [copy.deepcopy(run) for _ in range(3)],
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

        missing_topics = base_aggregate()
        del missing_topics["preflight"]["replay_topics"]
        expect_failure(
            root,
            "missing_replay_topics",
            missing_topics,
            "preflight is missing replay_topics",
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

        rviz_enabled = base_aggregate()
        replace_command_arg(
            rviz_enabled["runs"][0],
            "use_rviz:=false",
            "use_rviz:=true",
        )
        expect_failure(root, "rviz_enabled", rviz_enabled, "use_rviz:=false")

        runtime_error = base_aggregate()
        runtime_error["runs"][0]["runtime_error_count"] = 1
        expect_failure(root, "runtime_error", runtime_error, "runtime_error_count=1")

        reset_after_init = base_aggregate()
        reset_after_init["runs"][0]["reset_count_after_init"] = 1
        expect_failure(
            root,
            "reset_after_init",
            reset_after_init,
            "reset_count_after_init=1",
        )

    print("final stability gate selftest passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
