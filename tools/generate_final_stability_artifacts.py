#!/usr/bin/env python3
"""Generate guarded final ORB stability artifacts from a passing replay aggregate."""

from __future__ import annotations

import argparse
import json
import re
import shutil
import sys
from pathlib import Path
from typing import Any


WORKSPACE = Path("/home/gxxl/slam_ws_v2")
EXTRA_WS = WORKSPACE / "extra_ws"
PACKAGE_ROOT = EXTRA_WS / "src" / "ORB_SLAM3_ROS2"
DEFAULT_OUTPUT_DIR = EXTRA_WS / "debug_runs" / "fix_until_stable"
DEFAULT_SETTINGS = PACKAGE_ROOT / "config" / "Monocular-Inertial" / "rmoncam_h30.yaml"
DEFAULT_LATEST_FILES = (
    WORKSPACE / "bags" / "LATEST_BAG.txt",
    WORKSPACE / "bags" / "goalDebug" / "LATEST_BAG.txt",
)
ALLOWED_IMAGE_TOPIC = "/sync/image_raw"
ALLOWED_IMU_TOPICS = {"/sync/imu", "/sync/imu_batch"}
ALLOWED_REPLAY_TOPICS = {
    ALLOWED_IMAGE_TOPIC,
    *ALLOWED_IMU_TOPICS,
    "/imu/pipeline_ready",
    "/rosout",
}
BANNED_SELECTED_TOPICS = {"/image_raw", "/imu/data_raw", "/imu/raw_hw"}
CAMERA_FPS_RE = re.compile(
    r"^\s*Camera\.fps\s*:\s*([0-9]+(?:\.[0-9]+)?)\s*(?:#.*)?$"
)


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"{path} is not a JSON object")
    return data


def read_latest_bag() -> Path:
    for latest_file in DEFAULT_LATEST_FILES:
        if latest_file.exists():
            content = latest_file.read_text(encoding="utf-8").strip()
            if content:
                return Path(content).expanduser().resolve()
    raise FileNotFoundError("no LATEST_BAG.txt found for final replay validation")


def require_aggregate_bag_matches_latest(aggregate: dict[str, Any]) -> Path:
    latest_bag = read_latest_bag()
    aggregate_bag = Path(str(aggregate.get("bag_path"))).expanduser().resolve()
    if aggregate_bag != latest_bag:
        raise ValueError(
            "aggregate bag_path does not match current LATEST_BAG: "
            f"aggregate={aggregate_bag}, latest={latest_bag}"
        )
    return latest_bag


def metric(run: dict[str, Any], dotted_key: str) -> Any:
    value: Any = run
    for key in dotted_key.split("."):
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    return value


def require_float(preflight: dict[str, Any], key: str) -> float:
    value = preflight.get(key)
    if value is None:
        raise ValueError(f"preflight is missing {key}")
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"preflight {key} is not numeric: {value}") from exc


def read_camera_fps(settings_file: Path) -> float | None:
    for line in settings_file.read_text(encoding="utf-8").splitlines():
        match = CAMERA_FPS_RE.match(line)
        if match:
            return float(match.group(1))
    return None


def require_settings_match_preflight(settings_file: Path, preflight: dict[str, Any]) -> None:
    preflight_settings = Path(str(preflight.get("settings_file"))).expanduser().resolve()
    selected_settings = settings_file.expanduser().resolve()
    if selected_settings != preflight_settings:
        raise ValueError(
            "settings file does not match replay preflight: "
            f"selected={selected_settings}, preflight={preflight_settings}"
        )

    preflight_fps = require_float(preflight, "settings_camera_fps")
    current_fps = read_camera_fps(settings_file)
    if current_fps is None:
        raise ValueError(f"settings file is missing Camera.fps: {settings_file}")
    if abs(current_fps - preflight_fps) > 1e-6:
        raise ValueError(
            "settings Camera.fps differs from replay preflight: "
            f"current={current_fps}, preflight={preflight_fps}"
        )


def require_final_replay_preflight(aggregate: dict[str, Any]) -> dict[str, Any]:
    preflight = aggregate.get("preflight")
    if not isinstance(preflight, dict) or not preflight:
        raise ValueError("aggregate is missing replay preflight metadata")

    selected_topics = preflight.get("selected_topics")
    if not isinstance(selected_topics, dict):
        raise ValueError("preflight is missing selected_topics")
    image_topic = selected_topics.get("image")
    imu_topic = selected_topics.get("imu")
    selected = {str(topic) for topic in (image_topic, imu_topic) if topic}
    banned = sorted(selected & BANNED_SELECTED_TOPICS)
    if banned:
        raise ValueError(f"preflight selected forbidden replay topics: {banned}")
    if image_topic != ALLOWED_IMAGE_TOPIC:
        raise ValueError(
            f"preflight image topic must be {ALLOWED_IMAGE_TOPIC}, got {image_topic}"
        )
    if imu_topic not in ALLOWED_IMU_TOPICS:
        raise ValueError(
            f"preflight IMU topic must be one of {sorted(ALLOWED_IMU_TOPICS)}"
        )
    replay_topics = preflight.get("replay_topics")
    if not isinstance(replay_topics, list) or not replay_topics:
        raise ValueError("preflight is missing replay_topics")
    replay_topic_set = {str(topic) for topic in replay_topics}
    banned_replay = sorted(replay_topic_set & BANNED_SELECTED_TOPICS)
    if banned_replay:
        raise ValueError(
            f"preflight replay topics include forbidden topics: {banned_replay}"
        )
    unsupported_replay = sorted(
        topic for topic in replay_topic_set
        if topic not in ALLOWED_REPLAY_TOPICS
    )
    if unsupported_replay:
        raise ValueError(
            f"preflight replay topics are not sync-only allowed topics: {unsupported_replay}"
        )
    missing_selected_replay = sorted(
        topic for topic in selected if topic not in replay_topic_set
    )
    if missing_selected_replay:
        raise ValueError(
            "preflight replay topics do not include selected ORB topics: "
            f"{missing_selected_replay}"
        )

    image_topic_stats = preflight.get("image_topic_stats")
    if not isinstance(image_topic_stats, dict):
        raise ValueError("preflight is missing image_topic_stats")
    if int(image_topic_stats.get("count") or 0) <= 0:
        raise ValueError("preflight image topic has no recorded messages")
    if image_topic_stats.get("fps_span") is None:
        raise ValueError("preflight image topic is missing fps_span")
    imu_topic_stats = preflight.get("imu_topic_stats")
    if not isinstance(imu_topic_stats, dict):
        raise ValueError("preflight is missing imu_topic_stats")
    if int(imu_topic_stats.get("count") or 0) <= 0:
        raise ValueError("preflight IMU topic has no recorded messages")
    if imu_topic_stats.get("fps_span") is None:
        raise ValueError("preflight IMU topic is missing fps_span")

    required_paths = ("settings_file", "qos_profile_overrides_path")
    for key in required_paths:
        if not isinstance(preflight.get(key), str) or not preflight.get(key):
            raise ValueError(f"preflight is missing {key}")
    if preflight.get("settings_camera_fps") is None:
        raise ValueError("preflight is missing settings_camera_fps")

    required_timeout_sec = require_float(preflight, "required_timeout_sec")
    timeout_sec = require_float(preflight, "timeout_sec")
    for key in ("bag_time_span_sec", "replay_duration_sec", "rate", "play_delay_sec"):
        require_float(preflight, key)
    start_offset_sec = require_float(preflight, "start_offset_sec")
    if start_offset_sec != 0.0:
        raise ValueError(
            "preflight replay did not start at the beginning of the bag: "
            f"start_offset_sec={start_offset_sec}"
        )
    bag_time_span_sec = require_float(preflight, "bag_time_span_sec")
    replay_duration_sec = require_float(preflight, "replay_duration_sec")
    if abs(replay_duration_sec - bag_time_span_sec) > 1e-6:
        raise ValueError(
            "preflight replay duration is not the full bag duration: "
            f"replay_duration_sec={replay_duration_sec}, "
            f"bag_time_span_sec={bag_time_span_sec}"
        )
    if timeout_sec <= 0.0:
        raise ValueError("preflight timeout_sec must be positive for final replay evidence")
    if timeout_sec < required_timeout_sec:
        raise ValueError(
            "preflight timeout_sec is too low for final replay evidence: "
            f"timeout_sec={timeout_sec}, required_timeout_sec={required_timeout_sec}"
        )

    return preflight


def require_command_argument(
    command_parts: set[str],
    key: str,
    expected_value: Any,
    missing: list[str],
) -> None:
    expected = f"{key}:={expected_value}"
    if expected not in command_parts:
        missing.append(expected)


def command_value(command_parts: set[str], key: str) -> str | None:
    prefix = f"{key}:="
    for part in command_parts:
        if part.startswith(prefix):
            return part[len(prefix):]
    return None


def require_command_float_argument(
    command_parts: set[str],
    key: str,
    expected_value: Any,
    missing: list[str],
) -> None:
    actual_value = command_value(command_parts, key)
    try:
        expected_float = float(expected_value)
        actual_float = float(actual_value) if actual_value is not None else None
    except (TypeError, ValueError):
        actual_float = None
        expected_float = None
    if actual_float is None or expected_float is None:
        missing.append(f"{key}:={expected_value}")
        return
    if abs(actual_float - expected_float) > 1e-9:
        missing.append(f"{key}:={expected_value}")


def require_ros2_wrapper_command(
    run: dict[str, Any],
    index: int,
    aggregate: dict[str, Any],
    preflight: dict[str, Any],
) -> None:
    command = run.get("command")
    if not isinstance(command, list):
        raise ValueError(f"run {index} is missing replay command list")
    command_parts = {str(part) for part in command}
    required_parts = (
        "ros2",
        "launch",
        "orb_slam3_ros2",
        "h30_bag_replay.launch.py",
    )
    missing = [part for part in required_parts if part not in command_parts]
    required_arguments = {
        "play_bag": "true",
        "sensor_type": "imu-monocular",
        "orb_time_offset_mode": "zero_for_sync",
        "debug_enable": "true",
        "debug_runtime_metrics": "true",
        "enable_video_recording": "false",
        "publish_pose_only_when_tracking_ok": "true",
        "min_pose_publish_inliers": "15",
        "min_pose_publish_tracked_mappoints": "15",
        "max_image_queue_size": "2",
        "max_imu_queue_size": "500",
        "drop_stale_images": "true",
        "use_rviz": "false",
        "use_pangolin": "false",
    }
    for key, expected_value in required_arguments.items():
        require_command_argument(command_parts, key, expected_value, missing)
    selected_topics = preflight.get("selected_topics", {})
    require_command_argument(
        command_parts,
        "bag_path",
        aggregate.get("bag_path"),
        missing,
    )
    require_command_argument(
        command_parts,
        "image_topic",
        selected_topics.get("image"),
        missing,
    )
    require_command_argument(
        command_parts,
        "imu_topic",
        selected_topics.get("imu"),
        missing,
    )
    require_command_argument(
        command_parts,
        "settings_file",
        preflight.get("settings_file"),
        missing,
    )
    require_command_argument(
        command_parts,
        "qos_profile_overrides_path",
        preflight.get("qos_profile_overrides_path"),
        missing,
    )
    require_command_float_argument(command_parts, "rate", preflight.get("rate"), missing)
    require_command_float_argument(command_parts, "duration", "0.0", missing)
    require_command_float_argument(command_parts, "start_offset", "0.0", missing)
    require_command_float_argument(command_parts, "cam_imu_time_offset", "0.0", missing)
    require_command_float_argument(command_parts, "max_image_age_ms", "200.0", missing)
    replay_topics = ",".join(str(topic) for topic in preflight.get("replay_topics", []))
    require_command_argument(command_parts, "replay_topics", replay_topics, missing)
    require_command_float_argument(
        command_parts,
        "play_delay",
        preflight.get("play_delay_sec"),
        missing,
    )
    if missing:
        raise ValueError(
            f"run {index} command is not the guarded ROS2 wrapper replay path; "
            f"missing: {', '.join(missing)}"
        )
    if any(
        f"image_topic:={topic}" in command_parts or f"imu_topic:={topic}" in command_parts
        for topic in BANNED_SELECTED_TOPICS
    ):
        raise ValueError(f"run {index} command references forbidden replay topics")


def require_metric_gate(
    failures: list[str],
    name: str,
    value: Any,
    predicate: Any,
) -> None:
    try:
        passed = predicate(value)
    except (TypeError, ValueError):
        passed = False
    if not passed:
        failures.append(f"{name}={value}")


def require_run_stability_gates(run: dict[str, Any], index: int) -> None:
    failures: list[str] = []
    require_metric_gate(
        failures,
        "imu_initialized",
        run.get("imu_initialized"),
        lambda value: value is True,
    )
    require_metric_gate(
        failures,
        "required_v2_tags_present",
        run.get("required_v2_tags_present"),
        lambda value: value is True,
    )
    require_metric_gate(
        failures,
        "reset_count_after_init",
        run.get("reset_count_after_init"),
        lambda value: int(value) == 0,
    )
    require_metric_gate(
        failures,
        "ok_ratio_after_init",
        run.get("ok_ratio_after_init"),
        lambda value: float(value) >= 0.90,
    )
    require_metric_gate(
        failures,
        "max_lost_streak_sec_after_init",
        run.get("max_lost_streak_sec_after_init"),
        lambda value: float(value) < 5.0,
    )
    require_metric_gate(
        failures,
        "reset_delta_last_60_sec",
        run.get("reset_delta_last_60_sec"),
        lambda value: int(value) == 0,
    )
    require_metric_gate(
        failures,
        "final_keyframes",
        run.get("final_keyframes"),
        lambda value: int(value) >= 20,
    )
    require_metric_gate(
        failures,
        "median_tracked_map_points_after_init",
        run.get("median_tracked_map_points_after_init"),
        lambda value: float(value) >= 80.0,
    )
    require_metric_gate(
        failures,
        "median_inliers_after_init",
        run.get("median_inliers_after_init"),
        lambda value: float(value) >= 50.0,
    )
    require_metric_gate(
        failures,
        "image_to_last_imu_ms.p50",
        metric(run, "image_to_last_imu_ms.p50"),
        lambda value: abs(float(value)) <= 20.0,
    )
    require_metric_gate(
        failures,
        "image_to_last_imu_ms.p95",
        metric(run, "image_to_last_imu_ms.p95"),
        lambda value: abs(float(value)) <= 20.0,
    )
    require_metric_gate(
        failures,
        "processed_image_age_ms.p95",
        metric(run, "processed_image_age_ms.p95"),
        lambda value: float(value) < 150.0,
    )
    require_metric_gate(
        failures,
        "img_buf_size_max",
        run.get("img_buf_size_max"),
        lambda value: float(value) <= 2.0,
    )
    require_metric_gate(
        failures,
        "controlled_backlog_pass",
        run.get("controlled_backlog_pass"),
        lambda value: value is True,
    )
    require_metric_gate(
        failures,
        "invalid_pose_publish_count",
        run.get("invalid_pose_publish_count"),
        lambda value: int(value) == 0,
    )
    require_metric_gate(
        failures,
        "runtime_error_count",
        run.get("runtime_error_count"),
        lambda value: int(value) == 0,
    )
    if failures:
        raise ValueError(
            f"run {index} does not satisfy final stability gates: "
            + ", ".join(failures)
        )


def require_passing_aggregate(aggregate: dict[str, Any], min_runs: int) -> list[dict[str, Any]]:
    runs = aggregate.get("runs")
    if not isinstance(runs, list):
        raise ValueError("aggregate JSON is missing a runs list")
    run_count = int(aggregate.get("run_count", len(runs)) or 0)
    pass_count = int(aggregate.get("pass_count", 0) or 0)
    all_passed = bool(aggregate.get("all_passed"))
    if run_count < min_runs or len(runs) < min_runs:
        raise ValueError(f"need at least {min_runs} replay runs, found {len(runs)}")
    if run_count != len(runs):
        raise ValueError(
            f"aggregate run_count does not match runs length: "
            f"run_count={run_count}, runs={len(runs)}"
        )
    if pass_count != len(runs) or not all_passed:
        raise ValueError(
            f"aggregate is not fully passing: pass_count={pass_count}, "
            f"runs={len(runs)}, all_passed={all_passed}"
        )
    preflight = aggregate.get("preflight")
    if not isinstance(preflight, dict):
        raise ValueError("aggregate is missing replay preflight metadata")
    first_command = runs[0].get("command") if runs else None
    for index, run in enumerate(runs, start=1):
        if not isinstance(run, dict):
            raise ValueError(f"run {index} is not a JSON object")
        if run.get("command") != first_command:
            raise ValueError(f"run {index} command differs from run 1 command")
        if int(run.get("run_index", 0) or 0) != index:
            raise ValueError(
                f"run {index} has run_index={run.get('run_index')}, expected {index}"
            )
        elapsed_wall_sec = run.get("elapsed_wall_sec")
        try:
            elapsed_ok = float(elapsed_wall_sec) > 0.0
        except (TypeError, ValueError):
            elapsed_ok = False
        if not elapsed_ok:
            raise ValueError(
                f"run {index} has invalid elapsed_wall_sec={elapsed_wall_sec}"
            )
        if not run.get("stability_pass"):
            raise ValueError(f"run {index} has stability_pass={run.get('stability_pass')}")
        if int(run.get("returncode", 1) or 0) != 0:
            raise ValueError(f"run {index} has returncode={run.get('returncode')}")
        errors = run.get("errors")
        if isinstance(errors, list) and errors:
            raise ValueError(f"run {index} has runtime errors: {errors[:3]}")
        require_ros2_wrapper_command(run, index, aggregate, preflight)
        required_metrics = (
            "run_index",
            "elapsed_wall_sec",
            "imu_initialized",
            "required_v2_tags_present",
            "first_imu_initialized_time",
            "ok_ratio_after_init",
            "max_lost_streak_sec_after_init",
            "reset_count_after_init",
            "reset_delta_last_60_sec",
            "final_keyframes",
            "median_inliers_after_init",
            "median_tracked_map_points_after_init",
            "image_to_last_imu_ms.p50",
            "image_to_last_imu_ms.p95",
            "track_ms.p50",
            "track_ms.p95",
            "processed_image_age_ms.p95",
            "img_buf_size_max",
            "dropped_img_count",
            "orb_processed_fps_est.p50",
            "pose_publish_blocked_count",
            "invalid_pose_publish_count",
            "runtime_error_count",
        )
        missing = [name for name in required_metrics if metric(run, name) is None]
        if missing:
            raise ValueError(
                f"run {index} is missing final report metrics: {', '.join(missing)}"
            )
        require_run_stability_gates(run, index)
    return runs


def render_metrics_table(runs: list[dict[str, Any]]) -> str:
    lines = [
        "| Run | IMU init time | OK ratio | Reset after init | Reset last 60s | "
        "Max lost streak s | Final KFs | Median inliers | Median MPs | "
        "IMU lag p50/p95 ms | Track p50/p95 ms | Image age p95 ms | "
        "Img buf max | Dropped images | ORB FPS p50 | Pose blocked | "
        "Invalid pose publishes | Runtime errors |",
        "| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | "
        "---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for index, run in enumerate(runs, start=1):
        lines.append(
            f"| {index} | {run.get('first_imu_initialized_time')} | "
            f"{run.get('ok_ratio_after_init')} | "
            f"{run.get('reset_count_after_init')} | "
            f"{run.get('reset_delta_last_60_sec')} | "
            f"{run.get('max_lost_streak_sec_after_init')} | "
            f"{run.get('final_keyframes')} | "
            f"{run.get('median_inliers_after_init')} | "
            f"{run.get('median_tracked_map_points_after_init')} | "
            f"{metric(run, 'image_to_last_imu_ms.p50')}/"
            f"{metric(run, 'image_to_last_imu_ms.p95')} | "
            f"{metric(run, 'track_ms.p50')}/{metric(run, 'track_ms.p95')} | "
            f"{metric(run, 'processed_image_age_ms.p95')} | "
            f"{run.get('img_buf_size_max')} | "
            f"{run.get('dropped_img_count')} | "
            f"{metric(run, 'orb_processed_fps_est.p50')} | "
            f"{run.get('pose_publish_blocked_count')} | "
            f"{run.get('invalid_pose_publish_count')} | "
            f"{run.get('runtime_error_count')} |"
        )
    return "\n".join(lines)


def render_commands(runs: list[dict[str, Any]]) -> str:
    lines = ["# Final Commands", ""]
    for index, run in enumerate(runs, start=1):
        command = run.get("command")
        if isinstance(command, list):
            command_text = " ".join(str(part) for part in command)
        else:
            command_text = str(command or "")
        lines.extend([
            f"## Replay {index}",
            "",
            "```bash",
            command_text,
            "```",
            "",
        ])
    return "\n".join(lines).rstrip() + "\n"


def render_report(
    aggregate: dict[str, Any],
    runs: list[dict[str, Any]],
    settings_path: Path,
    kept_commits: list[str],
    excluded_hypotheses: list[str],
    risks: list[str],
) -> str:
    bag_path = aggregate.get("bag_path")
    label = aggregate.get("label")
    preflight = aggregate.get("preflight", {})
    table = render_metrics_table(runs)
    first_command = runs[0].get("command") if runs else None
    if isinstance(first_command, list):
        launch_command = " ".join(str(part) for part in first_command)
    else:
        launch_command = str(first_command or "")

    commit_lines = "\n".join(f"- {item}" for item in kept_commits)
    excluded_lines = "\n".join(f"- {item}" for item in excluded_hypotheses)
    risk_lines = "\n".join(f"- {item}" for item in risks)

    return "\n".join([
        "# Final ORB Stability Report",
        "",
        "## Result",
        "",
        f"- label: `{label}`",
        f"- bag: `{bag_path}`",
        f"- run_count: `{aggregate.get('run_count')}`",
        f"- pass_count: `{aggregate.get('pass_count')}`",
        f"- all_passed: `{aggregate.get('all_passed')}`",
        "- selected_image_topic: "
        f"`{preflight.get('selected_topics', {}).get('image')}`",
        "- selected_imu_topic: "
        f"`{preflight.get('selected_topics', {}).get('imu')}`",
        f"- settings_camera_fps: `{preflight.get('settings_camera_fps')}`",
        f"- settings_file: `{preflight.get('settings_file')}`",
        f"- replay_topics: `{', '.join(preflight.get('replay_topics', []))}`",
        "- image_topic_fps_span: "
        f"`{preflight.get('image_topic_stats', {}).get('fps_span')}`",
        f"- timeout_sec: `{preflight.get('timeout_sec')}`",
        f"- required_timeout_sec: `{preflight.get('required_timeout_sec')}`",
        f"- start_offset_sec: `{preflight.get('start_offset_sec')}`",
        "",
        "## Launch Command",
        "",
        "```bash",
        launch_command,
        "```",
        "",
        "## Final Config",
        "",
        f"- source settings: `{settings_path}`",
        f"- copied to: `best_config.yaml`",
        "",
        "## Replay Metrics",
        "",
        table,
        "",
        "## Kept Commits",
        "",
        commit_lines,
        "",
        "## Excluded Hypotheses",
        "",
        excluded_lines,
        "",
        "## Remaining Risks",
        "",
        risk_lines,
        "",
    ])


def render_decision(aggregate: dict[str, Any], kept_commits: list[str], risks: list[str]) -> str:
    kept = "\n".join(f"- {item}" for item in kept_commits)
    risk_lines = "\n".join(f"- {item}" for item in risks)
    return "\n".join([
        "# Decision",
        "",
        "Accept this configuration only because the fixed H30 sync-only bag passed all "
        "guarded replay stability gates.",
        "",
        f"- label: `{aggregate.get('label')}`",
        f"- run_count: `{aggregate.get('run_count')}`",
        f"- pass_count: `{aggregate.get('pass_count')}`",
        f"- all_passed: `{aggregate.get('all_passed')}`",
        "",
        "## Kept Commits",
        "",
        kept,
        "",
        "## Remaining Risks",
        "",
        risk_lines,
        "",
    ])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("aggregate_json", type=Path)
    parser.add_argument("--settings-file", type=Path, default=DEFAULT_SETTINGS)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--min-runs", type=int, default=3)
    parser.add_argument(
        "--kept-commit",
        action="append",
        default=[],
        help="Commit and reason to include in the final report. Repeat as needed.",
    )
    parser.add_argument(
        "--excluded-hypothesis",
        action="append",
        default=[],
        help="Rejected hypothesis and evidence pointer. Repeat as needed.",
    )
    parser.add_argument(
        "--risk",
        action="append",
        default=[],
        help="Remaining risk to include in the final report. Repeat as needed.",
    )
    args = parser.parse_args()

    aggregate = load_json(args.aggregate_json)
    latest_bag = require_aggregate_bag_matches_latest(aggregate)
    preflight = require_final_replay_preflight(aggregate)
    runs = require_passing_aggregate(aggregate, args.min_runs)
    if not args.settings_file.exists():
        raise FileNotFoundError(args.settings_file)
    require_settings_match_preflight(args.settings_file, preflight)
    if not args.kept_commit:
        raise ValueError("at least one --kept-commit entry is required")
    if not args.excluded_hypothesis:
        raise ValueError("at least one --excluded-hypothesis entry is required")
    if not args.risk:
        raise ValueError("at least one --risk entry is required")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(args.settings_file, args.output_dir / "best_config.yaml")

    report = render_report(
        aggregate,
        runs,
        args.settings_file,
        args.kept_commit,
        args.excluded_hypothesis,
        args.risk,
    )
    (args.output_dir / "final_stability_report.md").write_text(report, encoding="utf-8")
    (args.output_dir / "stability_report.md").write_text(report, encoding="utf-8")
    (args.output_dir / "decision.md").write_text(
        render_decision(aggregate, args.kept_commit, args.risk),
        encoding="utf-8",
    )
    (args.output_dir / "final_commands.md").write_text(
        render_commands(runs),
        encoding="utf-8",
    )
    (args.output_dir / "final_patch_summary.md").write_text(
        "# Final Patch Summary\n\n"
        + "\n".join(f"- {item}" for item in args.kept_commit)
        + "\n",
        encoding="utf-8",
    )
    print(f"validated latest bag: {latest_bag}")
    print(f"wrote final artifacts to {args.output_dir}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
