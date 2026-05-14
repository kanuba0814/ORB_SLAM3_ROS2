#!/usr/bin/env python3
"""Run repeated offline ORB-SLAM3 bag replays and parse V2 metrics."""

from __future__ import annotations

import argparse
import csv
import json
import os
import sqlite3
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

from parse_runtime_orb_log import parse_runtime_log, render_markdown


WORKSPACE = Path("/home/gxxl/slam_ws_v2")
EXTRA_WS = WORKSPACE / "extra_ws"
PACKAGE_ROOT = EXTRA_WS / "src" / "ORB_SLAM3_ROS2"
DEFAULT_LATEST_FILES = (
    WORKSPACE / "bags" / "LATEST_BAG.txt",
    WORKSPACE / "bags" / "goalDebug" / "LATEST_BAG.txt",
)
ALLOWED_REPLAY_TOPICS = {
    "/sync/image_raw",
    "/sync/imu",
    "/sync/imu_batch",
    "/imu/pipeline_ready",
    "/rosout",
}
ALLOWED_IMAGE_TOPICS = {"/sync/image_raw"}
ALLOWED_IMU_TOPICS = {"/sync/imu", "/sync/imu_batch"}
BANNED_SELECTED_TOPICS = {"/image_raw", "/imu/data_raw", "/imu/raw_hw"}


def read_latest_bag(explicit_bag: str | None, latest_file: Path | None) -> Path:
    if explicit_bag:
        return Path(explicit_bag).expanduser().resolve()
    candidates = [latest_file] if latest_file else list(DEFAULT_LATEST_FILES)
    for candidate in candidates:
        if candidate and candidate.exists():
            content = candidate.read_text(encoding="utf-8").strip()
            if content:
                return Path(content).expanduser().resolve()
    raise FileNotFoundError("No bag path provided and no LATEST_BAG.txt found")


def launch_command(args: argparse.Namespace, bag_path: Path) -> list[str]:
    return [
        "ros2",
        "launch",
        "orb_slam3_ros2",
        "h30_bag_replay.launch.py",
        f"bag_path:={bag_path}",
        "play_bag:=true",
        f"sensor_type:={args.sensor_type}",
        f"image_topic:={args.image_topic}",
        f"imu_topic:={args.imu_topic}",
        f"settings_file:={args.settings_file}",
        f"cam_imu_time_offset:={args.cam_imu_time_offset}",
        f"orb_time_offset_mode:={args.orb_time_offset_mode}",
        f"debug_enable:={str(args.debug_enable).lower()}",
        f"debug_runtime_metrics:={str(args.debug_runtime_metrics).lower()}",
        f"enable_video_recording:={str(args.enable_video_recording).lower()}",
        "publish_pose_only_when_tracking_ok:="
        f"{str(args.publish_pose_only_when_tracking_ok).lower()}",
        f"max_image_queue_size:={args.max_image_queue_size}",
        f"max_imu_queue_size:={args.max_imu_queue_size}",
        f"drop_stale_images:={str(args.drop_stale_images).lower()}",
        f"max_image_age_ms:={args.max_image_age_ms}",
        f"use_rviz:={str(args.use_rviz).lower()}",
        f"use_pangolin:={str(args.use_pangolin).lower()}",
        f"rate:={args.rate}",
        f"duration:={args.duration}",
        f"qos_profile_overrides_path:={args.qos_profile_overrides_path}",
    ]


def read_bag_topics(bag_path: Path) -> set[str]:
    db_files = sorted(bag_path.glob("*.db3"))
    if not db_files:
        raise FileNotFoundError(f"no rosbag2 sqlite .db3 files found under {bag_path}")
    topics: set[str] = set()
    for db_file in db_files:
        with sqlite3.connect(db_file) as connection:
            for (name,) in connection.execute("select name from topics"):
                topics.add(str(name))
    return topics


def preflight_replay(args: argparse.Namespace, bag_path: Path) -> dict[str, Any]:
    if not bag_path.exists():
        raise FileNotFoundError(f"bag path does not exist: {bag_path}")
    if not bag_path.is_dir():
        raise NotADirectoryError(f"bag path is not a directory: {bag_path}")

    topics = read_bag_topics(bag_path)
    selected_topics = {args.image_topic, args.imu_topic}
    banned_selected = selected_topics & BANNED_SELECTED_TOPICS
    if banned_selected:
        raise ValueError(f"selected forbidden replay topics: {sorted(banned_selected)}")
    if args.image_topic not in ALLOWED_IMAGE_TOPICS:
        raise ValueError(f"image topic must be one of {sorted(ALLOWED_IMAGE_TOPICS)}")
    if args.imu_topic not in ALLOWED_IMU_TOPICS:
        raise ValueError(f"IMU topic must be one of {sorted(ALLOWED_IMU_TOPICS)}")

    missing = sorted(topic for topic in selected_topics if topic not in topics)
    if missing:
        raise ValueError(f"selected replay topics are missing from bag: {missing}")

    settings_file = Path(args.settings_file).expanduser()
    if not settings_file.exists():
        raise FileNotFoundError(f"settings file does not exist: {settings_file}")
    qos_path = Path(args.qos_profile_overrides_path).expanduser()
    if args.qos_profile_overrides_path and not qos_path.exists():
        raise FileNotFoundError(f"QoS override file does not exist: {qos_path}")

    ignored = sorted(topic for topic in topics if topic not in ALLOWED_REPLAY_TOPICS)
    if ignored:
        print(
            "preflight: ignoring non-replay bag topics: " + ", ".join(ignored),
            file=sys.stderr,
        )
    return {
        "bag_topics": sorted(topics),
        "ignored_bag_topics": ignored,
        "selected_topics": {
            "image": args.image_topic,
            "imu": args.imu_topic,
        },
        "settings_file": str(settings_file),
        "qos_profile_overrides_path": (
            str(qos_path) if args.qos_profile_overrides_path else ""
        ),
    }


def write_aggregate(
    label: str,
    bag_path: Path,
    summaries: list[dict[str, Any]],
    output_dir: Path,
    preflight: dict[str, Any] | None = None,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    aggregate = {
        "label": label,
        "bag_path": str(bag_path),
        "preflight": preflight or {},
        "run_count": len(summaries),
        "pass_count": sum(1 for item in summaries if item.get("stability_pass")),
        "all_passed": bool(summaries) and all(
            item.get("stability_pass") and item.get("returncode") == 0
            for item in summaries
        ),
        "runs": summaries,
    }
    (output_dir / f"{label}.json").write_text(
        json.dumps(aggregate, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    csv_path = output_dir / f"{label}.csv"
    fieldnames = [
        "run",
        "returncode",
        "stability_pass",
        "required_v2_tags_present",
        "imu_initialized",
        "ok_ratio_after_init",
        "reset_count_after_init",
        "reset_delta_last_60_sec",
        "final_keyframes",
        "median_tracked_map_points_after_init",
        "median_inliers_after_init",
        "image_to_last_imu_ms_p50",
        "image_to_last_imu_ms_p95",
        "track_ms_p50",
        "track_ms_p95",
        "input_image_fps_est_p50",
        "input_image_fps_est_p95",
        "input_imu_fps_est_p50",
        "orb_processed_fps_est_p50",
        "orb_processed_fps_est_p95",
        "processed_image_age_ms_p95",
        "oldest_img_age_ms_p95",
        "img_buf_size_max",
        "img_buf_size_final",
        "imu_buf_size_max",
        "dropped_img_count",
        "dropped_imu_count",
        "controlled_backlog_pass",
        "pose_publish_blocked_count",
        "invalid_pose_publish_count",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for index, item in enumerate(summaries, start=1):
            writer.writerow({
                "run": index,
                "returncode": item.get("returncode"),
                "stability_pass": item.get("stability_pass"),
                "required_v2_tags_present": item.get("required_v2_tags_present"),
                "imu_initialized": item.get("imu_initialized"),
                "ok_ratio_after_init": item.get("ok_ratio_after_init"),
                "reset_count_after_init": item.get("reset_count_after_init"),
                "reset_delta_last_60_sec": item.get("reset_delta_last_60_sec"),
                "final_keyframes": item.get("final_keyframes"),
                "median_tracked_map_points_after_init": item.get(
                    "median_tracked_map_points_after_init"),
                "median_inliers_after_init": item.get("median_inliers_after_init"),
                "image_to_last_imu_ms_p50": item.get(
                    "image_to_last_imu_ms", {}).get("p50"),
                "image_to_last_imu_ms_p95": item.get(
                    "image_to_last_imu_ms", {}).get("p95"),
                "track_ms_p50": item.get("track_ms", {}).get("p50"),
                "track_ms_p95": item.get("track_ms", {}).get("p95"),
                "input_image_fps_est_p50": item.get(
                    "input_image_fps_est", {}).get("p50"),
                "input_image_fps_est_p95": item.get(
                    "input_image_fps_est", {}).get("p95"),
                "input_imu_fps_est_p50": item.get(
                    "input_imu_fps_est", {}).get("p50"),
                "orb_processed_fps_est_p50": item.get(
                    "orb_processed_fps_est", {}).get("p50"),
                "orb_processed_fps_est_p95": item.get(
                    "orb_processed_fps_est", {}).get("p95"),
                "processed_image_age_ms_p95": item.get(
                    "processed_image_age_ms", {}).get("p95"),
                "oldest_img_age_ms_p95": item.get(
                    "oldest_img_age_ms", {}).get("p95"),
                "img_buf_size_max": item.get("img_buf_size_max"),
                "img_buf_size_final": item.get("img_buf_size_final"),
                "imu_buf_size_max": item.get("imu_buf_size_max"),
                "dropped_img_count": item.get("dropped_img_count"),
                "dropped_imu_count": item.get("dropped_imu_count"),
                "controlled_backlog_pass": item.get("controlled_backlog_pass"),
                "pose_publish_blocked_count": item.get("pose_publish_blocked_count"),
                "invalid_pose_publish_count": item.get("invalid_pose_publish_count"),
            })

    lines = [
        f"# {label}",
        "",
        f"- bag_path: `{bag_path}`",
        "- selected_image_topic: "
        f"`{(preflight or {}).get('selected_topics', {}).get('image')}`",
        "- selected_imu_topic: "
        f"`{(preflight or {}).get('selected_topics', {}).get('imu')}`",
        "- ignored_bag_topics: "
        f"`{', '.join((preflight or {}).get('ignored_bag_topics', []))}`",
        f"- run_count: `{len(summaries)}`",
        f"- pass_count: `{aggregate['pass_count']}`",
        f"- all_passed: `{aggregate['all_passed']}`",
        "",
        "| Run | Return | Pass | V2 Tags | IMU Init | OK Ratio | Reset After Init | Final KFs | Median Inliers | Median MPs | IMU Lag P95 ms | Image Age P95 ms | ORB FPS P50 | Input FPS P50 | Backlog OK | Img Buf Max | Invalid Pose Publishes |",
        "| ---: | ---: | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | ---: | ---: |",
    ]
    for index, item in enumerate(summaries, start=1):
        lines.append(
            f"| {index} | {item.get('returncode')} | "
            f"{item.get('stability_pass')} | "
            f"{item.get('required_v2_tags_present')} | "
            f"{item.get('imu_initialized')} | "
            f"{item.get('ok_ratio_after_init')} | "
            f"{item.get('reset_count_after_init')} | "
            f"{item.get('final_keyframes')} | "
            f"{item.get('median_inliers_after_init')} | "
            f"{item.get('median_tracked_map_points_after_init')} | "
            f"{item.get('image_to_last_imu_ms', {}).get('p95')} | "
            f"{item.get('processed_image_age_ms', {}).get('p95')} | "
            f"{item.get('orb_processed_fps_est', {}).get('p50')} | "
            f"{item.get('input_image_fps_est', {}).get('p50')} | "
            f"{item.get('controlled_backlog_pass')} | "
            f"{item.get('img_buf_size_max')} | "
            f"{item.get('invalid_pose_publish_count')} |"
        )
    (output_dir / f"{label}.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag-path")
    parser.add_argument("--latest-file", type=Path)
    parser.add_argument("--runs", type=int, default=3)
    parser.add_argument("--label", default="02_replay_baseline_after_wrapper_fixes")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=EXTRA_WS / "debug_runs" / "fix_until_stable",
    )
    parser.add_argument("--sensor-type", default="imu-monocular")
    parser.add_argument("--image-topic", default="/sync/image_raw")
    parser.add_argument("--imu-topic", default="/sync/imu")
    parser.add_argument(
        "--settings-file",
        default=str(PACKAGE_ROOT / "config" / "Monocular-Inertial" / "rmoncam_h30.yaml"),
    )
    parser.add_argument("--cam-imu-time-offset", default="0.0")
    parser.add_argument("--orb-time-offset-mode", default="zero_for_sync")
    parser.add_argument("--debug-enable", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--debug-runtime-metrics", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--enable-video-recording", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument(
        "--publish-pose-only-when-tracking-ok",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--max-image-queue-size", default="2")
    parser.add_argument("--max-imu-queue-size", default="500")
    parser.add_argument("--drop-stale-images", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--max-image-age-ms", default="200.0")
    parser.add_argument("--use-rviz", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--use-pangolin", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--rate", default="1.0")
    parser.add_argument("--duration", default="0.0")
    parser.add_argument(
        "--qos-profile-overrides-path",
        default=str(PACKAGE_ROOT / "config" / "bag_qos_override.yaml"),
    )
    parser.add_argument("--preflight", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument(
        "--preflight-only",
        action="store_true",
        help="Run local bag/topic/settings checks and exit before launching ROS.",
    )
    parser.add_argument("--timeout-sec", type=float, default=0.0)
    args = parser.parse_args()
    if args.runs < 1:
        parser.error("--runs must be at least 1")

    bag_path = read_latest_bag(args.bag_path, args.latest_file)
    preflight_info: dict[str, Any] | None = None
    if args.preflight:
        try:
            preflight_info = preflight_replay(args, bag_path)
        except Exception as exc:
            parser.error(f"preflight failed: {exc}")
    if args.preflight_only:
        print(f"preflight passed for {bag_path}")
        return 0
    args.output_dir.mkdir(parents=True, exist_ok=True)
    summaries: list[dict[str, Any]] = []

    for run_index in range(1, args.runs + 1):
        run_prefix = args.output_dir / f"{args.label}_run{run_index:02d}"
        log_path = run_prefix.with_suffix(".log")
        command = launch_command(args, bag_path)
        with log_path.open("w", encoding="utf-8", errors="replace") as log_file:
            log_file.write("$ " + " ".join(command) + "\n")
            log_file.flush()
            started = time.time()
            try:
                completed = subprocess.run(
                    command,
                    cwd=EXTRA_WS,
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                    text=True,
                    timeout=args.timeout_sec if args.timeout_sec > 0 else None,
                    check=False,
                    env=os.environ.copy(),
                )
                returncode = completed.returncode
            except subprocess.TimeoutExpired:
                returncode = 124
                log_file.write(f"\nTIMEOUT after {args.timeout_sec} seconds\n")
            elapsed = time.time() - started
            log_file.write(f"\nreturncode={returncode} elapsed_sec={elapsed:.3f}\n")

        summary = parse_runtime_log(log_path)
        summary["run_index"] = run_index
        summary["returncode"] = returncode
        summary["elapsed_wall_sec"] = elapsed
        summary["command"] = command
        summaries.append(summary)
        run_prefix.with_suffix(".json").write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        run_prefix.with_suffix(".md").write_text(render_markdown(summary), encoding="utf-8")

        if returncode != 0:
            print(f"run {run_index} returned {returncode}", file=sys.stderr)

    write_aggregate(args.label, bag_path, summaries, args.output_dir, preflight_info)
    return 0 if all(
        item.get("stability_pass") and item.get("returncode") == 0
        for item in summaries
    ) else 1


if __name__ == "__main__":
    raise SystemExit(main())
