#!/usr/bin/env python3
"""Run repeated offline ORB-SLAM3 bag replays and parse V2 metrics."""

from __future__ import annotations

import argparse
import csv
import json
import os
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


def write_aggregate(label: str, bag_path: Path, summaries: list[dict[str, Any]], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    aggregate = {
        "label": label,
        "bag_path": str(bag_path),
        "run_count": len(summaries),
        "pass_count": sum(1 for item in summaries if item.get("stability_pass")),
        "all_passed": bool(summaries) and all(item.get("stability_pass") for item in summaries),
        "runs": summaries,
    }
    (output_dir / f"{label}.json").write_text(
        json.dumps(aggregate, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    csv_path = output_dir / f"{label}.csv"
    fieldnames = [
        "run",
        "stability_pass",
        "imu_initialized",
        "ok_ratio_after_init",
        "reset_count_after_init",
        "reset_delta_last_60_sec",
        "final_keyframes",
        "median_tracked_map_points_after_init",
        "median_inliers_after_init",
        "track_ms_p50",
        "track_ms_p95",
        "processed_image_age_ms_p95",
        "img_buf_size_max",
        "dropped_img_count",
        "pose_publish_blocked_count",
        "invalid_pose_publish_count",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for index, item in enumerate(summaries, start=1):
            writer.writerow({
                "run": index,
                "stability_pass": item.get("stability_pass"),
                "imu_initialized": item.get("imu_initialized"),
                "ok_ratio_after_init": item.get("ok_ratio_after_init"),
                "reset_count_after_init": item.get("reset_count_after_init"),
                "reset_delta_last_60_sec": item.get("reset_delta_last_60_sec"),
                "final_keyframes": item.get("final_keyframes"),
                "median_tracked_map_points_after_init": item.get(
                    "median_tracked_map_points_after_init"),
                "median_inliers_after_init": item.get("median_inliers_after_init"),
                "track_ms_p50": item.get("track_ms", {}).get("p50"),
                "track_ms_p95": item.get("track_ms", {}).get("p95"),
                "processed_image_age_ms_p95": item.get(
                    "processed_image_age_ms", {}).get("p95"),
                "img_buf_size_max": item.get("img_buf_size_max"),
                "dropped_img_count": item.get("dropped_img_count"),
                "pose_publish_blocked_count": item.get("pose_publish_blocked_count"),
                "invalid_pose_publish_count": item.get("invalid_pose_publish_count"),
            })

    lines = [
        f"# {label}",
        "",
        f"- bag_path: `{bag_path}`",
        f"- run_count: `{len(summaries)}`",
        f"- pass_count: `{aggregate['pass_count']}`",
        f"- all_passed: `{aggregate['all_passed']}`",
        "",
        "| Run | Pass | IMU Init | OK Ratio | Reset After Init | Final KFs | Median Inliers | Median MPs | Image Age P95 ms | Img Buf Max | Invalid Pose Publishes |",
        "| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for index, item in enumerate(summaries, start=1):
        lines.append(
            f"| {index} | {item.get('stability_pass')} | "
            f"{item.get('imu_initialized')} | "
            f"{item.get('ok_ratio_after_init')} | "
            f"{item.get('reset_count_after_init')} | "
            f"{item.get('final_keyframes')} | "
            f"{item.get('median_inliers_after_init')} | "
            f"{item.get('median_tracked_map_points_after_init')} | "
            f"{item.get('processed_image_age_ms', {}).get('p95')} | "
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
    parser.add_argument("--timeout-sec", type=float, default=0.0)
    args = parser.parse_args()

    bag_path = read_latest_bag(args.bag_path, args.latest_file)
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

        if returncode not in (0, 130, 143):
            print(f"run {run_index} returned {returncode}", file=sys.stderr)

    write_aggregate(args.label, bag_path, summaries, args.output_dir)
    return 0 if all(item.get("stability_pass") for item in summaries) else 1


if __name__ == "__main__":
    raise SystemExit(main())
