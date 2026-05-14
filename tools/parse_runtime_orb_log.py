#!/usr/bin/env python3
"""Parse V2 ORB wrapper runtime logs and evaluate stability gates."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import statistics
from pathlib import Path
from typing import Any


KV_RE = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)=([^\s]+)")
TAGS = {
    "track": "[orb_track]",
    "queue": "[orb_queue]",
    "timing": "[orb_timing]",
    "imu_window": "[orb_imu_window]",
    "state_transition": "[orb_state_transition]",
    "pose_gate": "[orb_pose_gate]",
    "visual_quality": "[orb_visual_quality]",
    "event": "[orb_event]",
}
OK_STATES = {"OK", "OK_KLT"}
BAD_STATES = {"RECENTLY_LOST", "LOST", "NOT_INITIALIZED"}


def parse_value(value: str) -> Any:
    if value in {"true", "True"}:
        return True
    if value in {"false", "False"}:
        return False
    if value in {"nan", "NaN"}:
        return math.nan
    try:
        if any(ch in value for ch in ".eE"):
            return float(value)
        return int(value)
    except ValueError:
        return value


def extract_kv(line: str) -> dict[str, Any]:
    return {key: parse_value(value) for key, value in KV_RE.findall(line)}


def finite(values: list[Any]) -> list[float]:
    return [
        float(value)
        for value in values
        if isinstance(value, (int, float)) and math.isfinite(float(value))
    ]


def percentile(values: list[Any], pct: float) -> float | None:
    clean = finite(values)
    if not clean:
        return None
    clean.sort()
    index = (len(clean) - 1) * pct
    lo = math.floor(index)
    hi = math.ceil(index)
    if lo == hi:
        return clean[int(index)]
    return clean[lo] * (hi - index) + clean[hi] * (index - lo)


def median(values: list[Any]) -> float | None:
    clean = finite(values)
    return float(statistics.median(clean)) if clean else None


def get_time(record: dict[str, Any]) -> float | None:
    for key in ("track_time_sec", "image_time_sec"):
        value = record.get(key)
        if isinstance(value, (int, float)) and math.isfinite(float(value)):
            return float(value)
    return None


def parse_runtime_log(path: Path) -> dict[str, Any]:
    records = {name: [] for name in TAGS}
    errors: list[str] = []

    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            for name, tag in TAGS.items():
                if tag in line:
                    records[name].append(extract_kv(line))
                    break
            else:
                if "ERROR" in line or "[ERROR]" in line:
                    errors.append(line[-500:])

    track = records["track"]
    timing = records["timing"]
    queue = records["queue"]
    pose_gate = records["pose_gate"]
    visual = records["visual_quality"]

    first_init_index: int | None = None
    first_init_frame: int | None = None
    first_init_time: float | None = None
    for index, record in enumerate(track):
        if int(record.get("imu_initialized", 0) or 0) == 1:
            first_init_index = index
            frame = record.get("frame_id")
            first_init_frame = int(frame) if isinstance(frame, int) else None
            first_init_time = get_time(record)
            break

    post_init = track[first_init_index:] if first_init_index is not None else []
    state_counts: dict[str, int] = {}
    for record in post_init or track:
        state = str(record.get("tracking_state", "UNKNOWN"))
        state_counts[state] = state_counts.get(state, 0) + 1

    ok_count = sum(state_counts.get(state, 0) for state in OK_STATES)
    bad_count = sum(state_counts.get(state, 0) for state in BAD_STATES)
    denominator = len(post_init or track)

    reset_values = finite([record.get("reset_count") for record in track])
    reset_after_init = 0
    if post_init:
        init_reset = post_init[0].get("reset_count", 0)
        final_reset = post_init[-1].get("reset_count", init_reset)
        if isinstance(init_reset, (int, float)) and isinstance(final_reset, (int, float)):
            reset_after_init = int(max(0, final_reset - init_reset))

    final_time = get_time(track[-1]) if track else None
    last_60 = []
    if final_time is not None:
        last_60 = [
            record for record in track
            if (t := get_time(record)) is not None and final_time - t <= 60.0
        ]
    reset_delta_last_60 = 0
    if len(last_60) >= 2:
        start_reset = last_60[0].get("reset_count", 0)
        end_reset = last_60[-1].get("reset_count", start_reset)
        if isinstance(start_reset, (int, float)) and isinstance(end_reset, (int, float)):
            reset_delta_last_60 = int(max(0, end_reset - start_reset))

    lost_streak_max = 0.0
    lost_start: float | None = None
    lost_last: float | None = None
    for record in post_init or track:
        time_sec = get_time(record)
        if time_sec is None:
            continue
        if str(record.get("tracking_state")) in {"RECENTLY_LOST", "LOST"}:
            if lost_start is None:
                lost_start = time_sec
            lost_last = time_sec
        elif lost_start is not None:
            lost_streak_max = max(lost_streak_max, (lost_last or time_sec) - lost_start)
            lost_start = None
            lost_last = None
    if lost_start is not None and lost_last is not None:
        lost_streak_max = max(lost_streak_max, lost_last - lost_start)

    invalid_pose_publish_count = 0
    pose_publish_blocked_count = 0
    for record in pose_gate:
        publish = int(record.get("publish_pose", 0) or 0) == 1
        state = str(record.get("tracking_state", "UNKNOWN"))
        if publish and state not in OK_STATES:
            invalid_pose_publish_count += 1
        if not publish:
            pose_publish_blocked_count += 1

    summary = {
        "log_path": str(path),
        "record_counts": {name: len(value) for name, value in records.items()},
        "errors": errors[:20],
        "imu_initialized": first_init_index is not None,
        "first_imu_initialized_frame": first_init_frame,
        "first_imu_initialized_time": first_init_time,
        "tracking_state": track[-1].get("tracking_state") if track else None,
        "tracking_state_counts_after_init": state_counts,
        "ok_ratio_after_init": ok_count / denominator if denominator else 0.0,
        "bad_ratio_after_init": bad_count / denominator if denominator else 0.0,
        "max_lost_streak_sec_after_init": lost_streak_max,
        "reset_count": int(max(reset_values)) if reset_values else 0,
        "reset_count_after_init": reset_after_init,
        "reset_delta_last_60_sec": reset_delta_last_60,
        "final_keyframes": int(track[-1].get("num_keyframes", 0)) if track else 0,
        "max_keyframes": int(max(finite([r.get("num_keyframes") for r in track])) or 0)
        if track else 0,
        "median_tracked_map_points_after_init": median(
            [record.get("num_tracked_mappoints") for record in post_init]
        ),
        "median_inliers_after_init": median(
            [record.get("num_inliers") for record in post_init]
        ),
        "image_to_last_imu_ms": {
            "p50": percentile([r.get("image_to_last_imu_ms") for r in records["imu_window"]], 0.50),
            "p95": percentile([r.get("image_to_last_imu_ms") for r in records["imu_window"]], 0.95),
            "max": max(finite([r.get("image_to_last_imu_ms") for r in records["imu_window"]]), default=None),
        },
        "track_ms": {
            "p50": percentile([r.get("track_ms") for r in timing], 0.50),
            "p95": percentile([r.get("track_ms") for r in timing], 0.95),
            "max": max(finite([r.get("track_ms") for r in timing]), default=None),
        },
        "processed_image_age_ms": {
            "p95": percentile([r.get("processed_image_age_ms") for r in timing], 0.95),
            "max": max(finite([r.get("processed_image_age_ms") for r in timing]), default=None),
        },
        "img_buf_size_max": max(finite([r.get("img_buf_size") for r in queue]), default=0),
        "dropped_img_count": int(max(finite([r.get("dropped_img_count") for r in queue]), default=0)),
        "dropped_imu_count": int(max(finite([r.get("dropped_imu_count") for r in queue]), default=0)),
        "pose_publish_blocked_count": pose_publish_blocked_count,
        "invalid_pose_publish_count": invalid_pose_publish_count,
        "visual_quality_records": len(visual),
    }
    image_to_last_imu_p95 = summary["image_to_last_imu_ms"]["p95"]
    required_v2_tags_present = all(
        summary["record_counts"].get(name, 0) > 0
        for name in (
            "queue",
            "timing",
            "imu_window",
            "state_transition",
            "pose_gate",
            "visual_quality",
        )
    )
    summary["required_v2_tags_present"] = required_v2_tags_present

    summary["stability_pass"] = bool(
        summary["imu_initialized"]
        and summary["required_v2_tags_present"]
        and summary["reset_count_after_init"] == 0
        and summary["ok_ratio_after_init"] >= 0.90
        and summary["max_lost_streak_sec_after_init"] < 5.0
        and summary["reset_delta_last_60_sec"] == 0
        and summary["final_keyframes"] >= 20
        and (summary["median_tracked_map_points_after_init"] or 0) >= 80
        and (summary["median_inliers_after_init"] or 0) >= 50
        and image_to_last_imu_p95 is not None
        and abs(image_to_last_imu_p95) <= 20.0
        and (summary["processed_image_age_ms"]["p95"] or math.inf) < 150.0
        and summary["img_buf_size_max"] <= 2
        and summary["invalid_pose_publish_count"] == 0
    )
    return summary


def render_markdown(summary: dict[str, Any]) -> str:
    rows = [
        ("stability_pass", summary["stability_pass"]),
        ("required_v2_tags_present", summary["required_v2_tags_present"]),
        ("imu_initialized", summary["imu_initialized"]),
        ("ok_ratio_after_init", summary["ok_ratio_after_init"]),
        ("reset_count_after_init", summary["reset_count_after_init"]),
        ("reset_delta_last_60_sec", summary["reset_delta_last_60_sec"]),
        ("final_keyframes", summary["final_keyframes"]),
        ("median_tracked_map_points_after_init", summary["median_tracked_map_points_after_init"]),
        ("median_inliers_after_init", summary["median_inliers_after_init"]),
        ("image_to_last_imu_ms_p50", summary["image_to_last_imu_ms"]["p50"]),
        ("image_to_last_imu_ms_p95", summary["image_to_last_imu_ms"]["p95"]),
        ("track_ms_p50", summary["track_ms"]["p50"]),
        ("track_ms_p95", summary["track_ms"]["p95"]),
        ("processed_image_age_ms_p95", summary["processed_image_age_ms"]["p95"]),
        ("img_buf_size_max", summary["img_buf_size_max"]),
        ("dropped_img_count", summary["dropped_img_count"]),
        ("pose_publish_blocked_count", summary["pose_publish_blocked_count"]),
        ("invalid_pose_publish_count", summary["invalid_pose_publish_count"]),
    ]
    lines = ["# Runtime ORB Log Summary", "", "| Metric | Value |", "| --- | --- |"]
    lines.extend(f"| `{key}` | `{value}` |" for key, value in rows)
    return "\n".join(lines) + "\n"


def write_csv(summary: dict[str, Any], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    flat = {
        "log_path": summary["log_path"],
        "stability_pass": summary["stability_pass"],
        "required_v2_tags_present": summary["required_v2_tags_present"],
        "imu_initialized": summary["imu_initialized"],
        "ok_ratio_after_init": summary["ok_ratio_after_init"],
        "reset_count_after_init": summary["reset_count_after_init"],
        "reset_delta_last_60_sec": summary["reset_delta_last_60_sec"],
        "final_keyframes": summary["final_keyframes"],
        "median_tracked_map_points_after_init": summary["median_tracked_map_points_after_init"],
        "median_inliers_after_init": summary["median_inliers_after_init"],
        "image_to_last_imu_ms_p50": summary["image_to_last_imu_ms"]["p50"],
        "image_to_last_imu_ms_p95": summary["image_to_last_imu_ms"]["p95"],
        "track_ms_p50": summary["track_ms"]["p50"],
        "track_ms_p95": summary["track_ms"]["p95"],
        "processed_image_age_ms_p95": summary["processed_image_age_ms"]["p95"],
        "img_buf_size_max": summary["img_buf_size_max"],
        "dropped_img_count": summary["dropped_img_count"],
        "pose_publish_blocked_count": summary["pose_publish_blocked_count"],
        "invalid_pose_publish_count": summary["invalid_pose_publish_count"],
    }
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(flat))
        writer.writeheader()
        writer.writerow(flat)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log_path", type=Path)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-md", type=Path)
    parser.add_argument("--output-csv", type=Path)
    args = parser.parse_args()

    summary = parse_runtime_log(args.log_path)
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    else:
        print(json.dumps(summary, indent=2, sort_keys=True))
    if args.output_md:
        args.output_md.parent.mkdir(parents=True, exist_ok=True)
        args.output_md.write_text(render_markdown(summary), encoding="utf-8")
    if args.output_csv:
        write_csv(summary, args.output_csv)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
