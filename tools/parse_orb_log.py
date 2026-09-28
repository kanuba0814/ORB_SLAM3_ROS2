#!/usr/bin/env python3
# flake8: noqa
"""Parse ORB-SLAM3 ROS wrapper console logs into run metrics."""

from __future__ import annotations

import argparse
import json
import math
import re
import statistics
from collections import Counter
from pathlib import Path
from typing import Any


KV_RE = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)=([^\s]+)")


def _parse_value(value: str) -> Any:
    if value in ("true", "True"):
        return True
    if value in ("false", "False"):
        return False
    try:
        if any(ch in value for ch in ".eE"):
            parsed = float(value)
            return parsed
        return int(value)
    except ValueError:
        return value


def _extract_kv(line: str) -> dict[str, Any]:
    return {key: _parse_value(value) for key, value in KV_RE.findall(line)}


def _median(values: list[float]) -> float | None:
    clean = [value for value in values if value is not None and math.isfinite(value)]
    if not clean:
        return None
    return float(statistics.median(clean))


def _mean(values: list[float]) -> float | None:
    clean = [value for value in values if value is not None and math.isfinite(value)]
    if not clean:
        return None
    return float(statistics.mean(clean))


def _max(values: list[float]) -> float | None:
    clean = [value for value in values if value is not None and math.isfinite(value)]
    if not clean:
        return None
    return float(max(clean))


def parse_orb_log(log_path: Path) -> dict[str, Any]:
    track_records: list[dict[str, Any]] = []
    pose_records: list[dict[str, Any]] = []
    event_records: list[dict[str, Any]] = []
    warning_insufficient = 0
    errors: list[str] = []

    with log_path.open("r", encoding="utf-8", errors="replace") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if "[orb_track]" in line:
                track_records.append(_extract_kv(line))
            elif "[orb_pose]" in line:
                pose_records.append(_extract_kv(line))
            elif "[orb_event]" in line:
                event_records.append(_extract_kv(line))
            elif "Only " in line and "IMU meas" in line:
                warning_insufficient += 1
            elif "ERROR" in line or "[ERROR]" in line:
                errors.append(line[-500:])

    state_counts: dict[str, int] = {}
    event_code_counts: Counter[str] = Counter()
    event_reason_counts: Counter[str] = Counter()
    first_ok_frame: int | None = None
    first_imu_initialized_frame: int | None = None
    insufficient_imu_frames = warning_insufficient
    image_to_last_imu_ms: list[float] = []
    inliers: list[float] = []
    tracked_mappoints: list[float] = []
    keyframes: list[float] = []
    reset_counts: list[float] = []
    reset_samples: list[tuple[float, float]] = []
    pose_drifts: list[float] = []
    track_times: list[float] = []

    max_lost_streak_sec = 0.0
    current_lost_start: float | None = None
    current_lost_last: float | None = None

    for record in track_records:
        state = str(record.get("tracking_state", "UNKNOWN"))
        state_counts[state] = state_counts.get(state, 0) + 1
        frame = record.get("frame_id")
        if first_ok_frame is None and state in ("OK", "OK_KLT"):
            first_ok_frame = int(frame) if isinstance(frame, int) else None
        if (
            first_imu_initialized_frame is None
            and int(record.get("imu_initialized", 0) or 0) == 1
        ):
            first_imu_initialized_frame = int(frame) if isinstance(frame, int) else None
        if record.get("skip_reason") == "insufficient_imu":
            insufficient_imu_frames += 1

        for field, sink in (
            ("image_to_last_imu_ms", image_to_last_imu_ms),
            ("num_inliers", inliers),
            ("num_tracked_mappoints", tracked_mappoints),
            ("num_keyframes", keyframes),
            ("reset_count", reset_counts),
            ("pose_drift_m", pose_drifts),
            ("track_time_sec", track_times),
        ):
            value = record.get(field)
            if isinstance(value, (int, float)):
                sink.append(float(value))

        t = record.get("track_time_sec")
        reset_count = record.get("reset_count")
        if isinstance(t, (int, float)) and isinstance(reset_count, (int, float)):
            reset_samples.append((float(t), float(reset_count)))
        if isinstance(t, (int, float)) and state in ("LOST", "RECENTLY_LOST"):
            if current_lost_start is None:
                current_lost_start = float(t)
            current_lost_last = float(t)
        elif isinstance(t, (int, float)) and current_lost_start is not None:
            max_lost_streak_sec = max(
                max_lost_streak_sec,
                float(current_lost_last or t) - current_lost_start,
            )
            current_lost_start = None
            current_lost_last = None

    if current_lost_start is not None and current_lost_last is not None:
        max_lost_streak_sec = max(
            max_lost_streak_sec,
            current_lost_last - current_lost_start,
        )

    for record in event_records:
        event_code = record.get("event_code")
        reason = record.get("reason")
        if event_code is not None:
            event_code_counts[str(event_code)] += 1
        if reason is not None:
            event_reason_counts[str(reason)] += 1

    for record in pose_records:
        value = record.get("drift_translation_m")
        if isinstance(value, (int, float)):
            pose_drifts.append(float(value))

    total_tracks = len(track_records)
    ok_count = state_counts.get("OK", 0) + state_counts.get("OK_KLT", 0)
    lost_count = state_counts.get("LOST", 0)
    recently_lost_count = state_counts.get("RECENTLY_LOST", 0)
    late_reset_delta = 0
    if reset_samples:
        final_time = reset_samples[-1][0]
        final_reset = reset_samples[-1][1]
        baseline_reset = reset_samples[0][1]
        for sample_time, sample_reset in reversed(reset_samples):
            if final_time - sample_time >= 30.0:
                baseline_reset = sample_reset
                break
        late_reset_delta = int(max(0.0, final_reset - baseline_reset))

    result = {
        "log_path": str(log_path),
        "track_record_count": total_tracks,
        "tracking_state": track_records[-1].get("tracking_state")
        if track_records else None,
        "tracking_state_counts": state_counts,
        "ok_ratio": ok_count / total_tracks if total_tracks else 0.0,
        "lost_ratio": lost_count / total_tracks if total_tracks else 0.0,
        "recently_lost_ratio": recently_lost_count / total_tracks
        if total_tracks else 0.0,
        "first_ok_frame": first_ok_frame,
        "imu_initialized": first_imu_initialized_frame is not None,
        "first_imu_initialized_frame": first_imu_initialized_frame,
        "reset_count": int(max(reset_counts)) if reset_counts else 0,
        "late_reset_delta_30s": late_reset_delta,
        "keyframe_count": int(max(keyframes)) if keyframes else 0,
        "final_keyframe_count": int(keyframes[-1]) if keyframes else 0,
        "tracked_map_points": {
            "last": tracked_mappoints[-1] if tracked_mappoints else None,
            "median": _median(tracked_mappoints),
            "max": _max(tracked_mappoints),
        },
        "inlier_count": {
            "last": inliers[-1] if inliers else None,
            "median": _median(inliers),
            "max": _max(inliers),
        },
        "insufficient_imu_frames": insufficient_imu_frames,
        "image_to_last_imu_ms": {
            "last": image_to_last_imu_ms[-1] if image_to_last_imu_ms else None,
            "median": _median(image_to_last_imu_ms),
            "mean": _mean(image_to_last_imu_ms),
            "min": min(image_to_last_imu_ms) if image_to_last_imu_ms else None,
            "max": max(image_to_last_imu_ms) if image_to_last_imu_ms else None,
        },
        "final_pose_drift_m": pose_drifts[-1] if pose_drifts else None,
        "max_lost_streak_sec": max_lost_streak_sec,
        "duration_sec": (max(track_times) - min(track_times))
        if len(track_times) > 1 else 0.0,
        "event_count": len(event_records),
        "event_code_counts": dict(event_code_counts.most_common()),
        "event_reason_counts": dict(event_reason_counts.most_common()),
        "errors": errors[:20],
    }
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log_path", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--summary-md", type=Path)
    args = parser.parse_args()

    result = parse_orb_log(args.log_path)

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    else:
        print(json.dumps(result, indent=2, sort_keys=True))

    if args.summary_md:
        args.summary_md.parent.mkdir(parents=True, exist_ok=True)
        args.summary_md.write_text(render_summary(result), encoding="utf-8")

    return 0


def render_summary(result: dict[str, Any]) -> str:
    lines = [
        "# ORB Log Summary",
        "",
        f"- tracking_state: {result.get('tracking_state')}",
        f"- ok_ratio: {result.get('ok_ratio'):.3f}",
        f"- lost_ratio: {result.get('lost_ratio'):.3f}",
        f"- recently_lost_ratio: {result.get('recently_lost_ratio'):.3f}",
        f"- first_ok_frame: {result.get('first_ok_frame')}",
        f"- imu_initialized: {result.get('imu_initialized')}",
        f"- first_imu_initialized_frame: {result.get('first_imu_initialized_frame')}",
        f"- reset_count: {result.get('reset_count')}",
        f"- late_reset_delta_30s: {result.get('late_reset_delta_30s')}",
        f"- keyframe_count: {result.get('keyframe_count')}",
        f"- median_inliers: {result.get('inlier_count', {}).get('median')}",
        f"- insufficient_imu_frames: {result.get('insufficient_imu_frames')}",
        f"- image_to_last_imu_ms_median: "
        f"{result.get('image_to_last_imu_ms', {}).get('median')}",
        f"- final_pose_drift_m: {result.get('final_pose_drift_m')}",
        f"- event_code_counts: {result.get('event_code_counts')}",
        f"- event_reason_counts: {result.get('event_reason_counts')}",
    ]
    return "\n".join(lines) + "\n"


if __name__ == "__main__":
    raise SystemExit(main())
