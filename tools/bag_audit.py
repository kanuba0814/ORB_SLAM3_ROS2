#!/usr/bin/env python3
# flake8: noqa
"""Audit H30 rosbag2 topics and timestamps for ORB-SLAM3 replay."""

from __future__ import annotations

import argparse
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Any

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TARGET_TOPICS = (
    "/image_raw",
    "/imu/data_raw",
    "/sync/image_raw",
    "/sync/imu",
)


def _resolve_bag_uri(path: Path) -> Path:
    path = path.expanduser().resolve()
    if path.name == "metadata.yaml" or path.suffix == ".db3":
        return path.parent
    return path


def _stamp_to_sec(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _header_stamp(msg: Any) -> float | None:
    header = getattr(msg, "header", None)
    if header is None:
        return None
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    return _stamp_to_sec(stamp)


def _rate(stamps: list[float]) -> float | None:
    if len(stamps) < 2:
        return None
    duration = stamps[-1] - stamps[0]
    if duration <= 0.0:
        return None
    return (len(stamps) - 1) / duration


def _period_stats(stamps: list[float]) -> dict[str, float | None]:
    if len(stamps) < 2:
        return {"mean_sec": None, "std_sec": None, "count": 0}
    periods = [b - a for a, b in zip(stamps, stamps[1:])]
    return {
        "mean_sec": float(statistics.mean(periods)),
        "std_sec": float(statistics.pstdev(periods)) if len(periods) > 1 else 0.0,
        "count": len(periods),
    }


def _count_nans(value: Any) -> int:
    if isinstance(value, float):
        return 1 if math.isnan(value) else 0
    if isinstance(value, (str, bytes, bytearray, bool, int)):
        return 0
    if isinstance(value, (list, tuple)):
        return sum(_count_nans(item) for item in value)
    slots = getattr(value, "__slots__", None)
    if not slots:
        return 0
    total = 0
    for slot in slots:
        name = slot[1:] if slot.startswith("_") else slot
        if hasattr(value, name):
            total += _count_nans(getattr(value, name))
    return total


def _norm3(x: float, y: float, z: float) -> float:
    return math.sqrt(x * x + y * y + z * z)


def audit_bag(
    bag_path: Path,
    output_root: Path,
    static_seconds: float = 5.0,
) -> dict[str, Any]:
    bag_uri = _resolve_bag_uri(bag_path)
    if not bag_uri.exists():
        raise FileNotFoundError(f"bag path does not exist: {bag_uri}")

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_uri), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader.open(storage_options, converter_options)

    topics = reader.get_all_topics_and_types()
    topic_types = {topic.name: topic.type for topic in topics}
    message_classes: dict[str, Any | None] = {}
    for topic, type_name in topic_types.items():
        try:
            message_classes[topic] = get_message(type_name)
        except (AttributeError, ModuleNotFoundError, ValueError):
            message_classes[topic] = None

    counts: dict[str, int] = defaultdict(int)
    for topic in topic_types:
        counts[topic] = 0
    storage_stamps: dict[str, list[float]] = defaultdict(list)
    header_stamps: dict[str, list[float]] = defaultdict(list)
    monotonic: dict[str, dict[str, Any]] = {
        topic: {"header_stamp_monotonic": True, "violations": 0}
        for topic in topic_types
    }
    nan_counts: dict[str, int] = defaultdict(int)
    imu_norm_samples: dict[str, list[tuple[float, float, float]]] = defaultdict(list)

    while reader.has_next():
        topic, data, storage_time_ns = reader.read_next()
        counts[topic] += 1
        storage_stamps[topic].append(storage_time_ns * 1e-9)
        msg_type = message_classes.get(topic)
        if msg_type is None or topic not in TARGET_TOPICS:
            continue

        msg = deserialize_message(data, msg_type)
        nan_counts[topic] += _count_nans(msg)
        stamp = _header_stamp(msg)
        if stamp is not None:
            stamps = header_stamps[topic]
            if stamps and stamp < stamps[-1]:
                monotonic[topic]["header_stamp_monotonic"] = False
                monotonic[topic]["violations"] += 1
            stamps.append(stamp)

        if topic in ("/sync/imu", "/imu/data_raw"):
            acc = getattr(msg, "linear_acceleration", None)
            gyro = getattr(msg, "angular_velocity", None)
            if stamp is not None and acc is not None and gyro is not None:
                acc_norm = _norm3(acc.x, acc.y, acc.z)
                gyro_norm = _norm3(gyro.x, gyro.y, gyro.z)
                imu_norm_samples[topic].append((stamp, acc_norm, gyro_norm))

    topic_rates = {}
    for topic in TARGET_TOPICS:
        stamps = header_stamps.get(topic) or storage_stamps.get(topic, [])
        topic_rates[topic] = _rate(stamps)

    static_topic = "/sync/imu" if imu_norm_samples.get("/sync/imu") else "/imu/data_raw"
    static_samples = imu_norm_samples.get(static_topic, [])
    static_acc: list[float] = []
    static_gyro: list[float] = []
    if static_samples:
        start = static_samples[0][0]
        for stamp, acc_norm, gyro_norm in static_samples:
            if stamp - start <= static_seconds:
                static_acc.append(acc_norm)
                static_gyro.append(gyro_norm)

    result: dict[str, Any] = {
        "bag_path": str(bag_uri),
        "bag_name": bag_uri.name,
        "topic_list": [
            {"name": topic.name, "type": topic.type}
            for topic in sorted(topics, key=lambda item: item.name)
        ],
        "message_counts": dict(sorted(counts.items())),
        "rates_hz": topic_rates,
        "header_stamp_monotonic": monotonic,
        "sync_imu_period": _period_stats(header_stamps.get("/sync/imu", [])),
        "static_segment": {
            "topic": static_topic,
            "seconds": static_seconds,
            "sample_count": len(static_acc),
            "acc_norm_mean": float(statistics.mean(static_acc))
            if static_acc else None,
            "gyro_norm_mean": float(statistics.mean(static_gyro))
            if static_gyro else None,
        },
        "nan_counts": dict(sorted(nan_counts.items())),
    }

    output_dir = output_root / bag_uri.name
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "bag_audit.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (output_dir / "bag_audit.md").write_text(render_markdown(result), encoding="utf-8")
    return result


def render_markdown(result: dict[str, Any]) -> str:
    lines = [
        f"# Bag Audit: {result['bag_name']}",
        "",
        f"- bag_path: `{result['bag_path']}`",
        f"- topics: {len(result['topic_list'])}",
        "",
        "## Topic Counts",
        "",
        "| Topic | Count | Rate Hz | Header Monotonic | NaNs |",
        "| --- | ---: | ---: | --- | ---: |",
    ]
    counts = result["message_counts"]
    rates = result["rates_hz"]
    monotonic = result["header_stamp_monotonic"]
    nan_counts = result["nan_counts"]
    for topic in sorted(counts):
        rate = rates.get(topic)
        rate_text = f"{rate:.3f}" if isinstance(rate, float) else ""
        mono = monotonic.get(topic, {}).get("header_stamp_monotonic", "")
        lines.append(
            f"| `{topic}` | {counts[topic]} | {rate_text} | {mono} | "
            f"{nan_counts.get(topic, 0)} |"
        )

    sync_period = result["sync_imu_period"]
    static = result["static_segment"]
    lines.extend([
        "",
        "## Timing",
        "",
        f"- /sync/imu period mean sec: {sync_period.get('mean_sec')}",
        f"- /sync/imu period std sec: {sync_period.get('std_sec')}",
        f"- static topic: `{static.get('topic')}`",
        f"- static acc_norm mean: {static.get('acc_norm_mean')}",
        f"- static gyro_norm mean: {static.get('gyro_norm_mean')}",
    ])
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_path", type=Path)
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "debug_runs",
    )
    parser.add_argument("--static-seconds", type=float, default=5.0)
    args = parser.parse_args()

    result = audit_bag(args.bag_path, args.output_root, args.static_seconds)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
