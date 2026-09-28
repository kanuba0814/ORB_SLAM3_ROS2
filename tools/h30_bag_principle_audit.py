#!/usr/bin/env python3
# flake8: noqa
"""Principle-driven H30 rosbag2 audit for ORB-SLAM3 diagnosis.

This is intentionally bag-only: it reads recorded rosbag2 data and does not
launch camera, IMU, cam_imu_sync, or ORB-SLAM3 nodes.
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import statistics
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


REQUIRED_TOPICS = (
    "/image_raw",
    "/imu/data_raw",
    "/imu/raw_hw",
    "/sync/image_raw",
    "/sync/imu",
    "/sync/imu_batch",
)
DESERIALIZE_TOPICS = set(REQUIRED_TOPICS)
IMU_TOPICS = ("/imu/data_raw", "/imu/raw_hw", "/sync/imu", "/sync/imu_batch")
IMAGE_TOPICS = ("/image_raw", "/sync/image_raw")
RATE_TARGETS_HZ = {
    "/image_raw": 30.0,
    "/sync/image_raw": 30.0,
    "/imu/data_raw": 100.0,
    "/sync/imu": 100.0,
}


@dataclass
class TopicSamples:
    count: int = 0
    storage_stamps: list[float] = field(default_factory=list)
    header_stamps: list[float] = field(default_factory=list)
    header_monotonic: bool = True
    header_violations: int = 0
    nan_count: int = 0
    acc_xyz: list[tuple[float, float, float, float]] = field(default_factory=list)
    gyro_xyz: list[tuple[float, float, float, float]] = field(default_factory=list)
    image_encodings: Counter[str] = field(default_factory=Counter)
    image_sizes: Counter[str] = field(default_factory=Counter)


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


def _mean(values: list[float]) -> float | None:
    return float(statistics.mean(values)) if values else None


def _pstdev(values: list[float]) -> float | None:
    if not values:
        return None
    if len(values) == 1:
        return 0.0
    return float(statistics.pstdev(values))


def _percentile(values: list[float], pct: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return float(ordered[0])
    rank = (len(ordered) - 1) * pct / 100.0
    lo = math.floor(rank)
    hi = math.ceil(rank)
    if lo == hi:
        return float(ordered[lo])
    return float(ordered[lo] * (hi - rank) + ordered[hi] * (rank - lo))


def _stats(values: list[float]) -> dict[str, float | int | None]:
    return {
        "count": len(values),
        "min": min(values) if values else None,
        "p50": _percentile(values, 50.0),
        "p95": _percentile(values, 95.0),
        "max": max(values) if values else None,
        "mean": _mean(values),
        "std": _pstdev(values),
    }


def _rate(stamps: list[float]) -> float | None:
    if len(stamps) < 2:
        return None
    span = stamps[-1] - stamps[0]
    if span <= 0.0:
        return None
    return (len(stamps) - 1) / span


def _first_last(values: list[float]) -> dict[str, float | None]:
    return {
        "first": values[0] if values else None,
        "last": values[-1] if values else None,
    }


def _norm3(x: float, y: float, z: float) -> float:
    return math.sqrt(x * x + y * y + z * z)


def _vec_mean(samples: list[tuple[float, float, float, float]]) -> list[float] | None:
    if not samples:
        return None
    return [
        float(statistics.mean(item[index] for item in samples))
        for index in (1, 2, 3)
    ]


def _vec_norms(samples: list[tuple[float, float, float, float]]) -> list[float]:
    return [_norm3(item[1], item[2], item[3]) for item in samples]


def _slice_by_header_time(
    samples: list[tuple[float, float, float, float]],
    seconds: float,
) -> list[tuple[float, float, float, float]]:
    if not samples:
        return []
    start = samples[0][0]
    return [item for item in samples if item[0] - start <= seconds]


def _time_delta_stats(header_stamps: list[float], storage_stamps: list[float]) -> dict[str, Any]:
    deltas = [
        header - storage
        for header, storage in zip(header_stamps, storage_stamps)
    ]
    result = _stats(deltas)
    result["unit"] = "sec"
    return result


def _nearest_delta_ms(image_stamps: list[float], imu_stamps: list[float]) -> dict[str, Any]:
    deltas: list[float] = []
    if not image_stamps or not imu_stamps:
        result = _stats(deltas)
        result["unit"] = "ms"
        return result
    ordered_imu = sorted(imu_stamps)
    for stamp in image_stamps:
        index = bisect.bisect_left(ordered_imu, stamp)
        candidates = []
        if index > 0:
            candidates.append(ordered_imu[index - 1])
        if index < len(ordered_imu):
            candidates.append(ordered_imu[index])
        nearest = min(candidates, key=lambda imu_stamp: abs(stamp - imu_stamp))
        deltas.append((stamp - nearest) * 1000.0)
    result = _stats(deltas)
    result["unit"] = "ms"
    return result


def _deserialize_class(topic_types: dict[str, str]) -> dict[str, Any | None]:
    classes: dict[str, Any | None] = {}
    for topic, type_name in topic_types.items():
        if topic not in DESERIALIZE_TOPICS:
            classes[topic] = None
            continue
        try:
            classes[topic] = get_message(type_name)
        except (AttributeError, ModuleNotFoundError, ValueError):
            classes[topic] = None
    return classes


def _extract_imu_vectors(
    samples: TopicSamples,
    msg: Any,
    stamp: float | None,
) -> None:
    if stamp is None:
        return
    acc = getattr(msg, "linear_acceleration", None)
    gyro = getattr(msg, "angular_velocity", None)
    if acc is not None:
        samples.acc_xyz.append((stamp, float(acc.x), float(acc.y), float(acc.z)))
    if gyro is not None:
        samples.gyro_xyz.append((stamp, float(gyro.x), float(gyro.y), float(gyro.z)))


def _read_bag_samples(bag_uri: Path) -> tuple[dict[str, str], dict[str, TopicSamples]]:
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_uri), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader.open(storage_options, converter_options)

    topics = reader.get_all_topics_and_types()
    topic_types = {topic.name: topic.type for topic in topics}
    message_classes = _deserialize_class(topic_types)
    samples: dict[str, TopicSamples] = defaultdict(TopicSamples)
    for topic in topic_types:
        samples[topic]

    while reader.has_next():
        topic, data, storage_time_ns = reader.read_next()
        item = samples[topic]
        item.count += 1
        item.storage_stamps.append(storage_time_ns * 1e-9)

        msg_type = message_classes.get(topic)
        if msg_type is None:
            continue

        msg = deserialize_message(data, msg_type)
        item.nan_count += _count_nans(msg)
        stamp = _header_stamp(msg)
        if stamp is not None:
            if item.header_stamps and stamp < item.header_stamps[-1]:
                item.header_monotonic = False
                item.header_violations += 1
            item.header_stamps.append(stamp)
        if topic in IMU_TOPICS:
            _extract_imu_vectors(item, msg, stamp)
        if topic in IMAGE_TOPICS:
            encoding = getattr(msg, "encoding", "")
            width = getattr(msg, "width", None)
            height = getattr(msg, "height", None)
            step = getattr(msg, "step", None)
            if encoding:
                item.image_encodings[str(encoding)] += 1
            if width is not None and height is not None:
                item.image_sizes[f"{width}x{height}@step{step}"] += 1

    return topic_types, samples


def _topic_result(topic: str, type_name: str | None, samples: TopicSamples) -> dict[str, Any]:
    stamps_for_rate = samples.header_stamps or samples.storage_stamps
    return {
        "topic": topic,
        "type": type_name,
        "count": samples.count,
        "header_stamp": _first_last(samples.header_stamps),
        "bag_receive_time": _first_last(samples.storage_stamps),
        "rate_hz": _rate(stamps_for_rate),
        "header_stamp_monotonic": samples.header_monotonic,
        "header_stamp_violations": samples.header_violations,
        "header_minus_receive": _time_delta_stats(
            samples.header_stamps,
            samples.storage_stamps,
        ),
        "nan_count": samples.nan_count,
        "image_encodings": dict(samples.image_encodings.most_common()),
        "image_sizes": dict(samples.image_sizes.most_common()),
    }


def _imu_result(samples: TopicSamples, static_seconds: float) -> dict[str, Any]:
    static_acc = _slice_by_header_time(samples.acc_xyz, static_seconds)
    static_gyro = _slice_by_header_time(samples.gyro_xyz, static_seconds)
    acc_norms = _vec_norms(static_acc)
    gyro_norms = _vec_norms(static_gyro)
    all_gyro_norms = _vec_norms(samples.gyro_xyz)
    return {
        "static_seconds": static_seconds,
        "static_sample_count": len(static_acc),
        "mean_acc_xyz": _vec_mean(static_acc),
        "mean_gyro_xyz": _vec_mean(static_gyro),
        "acc_norm": _stats(acc_norms),
        "gyro_norm": _stats(gyro_norms),
        "whole_bag_gyro_norm": _stats(all_gyro_norms),
        "unit_hint": _imu_unit_hint(acc_norms, gyro_norms),
    }


def _imu_unit_hint(acc_norms: list[float], gyro_norms: list[float]) -> str:
    acc_mean = _mean(acc_norms)
    gyro_mean = _mean(gyro_norms)
    hints = []
    if acc_mean is None:
        hints.append("no_acc_samples")
    elif 8.0 <= acc_mean <= 11.5:
        hints.append("acc_mps2_like")
    elif 0.8 <= acc_mean <= 1.2:
        hints.append("acc_g_like_FAIL_for_ORB")
    else:
        hints.append("acc_norm_unexpected")
    if gyro_mean is None:
        hints.append("no_gyro_samples")
    elif gyro_mean < 0.2:
        hints.append("static_gyro_radps_like")
    else:
        hints.append("static_gyro_not_near_zero")
    return ",".join(hints)


def _status_from_checks(
    required_presence: dict[str, bool],
    topic_results: dict[str, dict[str, Any]],
    imu_results: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    failures = []
    warnings = []
    for topic, present in required_presence.items():
        if not present:
            if topic == "/sync/imu_batch":
                warnings.append(f"{topic} absent_or_empty")
            else:
                failures.append(f"{topic} absent_or_empty")
    for topic in IMAGE_TOPICS + IMU_TOPICS:
        result = topic_results.get(topic)
        if not result:
            continue
        if not result["header_stamp_monotonic"]:
            failures.append(f"{topic} header_stamp_not_monotonic")
        target_hz = RATE_TARGETS_HZ.get(topic)
        actual_hz = result.get("rate_hz")
        if target_hz and actual_hz:
            if actual_hz < target_hz * 0.5:
                warnings.append(
                    f"{topic} rate_low actual={actual_hz:.3f}Hz target~{target_hz:.1f}Hz"
                )
            elif actual_hz > target_hz * 2.2:
                warnings.append(
                    f"{topic} rate_high actual={actual_hz:.3f}Hz target~{target_hz:.1f}Hz"
                )
    for topic in ("/sync/imu", "/imu/data_raw"):
        imu = imu_results.get(topic)
        if not imu:
            continue
        hint = imu.get("unit_hint", "")
        if "acc_g_like_FAIL_for_ORB" in hint:
            failures.append(f"{topic} acceleration_looks_like_g_units")
        if "acc_norm_unexpected" in hint:
            warnings.append(f"{topic} static_acc_norm_unexpected")
        if "static_gyro_not_near_zero" in hint:
            warnings.append(f"{topic} static_gyro_not_near_zero")
    return {
        "status": "FAIL" if failures else "PASS",
        "failures": failures,
        "warnings": warnings,
    }


def audit_one_bag(bag_path: Path, label: str, static_seconds: float) -> dict[str, Any]:
    bag_uri = _resolve_bag_uri(bag_path)
    if not bag_uri.exists():
        raise FileNotFoundError(f"bag path does not exist: {bag_uri}")

    topic_types, samples = _read_bag_samples(bag_uri)
    required_presence = {
        topic: samples.get(topic, TopicSamples()).count > 0
        for topic in REQUIRED_TOPICS
    }
    topic_results = {
        topic: _topic_result(topic, topic_types.get(topic), samples.get(topic, TopicSamples()))
        for topic in sorted(set(topic_types) | set(REQUIRED_TOPICS))
    }
    imu_results = {
        topic: _imu_result(samples[topic], static_seconds)
        for topic in IMU_TOPICS
        if samples.get(topic) and samples[topic].acc_xyz
    }
    sync_alignment = {
        "sync_image_minus_nearest_sync_imu": _nearest_delta_ms(
            samples.get("/sync/image_raw", TopicSamples()).header_stamps,
            samples.get("/sync/imu", TopicSamples()).header_stamps,
        ),
        "raw_image_minus_nearest_raw_imu": _nearest_delta_ms(
            samples.get("/image_raw", TopicSamples()).header_stamps,
            samples.get("/imu/data_raw", TopicSamples()).header_stamps,
        ),
    }
    header_receive_medians = {
        topic: topic_results[topic]["header_minus_receive"].get("p50")
        for topic in REQUIRED_TOPICS
        if topic in topic_results
    }
    status = _status_from_checks(required_presence, topic_results, imu_results)
    return {
        "label": label,
        "bag_path": str(bag_uri),
        "bag_name": bag_uri.name,
        "required_presence": required_presence,
        "topic_results": topic_results,
        "imu_results": imu_results,
        "sync_alignment": sync_alignment,
        "header_minus_receive_p50_by_topic": header_receive_medians,
        "status": status,
    }


def _fmt(value: Any, digits: int = 6) -> str:
    if value is None:
        return ""
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def _topic_table_lines(bag: dict[str, Any]) -> list[str]:
    lines = [
        "| Topic | Count | Type | Header First | Header Last | Receive First | Receive Last | Rate Hz | Target Hz | Monotonic | Header-Receive p50 s | NaNs | Image Encoding/Size |",
        "| --- | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | ---: | ---: | --- |",
    ]
    for topic in REQUIRED_TOPICS:
        result = bag["topic_results"].get(topic, {})
        header = result.get("header_stamp", {})
        receive = result.get("bag_receive_time", {})
        delta = result.get("header_minus_receive", {})
        lines.append(
            "| `{topic}` | {count} | `{type}` | {hfirst} | {hlast} | "
            "{rfirst} | {rlast} | {rate} | {target} | {mono} | {delta} | {nan} | {image} |".format(
                topic=topic,
                count=result.get("count", 0),
                type=result.get("type"),
                hfirst=_fmt(header.get("first"), 3),
                hlast=_fmt(header.get("last"), 3),
                rfirst=_fmt(receive.get("first"), 3),
                rlast=_fmt(receive.get("last"), 3),
                rate=_fmt(result.get("rate_hz"), 3),
                target=_fmt(RATE_TARGETS_HZ.get(topic), 1),
                mono=result.get("header_stamp_monotonic", ""),
                delta=_fmt(delta.get("p50"), 6),
                nan=result.get("nan_count", 0),
                image=(
                    f"{result.get('image_encodings', {})} "
                    f"{result.get('image_sizes', {})}"
                    if topic in IMAGE_TOPICS else ""
                ),
            )
        )
    return lines


def _imu_lines(bag: dict[str, Any]) -> list[str]:
    lines = [
        "| Topic | Samples | Mean Acc XYZ | Acc Norm Mean | Acc Norm p95 | Mean Gyro XYZ | Gyro Norm Mean | Gyro Norm p95 | Unit Hint |",
        "| --- | ---: | --- | ---: | ---: | --- | ---: | ---: | --- |",
    ]
    for topic in IMU_TOPICS:
        result = bag["imu_results"].get(topic)
        if not result:
            lines.append(f"| `{topic}` | 0 |  |  |  |  |  |  | no_samples |")
            continue
        acc_norm = result["acc_norm"]
        gyro_norm = result["gyro_norm"]
        acc_xyz = result.get("mean_acc_xyz")
        gyro_xyz = result.get("mean_gyro_xyz")
        lines.append(
            "| `{topic}` | {samples} | {acc_xyz} | {acc_mean} | {acc_p95} | "
            "{gyro_xyz} | {gyro_mean} | {gyro_p95} | {hint} |".format(
                topic=topic,
                samples=result.get("static_sample_count", 0),
                acc_xyz=_fmt(acc_xyz),
                acc_mean=_fmt(acc_norm.get("mean")),
                acc_p95=_fmt(acc_norm.get("p95")),
                gyro_xyz=_fmt(gyro_xyz),
                gyro_mean=_fmt(gyro_norm.get("mean")),
                gyro_p95=_fmt(gyro_norm.get("p95")),
                hint=result.get("unit_hint", ""),
            )
        )
    return lines


def _alignment_lines(bag: dict[str, Any]) -> list[str]:
    lines = [
        "| Metric | Count | Min ms | P50 ms | P95 ms | Max ms |",
        "| --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    for name, result in bag["sync_alignment"].items():
        lines.append(
            "| `{name}` | {count} | {minv} | {p50} | {p95} | {maxv} |".format(
                name=name,
                count=result.get("count", 0),
                minv=_fmt(result.get("min"), 3),
                p50=_fmt(result.get("p50"), 3),
                p95=_fmt(result.get("p95"), 3),
                maxv=_fmt(result.get("max"), 3),
            )
        )
    return lines


def render_markdown(report: dict[str, Any]) -> str:
    lines = [
        "# 01 Bag Audit",
        "",
        f"- generated_for: `{report['output_root']}`",
        f"- main_bag: `{report['main']['bag_path']}`",
        f"- static_bag: `{report['static']['bag_path']}`" if report.get("static") else "- static_bag: ``",
        f"- overall_status: **{report['overall_status']}**",
        "",
        "## Conclusion",
        "",
    ]
    for label in ("main", "static"):
        bag = report.get(label)
        if not bag:
            continue
        status = bag["status"]
        lines.append(
            f"- {label}: {status['status']} failures={status['failures']} "
            f"warnings={status['warnings']}"
        )

    for label in ("main", "static"):
        bag = report.get(label)
        if not bag:
            continue
        lines.extend([
            "",
            f"## {label.capitalize()} Bag",
            "",
            f"- bag_name: `{bag['bag_name']}`",
            f"- bag_path: `{bag['bag_path']}`",
            "",
            "### Topics",
            "",
            *_topic_table_lines(bag),
            "",
            "### Sync Alignment",
            "",
            *_alignment_lines(bag),
            "",
            "### Static IMU Segment",
            "",
            *_imu_lines(bag),
        ])
    return "\n".join(lines) + "\n"


def audit(
    main_bag: Path,
    output_root: Path,
    static_bag: Path | None,
    static_seconds: float,
) -> dict[str, Any]:
    output_root = output_root.expanduser().resolve()
    main_result = audit_one_bag(main_bag, "main", static_seconds)
    static_result = (
        audit_one_bag(static_bag, "static", static_seconds)
        if static_bag
        else None
    )
    failures = list(main_result["status"]["failures"])
    if static_result:
        failures.extend(static_result["status"]["failures"])
    report = {
        "output_root": str(output_root),
        "main": main_result,
        "static": static_result,
        "overall_status": "FAIL" if failures else "PASS",
    }

    latest_dir = output_root / "latest"
    latest_dir.mkdir(parents=True, exist_ok=True)
    (latest_dir / "01_bag_audit.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (latest_dir / "01_bag_audit.md").write_text(
        render_markdown(report),
        encoding="utf-8",
    )
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--main-bag", required=True, type=Path)
    parser.add_argument("--static-bag", type=Path)
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "debug_runs",
    )
    parser.add_argument("--static-seconds", type=float, default=15.0)
    args = parser.parse_args()

    report = audit(
        main_bag=args.main_bag,
        static_bag=args.static_bag,
        output_root=args.output_root,
        static_seconds=args.static_seconds,
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
