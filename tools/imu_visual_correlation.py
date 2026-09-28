#!/usr/bin/env python3
# flake8: noqa
"""Rank IMU gyro axis hypotheses against image motion from an H30 rosbag."""

from __future__ import annotations

import argparse
import itertools
import json
import math
import re
import sqlite3
import struct
from pathlib import Path
from typing import Any

import cv2
import numpy as np


SOURCE_PACKAGE_ROOT = Path("/home/gxxl/slam_ws_v2/extra_ws/src/ORB_SLAM3_ROS2")
PACKAGE_ROOT = (
    SOURCE_PACKAGE_ROOT
    if SOURCE_PACKAGE_ROOT.exists()
    else Path(__file__).resolve().parents[1]
)
WORKSPACE_ROOT = PACKAGE_ROOT.parents[2]
DEFAULT_LATEST = WORKSPACE_ROOT / "bags" / "goalDebug" / "LATEST_BAG.txt"
TBC_MATRIX_RE = re.compile(
    r"IMU\.T_b_c1:\s*!!opencv-matrix\s+"
    r"rows:\s*4\s+cols:\s*4\s+dt:\s*f\s+data:\s*\[([^\]]+)\]",
    re.MULTILINE,
)
CDR_BASE = 4


def align(offset: int, alignment: int) -> int:
    return CDR_BASE + ((offset - CDR_BASE + (alignment - 1)) & ~(alignment - 1))


def read_u32(data: bytes, offset: int) -> tuple[int, int]:
    return struct.unpack_from("<I", data, offset)[0], offset + 4


def read_i32(data: bytes, offset: int) -> tuple[int, int]:
    return struct.unpack_from("<i", data, offset)[0], offset + 4


def read_f64(data: bytes, offset: int) -> tuple[float, int]:
    return struct.unpack_from("<d", data, offset)[0], offset + 8


def read_string(data: bytes, offset: int) -> tuple[str, int]:
    offset = align(offset, 4)
    length, offset = read_u32(data, offset)
    value = data[offset:offset + length - 1].decode(errors="replace")
    offset += length
    return value, align(offset, 4)


def parse_header(data: bytes) -> tuple[float, str, int]:
    offset = CDR_BASE
    sec, offset = read_i32(data, offset)
    nsec, offset = read_u32(data, offset)
    frame_id, offset = read_string(data, offset)
    return sec + nsec * 1e-9, frame_id, offset


def parse_imu(data: bytes) -> tuple[float, np.ndarray, np.ndarray]:
    stamp, _frame_id, offset = parse_header(data)
    offset = align(offset, 8)
    offset += 4 * 8  # orientation
    offset += 9 * 8  # orientation covariance
    gyro = []
    for _ in range(3):
        value, offset = read_f64(data, offset)
        gyro.append(value)
    offset += 9 * 8  # angular velocity covariance
    accel = []
    for _ in range(3):
        value, offset = read_f64(data, offset)
        accel.append(value)
    return stamp, np.array(gyro, dtype=float), np.array(accel, dtype=float)


def parse_image(data: bytes) -> tuple[float, np.ndarray]:
    stamp, _frame_id, offset = parse_header(data)
    height, offset = read_u32(data, offset)
    width, offset = read_u32(data, offset)
    encoding, offset = read_string(data, offset)
    offset += 1  # is_bigendian
    offset = align(offset, 4)
    _step, offset = read_u32(data, offset)
    length, offset = read_u32(data, offset)
    raw = np.frombuffer(data, dtype=np.uint8, count=length, offset=offset)
    if encoding == "mono8":
        return stamp, raw.reshape(height, width).copy()
    if encoding in ("yuv422_yuy2", "yuyv", "yuyv422"):
        return stamp, cv2.cvtColor(
            raw.reshape(height, width, 2), cv2.COLOR_YUV2GRAY_YUY2
        )
    raise ValueError(f"unsupported image encoding: {encoding}")


def resolve_bag_path(path: Path) -> Path:
    path = path.expanduser().resolve()
    if path.name == "metadata.yaml" or path.suffix == ".db3":
        return path.parent
    return path


def read_latest_bag(latest_file: Path) -> Path:
    raw = latest_file.expanduser().read_text(encoding="utf-8").strip()
    if not raw:
        raise ValueError(f"latest bag file is empty: {latest_file}")
    return resolve_bag_path(Path(raw).expanduser())


def find_db3(bag_path: Path) -> Path:
    if bag_path.suffix == ".db3":
        return bag_path
    dbs = sorted(bag_path.glob("*.db3"))
    if not dbs:
        raise FileNotFoundError(f"no sqlite3 .db3 file found under {bag_path}")
    return dbs[0]


def topic_id(connection: sqlite3.Connection, name: str) -> int:
    row = connection.execute(
        "select id from topics where name = ? limit 1", (name,)
    ).fetchone()
    if row is None:
        raise KeyError(f"topic not found in bag: {name}")
    return int(row[0])


def load_tbc_rotation(settings_file: Path) -> np.ndarray:
    text = settings_file.read_text(encoding="utf-8")
    match = TBC_MATRIX_RE.search(text)
    if not match:
        raise ValueError(f"IMU.T_b_c1 not found in {settings_file}")
    values = [float(item) for item in re.findall(r"[-+0-9.eE]+", match.group(1))]
    if len(values) != 16:
        raise ValueError(f"IMU.T_b_c1 has {len(values)} values, expected 16")
    return np.array(values, dtype=float).reshape(4, 4)[:3, :3]


def corrcoef(left: np.ndarray, right: np.ndarray, mask: np.ndarray) -> float | None:
    if int(mask.sum()) < 5:
        return None
    return float(np.corrcoef(left[mask], right[mask])[0, 1])


def collect_motion_rows(
    db_path: Path,
    image_topic: str,
    imu_topic: str,
    start_offset: float,
    duration: float,
    min_tracks: int,
) -> tuple[list[dict[str, Any]], int, int]:
    connection = sqlite3.connect(db_path)
    image_topic_id = topic_id(connection, image_topic)
    imu_topic_id = topic_id(connection, imu_topic)
    first_blob = connection.execute(
        "select data from messages where topic_id = ? order by timestamp limit 1",
        (image_topic_id,),
    ).fetchone()[0]
    first_time, _ = parse_image(first_blob)
    start = first_time + start_offset
    end = start + duration if duration > 0.0 else float("inf")

    imu_rows = []
    for (data,) in connection.execute(
        "select data from messages where topic_id = ? order by timestamp",
        (imu_topic_id,),
    ):
        stamp, gyro, accel = parse_imu(data)
        if stamp < start - 0.2:
            continue
        if stamp > end + 0.2:
            break
        imu_rows.append((stamp, gyro, accel))
    imu_t = np.array([row[0] for row in imu_rows])
    imu_g = np.array([row[1] for row in imu_rows])

    motion_rows: list[dict[str, Any]] = []
    image_count = 0
    last_stamp: float | None = None
    last_image: np.ndarray | None = None
    for (data,) in connection.execute(
        "select data from messages where topic_id = ? order by timestamp",
        (image_topic_id,),
    ):
        stamp, image = parse_image(data)
        if stamp < start:
            continue
        if stamp > end:
            break
        image_count += 1
        if last_image is not None and last_stamp is not None:
            dt = stamp - last_stamp
            if 0.05 < dt < 0.2:
                p0 = cv2.goodFeaturesToTrack(
                    last_image,
                    maxCorners=800,
                    qualityLevel=0.01,
                    minDistance=8,
                    blockSize=7,
                )
                if p0 is not None and len(p0) >= min_tracks:
                    p1, status, _err = cv2.calcOpticalFlowPyrLK(
                        last_image, image, p0, None, winSize=(21, 21), maxLevel=3
                    )
                    good = status.reshape(-1) == 1
                    prev = p0.reshape(-1, 2)[good]
                    curr = p1.reshape(-1, 2)[good]
                    if len(prev) >= min_tracks:
                        disp = curr - prev
                        dx_rate = float(np.median(disp[:, 0])) / dt
                        dy_rate = float(np.median(disp[:, 1])) / dt
                        rot_rate = None
                        affine, _inliers = cv2.estimateAffinePartial2D(
                            prev,
                            curr,
                            method=cv2.RANSAC,
                            ransacReprojThreshold=3.0,
                            maxIters=500,
                        )
                        if affine is not None:
                            rot_rate = math.atan2(affine[1, 0], affine[0, 0]) / dt
                        lo = np.searchsorted(imu_t, last_stamp, "left")
                        hi = np.searchsorted(imu_t, stamp, "right")
                        if hi > lo:
                            gyro = imu_g[lo:hi].mean(axis=0)
                            motion_rows.append({
                                "dx_px_s": dx_rate,
                                "dy_px_s": dy_rate,
                                "rot_rad_s": rot_rate,
                                "gyro_raw": gyro.tolist(),
                                "tracks": int(len(prev)),
                            })
        last_stamp = stamp
        last_image = image

    return motion_rows, image_count, len(imu_rows)


def rank_candidates(rows: list[dict[str, Any]], r_bc: np.ndarray, min_tracks: int) -> list[dict[str, Any]]:
    dx = np.array([row["dx_px_s"] for row in rows], dtype=float)
    dy = np.array([row["dy_px_s"] for row in rows], dtype=float)
    rot = np.array([
        row["rot_rad_s"] if row["rot_rad_s"] is not None else np.nan
        for row in rows
    ], dtype=float)
    raw_gyro = np.array([row["gyro_raw"] for row in rows], dtype=float)
    tracks = np.array([row["tracks"] for row in rows], dtype=float)
    base_mask = tracks >= min_tracks

    axes = {"x": 0, "y": 1, "z": 2}
    ranked = []
    for order in ("xyz", "xzy", "yxz", "yzx", "zxy", "zyx"):
        axis_indices = [axes[axis] for axis in order]
        for signs in itertools.product((-1.0, 1.0), repeat=3):
            body = raw_gyro[:, axis_indices] * np.array(signs)
            # T_b_c1 maps camera vectors to the body/IMU frame, so transpose it
            # to compare body-frame gyro candidates with image-frame motion.
            camera = (r_bc.T @ body.T).T
            c_dx_cy = corrcoef(dx, camera[:, 1], base_mask)
            c_dy_cx = corrcoef(dy, camera[:, 0], base_mask)
            c_rot_cz = corrcoef(rot, camera[:, 2], base_mask & np.isfinite(rot))
            values = [value for value in (c_dx_cy, c_dy_cx, c_rot_cz) if value is not None]
            score = float(sum(abs(value) for value in values))
            ranked.append({
                "gyro_axis_order": order,
                "gyro_scale": signs,
                "score_abs_corr_sum": score,
                "corr_dx_vs_camera_gyro_y": c_dx_cy,
                "corr_dy_vs_camera_gyro_x": c_dy_cx,
                "corr_rot_vs_camera_gyro_z": c_rot_cz,
            })
    return sorted(ranked, key=lambda item: item["score_abs_corr_sum"], reverse=True)


def render_markdown(result: dict[str, Any], limit: int) -> str:
    def fmt(value: float | None) -> str:
        return f"{value:+.3f}" if value is not None else "n/a"

    lines = [
        "# IMU Visual Correlation",
        "",
        f"- bag_path: `{result['bag_path']}`",
        f"- image_topic: `{result['image_topic']}`",
        f"- imu_topic: `{result['imu_topic']}`",
        f"- window_sec_from_first_image: {result['start_offset']}-{result['start_offset'] + result['duration']}",
        f"- image_count: {result['image_count']}",
        f"- image_pairs_used: {result['image_pairs_used']}",
        f"- imu_samples: {result['imu_samples']}",
        "",
        "## Raw Axis Correlation",
        "",
        "| Visual motion | gyro_x | gyro_y | gyro_z |",
        "| --- | ---: | ---: | ---: |",
    ]
    for visual, row in result["raw_correlations"].items():
        lines.append(
            f"| `{visual}` | {fmt(row['gyro_x'])} | {fmt(row['gyro_y'])} | {fmt(row['gyro_z'])} |"
        )
    lines.extend([
        "",
        "## Candidate Ranking",
        "",
        "| Rank | gyro_axis_order | gyro_scale_xyz | Score | dx~cam_gyro_y | dy~cam_gyro_x | rot~cam_gyro_z |",
        "| ---: | --- | --- | ---: | ---: | ---: | ---: |",
    ])
    for index, item in enumerate(result["candidate_ranking"][:limit], start=1):
        scale = ",".join(f"{value:+.0f}" for value in item["gyro_scale"])
        lines.append(
            f"| {index} | `{item['gyro_axis_order']}` | `{scale}` | "
            f"{item['score_abs_corr_sum']:.3f} | "
            f"{fmt(item['corr_dx_vs_camera_gyro_y'])} | "
            f"{fmt(item['corr_dy_vs_camera_gyro_x'])} | "
            f"{fmt(item['corr_rot_vs_camera_gyro_z'])} |"
        )
    lines.extend([
        "",
        "## Interpretation",
        "",
        "This is a host-side heuristic. It ranks IMU gyro frame hypotheses against image motion, but replay metrics remain the acceptance gate.",
    ])
    return "\n".join(lines) + "\n"


def analyze(args: argparse.Namespace) -> dict[str, Any]:
    bag_path = resolve_bag_path(args.bag_path) if args.bag_path else read_latest_bag(args.latest_file)
    db_path = find_db3(bag_path)
    settings_file = args.settings_file.expanduser().resolve()
    r_bc = load_tbc_rotation(settings_file)
    rows, image_count, imu_samples = collect_motion_rows(
        db_path,
        args.image_topic,
        args.imu_topic,
        args.start_offset,
        args.duration,
        args.min_tracks,
    )

    raw_gyro = np.array([row["gyro_raw"] for row in rows], dtype=float)
    tracks = np.array([row["tracks"] for row in rows], dtype=float)
    mask = tracks >= args.min_tracks
    raw_correlations = {}
    for visual in ("dx_px_s", "dy_px_s", "rot_rad_s"):
        values = np.array([
            row[visual] if row[visual] is not None else np.nan
            for row in rows
        ], dtype=float)
        raw_correlations[visual] = {}
        for axis, name in enumerate(("gyro_x", "gyro_y", "gyro_z")):
            raw_correlations[visual][name] = corrcoef(
                values, raw_gyro[:, axis], mask & np.isfinite(values)
            )

    return {
        "bag_path": str(bag_path),
        "db_path": str(db_path),
        "settings_file": str(settings_file),
        "image_topic": args.image_topic,
        "imu_topic": args.imu_topic,
        "start_offset": args.start_offset,
        "duration": args.duration,
        "min_tracks": args.min_tracks,
        "image_count": image_count,
        "image_pairs_used": len(rows),
        "imu_samples": imu_samples,
        "raw_correlations": raw_correlations,
        "candidate_ranking": rank_candidates(rows, r_bc, args.min_tracks),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag-path", type=Path)
    parser.add_argument("--latest-file", type=Path, default=DEFAULT_LATEST)
    parser.add_argument(
        "--settings-file",
        type=Path,
        default=PACKAGE_ROOT / "config" / "Monocular-Inertial" / "rmoncam_h30.yaml",
    )
    parser.add_argument("--image-topic", default="/sync/image_raw")
    parser.add_argument("--imu-topic", default="/sync/imu")
    parser.add_argument("--start-offset", type=float, default=70.0)
    parser.add_argument("--duration", type=float, default=70.0)
    parser.add_argument("--min-tracks", type=int, default=60)
    parser.add_argument("--top", type=int, default=12)
    parser.add_argument(
        "--output-root",
        type=Path,
        default=PACKAGE_ROOT / "debug_runs",
    )
    args = parser.parse_args()

    result = analyze(args)
    output_dir = args.output_root.expanduser().resolve() / Path(result["bag_path"]).name
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "imu_visual_correlation.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (output_dir / "imu_visual_correlation.md").write_text(
        render_markdown(result, args.top),
        encoding="utf-8",
    )
    print(output_dir / "imu_visual_correlation.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
