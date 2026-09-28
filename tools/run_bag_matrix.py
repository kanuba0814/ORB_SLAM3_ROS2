#!/usr/bin/env python3
# flake8: noqa
"""Run repeatable H30 bag replay experiments for ORB-SLAM3."""

from __future__ import annotations

import argparse
import json
import os
import re
import signal
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from parse_orb_log import parse_orb_log


SOURCE_PACKAGE_ROOT = Path('/home/gxxl/slam_ws_v2/extra_ws/src/ORB_SLAM3_ROS2')
PACKAGE_ROOT = (
    SOURCE_PACKAGE_ROOT
    if SOURCE_PACKAGE_ROOT.exists()
    else Path(__file__).resolve().parents[1]
)
WORKSPACE_ROOT = PACKAGE_ROOT.parents[2]
DEFAULT_LATEST = WORKSPACE_ROOT / "bags" / "goalDebug" / "LATEST_BAG.txt"
EXPERIMENT_PRESETS = {
    "high_fps": (
        "vi_fps_60",
        "vi_fps_90",
        "vi_nfeatures_800_fps_60",
        "vi_nfeatures_1500_fps_60",
        "vi_nfeatures_1500_fps_90",
        "vi_nfeatures_1500_insert_kfs_fps_60",
    ),
    "tbc_body": (
        "vi_nfeatures_1500_tbc_body_rx180",
        "vi_nfeatures_1500_tbc_body_ry180",
        "vi_nfeatures_1500_tbc_body_rz180",
    ),
    "tracking_thresholds": (
        "vi_nfeatures_800_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_tracking_inliers_25_25_10",
        "vi_nfeatures_1500_fps90_tracking_inliers_30_30_10",
    ),
    "imu_init_window_fps90": (
        "vi_nfeatures_1500_fps90_init3_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_init5_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_init3_fast10_3_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_init5_no_ba2_reset_tracking_inliers_30_30_10",
    ),
    "imu_init_window_default": (
        "vi_nfeatures_1500_fps10_init3",
        "vi_nfeatures_1500_fps10_init5",
        "vi_nfeatures_1500_fps90_init3",
        "vi_nfeatures_1500_fps10_init3_fast10_3",
    ),
    "sync_offset_sweep_fps90": (
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_m50ms",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_m30ms",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_m15ms",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_m10ms",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p30ms",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p50ms",
    ),
    "sync_offset_fine_fps90": (
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_zero",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p4ms",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p8ms",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p12ms",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p15ms",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p20ms",
    ),
    "post_init_recovery_fps90": (
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms_reloc0_no_ba2_reset",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_reloc0_no_ba2_reset",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_reloc10_no_ba2_reset",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms_reloc0_no_ba2_reset_insert_kfs",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_reloc0_no_ba2_reset_insert_kfs",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms_reloc0_no_ba2_reset_no_lm_small_motion_reset",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_reloc0_no_ba2_reset_no_lm_small_motion_reset",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms_reloc0_no_ba2_reset_insert_kfs_no_lm_small_motion_reset",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_reloc0_no_ba2_reset_insert_kfs_no_lm_small_motion_reset",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms_reloc0_no_ba2_reset_post5",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_reloc0_no_ba2_reset_post5",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_reloc0_no_ba2_reset_post2",
    ),
    "post_init_lm_small_motion_fps90": (
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms_reloc0_no_ba2_reset_no_lm_small_motion_reset",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_reloc0_no_ba2_reset_no_lm_small_motion_reset",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms_reloc0_no_ba2_reset_insert_kfs_no_lm_small_motion_reset",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_reloc0_no_ba2_reset_insert_kfs_no_lm_small_motion_reset",
    ),
    "post_init_lm_only_fps90": (
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms_no_lm_small_motion_reset",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_no_lm_small_motion_reset",
    ),
    "post_init_window_offset_fps90": (
        "vi_nfeatures_1500_fps90_init5_fast10_3_sync_offset_p6ms",
        "vi_nfeatures_1500_fps90_init5_fast10_3_sync_offset_p10ms",
        "vi_nfeatures_1500_fps90_init10_fast10_3_sync_offset_p6ms",
        "vi_nfeatures_1500_fps90_init10_fast10_3_sync_offset_p10ms",
    ),
    "best_vi_p10_fps90": (
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms",
    ),
    "best_vi_zero_fps90": (
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_zero",
    ),
    "best_vi_zero_tbc_fps90": (
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_zero_tbc_inverse",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_zero_tbc_body_rx180",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_zero_tbc_body_ry180",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_zero_tbc_body_rz180",
    ),
    "best_vi_zero_noise_fps90": (
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_zero_imu_noise_x20",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_zero_imu_noise_x50",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_zero_imu_noise_x100",
    ),
    "best_vi_zero_scale_gate_fps90": (
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_zero_min_scale_0p2",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_zero_min_scale_0p4",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_zero_min_scale_0p45",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_zero_min_scale_0p5",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_zero_min_scale_0p55",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_zero_min_scale_0p6",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_zero_min_scale_1p0",
    ),
    "best_vi_scale_gate_recovery_fps90": (
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms_min_scale_0p5",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_min_scale_0p5",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_zero_min_scale_0p5_visual_pre_ba2",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms_min_scale_0p5_visual_pre_ba2",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_min_scale_0p5_visual_pre_ba2",
    ),
    "best_vi_raw_m15_fps90": (
        "vi_raw_nfeatures_1500_fps90_init3_fast10_3_offset_m15ms_imu200",
    ),
    "best_vi_p10_gyro_sign_fps90": (
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms_gyro_flip_xyz",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_gyro_flip_xyz",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_acc_gyro_flip_xyz",
    ),
    "best_vi_visual_pre_ba2_fps90": (
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms_visual_pre_ba2",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_visual_pre_ba2",
    ),
    "best_vi_visual_pre_ba2_lm_policy_fps90": (
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms_visual_pre_ba2_no_lm_small_motion_reset",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_visual_pre_ba2_no_lm_small_motion_reset",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms_visual_pre_ba2_no_lm_no_ba2_reset",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_visual_pre_ba2_no_lm_no_ba2_reset",
    ),
    "reset_policy": (
        "vi_nfeatures_1500_fps90_reloc10_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_no_ba2_reset_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_reloc10_no_ba2_reset_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps60_reloc10_no_ba2_reset_tracking_inliers_30_30_10",
    ),
    "imu_noise_fps90": (
        "vi_nfeatures_1500_fps90_imu_noise_x20_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_imu_noise_x50_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_imu_noise_x100_tracking_inliers_30_30_10",
    ),
    "imu_noise_reset_policy_fps90": (
        "vi_nfeatures_1500_fps90_imu_noise_x50_no_ba2_reset_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_imu_noise_x50_reloc10_no_ba2_reset_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_imu_noise_x100_no_ba2_reset_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_imu_noise_x200_no_ba2_reset_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_imu_noise_x500_no_ba2_reset_tracking_inliers_30_30_10",
    ),
    "imu_signs_fps90": (
        "vi_nfeatures_1500_fps90_acc_flip_xyz_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_gyro_flip_xyz_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_acc_gyro_flip_xyz_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_acc_flip_z_gyro_flip_z_tracking_inliers_30_30_10",
    ),
    "imu_gyro_ranked_fps90": (
        "vi_nfeatures_1500_fps90_gyro_flip_xyz_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_gyro_flip_xz_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_gyro_flip_y_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_gyro_flip_xy_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_gyro_flip_z_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_gyro_flip_x_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_gyro_flip_yz_tracking_inliers_30_30_10",
    ),
    "imu_gyro_axis_fps90": (
        "vi_nfeatures_1500_fps90_gyro_flip_xz_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_gyro_flip_y_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_gyro_flip_xy_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_gyro_flip_z_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_gyro_flip_x_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_gyro_flip_yz_tracking_inliers_30_30_10",
    ),
    "imu_gyro_order_fps90": (
        "vi_nfeatures_1500_fps90_gyro_order_xzy_flip_xyz_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_gyro_order_yxz_flip_xyz_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_gyro_order_yzx_flip_xyz_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_gyro_order_zxy_flip_xyz_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_gyro_order_zyx_flip_xyz_tracking_inliers_30_30_10",
    ),
    "tbc_gyro_ranked_fps90": (
        "vi_nfeatures_1500_fps90_tbc_inverse_gyro_flip_xyz_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_tbc_body_rx180_gyro_flip_xyz_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_tbc_body_ry180_gyro_flip_xyz_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_tbc_body_rz180_gyro_flip_xyz_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_tbc_inverse_gyro_flip_xz_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_tbc_body_rx180_gyro_flip_xz_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_tbc_body_ry180_gyro_flip_xz_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_tbc_body_rz180_gyro_flip_xz_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_tbc_inverse_gyro_flip_y_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_tbc_body_rx180_gyro_flip_y_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_tbc_body_ry180_gyro_flip_y_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_tbc_body_rz180_gyro_flip_y_tracking_inliers_30_30_10",
    ),
}
TBC_MATRIX_RE = re.compile(
    r"(IMU\.T_b_c1:\s*!!opencv-matrix\s+"
    r"rows:\s*4\s+cols:\s*4\s+dt:\s*f\s+data:\s*\[)([^\]]+)(\])",
    re.MULTILINE,
)
VI_GATE_KEYS = (
    "imu_initialized",
    "ok_ratio_gt_0_8",
    "reset_count_ok",
    "median_inliers_gt_50",
    "final_keyframes_gt_20",
    "insufficient_imu_near_zero",
    "image_to_last_imu_ms_0_12",
)


@dataclass
class Experiment:
    name: str
    sensor_type: str
    image_topic: str
    imu_topic: str
    settings_file: Path
    cam_imu_time_offset: float
    orb_time_offset_mode: str = "legacy_warn"
    extra: dict[str, Any] = field(default_factory=dict)


def _resolve_bag_path(path: Path) -> Path:
    path = path.expanduser().resolve()
    if path.name == "metadata.yaml" or path.suffix == ".db3":
        return path.parent
    return path


def read_latest_bag(latest_file: Path) -> Path:
    if not latest_file.exists():
        raise FileNotFoundError(
            f"latest bag file does not exist: {latest_file}. "
            "Create it with the main H30 bag directory path."
        )
    raw = latest_file.read_text(encoding="utf-8").strip()
    if not raw:
        raise ValueError(f"latest bag file is empty: {latest_file}")
    return _resolve_bag_path(Path(os.path.expanduser(raw)))


def find_smoke_bags(goal_debug_dir: Path) -> list[Path]:
    if not goal_debug_dir.exists():
        return []
    return sorted(
        path for path in goal_debug_dir.iterdir()
        if path.is_dir() and path.name.startswith("h30_sync_raw_smoke_")
    )


def _replace_yaml_scalar(text: str, key: str, value: Any) -> str:
    pattern = re.compile(rf"^({re.escape(key)}:\s*)([^\n#]+)(.*)$", re.MULTILINE)
    replacement = rf"\g<1>{value}\3"
    new_text, count = pattern.subn(replacement, text, count=1)
    if count == 0:
        new_text += f"\n{key}: {value}\n"
    return new_text


def _read_yaml_scalar(text: str, key: str) -> float:
    pattern = re.compile(rf"^{re.escape(key)}:\s*([-+0-9.eE]+)", re.MULTILINE)
    match = pattern.search(text)
    if not match:
        raise KeyError(key)
    return float(match.group(1))


def write_variant(base_file: Path, output_file: Path, updates: dict[str, Any]) -> Path:
    text = base_file.read_text(encoding="utf-8")
    for key, value in updates.items():
        text = _replace_yaml_scalar(text, key, value)
    output_file.parent.mkdir(parents=True, exist_ok=True)
    output_file.write_text(text, encoding="utf-8")
    return output_file


def write_noise_variant(
    base_file: Path,
    output_file: Path,
    scale: float,
    updates: dict[str, Any] | None = None,
) -> Path:
    text = base_file.read_text(encoding="utf-8")
    # The checked-in H30 file currently uses x10 datasheet noise.  Convert to
    # the requested datasheet multiplier without touching the source YAML.
    factor = scale / 10.0
    for key in (
        "IMU.NoiseGyro",
        "IMU.NoiseAcc",
        "IMU.GyroWalk",
        "IMU.AccWalk",
    ):
        value = _read_yaml_scalar(text, key) * factor
        text = _replace_yaml_scalar(text, key, f"{value:.8e}")
    if updates:
        for key, value in updates.items():
            text = _replace_yaml_scalar(text, key, value)
    output_file.parent.mkdir(parents=True, exist_ok=True)
    output_file.write_text(text, encoding="utf-8")
    return output_file


def _format_tbc_rows(rows: list[list[float]]) -> str:
    return (
        f"{rows[0][0]: .8f}, {rows[0][1]: .8f}, "
        f"{rows[0][2]: .8f}, {rows[0][3]: .8f},\n"
        f"          {rows[1][0]: .8f}, {rows[1][1]: .8f}, "
        f"{rows[1][2]: .8f}, {rows[1][3]: .8f},\n"
        f"          {rows[2][0]: .8f}, {rows[2][1]: .8f}, "
        f"{rows[2][2]: .8f}, {rows[2][3]: .8f},\n"
        f"          {rows[3][0]: .1f},         {rows[3][1]: .1f},"
        f"         {rows[3][2]: .1f},         {rows[3][3]: .1f}"
    )


def _read_tbc_rows(text: str, base_file: Path) -> list[list[float]]:
    match = TBC_MATRIX_RE.search(text)
    if not match:
        raise ValueError(f"IMU.T_b_c1 matrix not found in {base_file}")

    values = [float(item) for item in re.findall(r"[-+0-9.eE]+", match.group(2))]
    if len(values) != 16:
        raise ValueError(f"IMU.T_b_c1 matrix has {len(values)} values, expected 16")
    return [values[index:index + 4] for index in range(0, 16, 4)]


def _replace_tbc_rows(text: str, rows: list[list[float]]) -> str:
    matrix_text = _format_tbc_rows(rows)
    return TBC_MATRIX_RE.sub(rf"\g<1>{matrix_text}\3", text, count=1)


def _matmul4(lhs: list[list[float]], rhs: list[list[float]]) -> list[list[float]]:
    return [
        [
            sum(lhs[row][inner] * rhs[inner][col] for inner in range(4))
            for col in range(4)
        ]
        for row in range(4)
    ]


def write_tbc_inverse_variant(
    base_file: Path,
    output_file: Path,
    updates: dict[str, Any] | None = None,
) -> Path:
    text = base_file.read_text(encoding="utf-8")
    if updates:
        for key, value in updates.items():
            text = _replace_yaml_scalar(text, key, value)

    rows = _read_tbc_rows(text, base_file)
    rotation = [row[:3] for row in rows[:3]]
    translation = [rows[index][3] for index in range(3)]
    rotation_t = [[rotation[row][col] for row in range(3)] for col in range(3)]
    inv_translation = [
        -sum(rotation_t[row][col] * translation[col] for col in range(3))
        for row in range(3)
    ]
    inverse_rows = [
        rotation_t[row] + [inv_translation[row]]
        for row in range(3)
    ] + [[0.0, 0.0, 0.0, 1.0]]

    text = _replace_tbc_rows(text, inverse_rows)
    output_file.parent.mkdir(parents=True, exist_ok=True)
    output_file.write_text(text, encoding="utf-8")
    return output_file


def write_tbc_body_rotation_variant(
    base_file: Path,
    output_file: Path,
    body_rotation: list[list[float]],
    updates: dict[str, Any] | None = None,
) -> Path:
    text = base_file.read_text(encoding="utf-8")
    if updates:
        for key, value in updates.items():
            text = _replace_yaml_scalar(text, key, value)

    rows = _read_tbc_rows(text, base_file)
    body_transform = [
        body_rotation[0] + [0.0],
        body_rotation[1] + [0.0],
        body_rotation[2] + [0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    text = _replace_tbc_rows(text, _matmul4(body_transform, rows))
    output_file.parent.mkdir(parents=True, exist_ok=True)
    output_file.write_text(text, encoding="utf-8")
    return output_file


def first_batch() -> list[Experiment]:
    mono = PACKAGE_ROOT / "config" / "Monocular" / "rmoncam.yaml"
    vi = PACKAGE_ROOT / "config" / "Monocular-Inertial" / "rmoncam_h30.yaml"
    return [
        Experiment(
            name="mono_sync_image",
            sensor_type="monocular",
            image_topic="/sync/image_raw",
            imu_topic="/sync/imu",
            settings_file=mono,
            cam_imu_time_offset=0.0,
            orb_time_offset_mode="zero_for_sync",
        ),
        Experiment(
            name="vi_sync_offset_zero",
            sensor_type="imu-monocular",
            image_topic="/sync/image_raw",
            imu_topic="/sync/imu",
            settings_file=vi,
            cam_imu_time_offset=0.0,
            orb_time_offset_mode="zero_for_sync",
        ),
        Experiment(
            name="vi_sync_offset_minus_15ms",
            sensor_type="imu-monocular",
            image_topic="/sync/image_raw",
            imu_topic="/sync/imu",
            settings_file=vi,
            cam_imu_time_offset=-0.01574,
            orb_time_offset_mode="legacy_warn",
        ),
        Experiment(
            name="vi_raw_offset_minus_15ms",
            sensor_type="imu-monocular",
            image_topic="/image_raw",
            imu_topic="/imu/data_raw",
            settings_file=vi,
            cam_imu_time_offset=-0.01574,
            orb_time_offset_mode="legacy_warn",
        ),
    ]


def first_batch_names() -> list[str]:
    return [experiment.name for experiment in first_batch()]


def second_batch_names() -> list[str]:
    names = []
    names.extend(f"vi_nfeatures_{nfeatures}" for nfeatures in (800, 1200, 1500))
    names.extend(f"vi_fast_{ini}_{min_fast}" for ini, min_fast in ((20, 7), (15, 5), (10, 3)))
    names.extend(f"vi_rgb_{rgb}" for rgb in (0, 1))
    names.append("vi_fps_10")
    names.extend(f"vi_fps_{fps}" for fps in (60, 90))
    names.extend(f"vi_nfeatures_{nfeatures}_fps_10" for nfeatures in (800, 1500))
    names.extend(
        f"vi_nfeatures_{nfeatures}_fps_{fps}"
        for nfeatures, fps in ((800, 60), (1500, 60), (1500, 90))
    )
    names.append("vi_nfeatures_1500_insert_kfs_fps_60")
    names.append("vi_tbc_inverse")
    names.extend(f"vi_nfeatures_{nfeatures}_tbc_inverse" for nfeatures in (800, 1500))
    names.extend(
        f"vi_nfeatures_1500_tbc_body_{name}"
        for name in ("rx180", "ry180", "rz180")
    )
    names.extend((
        "vi_nfeatures_800_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_tracking_inliers_25_25_10",
        "vi_nfeatures_1500_fps90_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps10_init3",
        "vi_nfeatures_1500_fps10_init5",
        "vi_nfeatures_1500_fps90_init3",
        "vi_nfeatures_1500_fps10_init3_fast10_3",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_m50ms",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_m30ms",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_m15ms",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_m10ms",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_zero",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p4ms",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p8ms",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p12ms",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p15ms",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p20ms",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p30ms",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p50ms",
        "vi_nfeatures_1500_fps90_init5_fast10_3_sync_offset_p6ms",
        "vi_nfeatures_1500_fps90_init5_fast10_3_sync_offset_p10ms",
        "vi_nfeatures_1500_fps90_init10_fast10_3_sync_offset_p6ms",
        "vi_nfeatures_1500_fps90_init10_fast10_3_sync_offset_p10ms",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms_gyro_flip_xyz",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_gyro_flip_xyz",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_acc_gyro_flip_xyz",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_zero_tbc_inverse",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_zero_tbc_body_rx180",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_zero_tbc_body_ry180",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_zero_tbc_body_rz180",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_zero_imu_noise_x20",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_zero_imu_noise_x50",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_zero_imu_noise_x100",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_zero_min_scale_0p2",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_zero_min_scale_0p4",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_zero_min_scale_0p45",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_zero_min_scale_0p5",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_zero_min_scale_0p55",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_zero_min_scale_0p6",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_zero_min_scale_1p0",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms_min_scale_0p5",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_min_scale_0p5",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_zero_min_scale_0p5_visual_pre_ba2",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms_min_scale_0p5_visual_pre_ba2",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_min_scale_0p5_visual_pre_ba2",
        "vi_raw_nfeatures_1500_fps90_init3_fast10_3_offset_m15ms_imu200",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms_visual_pre_ba2",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_visual_pre_ba2",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms_visual_pre_ba2_no_lm_small_motion_reset",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_visual_pre_ba2_no_lm_small_motion_reset",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms_visual_pre_ba2_no_lm_no_ba2_reset",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_visual_pre_ba2_no_lm_no_ba2_reset",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms_no_lm_small_motion_reset",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_no_lm_small_motion_reset",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms_reloc0_no_ba2_reset",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_reloc0_no_ba2_reset",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_reloc10_no_ba2_reset",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms_reloc0_no_ba2_reset_insert_kfs",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_reloc0_no_ba2_reset_insert_kfs",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms_reloc0_no_ba2_reset_no_lm_small_motion_reset",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_reloc0_no_ba2_reset_no_lm_small_motion_reset",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms_reloc0_no_ba2_reset_insert_kfs_no_lm_small_motion_reset",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_reloc0_no_ba2_reset_insert_kfs_no_lm_small_motion_reset",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms_reloc0_no_ba2_reset_post5",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_reloc0_no_ba2_reset_post5",
        "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_reloc0_no_ba2_reset_post2",
        "vi_nfeatures_1500_fps90_init3_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_init5_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_init3_fast10_3_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_init5_no_ba2_reset_tracking_inliers_30_30_10",
    ))
    names.extend((
        "vi_nfeatures_1500_fps90_reloc10_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_no_ba2_reset_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_reloc10_no_ba2_reset_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps60_reloc10_no_ba2_reset_tracking_inliers_30_30_10",
    ))
    names.extend(
        f"vi_nfeatures_1500_fps90_imu_noise_x{scale}_tracking_inliers_30_30_10"
        for scale in (20, 50, 100)
    )
    names.extend((
        "vi_nfeatures_1500_fps90_imu_noise_x50_no_ba2_reset_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_imu_noise_x50_reloc10_no_ba2_reset_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_imu_noise_x100_no_ba2_reset_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_imu_noise_x200_no_ba2_reset_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_imu_noise_x500_no_ba2_reset_tracking_inliers_30_30_10",
    ))
    names.extend((
        "vi_nfeatures_1500_fps90_acc_flip_xyz_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_gyro_flip_xyz_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_acc_gyro_flip_xyz_tracking_inliers_30_30_10",
        "vi_nfeatures_1500_fps90_acc_flip_z_gyro_flip_z_tracking_inliers_30_30_10",
    ))
    names.extend(
        f"vi_nfeatures_1500_fps90_gyro_flip_{axes}_tracking_inliers_30_30_10"
        for axes in ("xz", "y", "xy", "z", "x", "yz")
    )
    names.extend(
        f"vi_nfeatures_1500_fps90_gyro_order_{order}_flip_xyz_tracking_inliers_30_30_10"
        for order in ("xzy", "yxz", "yzx", "zxy", "zyx")
    )
    names.extend(
        f"vi_nfeatures_1500_fps90_{tbc}_gyro_flip_{axes}_tracking_inliers_30_30_10"
        for axes in ("xyz", "xz", "y")
        for tbc in ("tbc_inverse", "tbc_body_rx180", "tbc_body_ry180", "tbc_body_rz180")
    )
    names.append("vi_insert_kfs_when_lost_1")
    names.extend(
        f"vi_nfeatures_{nfeatures}_insert_kfs_when_lost_1"
        for nfeatures in (600, 700, 800, 900, 1000, 1500, 2000)
    )
    names.extend(f"vi_imu_noise_x{scale}" for scale in (5, 10, 20))
    return names


def experiment_names_for_phase(phase: str) -> list[str]:
    names = first_batch_names()
    if phase in ("second", "all"):
        if phase == "second":
            names = []
        names.extend(second_batch_names())
    return names


def second_batch(variant_dir: Path) -> list[Experiment]:
    base = PACKAGE_ROOT / "config" / "Monocular-Inertial" / "rmoncam_h30.yaml"
    experiments: list[Experiment] = []

    for nfeatures in (800, 1200, 1500):
        config = write_variant(
            base,
            variant_dir / f"rmoncam_h30_nfeatures_{nfeatures}.yaml",
            {"ORBextractor.nFeatures": nfeatures},
        )
        experiments.append(_variant_experiment(f"vi_nfeatures_{nfeatures}", config))

    for ini, min_fast in ((20, 7), (15, 5), (10, 3)):
        config = write_variant(
            base,
            variant_dir / f"rmoncam_h30_fast_{ini}_{min_fast}.yaml",
            {
                "ORBextractor.iniThFAST": ini,
                "ORBextractor.minThFAST": min_fast,
            },
        )
        experiments.append(_variant_experiment(f"vi_fast_{ini}_{min_fast}", config))

    for rgb in (0, 1):
        config = write_variant(
            base,
            variant_dir / f"rmoncam_h30_rgb_{rgb}.yaml",
            {"Camera.RGB": rgb},
        )
        experiments.append(_variant_experiment(f"vi_rgb_{rgb}", config))

    config = write_variant(
        base,
        variant_dir / "rmoncam_h30_fps_10.yaml",
        {"Camera.fps": 10},
    )
    experiments.append(_variant_experiment("vi_fps_10", config))

    for fps in (60, 90):
        config = write_variant(
            base,
            variant_dir / f"rmoncam_h30_fps_{fps}.yaml",
            {"Camera.fps": fps},
        )
        experiments.append(_variant_experiment(f"vi_fps_{fps}", config))

    for nfeatures in (800, 1500):
        config = write_variant(
            base,
            variant_dir / f"rmoncam_h30_nfeatures_{nfeatures}_fps_10.yaml",
            {
                "Camera.fps": 10,
                "ORBextractor.nFeatures": nfeatures,
            },
        )
        experiments.append(
            _variant_experiment(f"vi_nfeatures_{nfeatures}_fps_10", config)
        )

    for nfeatures, fps in ((800, 60), (1500, 60), (1500, 90)):
        config = write_variant(
            base,
            variant_dir / f"rmoncam_h30_nfeatures_{nfeatures}_fps_{fps}.yaml",
            {
                "Camera.fps": fps,
                "ORBextractor.nFeatures": nfeatures,
            },
        )
        experiments.append(
            _variant_experiment(f"vi_nfeatures_{nfeatures}_fps_{fps}", config)
        )

    config = write_variant(
        base,
        variant_dir / "rmoncam_h30_nfeatures_1500_insert_kfs_fps_60.yaml",
        {
            "Camera.fps": 60,
            "IMU.InsertKFsWhenLost": 1,
            "ORBextractor.nFeatures": 1500,
        },
    )
    experiments.append(
        _variant_experiment("vi_nfeatures_1500_insert_kfs_fps_60", config)
    )

    config = write_tbc_inverse_variant(
        base,
        variant_dir / "rmoncam_h30_tbc_inverse.yaml",
    )
    experiments.append(_variant_experiment("vi_tbc_inverse", config))

    for nfeatures in (800, 1500):
        config = write_tbc_inverse_variant(
            base,
            variant_dir / f"rmoncam_h30_nfeatures_{nfeatures}_tbc_inverse.yaml",
            {"ORBextractor.nFeatures": nfeatures},
        )
        experiments.append(
            _variant_experiment(f"vi_nfeatures_{nfeatures}_tbc_inverse", config)
        )

    tbc_body_rotations = {
        "rx180": [
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
        ],
        "ry180": [
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, -1.0],
        ],
        "rz180": [
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
    }
    for name, rotation in tbc_body_rotations.items():
        config = write_tbc_body_rotation_variant(
            base,
            variant_dir / f"rmoncam_h30_nfeatures_1500_tbc_body_{name}.yaml",
            rotation,
            {"ORBextractor.nFeatures": 1500},
        )
        experiments.append(
            _variant_experiment(f"vi_nfeatures_1500_tbc_body_{name}", config)
        )

    tracking_threshold_variants = [
        (
            "vi_nfeatures_800_tracking_inliers_30_30_10",
            {
                "ORBextractor.nFeatures": 800,
                "Tracking.MinInliersRecentReloc": 30,
                "Tracking.MinInliersImuMonoPreInit": 30,
                "Tracking.MinInliersImuMonoPostInit": 10,
                "Tracking.MinInliersVisual": 20,
            },
        ),
        (
            "vi_nfeatures_1500_tracking_inliers_30_30_10",
            {
                "ORBextractor.nFeatures": 1500,
                "Tracking.MinInliersRecentReloc": 30,
                "Tracking.MinInliersImuMonoPreInit": 30,
                "Tracking.MinInliersImuMonoPostInit": 10,
                "Tracking.MinInliersVisual": 20,
            },
        ),
        (
            "vi_nfeatures_1500_tracking_inliers_25_25_10",
            {
                "ORBextractor.nFeatures": 1500,
                "Tracking.MinInliersRecentReloc": 25,
                "Tracking.MinInliersImuMonoPreInit": 25,
                "Tracking.MinInliersImuMonoPostInit": 10,
                "Tracking.MinInliersVisual": 15,
            },
        ),
        (
            "vi_nfeatures_1500_fps90_tracking_inliers_30_30_10",
            {
                "Camera.fps": 90,
                "ORBextractor.nFeatures": 1500,
                "Tracking.MinInliersRecentReloc": 30,
                "Tracking.MinInliersImuMonoPreInit": 30,
                "Tracking.MinInliersImuMonoPostInit": 10,
                "Tracking.MinInliersVisual": 20,
            },
        ),
    ]
    for name, updates in tracking_threshold_variants:
        config = write_variant(
            base,
            variant_dir / f"rmoncam_h30_{name.removeprefix('vi_')}.yaml",
            updates,
        )
        experiments.append(_variant_experiment(name, config))

    init_window_default_variants = [
        (
            "vi_nfeatures_1500_fps10_init3",
            {"Camera.fps": 10, "Tracking.ImuMonoInitMaxTime": 3.0},
        ),
        (
            "vi_nfeatures_1500_fps10_init5",
            {"Camera.fps": 10, "Tracking.ImuMonoInitMaxTime": 5.0},
        ),
        (
            "vi_nfeatures_1500_fps90_init3",
            {"Camera.fps": 90, "Tracking.ImuMonoInitMaxTime": 3.0},
        ),
        (
            "vi_nfeatures_1500_fps10_init3_fast10_3",
            {
                "Camera.fps": 10,
                "Tracking.ImuMonoInitMaxTime": 3.0,
                "ORBextractor.iniThFAST": 10,
                "ORBextractor.minThFAST": 3,
            },
        ),
    ]
    for name, updates in init_window_default_variants:
        config = write_variant(
            base,
            variant_dir / f"rmoncam_h30_{name.removeprefix('vi_')}.yaml",
            {"ORBextractor.nFeatures": 1500, **updates},
        )
        experiments.append(_variant_experiment(name, config))

    offset_sweep_config = write_variant(
        base,
        variant_dir / "rmoncam_h30_nfeatures_1500_fps90_init3_fast10_3.yaml",
        {
            "Camera.fps": 90,
            "ORBextractor.nFeatures": 1500,
            "ORBextractor.iniThFAST": 10,
            "ORBextractor.minThFAST": 3,
            "Tracking.ImuMonoInitMaxTime": 3.0,
            "Tracking.MinInliersRecentReloc": 30,
            "Tracking.MinInliersImuMonoPreInit": 30,
            "Tracking.MinInliersImuMonoPostInit": 10,
            "Tracking.MinInliersVisual": 20,
        },
    )
    for suffix, offset in (
        ("m50ms", -0.050),
        ("m30ms", -0.030),
        ("m15ms", -0.01574),
        ("m10ms", -0.010),
        ("zero", 0.0),
        ("p4ms", 0.004),
        ("p6ms", 0.006),
        ("p8ms", 0.008),
        ("p10ms", 0.010),
        ("p12ms", 0.012),
        ("p15ms", 0.015),
        ("p20ms", 0.020),
        ("p30ms", 0.030),
        ("p50ms", 0.050),
    ):
        experiments.append(_variant_experiment(
            f"vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_{suffix}",
            offset_sweep_config,
            cam_imu_time_offset=offset,
            orb_time_offset_mode="zero_for_sync" if suffix == "zero" else "legacy_warn",
        ))

    for name, offset, extra in (
        (
            "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms_gyro_flip_xyz",
            0.006,
            {
                "imu_gyro_scale_x": -1.0,
                "imu_gyro_scale_y": -1.0,
                "imu_gyro_scale_z": -1.0,
            },
        ),
        (
            "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_gyro_flip_xyz",
            0.010,
            {
                "imu_gyro_scale_x": -1.0,
                "imu_gyro_scale_y": -1.0,
                "imu_gyro_scale_z": -1.0,
            },
        ),
        (
            "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_acc_gyro_flip_xyz",
            0.010,
            {
                "imu_accel_scale_x": -1.0,
                "imu_accel_scale_y": -1.0,
                "imu_accel_scale_z": -1.0,
                "imu_gyro_scale_x": -1.0,
                "imu_gyro_scale_y": -1.0,
                "imu_gyro_scale_z": -1.0,
            },
        ),
    ):
        experiments.append(_variant_experiment(
            name,
            offset_sweep_config,
            extra=extra,
            cam_imu_time_offset=offset,
            orb_time_offset_mode="legacy_warn",
        ))

    tbc_zero_configs: dict[str, Path] = {}
    tbc_zero_configs["tbc_inverse"] = write_tbc_inverse_variant(
        base,
        variant_dir / (
            "rmoncam_h30_nfeatures_1500_fps90_init3_fast10_3"
            "_sync_offset_zero_tbc_inverse.yaml"
        ),
        {
            "Camera.fps": 90,
            "ORBextractor.nFeatures": 1500,
            "ORBextractor.iniThFAST": 10,
            "ORBextractor.minThFAST": 3,
            "Tracking.ImuMonoInitMaxTime": 3.0,
            "Tracking.MinInliersRecentReloc": 30,
            "Tracking.MinInliersImuMonoPreInit": 30,
            "Tracking.MinInliersImuMonoPostInit": 10,
            "Tracking.MinInliersVisual": 20,
        },
    )
    for tbc_name, rotation in tbc_body_rotations.items():
        tbc_zero_configs[f"tbc_body_{tbc_name}"] = write_tbc_body_rotation_variant(
            base,
            variant_dir / (
                f"rmoncam_h30_nfeatures_1500_fps90_init3_fast10_3"
                f"_sync_offset_zero_tbc_body_{tbc_name}.yaml"
            ),
            rotation,
            {
                "Camera.fps": 90,
                "ORBextractor.nFeatures": 1500,
                "ORBextractor.iniThFAST": 10,
                "ORBextractor.minThFAST": 3,
                "Tracking.ImuMonoInitMaxTime": 3.0,
                "Tracking.MinInliersRecentReloc": 30,
                "Tracking.MinInliersImuMonoPreInit": 30,
                "Tracking.MinInliersImuMonoPostInit": 10,
                "Tracking.MinInliersVisual": 20,
            },
        )
    for tbc_name, config in tbc_zero_configs.items():
        experiments.append(_variant_experiment(
            f"vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_zero_{tbc_name}",
            config,
        ))

    zero_noise_updates = {
        "Camera.fps": 90,
        "ORBextractor.nFeatures": 1500,
        "ORBextractor.iniThFAST": 10,
        "ORBextractor.minThFAST": 3,
        "Tracking.ImuMonoInitMaxTime": 3.0,
        "Tracking.MinInliersRecentReloc": 30,
        "Tracking.MinInliersImuMonoPreInit": 30,
        "Tracking.MinInliersImuMonoPostInit": 10,
        "Tracking.MinInliersVisual": 20,
    }
    for scale in (20, 50, 100):
        name = f"vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_zero_imu_noise_x{scale}"
        config = write_noise_variant(
            base,
            variant_dir / f"rmoncam_h30_{name.removeprefix('vi_')}.yaml",
            float(scale),
            zero_noise_updates,
        )
        experiments.append(_variant_experiment(name, config))

    for scale in (0.2, 0.4, 0.45, 0.5, 0.55, 0.6, 1.0):
        scale_tag = str(scale).replace(".", "p")
        name = (
            "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_zero"
            f"_min_scale_{scale_tag}"
        )
        updates = dict(zero_noise_updates)
        updates["LocalMapping.MinInitialScale"] = scale
        config = write_variant(
            base,
            variant_dir / f"rmoncam_h30_{name.removeprefix('vi_')}.yaml",
            updates,
        )
        experiments.append(_variant_experiment(name, config))

    scale_gate_recovery_updates = dict(zero_noise_updates)
    scale_gate_recovery_updates["LocalMapping.MinInitialScale"] = 0.5
    for suffix, offset in (("p6ms", 0.006), ("p10ms", 0.010)):
        name = (
            "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_"
            f"{suffix}_min_scale_0p5"
        )
        config = write_variant(
            base,
            variant_dir / f"rmoncam_h30_{name.removeprefix('vi_')}.yaml",
            scale_gate_recovery_updates,
        )
        experiments.append(_variant_experiment(
            name,
            config,
            cam_imu_time_offset=offset,
            orb_time_offset_mode="legacy_warn",
        ))

    for suffix, offset, offset_mode in (
        ("zero", 0.0, "zero_for_sync"),
        ("p6ms", 0.006, "legacy_warn"),
        ("p10ms", 0.010, "legacy_warn"),
    ):
        name = (
            "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_"
            f"{suffix}_min_scale_0p5_visual_pre_ba2"
        )
        updates = dict(scale_gate_recovery_updates)
        updates["Tracking.UseVisualPoseOptimizationBeforeInertialBA2"] = 1
        config = write_variant(
            base,
            variant_dir / f"rmoncam_h30_{name.removeprefix('vi_')}.yaml",
            updates,
        )
        experiments.append(_variant_experiment(
            name,
            config,
            cam_imu_time_offset=offset,
            orb_time_offset_mode=offset_mode,
        ))

    raw_best_config = write_variant(
        base,
        variant_dir / "rmoncam_h30_raw_nfeatures_1500_fps90_init3_fast10_3_imu200.yaml",
        {
            "Camera.fps": 90,
            "ORBextractor.nFeatures": 1500,
            "ORBextractor.iniThFAST": 10,
            "ORBextractor.minThFAST": 3,
            "Tracking.ImuMonoInitMaxTime": 3.0,
            "Tracking.MinInliersRecentReloc": 30,
            "Tracking.MinInliersImuMonoPreInit": 30,
            "Tracking.MinInliersImuMonoPostInit": 10,
            "Tracking.MinInliersVisual": 20,
            "IMU.Frequency": 200.0,
        },
    )
    experiments.append(Experiment(
        name="vi_raw_nfeatures_1500_fps90_init3_fast10_3_offset_m15ms_imu200",
        sensor_type="imu-monocular",
        image_topic="/image_raw",
        imu_topic="/imu/data_raw",
        settings_file=raw_best_config,
        cam_imu_time_offset=-0.01574,
        orb_time_offset_mode="legacy_warn",
    ))

    recovery_variants = [
        (
            "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms_visual_pre_ba2",
            0.006,
            {
                "Tracking.UseVisualPoseOptimizationBeforeInertialBA2": 1,
            },
        ),
        (
            "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_visual_pre_ba2",
            0.010,
            {
                "Tracking.UseVisualPoseOptimizationBeforeInertialBA2": 1,
            },
        ),
        (
            "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms_visual_pre_ba2_no_lm_small_motion_reset",
            0.006,
            {
                "Tracking.UseVisualPoseOptimizationBeforeInertialBA2": 1,
                "LocalMapping.ResetOnSmallMotionBeforeInertialBA2": 0,
            },
        ),
        (
            "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_visual_pre_ba2_no_lm_small_motion_reset",
            0.010,
            {
                "Tracking.UseVisualPoseOptimizationBeforeInertialBA2": 1,
                "LocalMapping.ResetOnSmallMotionBeforeInertialBA2": 0,
            },
        ),
        (
            "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms_visual_pre_ba2_no_lm_no_ba2_reset",
            0.006,
            {
                "Tracking.UseVisualPoseOptimizationBeforeInertialBA2": 1,
                "Tracking.RecentRelocMaxFrames": 0,
                "Tracking.ResetActiveMapBeforeInertialBA2": 0,
                "LocalMapping.ResetOnSmallMotionBeforeInertialBA2": 0,
            },
        ),
        (
            "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_visual_pre_ba2_no_lm_no_ba2_reset",
            0.010,
            {
                "Tracking.UseVisualPoseOptimizationBeforeInertialBA2": 1,
                "Tracking.RecentRelocMaxFrames": 0,
                "Tracking.ResetActiveMapBeforeInertialBA2": 0,
                "LocalMapping.ResetOnSmallMotionBeforeInertialBA2": 0,
            },
        ),
        (
            "vi_nfeatures_1500_fps90_init5_fast10_3_sync_offset_p6ms",
            0.006,
            {
                "Tracking.ImuMonoInitMaxTime": 5.0,
            },
        ),
        (
            "vi_nfeatures_1500_fps90_init5_fast10_3_sync_offset_p10ms",
            0.010,
            {
                "Tracking.ImuMonoInitMaxTime": 5.0,
            },
        ),
        (
            "vi_nfeatures_1500_fps90_init10_fast10_3_sync_offset_p6ms",
            0.006,
            {
                "Tracking.ImuMonoInitMaxTime": 10.0,
            },
        ),
        (
            "vi_nfeatures_1500_fps90_init10_fast10_3_sync_offset_p10ms",
            0.010,
            {
                "Tracking.ImuMonoInitMaxTime": 10.0,
            },
        ),
        (
            "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms_no_lm_small_motion_reset",
            0.006,
            {
                "LocalMapping.ResetOnSmallMotionBeforeInertialBA2": 0,
            },
        ),
        (
            "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_no_lm_small_motion_reset",
            0.010,
            {
                "LocalMapping.ResetOnSmallMotionBeforeInertialBA2": 0,
            },
        ),
        (
            "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms_reloc0_no_ba2_reset",
            0.006,
            {
                "Tracking.RecentRelocMaxFrames": 0,
                "Tracking.ResetActiveMapBeforeInertialBA2": 0,
            },
        ),
        (
            "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_reloc0_no_ba2_reset",
            0.010,
            {
                "Tracking.RecentRelocMaxFrames": 0,
                "Tracking.ResetActiveMapBeforeInertialBA2": 0,
            },
        ),
        (
            "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_reloc10_no_ba2_reset",
            0.010,
            {
                "Tracking.RecentRelocMaxFrames": 10,
                "Tracking.ResetActiveMapBeforeInertialBA2": 0,
            },
        ),
        (
            "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms_reloc0_no_ba2_reset_insert_kfs",
            0.006,
            {
                "IMU.InsertKFsWhenLost": 1,
                "Tracking.RecentRelocMaxFrames": 0,
                "Tracking.ResetActiveMapBeforeInertialBA2": 0,
            },
        ),
        (
            "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_reloc0_no_ba2_reset_insert_kfs",
            0.010,
            {
                "IMU.InsertKFsWhenLost": 1,
                "Tracking.RecentRelocMaxFrames": 0,
                "Tracking.ResetActiveMapBeforeInertialBA2": 0,
            },
        ),
        (
            "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms_reloc0_no_ba2_reset_no_lm_small_motion_reset",
            0.006,
            {
                "Tracking.RecentRelocMaxFrames": 0,
                "Tracking.ResetActiveMapBeforeInertialBA2": 0,
                "LocalMapping.ResetOnSmallMotionBeforeInertialBA2": 0,
            },
        ),
        (
            "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_reloc0_no_ba2_reset_no_lm_small_motion_reset",
            0.010,
            {
                "Tracking.RecentRelocMaxFrames": 0,
                "Tracking.ResetActiveMapBeforeInertialBA2": 0,
                "LocalMapping.ResetOnSmallMotionBeforeInertialBA2": 0,
            },
        ),
        (
            "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms_reloc0_no_ba2_reset_insert_kfs_no_lm_small_motion_reset",
            0.006,
            {
                "IMU.InsertKFsWhenLost": 1,
                "Tracking.RecentRelocMaxFrames": 0,
                "Tracking.ResetActiveMapBeforeInertialBA2": 0,
                "LocalMapping.ResetOnSmallMotionBeforeInertialBA2": 0,
            },
        ),
        (
            "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_reloc0_no_ba2_reset_insert_kfs_no_lm_small_motion_reset",
            0.010,
            {
                "IMU.InsertKFsWhenLost": 1,
                "Tracking.RecentRelocMaxFrames": 0,
                "Tracking.ResetActiveMapBeforeInertialBA2": 0,
                "LocalMapping.ResetOnSmallMotionBeforeInertialBA2": 0,
            },
        ),
        (
            "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p6ms_reloc0_no_ba2_reset_post5",
            0.006,
            {
                "Tracking.RecentRelocMaxFrames": 0,
                "Tracking.ResetActiveMapBeforeInertialBA2": 0,
                "Tracking.MinInliersImuMonoPostInit": 5,
            },
        ),
        (
            "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_reloc0_no_ba2_reset_post5",
            0.010,
            {
                "Tracking.RecentRelocMaxFrames": 0,
                "Tracking.ResetActiveMapBeforeInertialBA2": 0,
                "Tracking.MinInliersImuMonoPostInit": 5,
            },
        ),
        (
            "vi_nfeatures_1500_fps90_init3_fast10_3_sync_offset_p10ms_reloc0_no_ba2_reset_post2",
            0.010,
            {
                "Tracking.RecentRelocMaxFrames": 0,
                "Tracking.ResetActiveMapBeforeInertialBA2": 0,
                "Tracking.MinInliersImuMonoPostInit": 2,
            },
        ),
    ]
    for name, offset, extra_updates in recovery_variants:
        updates = {
            "Camera.fps": 90,
            "ORBextractor.nFeatures": 1500,
            "ORBextractor.iniThFAST": 10,
            "ORBextractor.minThFAST": 3,
            "Tracking.ImuMonoInitMaxTime": 3.0,
            "Tracking.MinInliersRecentReloc": 30,
            "Tracking.MinInliersImuMonoPreInit": 30,
            "Tracking.MinInliersImuMonoPostInit": 10,
            "Tracking.MinInliersVisual": 20,
        }
        updates.update(extra_updates)
        config = write_variant(
            base,
            variant_dir / f"rmoncam_h30_{name.removeprefix('vi_')}.yaml",
            updates,
        )
        experiments.append(_variant_experiment(
            name,
            config,
            cam_imu_time_offset=offset,
            orb_time_offset_mode="legacy_warn",
        ))

    init_window_variants = [
        (
            "vi_nfeatures_1500_fps90_init3_tracking_inliers_30_30_10",
            {
                "Tracking.ImuMonoInitMaxTime": 3.0,
            },
        ),
        (
            "vi_nfeatures_1500_fps90_init5_tracking_inliers_30_30_10",
            {
                "Tracking.ImuMonoInitMaxTime": 5.0,
            },
        ),
        (
            "vi_nfeatures_1500_fps90_init3_fast10_3_tracking_inliers_30_30_10",
            {
                "Tracking.ImuMonoInitMaxTime": 3.0,
                "ORBextractor.iniThFAST": 10,
                "ORBextractor.minThFAST": 3,
            },
        ),
        (
            "vi_nfeatures_1500_fps90_init5_no_ba2_reset_tracking_inliers_30_30_10",
            {
                "Tracking.ImuMonoInitMaxTime": 5.0,
                "Tracking.ResetActiveMapBeforeInertialBA2": 0,
            },
        ),
    ]
    for name, extra_updates in init_window_variants:
        updates = {
            "Camera.fps": 90,
            "ORBextractor.nFeatures": 1500,
            "Tracking.MinInliersRecentReloc": 30,
            "Tracking.MinInliersImuMonoPreInit": 30,
            "Tracking.MinInliersImuMonoPostInit": 10,
            "Tracking.MinInliersVisual": 20,
        }
        updates.update(extra_updates)
        config = write_variant(
            base,
            variant_dir / f"rmoncam_h30_{name.removeprefix('vi_')}.yaml",
            updates,
        )
        experiments.append(_variant_experiment(name, config))

    reset_policy_variants = [
        (
            "vi_nfeatures_1500_fps90_reloc10_tracking_inliers_30_30_10",
            {
                "Camera.fps": 90,
                "ORBextractor.nFeatures": 1500,
                "Tracking.MinInliersRecentReloc": 30,
                "Tracking.MinInliersImuMonoPreInit": 30,
                "Tracking.MinInliersImuMonoPostInit": 10,
                "Tracking.MinInliersVisual": 20,
                "Tracking.RecentRelocMaxFrames": 10,
            },
        ),
        (
            "vi_nfeatures_1500_fps90_no_ba2_reset_tracking_inliers_30_30_10",
            {
                "Camera.fps": 90,
                "ORBextractor.nFeatures": 1500,
                "Tracking.MinInliersRecentReloc": 30,
                "Tracking.MinInliersImuMonoPreInit": 30,
                "Tracking.MinInliersImuMonoPostInit": 10,
                "Tracking.MinInliersVisual": 20,
                "Tracking.ResetActiveMapBeforeInertialBA2": 0,
            },
        ),
        (
            "vi_nfeatures_1500_fps90_reloc10_no_ba2_reset_tracking_inliers_30_30_10",
            {
                "Camera.fps": 90,
                "ORBextractor.nFeatures": 1500,
                "Tracking.MinInliersRecentReloc": 30,
                "Tracking.MinInliersImuMonoPreInit": 30,
                "Tracking.MinInliersImuMonoPostInit": 10,
                "Tracking.MinInliersVisual": 20,
                "Tracking.RecentRelocMaxFrames": 10,
                "Tracking.ResetActiveMapBeforeInertialBA2": 0,
            },
        ),
        (
            "vi_nfeatures_1500_fps60_reloc10_no_ba2_reset_tracking_inliers_30_30_10",
            {
                "Camera.fps": 60,
                "ORBextractor.nFeatures": 1500,
                "Tracking.MinInliersRecentReloc": 30,
                "Tracking.MinInliersImuMonoPreInit": 30,
                "Tracking.MinInliersImuMonoPostInit": 10,
                "Tracking.MinInliersVisual": 20,
                "Tracking.RecentRelocMaxFrames": 10,
                "Tracking.ResetActiveMapBeforeInertialBA2": 0,
            },
        ),
    ]
    for name, updates in reset_policy_variants:
        config = write_variant(
            base,
            variant_dir / f"rmoncam_h30_{name.removeprefix('vi_')}.yaml",
            updates,
        )
        experiments.append(_variant_experiment(name, config))

    imu_noise_fps90_updates = {
        "Camera.fps": 90,
        "ORBextractor.nFeatures": 1500,
        "Tracking.MinInliersRecentReloc": 30,
        "Tracking.MinInliersImuMonoPreInit": 30,
        "Tracking.MinInliersImuMonoPostInit": 10,
        "Tracking.MinInliersVisual": 20,
    }
    for scale in (20, 50, 100):
        name = f"vi_nfeatures_1500_fps90_imu_noise_x{scale}_tracking_inliers_30_30_10"
        config = write_noise_variant(
            base,
            variant_dir / f"rmoncam_h30_{name.removeprefix('vi_')}.yaml",
            float(scale),
            imu_noise_fps90_updates,
        )
        experiments.append(_variant_experiment(name, config))

    noise_reset_variants = [
        (50, False),
        (50, True),
        (100, False),
        (200, False),
        (500, False),
    ]
    for scale, reloc10 in noise_reset_variants:
        suffix = f"imu_noise_x{scale}"
        updates = dict(imu_noise_fps90_updates)
        updates["Tracking.ResetActiveMapBeforeInertialBA2"] = 0
        if reloc10:
            suffix += "_reloc10"
            updates["Tracking.RecentRelocMaxFrames"] = 10
        name = (
            f"vi_nfeatures_1500_fps90_{suffix}"
            "_no_ba2_reset_tracking_inliers_30_30_10"
        )
        config = write_noise_variant(
            base,
            variant_dir / f"rmoncam_h30_{name.removeprefix('vi_')}.yaml",
            float(scale),
            updates,
        )
        experiments.append(_variant_experiment(name, config))

    imu_sign_config = write_variant(
        base,
        variant_dir / "rmoncam_h30_nfeatures_1500_fps90_tracking_inliers_30_30_10.yaml",
        {
            "Camera.fps": 90,
            "ORBextractor.nFeatures": 1500,
            "Tracking.MinInliersRecentReloc": 30,
            "Tracking.MinInliersImuMonoPreInit": 30,
            "Tracking.MinInliersImuMonoPostInit": 10,
            "Tracking.MinInliersVisual": 20,
        },
    )
    imu_sign_variants = [
        (
            "vi_nfeatures_1500_fps90_acc_flip_xyz_tracking_inliers_30_30_10",
            {
                "imu_accel_scale_x": -1.0,
                "imu_accel_scale_y": -1.0,
                "imu_accel_scale_z": -1.0,
            },
        ),
        (
            "vi_nfeatures_1500_fps90_gyro_flip_xyz_tracking_inliers_30_30_10",
            {
                "imu_gyro_scale_x": -1.0,
                "imu_gyro_scale_y": -1.0,
                "imu_gyro_scale_z": -1.0,
            },
        ),
        (
            "vi_nfeatures_1500_fps90_acc_gyro_flip_xyz_tracking_inliers_30_30_10",
            {
                "imu_accel_scale_x": -1.0,
                "imu_accel_scale_y": -1.0,
                "imu_accel_scale_z": -1.0,
                "imu_gyro_scale_x": -1.0,
                "imu_gyro_scale_y": -1.0,
                "imu_gyro_scale_z": -1.0,
            },
        ),
        (
            "vi_nfeatures_1500_fps90_acc_flip_z_gyro_flip_z_tracking_inliers_30_30_10",
            {
                "imu_accel_scale_z": -1.0,
                "imu_gyro_scale_z": -1.0,
            },
        ),
    ]
    for name, extra in imu_sign_variants:
        experiments.append(_variant_experiment(name, imu_sign_config, extra=extra))

    gyro_axis_variants = {
        "xz": {"imu_gyro_scale_x": -1.0, "imu_gyro_scale_z": -1.0},
        "y": {"imu_gyro_scale_y": -1.0},
        "xy": {"imu_gyro_scale_x": -1.0, "imu_gyro_scale_y": -1.0},
        "z": {"imu_gyro_scale_z": -1.0},
        "x": {"imu_gyro_scale_x": -1.0},
        "yz": {"imu_gyro_scale_y": -1.0, "imu_gyro_scale_z": -1.0},
    }
    for axes, extra in gyro_axis_variants.items():
        experiments.append(_variant_experiment(
            f"vi_nfeatures_1500_fps90_gyro_flip_{axes}_tracking_inliers_30_30_10",
            imu_sign_config,
            extra=extra,
        ))

    for order in ("xzy", "yxz", "yzx", "zxy", "zyx"):
        experiments.append(_variant_experiment(
            f"vi_nfeatures_1500_fps90_gyro_order_{order}_flip_xyz_tracking_inliers_30_30_10",
            imu_sign_config,
            extra={
                "imu_gyro_axis_order": order,
                "imu_gyro_scale_x": -1.0,
                "imu_gyro_scale_y": -1.0,
                "imu_gyro_scale_z": -1.0,
            },
        ))

    tbc_gyro_updates = {
        "Camera.fps": 90,
        "ORBextractor.nFeatures": 1500,
        "Tracking.MinInliersRecentReloc": 30,
        "Tracking.MinInliersImuMonoPreInit": 30,
        "Tracking.MinInliersImuMonoPostInit": 10,
        "Tracking.MinInliersVisual": 20,
    }
    ranked_gyro_extras = {
        "xyz": {
            "imu_gyro_scale_x": -1.0,
            "imu_gyro_scale_y": -1.0,
            "imu_gyro_scale_z": -1.0,
        },
        "xz": {"imu_gyro_scale_x": -1.0, "imu_gyro_scale_z": -1.0},
        "y": {"imu_gyro_scale_y": -1.0},
    }

    tbc_configs: dict[str, Path] = {}
    tbc_configs["tbc_inverse"] = write_tbc_inverse_variant(
        base,
        variant_dir / "rmoncam_h30_nfeatures_1500_fps90_tbc_inverse_tracking_inliers_30_30_10.yaml",
        tbc_gyro_updates,
    )
    for tbc_name, rotation in tbc_body_rotations.items():
        tbc_configs[f"tbc_body_{tbc_name}"] = write_tbc_body_rotation_variant(
            base,
            variant_dir / (
                f"rmoncam_h30_nfeatures_1500_fps90_tbc_body_{tbc_name}"
                "_tracking_inliers_30_30_10.yaml"
            ),
            rotation,
            tbc_gyro_updates,
        )

    for axes, extra in ranked_gyro_extras.items():
        for tbc_name, config in tbc_configs.items():
            experiments.append(_variant_experiment(
                f"vi_nfeatures_1500_fps90_{tbc_name}_gyro_flip_{axes}_tracking_inliers_30_30_10",
                config,
                extra=extra,
            ))

    config = write_variant(
        base,
        variant_dir / "rmoncam_h30_insert_kfs_when_lost_1.yaml",
        {"IMU.InsertKFsWhenLost": 1},
    )
    experiments.append(_variant_experiment("vi_insert_kfs_when_lost_1", config))

    for nfeatures in (600, 700, 800, 900, 1000, 1500, 2000):
        config = write_variant(
            base,
            variant_dir / (
                f"rmoncam_h30_nfeatures_{nfeatures}_insert_kfs_when_lost_1.yaml"
            ),
            {
                "IMU.InsertKFsWhenLost": 1,
                "ORBextractor.nFeatures": nfeatures,
            },
        )
        experiments.append(
            _variant_experiment(
                f"vi_nfeatures_{nfeatures}_insert_kfs_when_lost_1",
                config,
            )
        )

    for scale in (5, 10, 20):
        config = write_noise_variant(
            base,
            variant_dir / f"rmoncam_h30_imu_noise_x{scale}.yaml",
            float(scale),
        )
        experiments.append(_variant_experiment(f"vi_imu_noise_x{scale}", config))

    return experiments


def _variant_experiment(
    name: str,
    settings_file: Path,
    extra: dict[str, Any] | None = None,
    cam_imu_time_offset: float = 0.0,
    orb_time_offset_mode: str = "zero_for_sync",
) -> Experiment:
    return Experiment(
        name=name,
        sensor_type="imu-monocular",
        image_topic="/sync/image_raw",
        imu_topic="/sync/imu",
        settings_file=settings_file,
        cam_imu_time_offset=cam_imu_time_offset,
        orb_time_offset_mode=orb_time_offset_mode,
        extra=extra or {},
    )


def _metadata_duration_sec(bag_path: Path) -> float | None:
    metadata = bag_path / "metadata.yaml"
    if not metadata.exists():
        return None
    text = metadata.read_text(encoding="utf-8", errors="replace")
    match = re.search(r"duration:\s*\n\s*nanoseconds:\s*(\d+)", text)
    if not match:
        return None
    return int(match.group(1)) / 1e9


def _audit_bag(bag_path: Path, output_root: Path) -> dict[str, Any]:
    # Keep rosbag2_py optional until a real bag audit/replay run is requested.
    # This lets host-side log parsing and report refresh work without ROS.
    from bag_audit import audit_bag

    return audit_bag(bag_path, output_root)


def build_command(
    experiment: Experiment,
    bag_path: Path,
    rate: float,
    start_offset: float,
    duration: float,
    play_delay: float,
) -> list[str]:
    command = [
        "ros2",
        "launch",
        "orb_slam3_ros2",
        "h30_bag_replay.launch.py",
        f"bag_path:={bag_path}",
        "play_bag:=true",
        f"sensor_type:={experiment.sensor_type}",
        f"image_topic:={experiment.image_topic}",
        f"imu_topic:={experiment.imu_topic}",
        f"settings_file:={experiment.settings_file}",
        f"cam_imu_time_offset:={experiment.cam_imu_time_offset}",
        f"orb_time_offset_mode:={experiment.orb_time_offset_mode}",
        "debug_enable:=true",
        "use_rviz:=false",
        "use_pangolin:=false",
        f"rate:={rate}",
        f"start_offset:={start_offset}",
        f"duration:={duration}",
        f"play_delay:={play_delay}",
    ]
    for key, value in sorted(experiment.extra.items()):
        command.append(f"{key}:={value}")
    return command


def _timeout_for_bag(
    bag_path: Path,
    rate: float,
    duration: float,
    play_delay: float,
) -> float:
    bag_duration = duration if duration > 0.0 else (_metadata_duration_sec(bag_path) or 60.0)
    return max(20.0, play_delay + bag_duration / max(rate, 1e-6) + 35.0)


def run_experiment(
    experiment: Experiment,
    bag_path: Path,
    output_dir: Path,
    rate: float,
    start_offset: float,
    duration: float,
    play_delay: float,
) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    command = build_command(
        experiment,
        bag_path,
        rate,
        start_offset,
        duration,
        play_delay,
    )
    (output_dir / "command.txt").write_text(" ".join(command) + "\n", encoding="utf-8")

    timeout = _timeout_for_bag(bag_path, rate, duration, play_delay)
    started = time.time()
    timed_out = False
    return_code: int | None = None
    with (output_dir / "console.log").open("w", encoding="utf-8", errors="replace") as log:
        process = subprocess.Popen(
            command,
            stdout=log,
            stderr=subprocess.STDOUT,
            cwd=str(PACKAGE_ROOT),
            text=True,
            start_new_session=True,
        )
        try:
            return_code = process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            timed_out = True
            os.killpg(process.pid, signal.SIGINT)
            try:
                return_code = process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                return_code = process.wait(timeout=10)

    metrics = parse_orb_log(output_dir / "console.log")
    result = {
        "experiment": experiment.__dict__ | {
            "settings_file": str(experiment.settings_file),
        },
        "bag_path": str(bag_path),
        "return_code": return_code,
        "timed_out": timed_out,
        "elapsed_wall_sec": time.time() - started,
        "metrics": metrics,
        "criteria": evaluate_criteria(experiment.name, metrics),
    }
    (output_dir / "result.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (output_dir / "summary.md").write_text(render_experiment_summary(result), encoding="utf-8")
    return result


def evaluate_criteria(name: str, metrics: dict[str, Any]) -> dict[str, Any]:
    median_inliers = metrics.get("inlier_count", {}).get("median")
    median_image_to_imu = metrics.get("image_to_last_imu_ms", {}).get("median")
    criteria: dict[str, Any] = {
        "tracking_entered_ok": metrics.get("first_ok_frame") is not None,
        "keyframes_gt_15": metrics.get("keyframe_count", 0) > 15,
        "no_long_lost": metrics.get("max_lost_streak_sec", 0.0) < 10.0,
    }
    if name == "mono_sync_image":
        criteria["duration_ge_60"] = metrics.get("duration_sec", 0.0) >= 60.0
        criteria["success"] = all(criteria.values())
        return criteria

    criteria.update({
        "imu_initialized": bool(metrics.get("imu_initialized")),
        "ok_ratio_gt_0_8": metrics.get("ok_ratio", 0.0) > 0.8,
        "reset_count_ok": (
            metrics.get("reset_count", 0) == 0
            or metrics.get("late_reset_delta_30s", 1) == 0
        ),
        "median_inliers_gt_50": (
            median_inliers is not None and median_inliers > 50.0
        ),
        "final_keyframes_gt_20": metrics.get("final_keyframe_count", 0) > 20,
        "insufficient_imu_near_zero": metrics.get("insufficient_imu_frames", 0) <= 2,
        "image_to_last_imu_ms_0_12": (
            median_image_to_imu is not None and 0.0 <= median_image_to_imu <= 12.0
        ),
    })
    criteria["success"] = all(criteria[key] for key in VI_GATE_KEYS)
    return criteria


def render_experiment_summary(result: dict[str, Any]) -> str:
    metrics = result["metrics"]
    criteria = result["criteria"]
    event_code_counts = metrics.get("event_code_counts", {})
    event_reason_counts = metrics.get("event_reason_counts", {})
    return "\n".join([
        f"# {result['experiment']['name']}",
        "",
        f"- bag_path: `{result['bag_path']}`",
        f"- settings_file: `{result['experiment']['settings_file']}`",
        f"- return_code: {result['return_code']}",
        f"- timed_out: {result['timed_out']}",
        f"- success: {criteria.get('success')}",
        f"- tracking_state: {metrics.get('tracking_state')}",
        f"- ok_ratio: {metrics.get('ok_ratio'):.3f}",
        f"- reset_count: {metrics.get('reset_count')}",
        f"- late_reset_delta_30s: {metrics.get('late_reset_delta_30s')}",
        f"- imu_initialized: {metrics.get('imu_initialized')}",
        f"- keyframe_count: {metrics.get('keyframe_count')}",
        f"- median_inliers: {metrics.get('inlier_count', {}).get('median')}",
        f"- insufficient_imu_frames: {metrics.get('insufficient_imu_frames')}",
        f"- image_to_last_imu_ms_median: "
        f"{metrics.get('image_to_last_imu_ms', {}).get('median')}",
        f"- event_code_counts: {event_code_counts}",
        f"- event_reason_counts: {event_reason_counts}",
        "",
        "## Criteria",
        "",
        json.dumps(criteria, indent=2, sort_keys=True),
    ]) + "\n"


def refresh_result_from_log(result_path: Path) -> dict[str, Any]:
    result = json.loads(result_path.read_text(encoding="utf-8"))
    log_path = result_path.parent / "console.log"
    result["metrics"] = parse_orb_log(log_path)
    result["criteria"] = evaluate_criteria(
        result["experiment"]["name"],
        result["metrics"],
    )
    result_path.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (result_path.parent / "summary.md").write_text(
        render_experiment_summary(result),
        encoding="utf-8",
    )
    return result


def choose_best(results: list[dict[str, Any]]) -> dict[str, Any] | None:
    if not results:
        return None

    def score(result: dict[str, Any]) -> tuple[float, float, float, float]:
        metrics = result["metrics"]
        return (
            1.0 if result["criteria"].get("success") else 0.0,
            float(metrics.get("ok_ratio", 0.0)),
            float(metrics.get("keyframe_count", 0)),
            float(metrics.get("inlier_count", {}).get("median") or 0.0),
        )

    return max(results, key=score)


def _format_top_counts(counts: dict[str, int], limit: int = 3) -> str:
    if not counts:
        return ""
    items = sorted(counts.items(), key=lambda item: item[1], reverse=True)
    return ", ".join(f"{name}={count}" for name, count in items[:limit])


def render_round_summary(results: list[dict[str, Any]], bag_path: Path) -> str:
    lines = [
        "# H30 Bag Matrix Summary",
        "",
        f"- bag_path: `{bag_path}`",
        f"- experiments: {len(results)}",
        "",
        "| Experiment | Success | OK Ratio | IMU Init | Keyframes | Median Inliers | Resets | Insufficient IMU |",
        "| --- | --- | ---: | --- | ---: | ---: | ---: | ---: |",
    ]
    for result in results:
        metrics = result["metrics"]
        lines.append(
            f"| `{result['experiment']['name']}` | "
            f"{result['criteria'].get('success')} | "
            f"{metrics.get('ok_ratio', 0.0):.3f} | "
            f"{metrics.get('imu_initialized')} | "
            f"{metrics.get('keyframe_count')} | "
            f"{metrics.get('inlier_count', {}).get('median')} | "
            f"{metrics.get('reset_count')}"
            f" (+{metrics.get('late_reset_delta_30s', 'n/a')} late) | "
            f"{metrics.get('insufficient_imu_frames')} |"
        )
    lines.extend([
        "",
        "## Top Event Reasons",
        "",
    ])
    for result in results:
        metrics = result["metrics"]
        reasons = _format_top_counts(metrics.get("event_reason_counts", {}))
        codes = _format_top_counts(metrics.get("event_code_counts", {}))
        lines.append(
            f"- `{result['experiment']['name']}`: "
            f"codes: {codes or 'n/a'}; reasons: {reasons or 'n/a'}"
        )
    return "\n".join(lines) + "\n"


def render_best_config(best: dict[str, Any] | None) -> str:
    if best is None:
        return "best_config: null\n"
    exp = best["experiment"]
    metrics = best["metrics"]
    event_code_counts = metrics.get("event_code_counts", {})
    event_reason_counts = metrics.get("event_reason_counts", {})
    return "\n".join([
        f"name: {exp['name']}",
        f"sensor_type: {exp['sensor_type']}",
        f"image_topic: {exp['image_topic']}",
        f"imu_topic: {exp['imu_topic']}",
        f"settings_file: {exp['settings_file']}",
        f"cam_imu_time_offset: {exp['cam_imu_time_offset']}",
        f"orb_time_offset_mode: {exp['orb_time_offset_mode']}",
        f"extra: {json.dumps(exp.get('extra', {}), sort_keys=True)}",
        "metrics:",
        f"  ok_ratio: {metrics.get('ok_ratio')}",
        f"  imu_initialized: {metrics.get('imu_initialized')}",
        f"  keyframe_count: {metrics.get('keyframe_count')}",
        f"  reset_count: {metrics.get('reset_count')}",
        f"  late_reset_delta_30s: {metrics.get('late_reset_delta_30s')}",
        f"  median_inliers: {metrics.get('inlier_count', {}).get('median')}",
        f"  image_to_last_imu_ms_median: "
        f"{metrics.get('image_to_last_imu_ms', {}).get('median')}",
        f"  event_code_counts: {json.dumps(event_code_counts, sort_keys=True)}",
        f"  event_reason_counts: {json.dumps(event_reason_counts, sort_keys=True)}",
    ]) + "\n"


def render_decision(results: list[dict[str, Any]]) -> str:
    by_name = {result["experiment"]["name"]: result for result in results}
    sync_zero = by_name.get("vi_sync_offset_zero")
    sync_legacy = by_name.get("vi_sync_offset_minus_15ms")
    raw_legacy = by_name.get("vi_raw_offset_minus_15ms")
    mono = by_name.get("mono_sync_image")

    lines = ["# Decision", ""]
    if mono and not mono["criteria"].get("success"):
        lines.append(
            "Monocular tracking did not meet the gate. Fix image/calibration/"
            "RGB/ORB feature settings before tuning IMU."
        )
    elif sync_zero and sync_legacy:
        zero_score = sync_zero["metrics"].get("ok_ratio", 0.0)
        legacy_score = sync_legacy["metrics"].get("ok_ratio", 0.0)
        if sync_zero["criteria"].get("success") and zero_score >= legacy_score:
            lines.append(
                "/sync/image_raw + /sync/imu with cam_imu_time_offset=0.0 "
                "is the recommended sync path. Update sync launches to "
                "zero_for_sync if this remains stable on full replay."
            )
        elif raw_legacy and raw_legacy["criteria"].get("success"):
            lines.append(
                "Raw image + raw IMU with -15.74ms succeeds while sync zero "
                "does not. Inspect cam_imu_sync header stamp semantics."
            )
        else:
            lines.append(
                "No VI configuration met the success gate. Prioritize offset, "
                "QoS, IMU units, and Tbc direction before ORB core changes."
            )
    elif results:
        best = choose_best(results)
        if best is None:
            lines.append("No focused experiment results were available.")
        else:
            exp = best["experiment"]
            metrics = best["metrics"]
            if best["criteria"].get("success"):
                lines.append(
                    f"Focused matrix success: `{exp['name']}` met the replay "
                    "gate. Use this configuration for the next full replay "
                    "validation before making it the default."
                )
            else:
                lines.append(
                    f"Focused matrix complete; no experiment met the replay "
                    f"gate. Best by score was `{exp['name']}` with "
                    f"ok_ratio={metrics.get('ok_ratio', 0.0):.3f}, "
                    f"imu_initialized={metrics.get('imu_initialized')}, "
                    f"keyframes={metrics.get('keyframe_count')}, "
                    f"reset_count={metrics.get('reset_count')}, "
                    f"late_reset_delta_30s="
                    f"{metrics.get('late_reset_delta_30s')}, "
                    f"median_inliers="
                    f"{metrics.get('inlier_count', {}).get('median')}."
                )
    else:
        lines.append("First-batch evidence is incomplete.")
    return "\n".join(lines) + "\n"


def update_latest(output_root: Path, summary: str, best_config: str, decision: str) -> None:
    latest = output_root / "latest"
    latest.mkdir(parents=True, exist_ok=True)
    (latest / ".generated_by_run_bag_matrix").write_text("1\n", encoding="utf-8")
    (latest / "summary.md").write_text(summary, encoding="utf-8")
    (latest / "best_config.yaml").write_text(best_config, encoding="utf-8")
    (latest / "decision.md").write_text(decision, encoding="utf-8")


def refresh_round(round_dir: Path) -> list[dict[str, Any]]:
    result_paths = sorted(round_dir.glob("*/result.json"))
    if not result_paths:
        raise ValueError(f"no result.json files found under {round_dir}")
    results = [refresh_result_from_log(path) for path in result_paths]
    bag_path = Path(results[0]["bag_path"])
    summary = render_round_summary(results, bag_path)
    best_config = render_best_config(choose_best(results))
    decision = render_decision(results)
    (round_dir / "summary.md").write_text(summary, encoding="utf-8")
    (round_dir / "best_config.yaml").write_text(best_config, encoding="utf-8")
    (round_dir / "decision.md").write_text(decision, encoding="utf-8")
    output_root = round_dir.parent.parent if round_dir.parent.name == bag_path.name else round_dir.parent
    update_latest(output_root, summary, best_config, decision)
    write_gate_report(output_root, bag_path=bag_path)
    return results


def collect_result_files(root: Path) -> list[Path]:
    return sorted(
        path for path in root.glob("**/result.json")
        if "config_variants" not in path.parts
    )


def load_result_file(path: Path) -> dict[str, Any] | None:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError):
        return None


def collect_results(
    output_root: Path,
    bag_path: Path | None = None,
) -> list[dict[str, Any]]:
    expected_bag = str(_resolve_bag_path(bag_path)) if bag_path is not None else None
    results: list[dict[str, Any]] = []
    for path in collect_result_files(output_root):
        result = load_result_file(path)
        if result is None:
            continue
        if expected_bag is not None and result.get("bag_path") != expected_bag:
            continue
        results.append(result)
    return results


def completed_first_batch_names(output_root: Path, bag_path: Path) -> set[str]:
    required = set(first_batch_names())
    completed: set[str] = set()
    for result in collect_results(output_root, bag_path=bag_path):
        experiment = result.get("experiment", {})
        name = experiment.get("name")
        if name in required:
            completed.add(name)
    return completed


def require_first_batch_completed(output_root: Path, bag_path: Path) -> None:
    required = set(first_batch_names())
    completed = completed_first_batch_names(output_root, bag_path)
    missing = sorted(required - completed)
    if missing:
        raise ValueError(
            "second-batch experiments require completed first-batch results "
            f"for bag {bag_path.name}; missing: {', '.join(missing)}"
        )


def selected_needs_completed_first_batch(experiments: list[Experiment]) -> bool:
    first_names = set(first_batch_names())
    selected_names = {experiment.name for experiment in experiments}
    has_second_batch = any(name not in first_names for name in selected_names)
    first_batch_in_same_round = first_names.issubset(selected_names)
    return has_second_batch and not first_batch_in_same_round


def render_leaderboard(results: list[dict[str, Any]], limit: int = 20) -> str:
    def score(result: dict[str, Any]) -> tuple[float, float, float, float, float]:
        metrics = result.get("metrics", {})
        return (
            1.0 if result.get("criteria", {}).get("success") else 0.0,
            float(metrics.get("ok_ratio", 0.0)),
            -float(metrics.get("late_reset_delta_30s", metrics.get("reset_count", 0))),
            float(metrics.get("keyframe_count", 0)),
            float(metrics.get("inlier_count", {}).get("median") or 0.0),
        )

    ranked = sorted(results, key=score, reverse=True)
    lines = [
        "# H30 Bag Replay Leaderboard",
        "",
        f"- results: {len(ranked)}",
        "",
        "| Rank | Success | OK Ratio | IMU Init | Keyframes | Median Inliers | Reset Late | Reset Count | Experiment | Matrix |",
        "| ---: | --- | ---: | --- | ---: | ---: | ---: | ---: | --- | --- |",
    ]
    for index, result in enumerate(ranked[:limit], start=1):
        metrics = result.get("metrics", {})
        exp = result.get("experiment", {})
        result_path = Path(metrics.get("log_path", "")).parent
        matrix = result_path.parent.name if result_path.name else ""
        lines.append(
            f"| {index} | {result.get('criteria', {}).get('success')} | "
            f"{metrics.get('ok_ratio', 0.0):.3f} | "
            f"{metrics.get('imu_initialized')} | "
            f"{metrics.get('keyframe_count')} | "
            f"{metrics.get('inlier_count', {}).get('median')} | "
            f"{metrics.get('late_reset_delta_30s', 'n/a')} | "
            f"{metrics.get('reset_count')} | "
            f"`{exp.get('name')}` | `{matrix}` |"
        )
    return "\n".join(lines) + "\n"


def write_leaderboard(output_root: Path, limit: int = 20) -> str:
    results = collect_results(output_root)
    if not results:
        raise ValueError(f"no result.json files found under {output_root}")
    markdown = render_leaderboard(results, limit=limit)
    latest = output_root / "latest"
    latest.mkdir(parents=True, exist_ok=True)
    (latest / "leaderboard.md").write_text(markdown, encoding="utf-8")
    return markdown


def _result_brief(result: dict[str, Any] | None) -> str:
    if result is None:
        return "n/a"
    exp = result.get("experiment", {})
    metrics = result.get("metrics", {})
    result_path = Path(metrics.get("log_path", "")).parent
    matrix = result_path.parent.name if result_path.name else ""
    return (
        f"`{exp.get('name')}` matrix=`{matrix}` "
        f"success={result.get('criteria', {}).get('success')} "
        f"ok_ratio={metrics.get('ok_ratio', 0.0):.3f} "
        f"imu_initialized={metrics.get('imu_initialized')} "
        f"keyframes={metrics.get('keyframe_count')} "
        f"reset_count={metrics.get('reset_count')} "
        f"late_reset_delta_30s={metrics.get('late_reset_delta_30s')} "
        f"median_inliers={metrics.get('inlier_count', {}).get('median')} "
        f"image_to_last_imu_ms_median="
        f"{metrics.get('image_to_last_imu_ms', {}).get('median')}"
    )


def _failed_criteria(
    result: dict[str, Any] | None,
    gate_keys: tuple[str, ...] | None = None,
) -> list[str]:
    if result is None:
        return []
    criteria = result.get("criteria", {})
    keys = gate_keys or tuple(
        key for key in criteria
        if key != "success"
    )
    return [
        key for key in keys
        if criteria.get(key) is False
    ]


def render_gate_report(
    results: list[dict[str, Any]],
    bag_path: Path | None = None,
) -> tuple[str, bool]:
    mono_results = [
        result for result in results
        if result.get("experiment", {}).get("name") == "mono_sync_image"
    ]
    vi_results = [
        result for result in results
        if result.get("experiment", {}).get("sensor_type") == "imu-monocular"
    ]
    passing_mono = [
        result for result in mono_results
        if result.get("criteria", {}).get("success")
    ]
    passing_vi = [
        result for result in vi_results
        if result.get("criteria", {}).get("success")
    ]
    best_mono = choose_best(mono_results)
    best_vi = choose_best(vi_results)
    vi_gate_passed = bool(passing_vi)

    lines = [
        "# H30 Replay Gate Report",
        "",
        f"- bag_path: `{_resolve_bag_path(bag_path)}`"
        if bag_path is not None else "- bag_path: all",
        f"- total_results: {len(results)}",
        f"- mono_gate_passed: {bool(passing_mono)}",
        f"- vi_gate_passed: {vi_gate_passed}",
        f"- passing_vi_count: {len(passing_vi)}",
        "",
        "## Best Evidence",
        "",
        f"- best_mono: {_result_brief(best_mono)}",
        f"- best_vi: {_result_brief(best_vi)}",
        f"- best_vi_failed_criteria: "
        f"{', '.join(_failed_criteria(best_vi, VI_GATE_KEYS)) or 'none'}",
    ]
    if passing_vi:
        lines.extend([
            "",
            "## Passing VI",
            "",
        ])
        for result in sorted(
            passing_vi,
            key=lambda item: item.get("metrics", {}).get("ok_ratio", 0.0),
            reverse=True,
        ):
            lines.append(f"- {_result_brief(result)}")
    else:
        lines.extend([
            "",
            "## Missing",
            "",
            "- No monocular-inertial replay result currently meets the VI gate.",
        ])
    return "\n".join(lines) + "\n", vi_gate_passed


def write_gate_report(
    output_root: Path,
    bag_path: Path | None = None,
) -> tuple[str, bool]:
    results = collect_results(output_root, bag_path=bag_path)
    if not results:
        suffix = f" for bag {_resolve_bag_path(bag_path)}" if bag_path else ""
        raise ValueError(f"no result.json files found under {output_root}{suffix}")
    markdown, vi_gate_passed = render_gate_report(results, bag_path=bag_path)
    latest = output_root / "latest"
    latest.mkdir(parents=True, exist_ok=True)
    (latest / "gate_report.md").write_text(markdown, encoding="utf-8")
    return markdown, vi_gate_passed


def select_experiments(
    phase: str,
    variant_dir: Path,
    only: set[str] | None = None,
) -> list[Experiment]:
    experiments = first_batch()
    if phase in ("second", "all"):
        if phase == "second":
            experiments = []
        experiments.extend(second_batch(variant_dir))

    if only:
        available = {experiment.name for experiment in experiments}
        unknown = sorted(only - available)
        if unknown:
            raise ValueError(
                "unknown experiment(s): "
                + ", ".join(unknown)
                + "; available: "
                + ", ".join(sorted(available))
            )
        experiments = [
            experiment for experiment in experiments if experiment.name in only
        ]

    if not experiments:
        raise ValueError("no experiments selected")
    return experiments


def select_experiment_names(
    phase: str,
    only: set[str] | None = None,
) -> list[str]:
    names = experiment_names_for_phase(phase)
    if only:
        available = set(names)
        unknown = sorted(only - available)
        if unknown:
            raise ValueError(
                "unknown experiment(s): "
                + ", ".join(unknown)
                + "; available: "
                + ", ".join(sorted(available))
            )
        names = [name for name in names if name in only]
    if not names:
        raise ValueError("no experiments selected")
    return names


def run_round(
    bag_path: Path,
    output_root: Path,
    phase: str,
    rate: float,
    start_offset: float,
    duration: float,
    play_delay: float,
    only: set[str] | None = None,
) -> tuple[Path, list[dict[str, Any]]]:
    bag_path = _resolve_bag_path(bag_path)
    bag_run_root = output_root / bag_path.name
    run_id = time.strftime("%Y%m%d_%H%M%S")
    round_dir = bag_run_root / f"matrix_{run_id}"
    variant_dir = round_dir / "config_variants"
    experiments = select_experiments(phase, variant_dir, only)
    if selected_needs_completed_first_batch(experiments):
        require_first_batch_completed(output_root, bag_path)
    _audit_bag(bag_path, output_root)

    results: list[dict[str, Any]] = []
    for experiment in experiments:
        exp_dir = round_dir / experiment.name
        print(f"[matrix] running {experiment.name}", flush=True)
        result = run_experiment(
            experiment,
            bag_path,
            exp_dir,
            rate=rate,
            start_offset=start_offset,
            duration=duration,
            play_delay=play_delay,
        )
        results.append(result)

    summary = render_round_summary(results, bag_path)
    best_config = render_best_config(choose_best(results))
    decision = render_decision(results)
    round_dir.mkdir(parents=True, exist_ok=True)
    (round_dir / "summary.md").write_text(summary, encoding="utf-8")
    (round_dir / "best_config.yaml").write_text(best_config, encoding="utf-8")
    (round_dir / "decision.md").write_text(decision, encoding="utf-8")
    update_latest(output_root, summary, best_config, decision)
    write_gate_report(output_root, bag_path=bag_path)
    return round_dir, results


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag-path", type=Path)
    parser.add_argument("--latest-file", type=Path, default=DEFAULT_LATEST)
    parser.add_argument(
        "--output-root",
        type=Path,
        default=PACKAGE_ROOT / "debug_runs",
    )
    parser.add_argument("--phase", choices=("first", "second", "all"), default="first")
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--start-offset", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument(
        "--play-delay",
        type=float,
        default=2.0,
        help="Seconds to wait before ros2 bag play starts inside the launch file.",
    )
    parser.add_argument(
        "--only",
        action="append",
        default=[],
        help="Run only the named experiment. Repeat for multiple names.",
    )
    parser.add_argument(
        "--preset",
        action="append",
        choices=sorted(EXPERIMENT_PRESETS),
        default=[],
        help="Run a named focused experiment group. Repeat to combine groups.",
    )
    parser.add_argument(
        "--list-experiments",
        action="store_true",
        help="Print selected experiment names and exit without opening the bag.",
    )
    parser.add_argument("--smoke-first", action="store_true")
    parser.add_argument("--smoke-duration", type=float, default=20.0)
    parser.add_argument(
        "--refresh-round",
        type=Path,
        help="Reparse an existing matrix_* directory from console.log files.",
    )
    parser.add_argument(
        "--leaderboard",
        action="store_true",
        help="Render a cross-round leaderboard from existing result.json files.",
    )
    parser.add_argument("--leaderboard-limit", type=int, default=20)
    parser.add_argument(
        "--gate-report",
        action="store_true",
        help="Render a cross-round gate report from existing result.json files.",
    )
    parser.add_argument(
        "--require-vi-success",
        action="store_true",
        help="Return non-zero with --gate-report when no VI result passes.",
    )
    args = parser.parse_args()

    if args.preset and args.phase == "first":
        parser.error("--preset selects second-batch experiments; pass --phase second or --phase all")

    output_root = args.output_root.expanduser().resolve()

    if args.refresh_round:
        try:
            results = refresh_round(args.refresh_round.expanduser().resolve())
        except ValueError as exc:
            parser.error(str(exc))
        print(render_round_summary(results, Path(results[0]["bag_path"])))
        return 0

    if args.leaderboard:
        try:
            print(write_leaderboard(output_root, limit=args.leaderboard_limit))
        except ValueError as exc:
            parser.error(str(exc))
        return 0

    if args.gate_report:
        try:
            gate_bag_path = (
                _resolve_bag_path(args.bag_path)
                if args.bag_path else read_latest_bag(args.latest_file)
            )
            markdown, vi_gate_passed = write_gate_report(
                output_root,
                bag_path=gate_bag_path,
            )
        except ValueError as exc:
            parser.error(str(exc))
        print(markdown)
        if args.require_vi_success and not vi_gate_passed:
            return 1
        return 0

    selected_names = set(args.only)
    for preset in args.preset:
        selected_names.update(EXPERIMENT_PRESETS[preset])

    if args.list_experiments:
        try:
            names = select_experiment_names(args.phase, selected_names or None)
        except ValueError as exc:
            parser.error(str(exc))
        for name in names:
            print(name)
        return 0

    bag_path = _resolve_bag_path(args.bag_path) if args.bag_path else read_latest_bag(args.latest_file)
    if args.smoke_first:
        smoke_bags = find_smoke_bags(args.latest_file.expanduser().resolve().parent)
        if smoke_bags:
            print(f"[matrix] smoke bag: {smoke_bags[-1]}", flush=True)
            run_round(
                smoke_bags[-1],
                output_root,
                phase="first",
                rate=args.rate,
                start_offset=0.0,
                duration=args.smoke_duration,
                play_delay=args.play_delay,
                only=None,
            )
        else:
            print("[matrix] no h30_sync_raw_smoke_* bag found; skipping smoke", flush=True)

    try:
        round_dir, results = run_round(
            bag_path,
            output_root,
            phase=args.phase,
            rate=args.rate,
            start_offset=args.start_offset,
            duration=args.duration,
            play_delay=args.play_delay,
            only=selected_names or None,
        )
    except ValueError as exc:
        parser.error(str(exc))
    print(f"[matrix] wrote {round_dir}", flush=True)
    print(render_round_summary(results, bag_path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
