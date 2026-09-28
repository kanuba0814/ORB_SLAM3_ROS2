#!/usr/bin/env python3
"""Run the InitGate test matrix: per-bag x per-config x N runs.

For each (bag, config, run) combination, launch h30_bag_replay.launch.py inside
the autoware_env_v2_host docker container, capture console.log, then parse it
via parse_runtime_orb_log.py to get stability_pass and InitGate-specific
metrics. Aggregate into a CSV + Markdown report.

Pass condition (per the InitGate plan):
- imu_initialized == True
- reset_count_after_init == 0  (core goal)
- ok_ratio_after_init >= 0.90
- final_pose_drift_m < 1.0 m (closed-loop bags)

Allowed exceptions (failure tolerated):
- bags listed in INIT_EXCEPTION_BAGS may have imu_initialized == False
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import shlex
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from parse_runtime_orb_log import parse_runtime_log


WORKSPACE_ROOT = Path("/home/gxxl/slam_ws_v2")
DOCKER_CONTAINER = "autoware_env_v2_host"
PACKAGE_ROOT = WORKSPACE_ROOT / "extra_ws" / "src" / "ORB_SLAM3_ROS2"

INIT_GATE_PHASE_RE = re.compile(
    r"\[init_gate\] phase=(?P<to>\w+) prev=(?P<from>\w+) t=(?P<t>[\d.]+) "
    r"prev_dur_sec=(?P<dur>[-\d.]+) reason=(?P<reason>\S+) "
    r"frame_id=(?P<frame_id>\d+)"
)
IMU_WARMUP_RE = re.compile(
    r"\[imu_warmup\] (?P<status>PASS|SKIP)"
)


@dataclass
class Config:
    name: str
    launch_overrides: dict[str, str] = field(default_factory=dict)


# Config variants from the InitGate plan (Phase 2.1)
CONFIGS: list[Config] = [
    Config(
        "baseline_no_gate",
        {"init_gate_enabled": "false", "imu_warmup_enabled": "false"},
    ),
    Config(
        "warmup_only",
        {"init_gate_enabled": "false", "imu_warmup_enabled": "true"},
    ),
    Config(
        "full_gate_default",
        {"init_gate_enabled": "true"},
    ),
    Config(
        "full_gate_tight_motion",
        {
            "init_gate_enabled": "true",
            "init_motion_gyro_threshold_degs": "5.0",
            "init_motion_sustained_sec": "0.5",
        },
    ),
    Config(
        "full_gate_loose_motion",
        {
            "init_gate_enabled": "true",
            "init_motion_gyro_threshold_degs": "1.5",
            "init_motion_sustained_sec": "0.2",
        },
    ),
    Config(
        "full_gate_short_timeout",
        {
            "init_gate_enabled": "true",
            "init_await_motion_timeout_sec": "10.0",
        },
    ),
]


# (bag_path, scenario_name, replay_duration_sec_or_None)
DEFAULT_BAGS: list[tuple[Path, str, float | None]] = [
    (WORKSPACE_ROOT / "bags/InitGate_test/hand_static_gentle", "hand_static_gentle", 85.0),
    (WORKSPACE_ROOT / "bags/InitGate_test/hand_static_rotate", "hand_static_rotate", None),
    (WORKSPACE_ROOT / "bags/InitGate_test/hand_pickup_walk", "hand_pickup_walk", None),
    (WORKSPACE_ROOT / "bags/goalDebug/h30_150hz_slowMove", "h30_150hz_slowMove", None),
    (WORKSPACE_ROOT / "bags/goalDebug/h30_150hz_150s", "h30_150hz_150s", None),
]

# Bags that may legitimately fail imu_init (per plan's allowed exceptions)
INIT_EXCEPTION_BAGS = {"hand_rough_shake", "h30_150hz_150s"}
# Bags where final pose drift is dominated by scenario degeneracies (pure
# rotation, sparse features) rather than InitGate behaviour: exclude from the
# `closed_loop_drift_pass` gate so the InitGate-level pass/fail isn't masked.
DRIFT_EXCEPTION_BAGS = {
    "hand_static_rotate",  # pure rotation: monocular-inertial scale is degenerate
    "h30_150hz_150s",      # legacy baseline bag also in INIT_EXCEPTION_BAGS
    "hand_rough_shake",
}


def metadata_duration_sec(bag_path: Path) -> float | None:
    meta = bag_path / "metadata.yaml"
    if not meta.exists():
        return None
    nano_re = re.compile(r"duration:\s*\n\s*nanoseconds:\s*(\d+)")
    m = nano_re.search(meta.read_text())
    if m:
        return int(m.group(1)) / 1e9
    return None


def docker_launch_command(
    bag_path: Path,
    duration: float,
    rate: float,
    config: Config,
) -> list[str]:
    """Build the docker exec command for one launch."""
    args = [
        "ros2", "launch", "orb_slam3_ros2", "h30_bag_replay.launch.py",
        "play_bag:=true",
        f"bag_path:={bag_path}",
        f"duration:={duration}",
        f"rate:={rate}",
    ]
    for key, value in config.launch_overrides.items():
        args.append(f"{key}:={value}")
    inner = (
        "source /opt/ros/humble/setup.bash && "
        "cd /home/gxxl/slam_ws_v2/extra_ws && "
        "source install/setup.bash && "
        + " ".join(shlex.quote(a) for a in args)
    )
    return ["docker", "exec", DOCKER_CONTAINER, "bash", "-lc", inner]


def parse_init_gate_log(log_path: Path) -> dict[str, Any]:
    """Extract InitGate-specific info from console.log."""
    phases: list[dict[str, Any]] = []
    warmup_status: str | None = None
    text = log_path.read_text(encoding="utf-8", errors="replace")
    for line in text.splitlines():
        m = INIT_GATE_PHASE_RE.search(line)
        if m:
            phases.append({
                "from": m.group("from"),
                "to": m.group("to"),
                "t_bag_sec": float(m.group("t")),
                "prev_dur_sec": float(m.group("dur")),
                "reason": m.group("reason"),
                "frame_id": int(m.group("frame_id")),
            })
            continue
        wm = IMU_WARMUP_RE.search(line)
        if wm:
            warmup_status = wm.group("status")

    def reach(phase_name: str) -> dict[str, Any] | None:
        for p in phases:
            if p["to"] == phase_name:
                return p
        return None

    return {
        "warmup_status": warmup_status,
        "phase_transitions": phases,
        "reached_await_motion": reach("AWAIT_MOTION") is not None,
        "reached_tracking": reach("TRACKING") is not None,
        "reached_imu_ready": reach("IMU_READY") is not None,
        "time_to_await_motion_sec": (reach("AWAIT_MOTION") or {}).get("prev_dur_sec"),
        "time_to_tracking_sec": (reach("TRACKING") or {}).get("prev_dur_sec"),
        "time_to_imu_ready_sec": (reach("IMU_READY") or {}).get("prev_dur_sec"),
    }


def run_one(
    bag_path: Path,
    bag_name: str,
    config: Config,
    run_index: int,
    duration: float,
    rate: float,
    output_root: Path,
    overall_timeout_sec: float,
) -> dict[str, Any]:
    out_dir = output_root / bag_name / config.name / f"run{run_index:02d}"
    out_dir.mkdir(parents=True, exist_ok=True)
    log_path = out_dir / "console.log"
    summary_path = out_dir / "summary.json"
    cmd = docker_launch_command(bag_path, duration, rate, config)
    (out_dir / "command.txt").write_text(" ".join(shlex.quote(c) for c in cmd) + "\n")

    started_ts = time.time()
    timed_out = False
    return_code: int | None = None
    with log_path.open("w", encoding="utf-8", errors="replace") as fh:
        process = subprocess.Popen(
            cmd, stdout=fh, stderr=subprocess.STDOUT, text=True
        )
        try:
            return_code = process.wait(timeout=overall_timeout_sec)
        except subprocess.TimeoutExpired:
            timed_out = True
            process.terminate()
            try:
                return_code = process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                process.kill()
                return_code = process.wait(timeout=10)
    elapsed = time.time() - started_ts

    try:
        metrics = parse_runtime_log(log_path)
    except Exception as exc:
        metrics = {"parse_error": str(exc)}
    gate_info = parse_init_gate_log(log_path)

    final_pose_drift_m = None
    # final pose drift from last orb_track record
    for line in reversed(log_path.read_text(errors="replace").splitlines()):
        if "[orb_track]" in line and "pose_drift_m=" in line:
            m = re.search(r"pose_drift_m=([\d.eE+-]+)", line)
            if m:
                try:
                    final_pose_drift_m = float(m.group(1))
                except ValueError:
                    pass
                break

    summary = {
        "bag_name": bag_name,
        "bag_path": str(bag_path),
        "config_name": config.name,
        "config_overrides": config.launch_overrides,
        "run_index": run_index,
        "elapsed_wall_sec": elapsed,
        "return_code": return_code,
        "timed_out": timed_out,
        "metrics": metrics,
        "init_gate": gate_info,
        "final_pose_drift_m": final_pose_drift_m,
    }
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    return summary


def init_gate_pass(summary: dict[str, Any]) -> bool:
    """Plan's core InitGate gate: reset_post=0, imu_init (with exceptions), ok_ratio."""
    bag_name = summary["bag_name"]
    metrics = summary.get("metrics") or {}
    if metrics.get("parse_error"):
        return False
    imu_init = bool(metrics.get("imu_initialized"))
    reset_post = int(metrics.get("reset_count_after_init") or 0)
    ok_ratio = float(metrics.get("ok_ratio_after_init") or 0.0)
    if reset_post != 0:
        return False
    if ok_ratio < 0.90:
        return False
    if not imu_init and bag_name not in INIT_EXCEPTION_BAGS:
        return False
    return True


def drift_pass(summary: dict[str, Any]) -> bool:
    """Closed-loop drift gate, skipped for known-degenerate bags."""
    bag_name = summary["bag_name"]
    if bag_name in DRIFT_EXCEPTION_BAGS:
        return True
    drift = summary.get("final_pose_drift_m")
    if drift is None:
        return True
    return drift < 1.0


def passed(summary: dict[str, Any]) -> bool:
    return init_gate_pass(summary) and drift_pass(summary)


def write_csv(rows: list[dict[str, Any]], path: Path) -> None:
    fields = [
        "bag", "config", "run",
        "warmup_status",
        "reached_imu_ready",
        "time_to_tracking_sec",
        "time_to_imu_ready_sec",
        "imu_initialized",
        "reset_count_after_init",
        "ok_ratio_after_init",
        "final_pose_drift_m",
        "final_keyframes",
        "stability_pass",
        "init_gate_pass",
        "drift_pass",
        "overall_pass",
        "timed_out",
        "return_code",
        "elapsed_wall_sec",
    ]
    with path.open("w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        for s in rows:
            m = s.get("metrics") or {}
            gate = s.get("init_gate") or {}
            writer.writerow({
                "bag": s["bag_name"],
                "config": s["config_name"],
                "run": s["run_index"],
                "warmup_status": gate.get("warmup_status"),
                "reached_imu_ready": gate.get("reached_imu_ready"),
                "time_to_tracking_sec": gate.get("time_to_tracking_sec"),
                "time_to_imu_ready_sec": gate.get("time_to_imu_ready_sec"),
                "imu_initialized": m.get("imu_initialized"),
                "reset_count_after_init": m.get("reset_count_after_init"),
                "ok_ratio_after_init": (
                    round(m["ok_ratio_after_init"], 3)
                    if m.get("ok_ratio_after_init") is not None else None
                ),
                "final_pose_drift_m": s.get("final_pose_drift_m"),
                "final_keyframes": m.get("final_keyframes"),
                "stability_pass": m.get("stability_pass"),
                "init_gate_pass": init_gate_pass(s),
                "drift_pass": drift_pass(s),
                "overall_pass": passed(s),
                "timed_out": s.get("timed_out"),
                "return_code": s.get("return_code"),
                "elapsed_wall_sec": round(s.get("elapsed_wall_sec", 0.0), 1),
            })


def write_md(rows: list[dict[str, Any]], path: Path) -> None:
    init_pass = sum(1 for r in rows if init_gate_pass(r))
    drift_p = sum(1 for r in rows if drift_pass(r))
    overall = sum(1 for r in rows if passed(r))
    lines = [
        "# InitGate Matrix Report",
        "",
        f"Total runs: {len(rows)} | InitGate pass (core goal: reset_post=0 + imu_init + ok_ratio>=0.9): "
        f"{init_pass}/{len(rows)} | drift<1m (closed-loop): {drift_p}/{len(rows)} | overall: {overall}/{len(rows)}",
        "",
        "Drift-exception bags (skipped from drift gate): " + ", ".join(sorted(DRIFT_EXCEPTION_BAGS)),
        "",
        "| Bag | Config | Run | warmup | tracking_sec | imu_ready_sec | imu_init | reset_post | ok_ratio | drift_m | kf | init_gate | drift | overall |",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    for s in rows:
        m = s.get("metrics") or {}
        gate = s.get("init_gate") or {}
        lines.append(
            "| {bag} | {cfg} | {run} | {wm} | {tr} | {imr} | {ii} | {rp} | {ok} | {dr} | {kf} | {ig} | {dp} | {op} |".format(
                bag=s["bag_name"],
                cfg=s["config_name"],
                run=s["run_index"],
                wm=gate.get("warmup_status") or "-",
                tr=f"{gate.get('time_to_tracking_sec'):.2f}" if isinstance(gate.get("time_to_tracking_sec"), (int, float)) else "-",
                imr=f"{gate.get('time_to_imu_ready_sec'):.2f}" if isinstance(gate.get("time_to_imu_ready_sec"), (int, float)) else "-",
                ii="Y" if m.get("imu_initialized") else "N",
                rp=m.get("reset_count_after_init"),
                ok=f"{m.get('ok_ratio_after_init'):.2f}" if isinstance(m.get("ok_ratio_after_init"), (int, float)) else "-",
                dr=f"{s.get('final_pose_drift_m'):.2f}" if isinstance(s.get("final_pose_drift_m"), (int, float)) else "-",
                kf=m.get("final_keyframes"),
                ig="PASS" if init_gate_pass(s) else "FAIL",
                dp="PASS" if drift_pass(s) else "FAIL",
                op="PASS" if passed(s) else "FAIL",
            )
        )
    path.write_text("\n".join(lines) + "\n")


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--bags-dir", type=Path, default=None,
        help="If set, discover bag subdirs here instead of DEFAULT_BAGS.",
    )
    parser.add_argument(
        "--output-dir", type=Path,
        default=WORKSPACE_ROOT / "extra_ws/debug_runs/init_gate",
    )
    parser.add_argument("--runs", type=int, default=3, help="Repeat count per (bag, config)")
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument(
        "--configs", nargs="*", default=None,
        help="Subset of config names; default is all.",
    )
    parser.add_argument(
        "--bags", nargs="*", default=None,
        help="Subset of bag scenario names; default is all from DEFAULT_BAGS.",
    )
    parser.add_argument(
        "--smoke", action="store_true",
        help="Single run on first bag and first config; for self-test.",
    )
    args = parser.parse_args(argv)

    if args.bags_dir is not None:
        bags = []
        for sub in sorted(args.bags_dir.iterdir()):
            if sub.is_dir() and (sub / "metadata.yaml").exists():
                bags.append((sub, sub.name, None))
    else:
        bags = list(DEFAULT_BAGS)

    if args.bags is not None:
        wanted = set(args.bags)
        bags = [b for b in bags if b[1] in wanted]

    configs = list(CONFIGS)
    if args.configs is not None:
        wanted = set(args.configs)
        configs = [c for c in configs if c.name in wanted]

    if args.smoke:
        bags = bags[:1]
        configs = configs[:1]
        args.runs = 1

    args.output_dir.mkdir(parents=True, exist_ok=True)

    plan = []
    for bag_path, bag_name, duration_override in bags:
        bag_duration = duration_override or metadata_duration_sec(bag_path) or 60.0
        for cfg in configs:
            for run_idx in range(args.runs):
                plan.append((bag_path, bag_name, cfg, run_idx, bag_duration))

    print(f"Plan: {len(plan)} runs "
          f"({len(bags)} bags x {len(configs)} configs x {args.runs} runs)",
          flush=True)
    total_eta = sum(d / args.rate + 30.0 for _, _, _, _, d in plan)
    print(f"Approx wall time: {total_eta/60.0:.1f} min", flush=True)

    rows: list[dict[str, Any]] = []
    for index, (bag_path, bag_name, cfg, run_idx, duration) in enumerate(plan, start=1):
        overall_timeout = (duration / args.rate) + 60.0
        print(f"[{index}/{len(plan)}] {bag_name} | {cfg.name} | run {run_idx} "
              f"(dur={duration:.1f}s, timeout={overall_timeout:.0f}s)", flush=True)
        summary = run_one(
            bag_path=bag_path,
            bag_name=bag_name,
            config=cfg,
            run_index=run_idx,
            duration=duration,
            rate=args.rate,
            output_root=args.output_dir,
            overall_timeout_sec=overall_timeout,
        )
        rows.append(summary)
        status = "PASS" if passed(summary) else "FAIL"
        m = summary.get("metrics") or {}
        print(f"   -> {status}  imu_init={m.get('imu_initialized')} "
              f"reset_post={m.get('reset_count_after_init')} "
              f"ok_ratio={m.get('ok_ratio_after_init')}", flush=True)

    csv_path = args.output_dir / "summary.csv"
    md_path = args.output_dir / "summary.md"
    write_csv(rows, csv_path)
    write_md(rows, md_path)
    print(f"\nWrote {csv_path}")
    print(f"Wrote {md_path}")
    fails = [r for r in rows if not passed(r)]
    print(f"Failed: {len(fails)} / {len(rows)}")
    return 0 if not fails else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
