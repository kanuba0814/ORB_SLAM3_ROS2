#!/usr/bin/env python3
"""Generate guarded final ORB stability artifacts from a passing replay aggregate."""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path
from typing import Any


WORKSPACE = Path("/home/gxxl/slam_ws_v2")
EXTRA_WS = WORKSPACE / "extra_ws"
PACKAGE_ROOT = EXTRA_WS / "src" / "ORB_SLAM3_ROS2"
DEFAULT_OUTPUT_DIR = EXTRA_WS / "debug_runs" / "fix_until_stable"
DEFAULT_SETTINGS = PACKAGE_ROOT / "config" / "Monocular-Inertial" / "rmoncam_h30.yaml"


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"{path} is not a JSON object")
    return data


def metric(run: dict[str, Any], dotted_key: str) -> Any:
    value: Any = run
    for key in dotted_key.split("."):
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    return value


def require_passing_aggregate(aggregate: dict[str, Any], min_runs: int) -> list[dict[str, Any]]:
    runs = aggregate.get("runs")
    if not isinstance(runs, list):
        raise ValueError("aggregate JSON is missing a runs list")
    run_count = int(aggregate.get("run_count", len(runs)) or 0)
    pass_count = int(aggregate.get("pass_count", 0) or 0)
    all_passed = bool(aggregate.get("all_passed"))
    if run_count < min_runs or len(runs) < min_runs:
        raise ValueError(f"need at least {min_runs} replay runs, found {len(runs)}")
    if pass_count != len(runs) or not all_passed:
        raise ValueError(
            f"aggregate is not fully passing: pass_count={pass_count}, "
            f"runs={len(runs)}, all_passed={all_passed}"
        )
    for index, run in enumerate(runs, start=1):
        if not isinstance(run, dict):
            raise ValueError(f"run {index} is not a JSON object")
        if not run.get("stability_pass"):
            raise ValueError(f"run {index} has stability_pass={run.get('stability_pass')}")
        if int(run.get("returncode", 1) or 0) != 0:
            raise ValueError(f"run {index} has returncode={run.get('returncode')}")
        if not run.get("controlled_backlog_pass"):
            raise ValueError(
                f"run {index} has controlled_backlog_pass="
                f"{run.get('controlled_backlog_pass')}"
            )
        required_metrics = (
            "first_imu_initialized_time",
            "ok_ratio_after_init",
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
        )
        missing = [name for name in required_metrics if metric(run, name) is None]
        if missing:
            raise ValueError(
                f"run {index} is missing final report metrics: {', '.join(missing)}"
            )
    return runs


def render_metrics_table(runs: list[dict[str, Any]]) -> str:
    lines = [
        "| Run | IMU init time | OK ratio | Reset after init | Final KFs | "
        "Median inliers | Median MPs | Track p50/p95 ms | Image age p95 ms | "
        "Img buf max | Dropped images | ORB FPS p50 | Pose blocked | Invalid pose publishes |",
        "| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | "
        "---: | ---: | ---: | ---: | ---: |",
    ]
    for index, run in enumerate(runs, start=1):
        lines.append(
            f"| {index} | {run.get('first_imu_initialized_time')} | "
            f"{run.get('ok_ratio_after_init')} | "
            f"{run.get('reset_count_after_init')} | "
            f"{run.get('final_keyframes')} | "
            f"{run.get('median_inliers_after_init')} | "
            f"{run.get('median_tracked_map_points_after_init')} | "
            f"{metric(run, 'track_ms.p50')}/{metric(run, 'track_ms.p95')} | "
            f"{metric(run, 'processed_image_age_ms.p95')} | "
            f"{run.get('img_buf_size_max')} | "
            f"{run.get('dropped_img_count')} | "
            f"{metric(run, 'orb_processed_fps_est.p50')} | "
            f"{run.get('pose_publish_blocked_count')} | "
            f"{run.get('invalid_pose_publish_count')} |"
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
    runs = require_passing_aggregate(aggregate, args.min_runs)
    if not args.settings_file.exists():
        raise FileNotFoundError(args.settings_file)
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
    print(f"wrote final artifacts to {args.output_dir}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
