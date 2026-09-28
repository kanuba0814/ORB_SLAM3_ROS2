#!/usr/bin/env python3
# flake8: noqa
"""Extract matching /rosout messages from a rosbag2 sqlite bag."""

from __future__ import annotations

import argparse
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rcl_interfaces.msg import Log


def _resolve_bag_uri(path: Path) -> Path:
    path = path.expanduser().resolve()
    if path.name == "metadata.yaml" or path.suffix == ".db3":
        return path.parent
    return path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_path", type=Path)
    parser.add_argument("--contains", action="append", default=[])
    parser.add_argument("--limit", type=int, default=200)
    args = parser.parse_args()

    bag_uri = _resolve_bag_uri(args.bag_path)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_uri), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )

    printed = 0
    while reader.has_next():
        topic, data, storage_time_ns = reader.read_next()
        if topic != "/rosout":
            continue
        msg = deserialize_message(data, Log)
        text = msg.msg
        if args.contains and not any(pattern in text for pattern in args.contains):
            continue
        print(f"{storage_time_ns * 1e-9:.9f} {msg.name}: {text}")
        printed += 1
        if printed >= args.limit:
            break
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
