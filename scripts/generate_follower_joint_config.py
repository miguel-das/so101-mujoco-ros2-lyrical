#!/usr/bin/env python3
"""Convert a LeRobot SO-101 follower calibration to Feetech joint YAML."""

import argparse
import json
from pathlib import Path


JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
FIELDS = ("id", "homing_offset", "range_min", "range_max")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("calibration_json", type=Path, help="LeRobot follower calibration JSON")
    args = parser.parse_args()
    output = Path(__file__).resolve().parents[1] / "src/so101_bringup/config/follower_joints.yaml"

    try:
        calibration = json.loads(args.calibration_json.expanduser().read_text(encoding="utf-8"))
        if not isinstance(calibration, dict):
            raise ValueError("expected a JSON object mapping joint names to calibration fields")

        lines = ["joints:"]
        for joint in JOINTS:
            if joint not in calibration:
                raise ValueError(f"missing joint '{joint}'")
            values = calibration[joint]
            if not isinstance(values, dict):
                raise ValueError(f"joint '{joint}' must contain a JSON object")
            lines.append(f"  {joint}:")
            for field in FIELDS:
                if field not in values:
                    raise ValueError(f"joint '{joint}' is missing field '{field}'")
                value = values[field]
                if type(value) is not int:
                    raise ValueError(f"joint '{joint}' field '{field}' must be an integer")
                lines.append(f"    {field}: {value}")

        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text("\n".join(lines) + "\n", encoding="utf-8")
    except (OSError, ValueError) as exc:
        parser.exit(1, f"error: {exc}\n")

    print(output)


if __name__ == "__main__":
    main()
