"""Validate that a bounded follow test contained sustained tracking motion."""

import csv
import math
import sys


def finite(value: str) -> float:
    number = float(value)
    return number if math.isfinite(number) else math.nan


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("usage: analyze_follow_validation.py RESULTS.csv")

    with open(sys.argv[1], newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))

    if len(rows) < 20:
        raise SystemExit(f"FAIL: only {len(rows)} telemetry samples")

    if "tracking_enabled" in rows[0]:
        active_rows = [
            row
            for row in rows
            if row["tracking_enabled"].strip().lower() == "true"
        ]
    else:
        active_rows = rows

    if len(active_rows) < 20:
        raise SystemExit(
            f"FAIL: only {len(active_rows)} enabled telemetry samples"
        )

    elapsed = [float(row["elapsed_seconds"]) for row in active_rows]
    duration = elapsed[-1] - elapsed[0]
    commands = [
        (float(row["command_x"]), float(row["command_yaw"]))
        for row in active_rows
    ]
    commanded_samples = sum(
        abs(forward) >= 0.02 or abs(yaw) >= 0.02
        for forward, yaw in commands
    )

    positions = [
        (
            float(row["pose_x"]),
            float(row["pose_y"]),
            float(row["pose_z"]),
        )
        for row in active_rows
    ]
    horizontal_path = sum(
        math.hypot(current[0] - previous[0], current[1] - previous[1])
        for previous, current in zip(positions, positions[1:])
    )
    target_ages = [
        finite(row["target_age_seconds"]) for row in active_rows
    ]
    fresh_targets = sum(
        math.isfinite(age) and age <= 3.0 for age in target_ages
    )
    fresh_ratio = fresh_targets / len(active_rows)
    minimum_altitude = min(position[2] for position in positions)
    distances = [
        finite(row.get("estimated_distance_metres", "nan"))
        for row in active_rows
    ]
    distances = [distance for distance in distances if math.isfinite(distance)]
    desired_distance = 2.5
    final_distance = math.nan
    if distances:
        final_count = max(1, len(distances) // 5)
        final_values = sorted(distances[-final_count:])
        final_distance = final_values[len(final_values) // 2]
    final_distance_error = abs(final_distance - desired_distance)

    print(f"samples={len(active_rows)}")
    print(f"duration_seconds={duration:.3f}")
    print(f"commanded_samples={commanded_samples}")
    print(f"horizontal_path_metres={horizontal_path:.3f}")
    print(f"fresh_target_ratio={fresh_ratio:.3f}")
    print(f"minimum_altitude_metres={minimum_altitude:.3f}")
    print(f"final_estimated_distance_metres={final_distance:.3f}")
    print(f"final_distance_error_metres={final_distance_error:.3f}")

    failures = []
    if duration < 12.0:
        failures.append("telemetry duration was under 12 seconds")
    if commanded_samples < 10:
        failures.append("fewer than 10 nonzero command samples")
    if horizontal_path < 0.15:
        failures.append("drone path was under 0.15 metres")
    if fresh_ratio < 0.5:
        failures.append("runner target was fresh for under half the test")
    if minimum_altitude < 1.0:
        failures.append("vehicle was not airborne for the validation")
    if not distances:
        failures.append("no runner distance estimates were recorded")
    elif final_distance_error > 0.75:
        failures.append("final trailing distance error exceeded 0.75 metres")

    if failures:
        raise SystemExit("FAIL: " + "; ".join(failures))
    print("FOLLOW_VALIDATION_PASS")


if __name__ == "__main__":
    main()
