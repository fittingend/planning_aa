#!/usr/bin/env python3
"""Compare JSON captures from compare_runtime_probe.py, ignoring run IDs and stamps."""

import argparse
import json
import math


def maximum(values):
    return max(values, default=0.0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("original")
    parser.add_argument("adaptive")
    args = parser.parse_args()
    with open(args.original, encoding="utf-8") as file:
        original = json.load(file)
    with open(args.adaptive, encoding="utf-8") as file:
        adaptive = json.load(file)

    keys = ("start_x", "start_y", "goal_x", "goal_y")
    same_input = all(original["input"][key] == adaptive["input"][key]
                     for key in keys)
    same_response = original["response"] == adaptive["response"]
    same_route_count = len(original["routes"]) == len(adaptive["routes"]) >= 1
    route = {"segments_equal": False, "allow_modification_equal": False,
             "start_max_difference": None,
             "goal_max_difference": None}
    if same_route_count:
        a = original["routes"][0]
        b = adaptive["routes"][0]
        route = {
            "segments_equal": a["segments"] == b["segments"],
            "allow_modification_equal": a["allow_modification"] == b["allow_modification"],
            "start_max_difference": maximum(abs(x - y) for x, y in zip(a["start"], b["start"])),
            "goal_max_difference": maximum(abs(x - y) for x, y in zip(a["goal"], b["goal"])),
        }

    trajectories = []
    same_count = len(original["trajectories"]) == len(adaptive["trajectories"]) >= 3
    for a, b in zip(original["trajectories"], adaptive["trajectories"]):
        points_a, points_b = a["points"], b["points"]
        paired = list(zip(points_a, points_b))
        item = {"points_original": len(points_a), "points_adaptive": len(points_b),
                "frame_equal": a["frame"] == b["frame"]}
        if len(points_a) == len(points_b) and paired:
            item["position_max_m"] = maximum(
                math.dist(x["pose"][:3], y["pose"][:3]) for x, y in paired)
            item["quaternion_max"] = maximum(
                abs(u - v) for x, y in paired
                for u, v in zip(x["pose"][3:], y["pose"][3:]))
            for key in ("longitudinal_velocity_mps", "lateral_velocity_mps",
                        "acceleration_mps2", "heading_rate_rps",
                        "front_wheel_angle_rad", "rear_wheel_angle_rad",
                        "time_from_start_ns"):
                item[key + "_max_difference"] = maximum(
                    abs(x[key] - y[key]) for x, y in paired)
        trajectories.append(item)

    passed = (same_input and same_response and same_count and same_route_count and
              route["segments_equal"] and route["allow_modification_equal"] and
              route["start_max_difference"] < 1e-6 and
              route["goal_max_difference"] < 1e-6 and
              all(item["points_original"] == item["points_adaptive"] and
                  item["frame_equal"] and item["position_max_m"] < 1e-4 and
                  item["quaternion_max"] < 1e-5 and
                  item["longitudinal_velocity_mps_max_difference"] < 1e-5 and
                  item["lateral_velocity_mps_max_difference"] < 1e-5 and
                  item["acceleration_mps2_max_difference"] < 1e-5 and
                  item["heading_rate_rps_max_difference"] < 1e-5 and
                  item["front_wheel_angle_rad_max_difference"] < 1e-4 and
                  item["rear_wheel_angle_rad_max_difference"] < 1e-4 and
                  item["time_from_start_ns_max_difference"] == 0
                  for item in trajectories))
    print(json.dumps({"passed": passed, "same_input": same_input,
                      "same_response": same_response, "route": route,
                      "trajectories": trajectories}, indent=2))
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
