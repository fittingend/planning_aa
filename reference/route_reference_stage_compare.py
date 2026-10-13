"""Trace source-derived RouteHandler geometry against recorded root path markers.

This uses the same map, route, and latest odometry as the independent replay.
The original ROS RouteHandler cannot be linked in the ROS-less build; the
geometry and ordering here follow its checked-in source, while marker pairing
and callback odometry remain inferred from bag receipt times.
"""

import argparse
import bisect
import math
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from offline_rosbag_cdr import (
    decode_lanelet_map, decode_lanelet_route, decode_marker_array,
    decode_odometry, messages,
)

PREFIX = "/planning/scenario_planning/lane_driving/behavior_planning/"


def project(line, point):
    arc = 0.0
    best = (float("inf"), 0.0)
    for a, b in zip(line, line[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        if length < 1e-9:
            continue
        ratio = max(0.0, min(1.0, ((point[0] - a[0]) * dx +
                                      (point[1] - a[1]) * dy) / length**2))
        error = math.hypot(point[0] - a[0] - ratio * dx,
                           point[1] - a[1] - ratio * dy)
        if error < best[0]:
            best = error, arc + ratio * length
        arc += length
    return best


def at_arc(line, arc):
    traveled = 0.0
    for a, b in zip(line, line[1:]):
        length = math.dist(a, b)
        if traveled + length >= arc:
            ratio = (arc - traveled) / length if length else 0.0
            return (a[0] + ratio * (b[0] - a[0]),
                    a[1] + ratio * (b[1] - a[1]))
        traveled += length
    return line[-1]


def length(line):
    return sum(math.dist(a, b) for a, b in zip(line, line[1:]))


def read_geometry(probe, map_data, segments):
    sections = [[item[0] for item in section["primitives"]]
                for section in segments]
    with tempfile.NamedTemporaryFile() as map_file:
        map_file.write(map_data)
        map_file.flush()
        data = "\n".join([str(len(sections))] +
                         [" ".join(map(str, [len(ids), *ids]))
                          for ids in sections]) + "\n"
        result = subprocess.run([str(probe), map_file.name], input=data,
                                text=True, capture_output=True, check=True)
    tokens = iter(result.stdout.split())
    count = int(next(tokens))
    if count != len(sections):
        raise ValueError("geometry section count mismatch")
    centerlines = {}
    for _ in sections:
        for _ in range(int(next(tokens))):
            lane_id, n = int(next(tokens)), int(next(tokens))
            speed = float(next(tokens))
            centerlines[lane_id] = ([(float(next(tokens)), float(next(tokens)))
                                     for _ in range(n)], speed)
    successors = {}
    for ids in sections[:-1]:
        for _ in ids:
            lane_id, next_id = int(next(tokens)), int(next(tokens))
            if next_id:
                successors[lane_id] = next_id
    return sections, centerlines, successors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("probe", type=Path)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--frames", type=int, default=110)
    parser.add_argument("--offset", type=int, default=0)
    parser.add_argument("--backward-length", type=float, default=7.0)
    parser.add_argument("--odometry-clock", choices=("behavior", "marker"),
                        default="behavior")
    args = parser.parse_args()
    route_record = next(messages(args.bag, "/planning/mission_planning/route"))
    route = decode_lanelet_route(route_record[1])
    map_data = decode_lanelet_map(next(messages(
        args.bag, "/map/vector_map"))[1])
    sections, centerlines, successors = read_geometry(
        args.probe, map_data, route["segments"])
    odometry = list(messages(args.bag, "/localization/kinematic_state"))
    times = [stamp for stamp, _ in odometry]
    behavior = [(stamp, raw) for stamp, raw in messages(
        args.bag, PREFIX + "path_with_lane_id")
        if stamp >= max(route_record[0], times[0])]
    markers = list(messages(args.bag, PREFIX +
                            "behavior_path_planner/debug/root_reference_path"))
    marker_times = [stamp for stamp, _ in markers]
    current = None
    all_ids = [lane_id for section in sections for lane_id in section]
    print("frame lane sequence ego_arc raw_start_arc predicted_root_arc "
          "marker_root_arc error_m odom_age_ms marker_age_ms")
    for frame, (receipt, _) in enumerate(
            behavior[args.offset:args.offset + args.frames], args.offset):
        marker_index = bisect.bisect_right(marker_times, receipt) - 1
        if marker_index < 0:
            continue
        input_time = (marker_times[marker_index]
                      if args.odometry_clock == "marker" else receipt)
        odom_index = bisect.bisect_right(times, input_time) - 1
        if odom_index < 0:
            continue
        odom = decode_odometry(odometry[odom_index][1])
        xy = odom["x"], odom["y"]
        # PlannerManager first searches prior lane and its route neighbors;
        # falling back to the closest route lane is explicit here.
        if current is None:
            candidates = all_ids
        else:
            candidates = [current]
            candidates.extend(id_ for id_, next_id in successors.items()
                              if next_id == current)
            if current in successors:
                candidates.append(successors[current])
        current = min(candidates, key=lambda id_: project(
            centerlines[id_][0], xy)[0])
        current_section = next(i for i, ids in enumerate(sections)
                               if current in ids)
        sequence = [current]
        while sequence[-1] in successors and len(sequence) < len(sections):
            sequence.append(successors[sequence[-1]])
        # RouteHandler::getLaneletSequence includes a previous lane when the
        # ego is less than backward+10 m into the current lane.
        ego_lane_arc = project(centerlines[current][0], xy)[1]
        margin = args.backward_length + 10.0
        if ego_lane_arc < margin and current_section:
            predecessors = [id_ for id_, next_id in successors.items()
                            if next_id == current]
            if predecessors:
                sequence.insert(0, predecessors[0])
        line = []
        for lane_id in sequence:
            lane = centerlines[lane_id][0]
            line.extend(lane if not line else lane[1:])
        ego_arc = project(line, xy)[1]
        goal_xy = route["goal"][:2]
        goal_arc = project(line, goal_xy)[1]
        raw_start = max(0.0, ego_arc - margin)
        raw_end = min(length(line), ego_arc + 300.0, goal_arc)
        if raw_end < raw_start:
            raw_end = raw_start
        sample_arcs = []
        sample = raw_start
        while sample < raw_end:
            sample_arcs.append(sample)
            sample += 2.0
        sample_arcs.append(raw_end)
        # motion_utils::cropBackwardPoints starts at the sample after the
        # threshold-crossing segment (begin + i), not at begin + i - 1.
        # The threshold may coincide with a 2 m sample; the source starts
        # with the following sample. Use an epsilon here because this probe
        # projects flattened 2D centerlines instead of ROS's 3D path points.
        threshold = ego_arc - args.backward_length - 2.0
        begin = max(0, min(len(sample_arcs) - 2,
                           bisect.bisect_right(sample_arcs,
                                               threshold + 1e-7)))
        predicted_arc = sample_arcs[begin]
        predicted_xy = at_arc(line, predicted_arc)
        marker_points = decode_marker_array(markers[marker_index][1])[0]["points"]
        marker_xy = marker_points[0][:2]
        marker_arc = project(line, marker_xy)[1]
        error = math.dist(predicted_xy, marker_xy)
        print(f"{frame} {current} {','.join(map(str, sequence))} "
              f"{ego_arc:.3f} {raw_start:.3f} {predicted_arc:.3f} "
              f"{marker_arc:.3f} {error:.3f} "
              f"{(receipt-times[odom_index])*1e-6:.1f} "
              f"{(receipt-marker_times[marker_index])*1e-6:.1f}")


if __name__ == "__main__":
    main()
