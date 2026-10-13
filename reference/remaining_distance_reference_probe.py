"""Observe the original remaining-distance node's gated parking output."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from autoware_internal_msgs.msg import MissionRemainingDistanceTime
from tier4_planning_msgs.msg import Scenario


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    with args.output.with_suffix(".log").open("w") as log:
        process = subprocess.Popen(
            ["ros2", "launch", "/probe/reference/remaining_distance_reference_probe.launch.xml"],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = Node("remaining_distance_reference_probe")
        scenario_pub = node.create_publisher(Scenario, "/probe/scenario", 10)
        received = []
        node.create_subscription(
            MissionRemainingDistanceTime, "/probe/mission_remaining_distance_time",
            lambda msg: received.append((time.monotonic_ns(),
                                         msg.remaining_distance,
                                         msg.remaining_time)), 10)
        try:
            def spin(seconds, scenario=None):
                deadline = time.monotonic() + seconds
                while time.monotonic() < deadline:
                    if scenario is not None:
                        message = Scenario()
                        message.current_scenario = scenario
                        message.activating_scenarios = [scenario]
                        scenario_pub.publish(message)
                    rclpy.spin_once(node, timeout_sec=0.05)

            spin(0.6)
            before = len(received)
            spin(0.6, Scenario.LANEDRIVING)
            lane = len(received) - before
            spin(1.2, Scenario.PARKING)
            parking = received[before + lane:]
            topic = "/probe/mission_remaining_distance_time"
            graph = {
                "types": dict(node.get_topic_names_and_types()).get(topic, []),
                "publishers": [{
                    "durability": str(info.qos_profile.durability),
                    "reliability": str(info.qos_profile.reliability),
                    "depth": info.qos_profile.depth,
                } for info in node.get_publishers_info_by_topic(topic)],
            }
            result = {
                "before_scenario_count": before,
                "lane_without_map_route_count": lane,
                "parking_count": len(parking),
                "parking_values": [[distance, remaining] for _, distance, remaining in parking],
                "parking_intervals_ms": [(b[0] - a[0]) / 1e6
                                         for a, b in zip(parking, parking[1:])],
                "graph": graph,
            }
            if before or lane or not parking or any(
                    distance != 0.0 or remaining != 0.0
                    for _, distance, remaining in parking):
                raise AssertionError("원본 parking 발행 게이트가 예상과 다릅니다")
            args.output.write_text(json.dumps(result, indent=2) + "\n")
            print(json.dumps({"parking_count": len(parking), "graph": graph}))
        finally:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
