#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Observe route selector responses when its mission planner service is absent."""

import json
import os
from pathlib import Path
import signal
import subprocess

import rclpy
from rclpy.node import Node
from tier4_planning_msgs.srv import ClearRoute, SetWaypointRoute


OUTPUT = Path("/probe/reference/route_selector_unready_reference.json")


def call(node, name, service_type, request):
    client = node.create_client(service_type, "/probe/route_selector/" + name)
    assert client.wait_for_service(timeout_sec=10.0), name
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    assert future.done() and future.result() is not None, name
    status = future.result().status
    return {"service": name, "success": bool(status.success),
            "code": int(status.code), "message": status.message}


def main():
    log = OUTPUT.with_suffix(".log")
    with log.open("w") as stream:
        process = subprocess.Popen(
            ["ros2", "launch", "/probe/reference/route_selector_unready_probe.launch.xml"],
            stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        node = None
        try:
            rclpy.init()
            node = Node("planning_aa_route_selector_unready_probe")
            waypoint = SetWaypointRoute.Request()
            waypoint.header.frame_id = "map"
            waypoint.goal_pose.orientation.w = 1.0
            result = [
                call(node, "main/set_waypoint_route", SetWaypointRoute, waypoint),
                call(node, "mrm/set_waypoint_route", SetWaypointRoute, waypoint),
                call(node, "mrm/clear_route", ClearRoute, ClearRoute.Request()),
            ]
            assert all(not item["success"] and item["code"] == 50001
                       for item in result), result
            OUTPUT.write_text(json.dumps(result, indent=2) + "\n")
            print(json.dumps(result))
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)


if __name__ == "__main__":
    main()
