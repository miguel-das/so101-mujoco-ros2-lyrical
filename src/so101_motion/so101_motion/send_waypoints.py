"""Send an n x 5 waypoint file to motion_server's ~/move_through_waypoints action.

Each row is one waypoint: x, y, z (m), pitch, roll (rad). The file is either a
CSV (comma-separated, `#` comments) or a YAML list of rows, optionally under a
`waypoints` key next to a `frame_id`.

    ros2 run so101_motion send_waypoints square.csv --frame base_link

Prints the goal's feedback and result. Ctrl+C cancels the goal.
"""

import argparse
import signal
import sys
import threading
import time

import numpy as np
import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from so101_interfaces.action import MoveThroughWaypoints

STATES = {
    MoveThroughWaypoints.Feedback.PLANNING: "planning",
    MoveThroughWaypoints.Feedback.WAITING_FOR_CONFIRMATION: "waiting for Next in RViz",
    MoveThroughWaypoints.Feedback.EXECUTING: "executing",
}


def load(path):
    """Waypoint matrix and frame_id (None if the file gives none)."""
    frame = None
    if path.endswith((".yaml", ".yml")):
        with open(path) as f:
            data = yaml.safe_load(f)
        if isinstance(data, dict):
            frame = data.get("frame_id")
            data = data.get("waypoints")
        waypoints = np.array(data, dtype=float, ndmin=2)
    else:
        waypoints = np.loadtxt(path, delimiter=",", comments="#", ndmin=2)
    if waypoints.ndim != 2 or waypoints.shape[1] != 5:
        raise ValueError(f"expected n x 5 waypoints (x, y, z, pitch, roll), got shape {waypoints.shape}")
    return waypoints, frame


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("file", help="CSV or YAML file with one x, y, z, pitch, roll row per waypoint")
    parser.add_argument("--frame", help="frame of x, y, z (default: the file's frame_id, else base_link)")
    parser.add_argument("--action", default="/motion_server/move_through_waypoints")
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    try:
        waypoints, frame = load(args.file)
    except (OSError, ValueError, yaml.YAMLError) as e:
        sys.exit(f"Cannot read {args.file}: {e}")
    frame = args.frame or frame or "base_link"
    print(f"{len(waypoints)} waypoints in {frame} (x, y, z, pitch, roll):")
    print(np.array2string(waypoints, precision=4, suppress_small=True))

    # Ctrl+C only sets a flag: interrupting rclpy mid-spin would leave it unable
    # to send the cancel request.
    interrupted = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: interrupted.set())
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("send_waypoints")

    def wait(future, timeout=None):
        """Spin until `future` is done; False if Ctrl+C or `timeout` (s) came first."""
        end = None if timeout is None else time.monotonic() + timeout
        while not future.done():
            if interrupted.is_set() or (end is not None and time.monotonic() > end):
                return False
            rclpy.spin_once(node, timeout_sec=0.1)
        return True

    try:
        client = ActionClient(node, MoveThroughWaypoints, args.action)
        if not client.wait_for_server(timeout_sec=10.0):
            sys.exit(f"Action server {args.action} not available")
        goal = MoveThroughWaypoints.Goal(waypoints=waypoints.ravel().tolist(), frame_id=frame)
        future = client.send_goal_async(
            goal, feedback_callback=lambda msg: print(f"... {STATES[msg.feedback.state]}")
        )
        if not wait(future):
            sys.exit("Interrupted before the goal was accepted")
        goal_handle = future.result()
        if not goal_handle.accepted:
            sys.exit("Goal rejected: motion_server is planning or executing another goal")

        result = goal_handle.get_result_async()
        if not wait(result):
            print("Canceling goal")
            interrupted.clear()
            wait(goal_handle.cancel_goal_async(), timeout=5.0)
            wait(result, timeout=10.0)
        if not result.done():
            sys.exit("No result from motion_server")
        result = result.result().result
        print(("Succeeded: " if result.success else "Failed: ") + result.message)
        sys.exit(0 if result.success else 1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
