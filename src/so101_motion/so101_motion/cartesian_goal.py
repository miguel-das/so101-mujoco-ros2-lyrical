"""Plan (and optionally execute) the SO-101 arm to a Cartesian goal with MoveItPy.

Send a TCP position on ~/goal (geometry_msgs/PointStamped). The gripper
approaches it `approach_pitch` rad below horizontal (pi/2 = straight down),
keeping the current wrist roll. The goal is converted to a joint-space goal
with ApproachIK (the collision-free solution closest to the current state)
and planned with the default MoveItPy pipeline.
"""

import copy
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from moveit.planning import MoveItPy
from rclpy.node import Node

from so101_motion.approach_ik import ApproachIK


class CartesianGoal(Node):
    def __init__(self, moveit):
        super().__init__("cartesian_goal")
        self.declare_parameter("planning_group", "arm")
        self.declare_parameter("tip_link", "gripper_frame_link")
        self.declare_parameter("approach_pitch", math.pi / 2)
        self.declare_parameter("execute", False)

        self._group = self.get_parameter("planning_group").value
        self._tip_link = self.get_parameter("tip_link").value
        self._moveit = moveit
        self._robot_model = moveit.get_robot_model()
        self._arm = moveit.get_planning_component(self._group)
        self._scene_monitor = moveit.get_planning_scene_monitor()
        self._ik = ApproachIK(self._robot_model, self._group, self._tip_link)

        self.create_subscription(PointStamped, "~/goal", self._on_goal, 1)
        self.get_logger().info(
            f"Ready. Publish a geometry_msgs/PointStamped on {self.resolve_topic_name('~/goal')}"
        )

    def _on_goal(self, msg):
        pitch = self.get_parameter("approach_pitch").value
        try:
            position = self._to_model_frame(msg)
        except ValueError as e:
            self.get_logger().error(str(e))
            return
        self.get_logger().info(
            f"Goal {np.round(position, 4).tolist()} in {self._robot_model.model_frame}, "
            f"approach pitch {math.degrees(pitch):.1f} deg"
        )

        self._arm.set_start_state_to_current_state()
        start = self._arm.get_start_state()
        current = np.asarray(start.get_joint_group_positions(self._group))
        solutions = sorted(
            self._ik.solve(position, pitch, roll=current[-1]),
            key=lambda q: np.linalg.norm(q - current),
        )
        q = next((q for q in solutions if self._collision_free(start, q)), None)
        if q is None:
            self.get_logger().error(
                "No collision-free IK solution: goal unreachable with this approach pitch"
            )
            return
        pos_err, pitch_err = self._ik.errors(q, position, pitch)
        self.get_logger().info(
            f"IK: q = {np.round(q, 3).tolist()} "
            f"(pos err {pos_err * 1e3:.2f} mm, pitch err {math.degrees(pitch_err):.3f} deg)"
        )

        self._arm.set_goal_state(robot_state=self._state(start, q))
        result = self._arm.plan()
        if not result:
            self.get_logger().error("Planning failed")
            return
        self.get_logger().info(
            f"Planned trajectory: {result.trajectory.duration:.2f} s"
        )

        if self.get_parameter("execute").value:
            self._moveit.execute(result.trajectory, controllers=[])
            self.get_logger().info("Execution finished")

    def _to_model_frame(self, msg):
        p = np.array([msg.point.x, msg.point.y, msg.point.z])
        frame = msg.header.frame_id
        if not frame or frame == self._robot_model.model_frame:
            return p
        with self._scene_monitor.read_only() as scene:
            if not scene.knows_frame_transform(frame):
                raise ValueError(f"Unknown frame '{frame}'")
            T = scene.get_frame_transform(frame)
        return T[:3, :3] @ p + T[:3, 3]

    def _state(self, start, q):
        """`start` with the arm group set to `q` (other joints, e.g. the gripper, unchanged)."""
        state = copy.copy(start)
        state.set_joint_group_positions(self._group, q)
        state.update()
        return state

    def _collision_free(self, start, q):
        with self._scene_monitor.read_only() as scene:
            return scene.is_state_valid(self._state(start, q), self._group)


def main():
    rclpy.init()
    moveit = MoveItPy(node_name="moveit_py")
    node = CartesianGoal(moveit)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        moveit.shutdown()
        rclpy.try_shutdown()
