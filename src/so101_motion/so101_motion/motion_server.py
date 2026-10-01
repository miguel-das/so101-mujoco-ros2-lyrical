"""Plan and execute SO-101 arm motions with MoveItPy, behind ROS actions.

~/move_to_point (so101_interfaces/MoveToPoint) moves the gripper TCP to a
point. The gripper approaches it `approach_pitch` rad below horizontal (pi/2 =
straight down), keeping the current wrist roll. The goal is converted to a
joint-space goal with ApproachIK (the collision-free solution closest to the
current state).

~/move_to_joints (so101_interfaces/MoveToJoints) moves the arm joints to
target positions, given in planning group order or by joint name (unnamed
joints keep their current position).

~/move_through_waypoints (so101_interfaces/MoveThroughWaypoints) reaches the
first of n (x, y, z, pitch, roll) waypoints with a free path, then moves the
TCP along straight segments through the others. Each segment is sampled every
`cartesian_step` m and solved with ApproachIK, keeping the solution closest to
the previous sample; it fails if a sample is unreachable or in collision, or if
a joint jumps more than `max_joint_step` rad between samples.

Free paths are planned with the default MoveItPy pipeline. With `confirm` (the
default), the plan is only previewed in RViz on /display_planned_path. Press
Next in the RvizVisualToolsGui panel to execute it, or Stop to discard it. A
new goal on any action replaces a plan still waiting. Stop or canceling the
goal also stops a running execution.
"""

import copy
import functools
import math
import threading

import numpy as np
import rclpy
from moveit.core.robot_state import robotStateToRobotStateMsg
from moveit.planning import MoveItPy
from moveit_msgs.msg import DisplayTrajectory
from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectoryPoint

from so101_interfaces.action import MoveThroughWaypoints, MoveToJoints, MoveToPoint
from so101_motion.approach_ik import ApproachIK

# sensor_msgs/Joy published by the rviz_visual_tools RvizVisualToolsGui panel
GUI_TOPIC = "/rviz_visual_tools_gui"
NEXT_BUTTON = 1
STOP_BUTTON = 4
# Topic of the RViz plan preview, also used by MoveIt's planning pipeline
DISPLAY_TOPIC = "/display_planned_path"
# Max pitch or roll change between two samples of a straight segment, in rad
ANGLE_STEP = 0.02


class GoalFailed(Exception):
    pass


class Decision:
    """What happened to a plan waiting for confirmation."""

    NEXT = "next"
    STOP = "stop"
    CANCEL = "cancel"
    REPLACED = "replaced"

    def __init__(self, goal_handle):
        self.goal_handle = goal_handle
        self.choice = None
        self._event = threading.Event()

    def decide(self, choice):
        self.choice = choice
        self._event.set()

    def wait(self):
        self._event.wait()
        return self.choice


class MotionServer(Node):
    def __init__(self, moveit):
        super().__init__("motion_server")
        self.declare_parameter("planning_group", "arm")
        self.declare_parameter("tip_link", "gripper_frame_link")
        self.declare_parameter("confirm", True)
        self.declare_parameter("cartesian_step", 0.005)
        self.declare_parameter("max_joint_step", 0.1)
        self.declare_parameter("max_velocity_scaling_factor", 0.1)
        self.declare_parameter("max_acceleration_scaling_factor", 0.1)

        self._group = self.get_parameter("planning_group").value
        self._tip_link = self.get_parameter("tip_link").value
        self._moveit = moveit
        self._robot_model = moveit.get_robot_model()
        self._arm = moveit.get_planning_component(self._group)
        self._scene_monitor = moveit.get_planning_scene_monitor()
        self._execution_manager = moveit.get_trajectory_execution_manager()
        self._ik = ApproachIK(self._robot_model, self._group, self._tip_link)
        jmg = self._robot_model.get_joint_model_group(self._group)
        self._joints = list(jmg.active_joint_model_names)
        self._bounds = [
            (b[0].min_position, b[0].max_position) for b in jmg.active_joint_model_bounds
        ]

        # Goals run in executor threads; these fields are shared with the
        # button and cancel callbacks and guarded by the lock.
        self._lock = threading.Lock()
        self._active = None  # goal handle planning or executing
        self._executing = False
        self._waiting = None  # Decision of the plan waiting for confirmation

        actions = [
            (MoveToPoint, "~/move_to_point", self._plan_point),
            (MoveToJoints, "~/move_to_joints", self._plan_joints),
            (MoveThroughWaypoints, "~/move_through_waypoints", self._plan_waypoints),
        ]
        group = ReentrantCallbackGroup()
        for action, name, plan in actions:
            ActionServer(
                self,
                action,
                name,
                execute_callback=functools.partial(self._run_goal, action=action, plan=plan),
                goal_callback=self._on_goal_request,
                cancel_callback=self._on_cancel_request,
                callback_group=group,
            )
        self.create_subscription(
            Joy,
            GUI_TOPIC,
            self._on_button,
            10,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self._display = self.create_publisher(DisplayTrajectory, DISPLAY_TOPIC, 1)
        names = ", ".join(self.resolve_topic_name(name) for _, name, _ in actions)
        self.get_logger().info(f"Ready. Actions: {names}")

    def _on_goal_request(self, goal):
        with self._lock:
            if self._active is not None:
                self.get_logger().warning("Goal rejected: still planning or executing")
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel_request(self, goal_handle):
        stop = False
        with self._lock:
            if self._waiting is not None and self._waiting.goal_handle is goal_handle:
                self._waiting.decide(Decision.CANCEL)
                self._waiting = None
            elif self._active is goal_handle and self._executing:
                stop = True
        if stop:
            self._execution_manager.stop_execution()
        return CancelResponse.ACCEPT

    def _on_button(self, msg):
        def pressed(i):
            return len(msg.buttons) > i and msg.buttons[i] != 0

        stop = False
        with self._lock:
            waiting = self._waiting
            if pressed(NEXT_BUTTON):
                if waiting is None:
                    self.get_logger().warning("No plan waiting: send a goal first")
                    return
                self._waiting = None
                self._active = waiting.goal_handle
                waiting.decide(Decision.NEXT)
            elif pressed(STOP_BUTTON):
                if waiting is not None:
                    self._waiting = None
                    waiting.decide(Decision.STOP)
                else:
                    stop = self._executing
        if stop:
            self.get_logger().info("Stopping execution")
            self._execution_manager.stop_execution()

    def _run_goal(self, goal_handle, action, plan):
        with self._lock:
            if self._active is not None:
                return self._abort(goal_handle, action, "Still planning or executing another goal")
            self._active = goal_handle
            if self._waiting is not None:
                self._waiting.decide(Decision.REPLACED)
                self._waiting = None
        try:
            return self._run(goal_handle, action, plan)
        finally:
            with self._lock:
                if self._active is goal_handle:
                    self._active = None
                    self._executing = False

    def _run(self, goal_handle, action, plan):
        self._feedback(goal_handle, action, action.Feedback.PLANNING)
        try:
            trajectory = plan(goal_handle.request)
        except GoalFailed as e:
            return self._abort(goal_handle, action, str(e))
        if goal_handle.is_cancel_requested:
            return self._canceled(goal_handle, action)

        if self.get_parameter("confirm").value:
            decision = Decision(goal_handle)
            with self._lock:
                self._waiting = decision
                self._active = None  # a new goal may replace this plan
            self._feedback(goal_handle, action, action.Feedback.WAITING_FOR_CONFIRMATION)
            self.get_logger().info(
                "Check the plan in RViz, then press Next to execute it or Stop to discard it"
            )
            choice = decision.wait()
            if choice == Decision.CANCEL:
                return self._canceled(goal_handle, action)
            if choice == Decision.STOP:
                return self._abort(goal_handle, action, "Plan discarded with Stop", error=False)
            if choice == Decision.REPLACED:
                return self._abort(goal_handle, action, "Plan replaced by a new goal", error=False)

        with self._lock:
            self._executing = True
        self._feedback(goal_handle, action, action.Feedback.EXECUTING)
        self.get_logger().info("Executing")
        status = self._moveit.execute(trajectory, controllers=[])
        with self._lock:
            self._executing = False
        if goal_handle.is_cancel_requested:
            return self._canceled(goal_handle, action)
        if status.status == "PREEMPTED":
            return self._abort(goal_handle, action, "Execution stopped", error=False)
        if not status:
            return self._abort(goal_handle, action, f"Execution ended with status {status.status}")
        self.get_logger().info("Execution finished")
        goal_handle.succeed()
        return action.Result(success=True, message="Goal reached")

    def _plan_point(self, goal):
        target = goal.target
        position = self._to_model_frame(
            [target.point.x, target.point.y, target.point.z], target.header.frame_id
        )
        pitch = goal.approach_pitch
        self.get_logger().info(
            f"Goal {np.round(position, 4).tolist()} in {self._robot_model.model_frame}, "
            f"approach pitch {math.degrees(pitch):.1f} deg"
        )

        start = self._current_state()
        current = np.asarray(start.get_joint_group_positions(self._group))
        q = self._closest_ik(start, current, position, pitch, current[-1])
        if q is None:
            raise GoalFailed("No collision-free IK solution: goal unreachable with this approach pitch")
        pos_err, pitch_err = self._ik.errors(q, position, pitch)
        self.get_logger().info(
            f"IK: q = {np.round(q, 3).tolist()} "
            f"(pos err {pos_err * 1e3:.2f} mm, pitch err {math.degrees(pitch_err):.3f} deg)"
        )
        return self._plan_to(start, q)

    def _plan_joints(self, goal):
        start = self._current_state()
        q = np.asarray(start.get_joint_group_positions(self._group))
        names = list(goal.joint_names)
        if not names:
            if len(goal.positions) != len(self._joints):
                raise GoalFailed(
                    f"Expected {len(self._joints)} positions ({', '.join(self._joints)}), "
                    f"got {len(goal.positions)}"
                )
            names = self._joints
        elif len(names) != len(goal.positions):
            raise GoalFailed(f"{len(names)} joint names but {len(goal.positions)} positions")
        unknown = [n for n in names if n not in self._joints]
        if unknown:
            raise GoalFailed(f"Not {self._group} joints: {unknown}. Valid joints: {self._joints}")
        if len(set(names)) != len(names):
            raise GoalFailed(f"Duplicate joint names: {names}")

        for name, position in zip(names, goal.positions):
            i = self._joints.index(name)
            lower, upper = self._bounds[i]
            if not lower <= position <= upper:
                raise GoalFailed(
                    f"{name} = {position:.3f} rad is outside its limits [{lower:.3f}, {upper:.3f}]"
                )
            q[i] = position
        self.get_logger().info(
            f"Goal {dict(zip(self._joints, np.round(q, 3).tolist()))}"
        )
        if not self._collision_free(start, q):
            raise GoalFailed("Goal joint positions are in collision")
        return self._plan_to(start, q)

    def _plan_waypoints(self, goal):
        values = np.asarray(goal.waypoints, dtype=float)
        if values.size == 0 or values.size % 5:
            raise GoalFailed(
                f"Expected n x 5 values (x, y, z, pitch, roll per waypoint), got {values.size}"
            )
        waypoints = values.reshape(-1, 5)
        for w in waypoints:
            w[:3] = self._to_model_frame(w[:3], goal.frame_id)
        self.get_logger().info(
            f"{len(waypoints)} waypoints in {self._robot_model.model_frame} "
            "(x, y, z, pitch, roll):\n" + np.array2string(np.round(waypoints, 4))
        )

        start = self._current_state()
        current = np.asarray(start.get_joint_group_positions(self._group))
        first = self._closest_ik(start, current, *self._split(waypoints[0]))
        if first is None:
            raise GoalFailed("Waypoint 1 is unreachable or in collision")

        path = [first]
        for k in range(1, len(waypoints)):
            path += self._straight_segment(start, path[-1], waypoints[k - 1], waypoints[k], k)

        # The free path's trajectory also carries the planning group that time
        # parameterization needs, which RobotTrajectory() can't set from Python.
        trajectory = self._plan_to(start, first, log=False)
        if np.max(np.abs(first - current)) > 1e-3:
            approach = trajectory.get_robot_trajectory_msg().joint_trajectory
            order = [list(approach.joint_names).index(j) for j in self._joints]
            path = [np.asarray(p.positions)[order] for p in approach.points][:-1] + path
        else:
            path = [current] + path
        self._retime(trajectory, start, path)
        self.get_logger().info(
            f"Planned trajectory: {trajectory.duration:.2f} s through {len(waypoints)} waypoints"
        )
        self._display.publish(
            DisplayTrajectory(
                model_id=self._robot_model.name,
                trajectory=[trajectory.get_robot_trajectory_msg()],
                trajectory_start=robotStateToRobotStateMsg(start),
            )
        )
        return trajectory

    def _straight_segment(self, start, q, a, b, k):
        """Joint positions along the straight segment from waypoint `a` to `b`, `a` excluded."""
        step = self.get_parameter("cartesian_step").value
        max_joint_step = self.get_parameter("max_joint_step").value
        samples = max(
            1,
            math.ceil(np.linalg.norm(b[:3] - a[:3]) / step),
            math.ceil(np.max(np.abs(b[3:] - a[3:])) / ANGLE_STEP),
        )
        segment = []
        for i in range(1, samples + 1):
            t = i / samples
            where = f"Segment {k} -> {k + 1} at {t:.0%}"
            position, pitch, roll = self._split(a + t * (b - a))
            solutions = self._ik.solve(position, pitch, roll)
            if not solutions:
                raise GoalFailed(f"{where}: unreachable")
            previous = q
            q = min(solutions, key=lambda s: np.max(np.abs(s - previous)))
            jump = np.max(np.abs(q - previous))
            if jump > max_joint_step:
                raise GoalFailed(
                    f"{where}: a joint jumps {jump:.2f} rad between samples "
                    "(near a singularity or the edge of the workspace)"
                )
            if not self._collision_free(start, q):
                raise GoalFailed(f"{where}: in collision")
            segment.append(q)
        return segment

    def _retime(self, trajectory, start, path):
        """Replace `trajectory` with a time-parameterized one through the joint positions in `path`."""
        msg = RobotTrajectoryMsg()
        msg.joint_trajectory.joint_names = self._joints
        msg.joint_trajectory.points = [
            JointTrajectoryPoint(positions=q.tolist()) for q in path
        ]
        trajectory.set_robot_trajectory_msg(start, msg)
        # A tight path tolerance keeps TOTG from rounding the straight segments
        if not trajectory.apply_totg_time_parameterization(
            self.get_parameter("max_velocity_scaling_factor").value,
            self.get_parameter("max_acceleration_scaling_factor").value,
            path_tolerance=0.001,
            resample_dt=0.05,
        ):
            raise GoalFailed("Time parameterization failed")

    def _closest_ik(self, start, current, position, pitch, roll):
        """Collision-free IK solution closest to `current`, or None."""
        solutions = sorted(
            self._ik.solve(position, pitch, roll),
            key=lambda q: np.linalg.norm(q - current),
        )
        return next((q for q in solutions if self._collision_free(start, q)), None)

    @staticmethod
    def _split(waypoint):
        return waypoint[:3], waypoint[3], waypoint[4]

    def _plan_to(self, start, q, log=True):
        self._arm.set_goal_state(robot_state=self._state(start, q))
        result = self._arm.plan()
        if not result:
            raise GoalFailed("Planning failed")
        if log:
            self.get_logger().info(
                f"Planned trajectory: {result.trajectory.duration:.2f} s"
            )
        return result.trajectory

    def _current_state(self):
        self._arm.set_start_state_to_current_state()
        return self._arm.get_start_state()

    def _feedback(self, goal_handle, action, state):
        goal_handle.publish_feedback(action.Feedback(state=state))

    def _abort(self, goal_handle, action, message, error=True):
        if error:
            self.get_logger().error(message)
        else:
            self.get_logger().info(message)
        goal_handle.abort()
        return action.Result(success=False, message=message)

    def _canceled(self, goal_handle, action):
        self.get_logger().info("Goal canceled")
        goal_handle.canceled()
        return action.Result(success=False, message="Canceled")

    def _to_model_frame(self, point, frame):
        p = np.asarray(point, dtype=float)
        if not frame or frame == self._robot_model.model_frame:
            return p
        with self._scene_monitor.read_only() as scene:
            if not scene.knows_frame_transform(frame):
                raise GoalFailed(f"Unknown frame '{frame}'")
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
    node = MotionServer(moveit)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        moveit.shutdown()
        rclpy.try_shutdown()
