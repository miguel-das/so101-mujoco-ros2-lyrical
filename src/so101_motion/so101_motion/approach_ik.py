"""Closed-form position + approach-pitch IK for the 5-DOF SO-101 arm.

The arm is a base yaw joint (shoulder_pan), three parallel pitch joints
(shoulder_lift, elbow_flex, wrist_flex) and a wrist roll whose axis lies in
the pitch plane. A goal is a TCP position, the approach pitch below the
horizontal pointing away from the base (pi/2 = straight down, beyond pi/2 =
tilted back toward the base) and the wrist roll. The approach yaw is not
part of the goal: it follows from the position through the base yaw joint.

With the roll fixed, choosing the approach pitch fixes the TCP offset from the
wrist pitch axis. The base yaw then places the TCP's constant lateral offset,
and the shoulder and elbow form a planar 2-link chain solved with the law of
cosines. This gives up to four solutions (arm reaching forward or back over
itself, elbow up or down).

All geometry is read from the MoveIt robot model at zero joint positions. The
model frame z axis is assumed vertical.
"""

import numpy as np
from moveit.core.robot_state import RobotState

UP = np.array([0.0, 0.0, 1.0])


def _wrap(q):
    return (q + np.pi) % (2 * np.pi) - np.pi


def _rotate(angle, v):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


def _angle(v):
    return np.arctan2(v[1], v[0])


class ApproachIK:
    """Joint order of `group`: base yaw, three pitch joints, wrist roll."""

    def __init__(self, robot_model, group, tip_link, approach_axis=2):
        self._group = group
        self._tip_link = tip_link
        self._axis = approach_axis
        self._state = RobotState(robot_model)
        self._state.set_to_default_values()
        bounds = robot_model.get_joint_model_group(group).active_joint_model_bounds
        self.lower = np.array([b[0].min_position for b in bounds])
        self.upper = np.array([b[0].max_position for b in bounds])

        (yaw_axis, self._origin), *pitch_axes = [self._joint_axis(i) for i in range(4)]
        self._yaw_sign = np.sign(yaw_axis @ UP)
        # Planar coordinates: (radial, up), with pitch rotations counter-clockwise about lateral.
        self._lateral = pitch_axes[0][0]
        self._radial = np.cross(UP, self._lateral)
        self._pitch_signs = np.array([np.sign(axis @ self._lateral) for axis, _ in pitch_axes])
        self._shoulder, elbow, self._wrist = (self._planar(point) for _, point in pitch_axes)
        self._upper_arm = elbow - self._shoulder
        self._forearm = self._wrist - elbow
        self._bend = _angle(self._forearm) - _angle(self._upper_arm)

    def forward(self, q):
        """TCP position and approach axis in the model frame."""
        T = self._tip(q)
        return T[:3, 3].copy(), T[:3, self._axis].copy()

    def pitch(self, q):
        """Approach pitch below the horizontal direction pointing away from the base yaw axis."""
        p, u = self.forward(q)
        outward = np.sign(u[:2] @ (p - self._origin)[:2])
        return np.arctan2(-u @ UP, outward * np.linalg.norm(u[:2]))

    def errors(self, q, position, pitch):
        """Position error (m) and approach pitch error (rad)."""
        p, _ = self.forward(q)
        return np.linalg.norm(position - p), abs(_wrap(self.pitch(q) - pitch))

    def solve(self, position, pitch, roll):
        """All joint solutions within limits reaching `position` with the given approach pitch and roll."""
        tcp, u = self.forward(np.array([0.0, 0.0, 0.0, 0.0, roll]))
        offset = self._planar(tcp) - self._wrist
        lateral = (tcp - self._origin) @ self._lateral
        tilt = _angle([u @ self._radial, u @ UP])

        target = np.asarray(position, dtype=float) - self._origin
        radial2 = target[0] ** 2 + target[1] ** 2 - lateral**2
        if radial2 < 0.0:
            return []
        l1, l2 = np.linalg.norm(self._upper_arm), np.linalg.norm(self._forearm)

        solutions = []
        for side in (1.0, -1.0):
            radial = side * np.sqrt(radial2)
            arm = radial * self._radial + lateral * self._lateral
            yaw = _angle(target) - _angle(arm)
            total_pitch = _angle([side * np.cos(pitch), -np.sin(pitch)]) - tilt
            wrist = np.array([radial, target[2]]) - _rotate(total_pitch, offset)
            reach = wrist - self._shoulder
            c = (reach @ reach - l1**2 - l2**2) / (2.0 * l1 * l2)
            if abs(c) > 1.0 + 1e-5:  # beyond full stretch or fold, with < 1 um slack for URDF rounding
                continue
            bend = np.arccos(np.clip(c, -1.0, 1.0))
            for bend in (bend, -bend):
                elbow = bend - self._bend
                shoulder = _angle(reach) - _angle(self._upper_arm + _rotate(elbow, self._forearm))
                pitches = self._pitch_signs * [shoulder, elbow, total_pitch - shoulder - elbow]
                q = _wrap(np.array([self._yaw_sign * yaw, *pitches, roll]))
                if np.all((q >= self.lower) & (q <= self.upper)):
                    solutions.append(q)
        return solutions

    def _tip(self, q):
        self._state.set_joint_group_positions(self._group, q)
        self._state.update(True)
        return self._state.get_global_link_transform(self._tip_link).copy()

    def _joint_axis(self, i, angle=1.0):
        """Direction and one point of joint i's axis at q = 0, from the motion of the tip."""
        q = np.zeros(self.lower.size)
        T0 = self._tip(q)
        q[i] = angle
        M = self._tip(q) @ np.linalg.inv(T0)
        R = M[:3, :3]
        direction = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
        point = np.linalg.lstsq(np.eye(3) - R, M[:3, 3], rcond=None)[0]
        return direction / np.linalg.norm(direction), point

    def _planar(self, point):
        d = point - self._origin
        return np.array([d @ self._radial, d @ UP])
