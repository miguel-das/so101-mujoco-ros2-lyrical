"""Position + approach-direction IK for the 5-DOF SO-101 arm.

After shoulder_pan the SO-101 is a planar 3R chain plus a wrist roll whose
axis lies in that plane. A full 6-D pose is therefore not reachable in
general: at a given TCP position the approach axis must lie (almost) in the
vertical plane through the pan axis and the target. The goal is posed as
TCP position (3) + approach direction (2); the rotation about the approach
axis is left to the solver.

Solved by damped least squares on a finite-difference Jacobian of the
MoveIt RobotState forward kinematics, so all geometry comes from the loaded
robot model.
"""

import numpy as np
from moveit.core.robot_state import RobotState


def radial_approach(position, pan_origin, pitch):
    """Unit approach vector pointing away from the pan axis, `pitch` rad below horizontal.

    pitch = pi/2 approaches straight down, pitch = 0 horizontally outward.
    Assumes the pan axis is vertical in the model frame.
    """
    dx, dy = position[0] - pan_origin[0], position[1] - pan_origin[1]
    if np.hypot(dx, dy) < 1e-6:
        raise ValueError("target lies on the shoulder_pan axis, approach azimuth is undefined")
    azimuth = np.arctan2(dy, dx)
    return np.array(
        [
            np.cos(pitch) * np.cos(azimuth),
            np.cos(pitch) * np.sin(azimuth),
            -np.sin(pitch),
        ]
    )


def _errors(p, u, position, approach):
    angle = np.arccos(np.clip(np.dot(u, approach), -1.0, 1.0))
    return np.linalg.norm(position - p), angle


class ApproachIK:
    def __init__(
        self,
        robot_model,
        group,
        tip_link,
        approach_axis=2,
        orientation_weight=0.1,
        position_tolerance=1e-4,
        angle_tolerance=1e-3,
        max_iterations=200,
        damping=1e-2,
        max_step=0.3,
        fd_step=1e-6,
    ):
        self._group = group
        self._tip_link = tip_link
        self._axis = approach_axis
        self._w = orientation_weight
        self._pos_tol = position_tolerance
        self._ang_tol = angle_tolerance
        self._max_iterations = max_iterations
        self._damping = damping
        self._max_step = max_step
        self._h = fd_step

        self._state = RobotState(robot_model)
        self._state.set_to_default_values()
        bounds = robot_model.get_joint_model_group(group).active_joint_model_bounds
        self.lower = np.array([b[0].min_position for b in bounds])
        self.upper = np.array([b[0].max_position for b in bounds])

    def link_position(self, link, q=None):
        if q is not None:
            self._set(q)
        return self._state.get_global_link_transform(link)[:3, 3].copy()

    def forward(self, q):
        """TCP position and approach axis in the model frame."""
        self._set(q)
        T = self._state.get_global_link_transform(self._tip_link)
        return T[:3, 3].copy(), T[:3, self._axis].copy()

    def errors(self, q, position, approach):
        """Position error (m) and approach-axis angle error (rad)."""
        return _errors(*self.forward(q), position, approach)

    def solve(self, position, approach, seeds, is_valid=None):
        """Return the first converged, bounded, valid solution over `seeds`, or None."""
        position = np.asarray(position, dtype=float)
        approach = np.asarray(approach, dtype=float)
        approach = approach / np.linalg.norm(approach)
        for seed in seeds:
            q = self._descend(np.clip(np.asarray(seed, dtype=float), self.lower, self.upper), position, approach)
            if q is not None and (is_valid is None or is_valid(q)):
                return q
        return None

    def random_seeds(self, count, rng):
        return [rng.uniform(self.lower, self.upper) for _ in range(count)]

    def _set(self, q):
        self._state.set_joint_group_positions(self._group, q)
        self._state.update(True)

    def _task(self, q):
        p, u = self.forward(q)
        return np.concatenate([p, self._w * u])

    def _descend(self, q, position, approach):
        target = np.concatenate([position, self._w * approach])
        n = q.size
        for _ in range(self._max_iterations):
            f = self._task(q)
            pos_err, ang_err = _errors(f[:3], f[3:] / self._w, position, approach)
            if pos_err < self._pos_tol and ang_err < self._ang_tol:
                return q
            J = np.empty((6, n))
            for i in range(n):
                dq = np.zeros(n)
                dq[i] = self._h
                J[:, i] = (self._task(q + dq) - f) / self._h
            e = target - f
            step = J.T @ np.linalg.solve(J @ J.T + self._damping**2 * np.eye(6), e)
            norm = np.linalg.norm(step)
            if norm > self._max_step:
                step *= self._max_step / norm
            q = np.clip(q + step, self.lower, self.upper)
        return None
