#!/usr/bin/env python3
"""Pure closed-loop control math for Go2W adaptive and quasi-static gestures.

This module deliberately has no DDS, terminal-input, or filesystem operations.
The runtime model below was transcribed from the audited Go2W MJCF whose SHA-256
is recorded in ``MODEL_SOURCE_SHA256``.  It is intentionally small enough to be
reviewed and deployed on the Jetson without an external unitree_mujoco checkout.

The torque ratios in this module are provisional application protections.  They
are not actuator certification limits and do not establish physical safety.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Iterable, Optional, Sequence

import numpy as np


MODEL_SOURCE_SHA256 = (
    "c8feaef4afdf360335727c80a826d1611950c562a3daaa5b5bfcf8b57f6859a6"
)
MODEL_MASS_KG = 19.126408
GRAVITY_M_S2 = 9.81
BODY_WEIGHT_N = MODEL_MASS_KG * GRAVITY_M_S2

LEG_NAMES = ("FR", "FL", "RR", "RL")
JOINT_NAMES = tuple(
    "{}_{}".format(leg, joint)
    for leg in LEG_NAMES
    for joint in ("hip", "thigh", "calf")
)

# Command-versus-measurement envelopes retain margin for KD and gravity load.
TRACKING_ENVELOPE_RAD = np.asarray([0.18, 0.14, 0.25] * 4, dtype=float)
TORQUE_LIMIT_NM = np.asarray([23.7, 23.7, 45.43] * 4, dtype=float)

JOINT_LOWER_RAD = np.asarray(
    [
        -1.0472,
        -1.5708,
        -2.7227,
        -1.0472,
        -1.5708,
        -2.7227,
        -1.0472,
        -0.5236,
        -2.7227,
        -1.0472,
        -0.5236,
        -2.7227,
    ],
    dtype=float,
)
JOINT_UPPER_RAD = np.asarray(
    [
        1.0472,
        3.4907,
        -0.83776,
        1.0472,
        3.4907,
        -0.83776,
        1.0472,
        4.5379,
        -0.83776,
        1.0472,
        4.5379,
        -0.83776,
    ],
    dtype=float,
)

HIP_ORIGINS_M = np.asarray(
    [
        [0.1934, -0.0465, 0.0],
        [0.1934, 0.0465, 0.0],
        [-0.1934, -0.0465, 0.0],
        [-0.1934, 0.0465, 0.0],
    ],
    dtype=float,
)
SIDE_SIGNS = np.asarray([-1.0, 1.0, -1.0, 1.0], dtype=float)
HIP_LATERAL_OFFSET_M = 0.0955
THIGH_LENGTH_M = 0.213
CALF_TO_WHEEL_M = 0.2264

BASE_MASS_KG = 6.921
BASE_COM_M = np.asarray([0.021112, 0.0, -0.005366], dtype=float)
HIP_MASS_KG = 0.678
THIGH_MASS_KG = 1.152
CALF_MASS_KG = 0.241352
WHEEL_MASS_KG = 0.98

HEIGHT_LOW_REL_M = -0.093178
HEIGHT_HIGH_REL_M = 0.076281
ROLL_RIGHT_REL_RAD = -0.395469
ROLL_LEFT_REL_RAD = 0.395469

FAST_TRANSITION_TIMEOUT_S = 8.0
STARTUP_TIMEOUT_S = 12.0
PRONE_RETURN_TIMEOUT_S = 15.0
HOLD_CONVERGENCE_TIMEOUT_S = 5.0
CONVERGENCE_DWELL_S = 0.30
MAX_CONVERGED_DQ_RAD_S = 0.20
# One milliradian prevents quantization and floating-point boundary chatter at
# exactly 50% without changing the command safety envelope or speed schedule.
CONVERGENCE_NUMERIC_MARGIN_RAD = 0.001

TORQUE_WARN_RATIO = 0.60
TORQUE_STOP_RATIO = 0.75
TORQUE_RETURN_RATIO = 0.85
TORQUE_ERROR_RATIO = 1.00
PERSISTENCE_S = 0.10

WBC_PERIOD_S = 0.010
WBC_MAX_DQ_RAD_S = 1.0
WBC_MAX_DDQ_RAD_S2 = 4.0
WBC_MAX_SOLVE_S = 0.010
# The initial acceleration-limited QP can need roughly 300 ADMM iterations to
# meet the strict residual bounds from a cold start.  The fixed ceiling remains
# comfortably inside the separately enforced 10 ms wall-time limit.
WBC_MAX_ITER = 1000
WBC_EPS_ABS = 1.0e-5
WBC_EPS_REL = 1.0e-5
# Acceptance is checked independently of OSQP's scaled stopping test.  The
# 5e-4 fixed bound rejects materially inaccurate solutions while avoiding a
# false trip at the 1e-4 floating-point boundary seen after warm starts.
WBC_MAX_PRIMAL_RESIDUAL = 5.0e-4
WBC_MAX_DUAL_RESIDUAL = 5.0e-4


def _vector(values: Sequence[float], length: int, name: str) -> np.ndarray:
    result = np.asarray(values, dtype=float).reshape(-1)
    if result.size != length:
        raise ValueError("{} must contain {} values".format(name, length))
    if not np.all(np.isfinite(result)):
        raise ValueError("{} contains a non-finite value".format(name))
    return result


def smoothstep(alpha: float) -> float:
    alpha = min(1.0, max(0.0, float(alpha)))
    return alpha * alpha * (3.0 - 2.0 * alpha)


def smooth_path(
    source: Sequence[float], target: Sequence[float], alpha: float
) -> np.ndarray:
    source_array = np.asarray(source, dtype=float)
    target_array = np.asarray(target, dtype=float)
    if source_array.shape != target_array.shape:
        raise ValueError("source and target shapes differ")
    return source_array + (target_array - source_array) * smoothstep(alpha)


@dataclass(frozen=True)
class ProgressDecision:
    q_ref: np.ndarray
    progress: float
    speed_scale: float
    tracking_ratio: float
    torque_ratio: float
    completed: bool
    request_return: bool
    emergency: bool
    timed_out: bool
    warning: Optional[str]
    reason: Optional[str]


class ReferenceGovernor:
    """Tracking-coupled smooth path with convergence and fail-closed gates."""

    def __init__(
        self,
        source: Sequence[float],
        target: Sequence[float],
        duration_s: float,
        timeout_s: float,
        envelopes: Sequence[float] = TRACKING_ENVELOPE_RAD,
        torque_limits: Sequence[float] = TORQUE_LIMIT_NM,
    ) -> None:
        self.source = _vector(source, 12, "source")
        self.target = _vector(target, 12, "target")
        self.envelopes = _vector(envelopes, 12, "envelopes")
        self.torque_limits = _vector(torque_limits, 12, "torque_limits")
        if duration_s <= 0.0 or not math.isfinite(duration_s):
            raise ValueError("duration_s must be positive and finite")
        if timeout_s < duration_s or not math.isfinite(timeout_s):
            raise ValueError("timeout_s must be finite and at least duration_s")
        if np.any(self.envelopes <= 0.0) or np.any(self.torque_limits <= 0.0):
            raise ValueError("envelopes and torque limits must be positive")
        self.duration_s = float(duration_s)
        self.timeout_s = float(timeout_s)
        self.progress = 0.0
        self.current_ref = self.source.copy()
        self.last_converged_ref = self.source.copy()
        self._tracking_over_s = 0.0
        self._torque_over_s = 0.0
        self._settled_s = 0.0

    @staticmethod
    def _tracking_speed_scale(ratio: float) -> float:
        if ratio <= 0.50:
            return 1.0
        if ratio < 0.90:
            return max(0.0, (0.90 - ratio) / 0.40)
        return 0.0

    def step(
        self,
        measured_q: Sequence[float],
        measured_dq: Sequence[float],
        tau_est: Sequence[float],
        dt_s: float,
        wall_elapsed_s: float,
        *,
        return_mode: bool = False,
    ) -> ProgressDecision:
        measured = _vector(measured_q, 12, "measured_q")
        velocity = _vector(measured_dq, 12, "measured_dq")
        torque = _vector(tau_est, 12, "tau_est")
        if dt_s < 0.0 or not math.isfinite(dt_s):
            raise ValueError("dt_s must be finite and nonnegative")
        if wall_elapsed_s < 0.0 or not math.isfinite(wall_elapsed_s):
            raise ValueError("wall_elapsed_s must be finite and nonnegative")

        tracking_ratio = float(
            np.max(np.abs(self.current_ref - measured) / self.envelopes)
        )
        torque_ratio = float(np.max(np.abs(torque) / self.torque_limits))
        timed_out = wall_elapsed_s > self.timeout_s
        emergency = torque_ratio >= TORQUE_ERROR_RATIO

        self._tracking_over_s = (
            self._tracking_over_s + dt_s if tracking_ratio >= 1.0 else 0.0
        )
        self._torque_over_s = (
            self._torque_over_s + dt_s
            if torque_ratio >= TORQUE_RETURN_RATIO
            else 0.0
        )
        tracking_return = self._tracking_over_s >= PERSISTENCE_S
        # Once a controlled return has begun, 75--100% torque slows the
        # retreat instead of recursively requesting the same return.  Reaching
        # 100% remains an immediate error in both directions.
        torque_return = (
            self._torque_over_s >= PERSISTENCE_S and not return_mode
        )
        request_return = timed_out or tracking_return or torque_return

        reason = None
        if emergency:
            reason = "tau_est reached 100% of the model torque range"
        elif timed_out:
            reason = "phase wall timeout exceeded {:.3f} s".format(self.timeout_s)
        elif tracking_return:
            reason = (
                "tracking envelope exceeded continuously for {:.3f} s"
            ).format(PERSISTENCE_S)
        elif torque_return:
            reason = (
                "tau_est exceeded 85% of the model range continuously for {:.3f} s"
            ).format(PERSISTENCE_S)

        warning = None
        if torque_ratio >= TORQUE_WARN_RATIO:
            warning = (
                "provisional tau_est protection active at {:.1%} of model range"
            ).format(torque_ratio)

        tracking_scale = self._tracking_speed_scale(tracking_ratio)
        if torque_ratio >= TORQUE_STOP_RATIO:
            torque_scale = 0.25 if return_mode and not emergency else 0.0
        else:
            torque_scale = 1.0
        speed_scale = min(tracking_scale, torque_scale)

        if request_return:
            self.current_ref = self.last_converged_ref.copy()
        elif not emergency:
            self.progress = min(
                1.0,
                self.progress + dt_s * speed_scale / self.duration_s,
            )
            self.current_ref = smooth_path(self.source, self.target, self.progress)

        within_convergence = bool(
            np.all(
                np.abs(self.current_ref - measured)
                <= 0.50 * self.envelopes + CONVERGENCE_NUMERIC_MARGIN_RAD
            )
        ) and float(np.max(np.abs(velocity))) <= MAX_CONVERGED_DQ_RAD_S
        self._settled_s = self._settled_s + dt_s if within_convergence else 0.0
        if self._settled_s >= CONVERGENCE_DWELL_S:
            self.last_converged_ref = self.current_ref.copy()
        completed = self.progress >= 1.0 and self._settled_s >= CONVERGENCE_DWELL_S

        return ProgressDecision(
            q_ref=self.current_ref.copy(),
            progress=self.progress,
            speed_scale=speed_scale,
            tracking_ratio=tracking_ratio,
            torque_ratio=torque_ratio,
            completed=completed,
            request_return=request_return,
            emergency=emergency,
            timed_out=timed_out,
            warning=warning,
            reason=reason,
        )


class ConvergenceGate:
    def __init__(
        self,
        dwell_s: float = CONVERGENCE_DWELL_S,
        envelopes: Sequence[float] = TRACKING_ENVELOPE_RAD,
    ) -> None:
        self.dwell_s = float(dwell_s)
        self.envelopes = _vector(envelopes, 12, "envelopes")
        self.accumulated_s = 0.0

    def update(
        self,
        target_q: Sequence[float],
        measured_q: Sequence[float],
        measured_dq: Sequence[float],
        dt_s: float,
    ) -> bool:
        target = _vector(target_q, 12, "target_q")
        measured = _vector(measured_q, 12, "measured_q")
        velocity = _vector(measured_dq, 12, "measured_dq")
        tracking_ratio = float(np.max(np.abs(target - measured) / self.envelopes))
        converged = (
            bool(
                np.all(
                    np.abs(target - measured)
                    <= 0.50 * self.envelopes + CONVERGENCE_NUMERIC_MARGIN_RAD
                )
            )
            and float(np.max(np.abs(velocity))) <= MAX_CONVERGED_DQ_RAD_S
        )
        self.accumulated_s = self.accumulated_s + dt_s if converged else 0.0
        return self.accumulated_s >= self.dwell_s


@dataclass(frozen=True)
class TaskProgressDecision:
    progress: float
    speed_scale: float
    tracking_ratio: float
    torque_ratio: float
    completed: bool
    request_return: bool
    emergency: bool
    timed_out: bool
    warning: Optional[str]
    reason: Optional[str]


class TaskProgressGovernor:
    """Reference-governor policy for task-space paths generated by WBC."""

    def __init__(
        self,
        duration_s: float,
        timeout_s: float,
        envelopes: Sequence[float] = TRACKING_ENVELOPE_RAD,
        torque_limits: Sequence[float] = TORQUE_LIMIT_NM,
    ) -> None:
        if duration_s <= 0.0 or not math.isfinite(duration_s):
            raise ValueError("duration_s must be positive and finite")
        if timeout_s < duration_s or not math.isfinite(timeout_s):
            raise ValueError("timeout_s must be finite and at least duration_s")
        self.duration_s = float(duration_s)
        self.timeout_s = float(timeout_s)
        self.envelopes = _vector(envelopes, 12, "envelopes")
        self.torque_limits = _vector(torque_limits, 12, "torque_limits")
        self.progress = 0.0
        self._tracking_over_s = 0.0
        self._torque_over_s = 0.0
        self._settled_s = 0.0

    def step(
        self,
        command_q: Sequence[float],
        measured_q: Sequence[float],
        measured_dq: Sequence[float],
        tau_est: Sequence[float],
        dt_s: float,
        wall_elapsed_s: float,
        *,
        task_within_tolerance: bool,
    ) -> TaskProgressDecision:
        command = _vector(command_q, 12, "command_q")
        measured = _vector(measured_q, 12, "measured_q")
        velocity = _vector(measured_dq, 12, "measured_dq")
        torque = _vector(tau_est, 12, "tau_est")
        if dt_s < 0.0 or not math.isfinite(dt_s):
            raise ValueError("dt_s must be finite and nonnegative")
        tracking_ratio = float(np.max(np.abs(command - measured) / self.envelopes))
        torque_ratio = float(np.max(np.abs(torque) / self.torque_limits))
        timed_out = wall_elapsed_s > self.timeout_s
        emergency = torque_ratio >= TORQUE_ERROR_RATIO
        self._tracking_over_s = (
            self._tracking_over_s + dt_s if tracking_ratio >= 1.0 else 0.0
        )
        self._torque_over_s = (
            self._torque_over_s + dt_s
            if torque_ratio >= TORQUE_RETURN_RATIO
            else 0.0
        )
        tracking_return = self._tracking_over_s >= PERSISTENCE_S
        torque_return = self._torque_over_s >= PERSISTENCE_S
        request_return = timed_out or tracking_return or torque_return

        reason = None
        if emergency:
            reason = "tau_est reached 100% of the model torque range"
        elif timed_out:
            reason = "task wall timeout exceeded {:.3f} s".format(self.timeout_s)
        elif tracking_return:
            reason = "WBC q_ref envelope exceeded continuously for 0.10 s"
        elif torque_return:
            reason = "tau_est exceeded 85% continuously for 0.10 s"
        warning = None
        if torque_ratio >= TORQUE_WARN_RATIO:
            warning = (
                "provisional tau_est protection active at {:.1%} of model range"
            ).format(torque_ratio)

        tracking_scale = ReferenceGovernor._tracking_speed_scale(tracking_ratio)
        torque_scale = 0.0 if torque_ratio >= TORQUE_STOP_RATIO else 1.0
        speed_scale = min(tracking_scale, torque_scale)
        if not request_return and not emergency:
            self.progress = min(
                1.0,
                self.progress + dt_s * speed_scale / self.duration_s,
            )

        settled = (
            self.progress >= 1.0
            and bool(task_within_tolerance)
            and float(np.max(np.abs(velocity))) <= MAX_CONVERGED_DQ_RAD_S
            and bool(
                np.all(
                    np.abs(command - measured)
                    <= 0.50 * self.envelopes + CONVERGENCE_NUMERIC_MARGIN_RAD
                )
            )
        )
        self._settled_s = self._settled_s + dt_s if settled else 0.0
        completed = self._settled_s >= CONVERGENCE_DWELL_S
        return TaskProgressDecision(
            progress=self.progress,
            speed_scale=speed_scale,
            tracking_ratio=tracking_ratio,
            torque_ratio=torque_ratio,
            completed=completed,
            request_return=request_return,
            emergency=emergency,
            timed_out=timed_out,
            warning=warning,
            reason=reason,
        )


class ContactValidityGate:
    def __init__(self, required_s: float = 0.5) -> None:
        if required_s <= 0.0 or not math.isfinite(required_s):
            raise ValueError("required_s must be positive and finite")
        self.required_s = float(required_s)
        self.valid_s = 0.0

    def update(self, valid: bool, dt_s: float) -> bool:
        if dt_s < 0.0 or not math.isfinite(dt_s):
            raise ValueError("dt_s must be finite and nonnegative")
        self.valid_s = self.valid_s + dt_s if valid else 0.0
        return self.valid_s >= self.required_s


def rotation_x(angle: float) -> np.ndarray:
    c = math.cos(float(angle))
    s = math.sin(float(angle))
    return np.asarray([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rotation_y(angle: float) -> np.ndarray:
    c = math.cos(float(angle))
    s = math.sin(float(angle))
    return np.asarray([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rotation_z(angle: float) -> np.ndarray:
    c = math.cos(float(angle))
    s = math.sin(float(angle))
    return np.asarray([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rpy_rotation(rpy: Sequence[float]) -> np.ndarray:
    values = _vector(rpy, 3, "rpy")
    return rotation_z(values[2]) @ rotation_y(values[1]) @ rotation_x(values[0])


@dataclass(frozen=True)
class LegKinematics:
    wheel_position: np.ndarray
    jacobian: np.ndarray
    joint_origins: np.ndarray
    joint_axes: np.ndarray


def leg_kinematics(q_leg: Sequence[float], leg_index: int) -> LegKinematics:
    if not 0 <= int(leg_index) < 4:
        raise ValueError("leg_index must be in [0, 3]")
    q = _vector(q_leg, 3, "q_leg")
    hip_origin = HIP_ORIGINS_M[int(leg_index)]
    side = SIDE_SIGNS[int(leg_index)]
    rx = rotation_x(q[0])
    r_thigh = rx @ rotation_y(q[1])
    r_calf = rx @ rotation_y(q[1] + q[2])

    thigh_origin = hip_origin + rx @ np.asarray(
        [0.0, side * HIP_LATERAL_OFFSET_M, 0.0]
    )
    calf_origin = thigh_origin + r_thigh @ np.asarray(
        [0.0, 0.0, -THIGH_LENGTH_M]
    )
    wheel_position = calf_origin + r_calf @ np.asarray(
        [0.0, 0.0, -CALF_TO_WHEEL_M]
    )

    axis_hip = np.asarray([1.0, 0.0, 0.0])
    axis_pitch = rx @ np.asarray([0.0, 1.0, 0.0])
    origins = np.vstack([hip_origin, thigh_origin, calf_origin])
    axes = np.vstack([axis_hip, axis_pitch, axis_pitch])
    jacobian = np.column_stack(
        [
            np.cross(axes[index], wheel_position - origins[index])
            for index in range(3)
        ]
    )
    return LegKinematics(
        wheel_position=wheel_position,
        jacobian=jacobian,
        joint_origins=origins,
        joint_axes=axes,
    )


def wheel_positions(q: Sequence[float]) -> np.ndarray:
    q_array = _vector(q, 12, "q")
    return np.vstack(
        [leg_kinematics(q_array[3 * leg : 3 * leg + 3], leg).wheel_position for leg in range(4)]
    )


def leg_jacobians(q: Sequence[float]) -> np.ndarray:
    q_array = _vector(q, 12, "q")
    return np.stack(
        [leg_kinematics(q_array[3 * leg : 3 * leg + 3], leg).jacobian for leg in range(4)]
    )


def _leg_com_positions(q_leg: np.ndarray, leg_index: int) -> tuple[np.ndarray, ...]:
    kinematics = leg_kinematics(q_leg, leg_index)
    q0, q1, q2 = q_leg
    side = SIDE_SIGNS[leg_index]
    front = leg_index in (0, 1)
    rx = rotation_x(q0)
    r_thigh = rx @ rotation_y(q1)
    r_calf = rx @ rotation_y(q1 + q2)
    hip_origin, thigh_origin, calf_origin = kinematics.joint_origins

    hip_com_local = np.asarray(
        [-0.0054 if front else 0.0054, side * 0.00194, -0.000105]
    )
    thigh_com_local = np.asarray([-0.00374, -side * 0.0223, -0.0327])
    calf_com_local = np.asarray([0.00629595, -side * 0.000622121, -0.141417])
    wheel_com_local = np.asarray([0.0, side * 0.04, 0.0])

    return (
        hip_origin + rx @ hip_com_local,
        thigh_origin + r_thigh @ thigh_com_local,
        calf_origin + r_calf @ calf_com_local,
        kinematics.wheel_position + r_calf @ wheel_com_local,
    )


def gravity_torques(
    q: Sequence[float], body_rpy: Sequence[float] = (0.0, 0.0, 0.0)
) -> np.ndarray:
    """Return the external generalized gravity torque on the 12 leg joints.

    This is the torque applied *to the robot* by gravity, rather than the
    opposite-sign motor compensation torque.  Keeping that distinction
    explicit matters when ``tau_est`` follows the Unitree/MuJoCo actuator
    convention.
    """

    q_array = _vector(q, 12, "q")
    rotation = rpy_rotation(body_rpy)
    result = np.zeros(12, dtype=float)
    masses = (HIP_MASS_KG, THIGH_MASS_KG, CALF_MASS_KG, WHEEL_MASS_KG)
    # Link geometry is expressed in the body frame.  Rotate world gravity into
    # that frame so roll/pitch gestures do not look like contact inconsistency.
    force_direction = rotation.T @ np.asarray([0.0, 0.0, -GRAVITY_M_S2])
    for leg in range(4):
        q_leg = q_array[3 * leg : 3 * leg + 3]
        kinematics = leg_kinematics(q_leg, leg)
        coms = _leg_com_positions(q_leg, leg)
        for joint in range(3):
            torque = 0.0
            for link in range(joint, 4):
                moment = np.cross(
                    coms[link] - kinematics.joint_origins[joint],
                    masses[link] * force_direction,
                )
                torque += float(np.dot(kinematics.joint_axes[joint], moment))
            result[3 * leg + joint] = torque
    return result


def whole_body_com(q: Sequence[float]) -> np.ndarray:
    """Return the audited model's whole-body COM in the base frame."""

    q_array = _vector(q, 12, "q")
    weighted_position = BASE_MASS_KG * BASE_COM_M
    for leg in range(4):
        q_leg = q_array[3 * leg : 3 * leg + 3]
        coms = _leg_com_positions(q_leg, leg)
        for mass, position in zip(
            (HIP_MASS_KG, THIGH_MASS_KG, CALF_MASS_KG, WHEEL_MASS_KG),
            coms,
        ):
            weighted_position = weighted_position + mass * position
    return weighted_position / MODEL_MASS_KG


def skew(vector: Sequence[float]) -> np.ndarray:
    x, y, z = _vector(vector, 3, "vector")
    return np.asarray([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def _load_qp_dependencies():
    try:
        import osqp  # type: ignore
        from scipy import sparse  # type: ignore
    except ImportError as error:
        raise RuntimeError(
            "OSQP runtime is unavailable; install pinned scipy==1.13.1 and "
            "osqp==1.1.3"
        ) from error
    return osqp, sparse


@dataclass(frozen=True)
class ContactEstimate:
    forces: np.ndarray
    gravity_torque: np.ndarray
    torque_contact: np.ndarray
    status: str
    iterations: int
    solve_time_s: float
    max_jacobian_condition: float
    torque_residual_ratio: float
    force_balance_residual_n: float
    moment_balance_residual_nm: float
    balance_residual_ratio: float
    total_vertical_load_n: float
    minimum_normal_load_n: float
    valid: bool
    reason: Optional[str]


def estimate_contact_forces(
    q: Sequence[float],
    tau_est: Sequence[float],
    body_rpy: Sequence[float] = (0.0, 0.0, 0.0),
    *,
    body_weight_n: float = BODY_WEIGHT_N,
    max_jacobian_condition: float = 200.0,
) -> ContactEstimate:
    """Estimate ground-on-wheel forces with a nonnegative-normal OSQP problem.

    Unitree's simulator bridge populates ``tau_est`` from MuJoCo's
    ``jointactuatorfrc`` sensor, so it has the actuator-effort sign.  In a
    quasi-static configuration the generalized joint equilibrium is

    ``tau_est + tau_gravity_external + J.T @ force_ground_on_wheel = 0``.

    Consequently the contact torque fitted below is
    ``-(tau_est + tau_gravity_external)``.  This is equivalent to subtracting
    the motor gravity-compensation and then converting motor load to the
    ground-on-wheel convention.  A synthetic sign convention must not be used
    here: doing so produces negative normal loads for the audited MJCF.
    """

    q_array = _vector(q, 12, "q")
    tau_array = _vector(tau_est, 12, "tau_est")
    rpy_array = _vector(body_rpy, 3, "body_rpy")
    if body_weight_n <= 0.0 or not math.isfinite(body_weight_n):
        raise ValueError("body_weight_n must be positive and finite")
    osqp, sparse = _load_qp_dependencies()

    jacobians = leg_jacobians(q_array)
    rotation = rpy_rotation(rpy_array)
    positions = (rotation @ wheel_positions(q_array).T).T
    conditions = np.asarray([np.linalg.cond(jacobian) for jacobian in jacobians])
    max_condition = float(np.max(conditions))
    gravity = gravity_torques(q_array, rpy_array)
    tau_contact = -(tau_array + gravity)

    torque_map = np.zeros((12, 12), dtype=float)
    for leg in range(4):
        torque_map[3 * leg : 3 * leg + 3, 3 * leg : 3 * leg + 3] = (
            jacobians[leg].T @ rotation.T
        )

    equilibrium = np.zeros((6, 12), dtype=float)
    for leg in range(4):
        equilibrium[0:3, 3 * leg : 3 * leg + 3] = np.eye(3)
        equilibrium[3:6, 3 * leg : 3 * leg + 3] = skew(positions[leg])
    upward_force = np.asarray([0.0, 0.0, body_weight_n])
    com_world = rotation @ whole_body_com(q_array)
    # Contact forces balance both gravity force and its moment about the base
    # origin.  A zero desired moment is only correct when the COM is exactly on
    # that origin's vertical line, which is not true during a roll gesture.
    desired_wrench = np.concatenate(
        [upward_force, np.cross(com_world, upward_force)]
    )

    def invalid_contact(status: str, reason: str, solve_time_s: float = 0.0):
        return ContactEstimate(
            forces=np.full((4, 3), np.nan),
            gravity_torque=gravity,
            torque_contact=tau_contact,
            status=status,
            iterations=0,
            solve_time_s=solve_time_s,
            max_jacobian_condition=max_condition,
            torque_residual_ratio=float("inf"),
            force_balance_residual_n=float("inf"),
            moment_balance_residual_nm=float("inf"),
            balance_residual_ratio=float("inf"),
            total_vertical_load_n=float("nan"),
            minimum_normal_load_n=float("nan"),
            valid=False,
            reason=reason,
        )

    equilibrium_weight = 0.02
    regularization = 1.0e-6
    hessian = (
        torque_map.T @ torque_map
        + equilibrium_weight * equilibrium.T @ equilibrium
        + regularization * np.eye(12)
    )
    gradient = -(
        torque_map.T @ tau_contact
        + equilibrium_weight * equilibrium.T @ desired_wrench
    )
    lower = np.asarray([-body_weight_n, -body_weight_n, 0.0] * 4)
    upper = np.asarray([body_weight_n, body_weight_n, body_weight_n] * 4)

    solver = osqp.OSQP()
    setup_kwargs = dict(
        P=sparse.csc_matrix(2.0 * hessian),
        q=2.0 * gradient,
        A=sparse.eye(12, format="csc"),
        l=lower,
        u=upper,
        verbose=False,
        max_iter=WBC_MAX_ITER,
        eps_abs=WBC_EPS_ABS,
        eps_rel=WBC_EPS_REL,
        polishing=False,
        warm_starting=True,
    )
    try:
        try:
            solver.setup(**setup_kwargs)
        except TypeError:
            setup_kwargs.pop("warm_starting", None)
            setup_kwargs["warm_start"] = True
            setup_kwargs.pop("polishing", None)
            setup_kwargs["polish"] = False
            solver.setup(**setup_kwargs)
    except Exception as error:
        return invalid_contact(
            "setup-exception",
            "contact-force QP setup raised {}: {}".format(
                type(error).__name__, error
            ),
        )

    started = time.perf_counter()
    try:
        result = solver.solve()
    except Exception as error:
        elapsed = time.perf_counter() - started
        return invalid_contact(
            "solve-exception",
            "contact-force QP solve raised {}: {}".format(
                type(error).__name__, error
            ),
            elapsed,
        )
    elapsed = time.perf_counter() - started
    info = result.info
    status = str(getattr(info, "status", "unknown")).lower()
    iterations = int(getattr(info, "iter", 0))
    solve_time = float(getattr(info, "solve_time", elapsed) or elapsed)
    solution = result.x

    if solution is None or "solved" not in status:
        forces = np.full((4, 3), np.nan)
        return ContactEstimate(
            forces=forces,
            gravity_torque=gravity,
            torque_contact=tau_contact,
            status=status,
            iterations=iterations,
            solve_time_s=solve_time,
            max_jacobian_condition=max_condition,
            torque_residual_ratio=float("inf"),
            force_balance_residual_n=float("inf"),
            moment_balance_residual_nm=float("inf"),
            balance_residual_ratio=float("inf"),
            total_vertical_load_n=float("nan"),
            minimum_normal_load_n=float("nan"),
            valid=False,
            reason="contact-force QP was not solved",
        )

    forces = np.asarray(solution, dtype=float).reshape(4, 3)
    finite = bool(np.all(np.isfinite(forces)))
    torque_residual = torque_map @ forces.reshape(-1) - tau_contact
    torque_residual_ratio = float(
        np.linalg.norm(torque_residual) / max(np.linalg.norm(tau_contact), 1.0)
    )
    wrench = equilibrium @ forces.reshape(-1)
    force_residual = float(np.linalg.norm(wrench[:3] - desired_wrench[:3]))
    moment_residual = float(
        np.linalg.norm(wrench[3:] - desired_wrench[3:])
    )
    moment_scale = body_weight_n * 0.25
    balance_ratio = max(force_residual / body_weight_n, moment_residual / moment_scale)
    total_vertical = float(np.sum(forces[:, 2]))
    minimum_normal = float(np.min(forces[:, 2]))

    reason = None
    if not finite:
        reason = "contact-force QP returned a non-finite solution"
    elif max_condition > max_jacobian_condition:
        reason = "leg Jacobian condition number exceeds the configured limit"
    elif torque_residual_ratio > 0.25:
        reason = "contact torque residual exceeds 25%"
    elif balance_ratio > 0.15:
        reason = "force or moment balance residual exceeds 15%"
    elif not 0.50 * body_weight_n <= total_vertical <= 1.50 * body_weight_n:
        reason = "total vertical load is inconsistent with model weight"
    elif minimum_normal < 0.01 * body_weight_n:
        reason = "at least one wheel lacks a positive normal load"

    return ContactEstimate(
        forces=forces,
        gravity_torque=gravity,
        torque_contact=tau_contact,
        status=status,
        iterations=iterations,
        solve_time_s=solve_time,
        max_jacobian_condition=max_condition,
        torque_residual_ratio=torque_residual_ratio,
        force_balance_residual_n=force_residual,
        moment_balance_residual_nm=moment_residual,
        balance_residual_ratio=balance_ratio,
        total_vertical_load_n=total_vertical,
        minimum_normal_load_n=minimum_normal,
        valid=reason is None,
        reason=reason,
    )


@dataclass(frozen=True)
class TaskSpaceEstimate:
    relative_height_m: float
    raw_height_m: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float


def estimate_task_space(
    q: Sequence[float],
    rpy: Sequence[float],
    contact_forces: Sequence[Sequence[float]],
    *,
    baseline_height_m: float,
) -> TaskSpaceEstimate:
    q_array = _vector(q, 12, "q")
    rpy_array = _vector(rpy, 3, "rpy")
    forces = np.asarray(contact_forces, dtype=float)
    if forces.shape != (4, 3) or not np.all(np.isfinite(forces)):
        raise ValueError("contact_forces must be a finite 4x3 array")
    if not math.isfinite(baseline_height_m):
        raise ValueError("baseline_height_m must be finite")
    positions = wheel_positions(q_array)
    rotation = rpy_rotation(rpy_array)
    individual_heights = np.asarray(
        [-float((rotation @ positions[leg])[2]) for leg in range(4)]
    )
    weights = np.maximum(forces[:, 2], 0.0)
    if float(np.sum(weights)) <= 1.0e-9:
        raise ValueError("contact forces have no positive normal load")
    raw_height = float(np.average(individual_heights, weights=weights))
    return TaskSpaceEstimate(
        relative_height_m=raw_height - baseline_height_m,
        raw_height_m=raw_height,
        roll_rad=float(rpy_array[0]),
        pitch_rad=float(rpy_array[1]),
        yaw_rad=float(rpy_array[2]),
    )


def contact_velocity_matrix(q: Sequence[float]) -> np.ndarray:
    """Map [base linear, base angular, 12 leg dq] to four wheel velocities."""

    q_array = _vector(q, 12, "q")
    positions = wheel_positions(q_array)
    jacobians = leg_jacobians(q_array)
    matrix = np.zeros((12, 18), dtype=float)
    for leg in range(4):
        rows = slice(3 * leg, 3 * leg + 3)
        matrix[rows, 0:3] = np.eye(3)
        matrix[rows, 3:6] = -skew(positions[leg])
        matrix[rows, 6 + 3 * leg : 6 + 3 * leg + 3] = jacobians[leg]
    return matrix


@dataclass(frozen=True)
class WBCTarget:
    relative_height_m: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float


@dataclass(frozen=True)
class WBCSolveResult:
    q_ref: np.ndarray
    generalized_velocity: np.ndarray
    status: str
    iterations: int
    solve_time_s: float
    primal_residual: float
    dual_residual: float
    contact_velocity_residual_m_s: float
    valid: bool
    reason: Optional[str]


class KinematicWBC:
    """100 Hz constrained kinematic QP; output is a position-PD reference."""

    def __init__(self, dt_s: float = WBC_PERIOD_S) -> None:
        if dt_s <= 0.0 or not math.isfinite(dt_s):
            raise ValueError("dt_s must be positive and finite")
        self.dt_s = float(dt_s)
        self.previous_dq = np.zeros(12, dtype=float)
        self.previous_solution: Optional[np.ndarray] = None

    def solve(
        self,
        measured_q: Sequence[float],
        current_q_ref: Sequence[float],
        posture_target_q: Sequence[float],
        task_estimate: TaskSpaceEstimate,
        task_target: WBCTarget,
        body_angular_velocity: Sequence[float] = (0.0, 0.0, 0.0),
    ) -> WBCSolveResult:
        q = _vector(measured_q, 12, "measured_q")
        q_ref = _vector(current_q_ref, 12, "current_q_ref")
        posture = _vector(posture_target_q, 12, "posture_target_q")
        body_gyro = _vector(
            body_angular_velocity, 3, "body_angular_velocity"
        )

        def invalid_result(status: str, reason: str, elapsed_s: float = 0.0):
            return WBCSolveResult(
                q_ref=q_ref.copy(),
                generalized_velocity=np.zeros(18),
                status=status,
                iterations=0,
                solve_time_s=elapsed_s,
                primal_residual=float("inf"),
                dual_residual=float("inf"),
                contact_velocity_residual_m_s=float("inf"),
                valid=False,
                reason=reason,
            )
        # Account for matrix assembly and OSQP setup as part of the 100 Hz
        # budget, rather than reporting only the final ADMM call.
        started = time.perf_counter()
        osqp, sparse = _load_qp_dependencies()

        desired_velocity = np.zeros(18, dtype=float)
        desired_velocity[0] = 0.0
        desired_velocity[1] = 0.0
        desired_velocity[2] = float(
            np.clip(
                3.0
                * (task_target.relative_height_m - task_estimate.relative_height_m),
                -0.25,
                0.25,
            )
        )
        desired_velocity[3] = float(
            np.clip(
                4.0 * (task_target.roll_rad - task_estimate.roll_rad)
                - 1.5 * body_gyro[0],
                -0.8,
                0.8,
            )
        )
        desired_velocity[4] = float(
            np.clip(
                4.0 * (task_target.pitch_rad - task_estimate.pitch_rad)
                - 1.5 * body_gyro[1],
                -0.8,
                0.8,
            )
        )
        desired_velocity[5] = float(
            np.clip(
                2.0 * (task_target.yaw_rad - task_estimate.yaw_rad)
                - 0.5 * body_gyro[2],
                -0.5,
                0.5,
            )
        )
        desired_velocity[6:] = np.clip(4.0 * (posture - q), -1.0, 1.0)

        weights = np.asarray(
            [12.0, 12.0, 80.0, 70.0, 70.0, 12.0] + [3.0] * 12,
            dtype=float,
        )
        diagonal = weights + 1.0e-5
        hessian = sparse.diags(2.0 * diagonal, format="csc")
        gradient = -2.0 * diagonal * desired_velocity

        contact_matrix = contact_velocity_matrix(q)
        identity = np.eye(18)
        constraints = sparse.csc_matrix(np.vstack([contact_matrix, identity]))

        lower_velocity = np.asarray([-0.5] * 3 + [-1.0] * 3 + [-1.0] * 12)
        upper_velocity = -lower_velocity
        acceleration_delta = WBC_MAX_DDQ_RAD_S2 * self.dt_s
        lower_dq = np.maximum(
            -WBC_MAX_DQ_RAD_S,
            self.previous_dq - acceleration_delta,
        )
        upper_dq = np.minimum(
            WBC_MAX_DQ_RAD_S,
            self.previous_dq + acceleration_delta,
        )
        lower_dq = np.maximum(lower_dq, (JOINT_LOWER_RAD - q_ref) / self.dt_s)
        upper_dq = np.minimum(upper_dq, (JOINT_UPPER_RAD - q_ref) / self.dt_s)
        lower_dq = np.maximum(
            lower_dq,
            (q - TRACKING_ENVELOPE_RAD - q_ref) / self.dt_s,
        )
        upper_dq = np.minimum(
            upper_dq,
            (q + TRACKING_ENVELOPE_RAD - q_ref) / self.dt_s,
        )
        lower_velocity[6:] = lower_dq
        upper_velocity[6:] = upper_dq

        if np.any(lower_velocity > upper_velocity):
            return WBCSolveResult(
                q_ref=q_ref.copy(),
                generalized_velocity=np.zeros(18),
                status="infeasible-bounds",
                iterations=0,
                solve_time_s=0.0,
                primal_residual=float("inf"),
                dual_residual=float("inf"),
                contact_velocity_residual_m_s=float("inf"),
                valid=False,
                reason="joint velocity, acceleration, position, or tracking bounds conflict",
            )

        lower = np.concatenate([np.zeros(12), lower_velocity])
        upper = np.concatenate([np.zeros(12), upper_velocity])
        solver = osqp.OSQP()
        setup_kwargs = dict(
            P=hessian,
            q=gradient,
            A=constraints,
            l=lower,
            u=upper,
            verbose=False,
            max_iter=WBC_MAX_ITER,
            eps_abs=WBC_EPS_ABS,
            eps_rel=WBC_EPS_REL,
            polishing=False,
            warm_starting=True,
        )
        try:
            try:
                solver.setup(**setup_kwargs)
            except TypeError:
                setup_kwargs.pop("warm_starting", None)
                setup_kwargs["warm_start"] = True
                setup_kwargs.pop("polishing", None)
                setup_kwargs["polish"] = False
                solver.setup(**setup_kwargs)
        except Exception as error:
            elapsed = time.perf_counter() - started
            return invalid_result(
                "setup-exception",
                "kinematic WBC QP setup raised {}: {}".format(
                    type(error).__name__, error
                ),
                elapsed,
            )
        if self.previous_solution is not None:
            try:
                solver.warm_start(x=self.previous_solution)
            except Exception as error:
                elapsed = time.perf_counter() - started
                return invalid_result(
                    "warm-start-exception",
                    "kinematic WBC warm start raised {}: {}".format(
                        type(error).__name__, error
                    ),
                    elapsed,
                )

        try:
            result = solver.solve()
        except Exception as error:
            elapsed = time.perf_counter() - started
            return invalid_result(
                "solve-exception",
                "kinematic WBC QP solve raised {}: {}".format(
                    type(error).__name__, error
                ),
                elapsed,
            )
        elapsed = time.perf_counter() - started
        info = result.info
        status = str(getattr(info, "status", "unknown")).lower()
        iterations = int(getattr(info, "iter", 0))
        solve_time = float(getattr(info, "solve_time", elapsed) or elapsed)
        primal_residual = float(
            getattr(info, "prim_res", getattr(info, "pri_res", float("inf")))
        )
        dual_residual = float(
            getattr(info, "dual_res", getattr(info, "dua_res", float("inf")))
        )
        solution = result.x

        reason = None
        if solution is None or "solved" not in status:
            reason = "kinematic WBC QP was not solved"
            generalized_velocity = np.zeros(18)
            next_q_ref = q_ref.copy()
            contact_residual = float("inf")
        else:
            generalized_velocity = np.asarray(solution, dtype=float)
            if not np.all(np.isfinite(generalized_velocity)):
                reason = "kinematic WBC QP returned a non-finite solution"
            elif elapsed > WBC_MAX_SOLVE_S:
                reason = "kinematic WBC solve exceeded 10 ms"
            elif (
                primal_residual > WBC_MAX_PRIMAL_RESIDUAL
                or dual_residual > WBC_MAX_DUAL_RESIDUAL
            ):
                reason = "kinematic WBC residual exceeded the configured bound"
            contact_vectors = (contact_matrix @ generalized_velocity).reshape(4, 3)
            contact_residual = float(
                max(np.linalg.norm(contact_vectors[leg]) for leg in range(4))
            )
            next_q_ref = q_ref + generalized_velocity[6:] * self.dt_s
            next_q_ref = np.minimum(np.maximum(next_q_ref, JOINT_LOWER_RAD), JOINT_UPPER_RAD)
            next_q_ref = np.minimum(
                np.maximum(next_q_ref, q - TRACKING_ENVELOPE_RAD),
                q + TRACKING_ENVELOPE_RAD,
            )

        valid = reason is None
        if valid:
            self.previous_dq = generalized_velocity[6:].copy()
            self.previous_solution = generalized_velocity.copy()
        return WBCSolveResult(
            q_ref=next_q_ref,
            generalized_velocity=generalized_velocity,
            status=status,
            iterations=iterations,
            solve_time_s=elapsed,
            primal_residual=primal_residual,
            dual_residual=dual_residual,
            contact_velocity_residual_m_s=contact_residual,
            valid=valid,
            reason=reason,
        )


def task_target_for_gesture(gesture: str, side: str, baseline_rpy: Sequence[float]) -> WBCTarget:
    baseline = _vector(baseline_rpy, 3, "baseline_rpy")
    if gesture == "height" and side == "low":
        height, roll = HEIGHT_LOW_REL_M, float(baseline[0])
    elif gesture == "height" and side == "high":
        height, roll = HEIGHT_HIGH_REL_M, float(baseline[0])
    elif gesture == "roll" and side == "right":
        height, roll = 0.0, float(baseline[0] + ROLL_RIGHT_REL_RAD)
    elif gesture == "roll" and side == "left":
        height, roll = 0.0, float(baseline[0] + ROLL_LEFT_REL_RAD)
    elif side == "standard":
        height, roll = 0.0, float(baseline[0])
    else:
        raise ValueError("unsupported task target: {} {}".format(gesture, side))
    return WBCTarget(
        relative_height_m=height,
        roll_rad=roll,
        pitch_rad=float(baseline[1]),
        yaw_rad=float(baseline[2]),
    )


def interpolate_task_target(source: WBCTarget, target: WBCTarget, alpha: float) -> WBCTarget:
    blend = smoothstep(alpha)
    return WBCTarget(
        relative_height_m=source.relative_height_m
        + (target.relative_height_m - source.relative_height_m) * blend,
        roll_rad=source.roll_rad + (target.roll_rad - source.roll_rad) * blend,
        pitch_rad=source.pitch_rad + (target.pitch_rad - source.pitch_rad) * blend,
        yaw_rad=source.yaw_rad + (target.yaw_rad - source.yaw_rad) * blend,
    )
