#!/usr/bin/env python3
"""Quasi-static kinematic WBC Go2W height and roll gestures.

The controller solves a constrained 100 Hz generalized-velocity QP, integrates
safe leg position references, and republishes those references through the
existing 500 Hz LowCmd position PD.  It never commands feed-forward torque and
is not a direct-torque dynamic WBC.
"""

from __future__ import print_function

import math
import queue
from types import SimpleNamespace
import sys
import threading
import time

import numpy as np

import go2w_closed_loop_control as closed_loop
import go2w_gesture_real as base
import go2w_gesture_real_adaptive as adaptive


CONTROLLER_TYPE = "quasi-static-kinematic-wbc"
SUPPORT_CONFIRMATION = "FOUR WHEELS LOADED AND BELLY CLEAR"
SUPPORT_CONFIRMATION_TIMEOUT_S = 120.0
CONTACT_GATE_TIMEOUT_S = 5.0
MAX_CONSECUTIVE_DEADLINE_MISSES = 5


def print_wbc_plan(gesture=None, timing=base.FAST_TIMING):
    adaptive.print_adaptive_plan(gesture, timing)
    print("WBC task stage: quasi-static constrained kinematic QP")
    print("  QP: 100 Hz; LowCmd position PD resend: 500 Hz; commanded tau: 0")
    print("  variables: [base twist 6, leg dq 12]")
    print("  hard constraints: four wheel-center velocities = 0")
    print("  joint bounds: |dq| <=1.0 rad/s, |ddq| <=4.0 rad/s^2")
    print(
        "  height targets relative to STANDARD: low -0.093178 m, high +0.076281 m"
    )
    print(
        "  roll targets relative to STANDARD: right -0.395469 rad, "
        "left +0.395469 rad"
    )
    print("  pitch stays at the STANDARD reference; x/y/yaw are held")
    print(
        "  contact forces use actuator/gravity equilibrium and J(q)^T f; "
        "foot_force is unused"
    )
    print("  requires 0.5 s continuously valid four-wheel load estimation")
    print("  hardware requires: {}".format(SUPPORT_CONFIRMATION))
    print("  this is not a direct-torque dynamic WBC")


class WBCGestureController(adaptive.AdaptiveGestureController):
    def __init__(
        self,
        *args,
        require_support_confirmation=True,
        **kwargs
    ):
        kwargs["controller_type"] = CONTROLLER_TYPE
        super().__init__(*args, **kwargs)
        self.require_support_confirmation = bool(require_support_confirmation)
        self._baseline_height_m = None
        self._baseline_rpy = None
        self._latest_contact_estimate = None
        self._wbc_solver = closed_loop.KinematicWBC()
        self._last_wbc_extra = {}

    @staticmethod
    def _task_is_within_tolerance(estimate, target):
        angular_tolerance = math.radians(2.0)
        return (
            abs(estimate.relative_height_m - target.relative_height_m) <= 0.015
            and abs(estimate.roll_rad - target.roll_rad) <= angular_tolerance
            and abs(estimate.pitch_rad - target.pitch_rad) <= angular_tolerance
        )

    @staticmethod
    def _contact_extra(contact):
        extra = {
            "contact_qp_status": contact.status,
            "contact_qp_iterations": contact.iterations,
            "contact_qp_solve_time_s": contact.solve_time_s,
            "contact_max_jacobian_condition": contact.max_jacobian_condition,
            "contact_torque_residual_ratio": contact.torque_residual_ratio,
            "contact_force_balance_residual_n": contact.force_balance_residual_n,
            "contact_moment_balance_residual_nm": contact.moment_balance_residual_nm,
            "contact_balance_residual_ratio": contact.balance_residual_ratio,
            "contact_total_vertical_load_n": contact.total_vertical_load_n,
            "contact_minimum_normal_load_n": contact.minimum_normal_load_n,
            "contact_valid": int(contact.valid),
            "contact_invalid_reason": contact.reason or "",
        }
        for leg_index, leg_name in enumerate(closed_loop.LEG_NAMES):
            for axis_index, axis_name in enumerate(("x", "y", "z")):
                extra["contact_{}_force_{}_n".format(leg_name, axis_name)] = float(
                    contact.forces[leg_index, axis_index]
                )
        return extra

    @staticmethod
    def _task_extra(estimate, target, qp_result=None):
        extra = {
            "task_target_relative_height_m": target.relative_height_m,
            "task_measured_relative_height_m": estimate.relative_height_m,
            "task_measured_raw_height_m": estimate.raw_height_m,
            "task_target_roll_rad": target.roll_rad,
            "task_measured_roll_rad": estimate.roll_rad,
            "task_target_pitch_rad": target.pitch_rad,
            "task_measured_pitch_rad": estimate.pitch_rad,
            "task_target_yaw_rad": target.yaw_rad,
            "task_measured_yaw_rad": estimate.yaw_rad,
        }
        if qp_result is not None:
            extra.update(
                {
                    "wbc_qp_status": qp_result.status,
                    "wbc_qp_iterations": qp_result.iterations,
                    "wbc_qp_solve_time_s": qp_result.solve_time_s,
                    "wbc_qp_primal_residual": qp_result.primal_residual,
                    "wbc_qp_dual_residual": qp_result.dual_residual,
                    "wbc_contact_velocity_residual_m_s": (
                        qp_result.contact_velocity_residual_m_s
                    ),
                    "wbc_qp_valid": int(qp_result.valid),
                    "wbc_qp_invalid_reason": qp_result.reason or "",
                }
            )
        return extra

    def _estimate_contacts(self, sample):
        try:
            contact = closed_loop.estimate_contact_forces(
                sample.pose,
                sample.tau_est[:12],
                sample.rpy,
            )
        except Exception as error:
            raise base.ControlledReturnRequested(
                "contact estimator failed closed with {}: {}".format(
                    type(error).__name__, error
                )
            ) from error
        self._latest_contact_estimate = contact
        return contact

    def _validate_preflight_state(self):
        super()._validate_preflight_state()
        closed_loop._load_qp_dependencies()

    def _estimate_task(self, sample, contact):
        if self._baseline_height_m is None:
            raise RuntimeError("WBC task baseline has not been calibrated")
        return closed_loop.estimate_task_space(
            sample.pose,
            sample.rpy,
            contact.forces,
            baseline_height_m=self._baseline_height_m,
        )

    def _record_wbc_tick(
        self,
        sample,
        q_ref,
        decision,
        phase,
        elapsed,
        *,
        event="",
        extra=None,
    ):
        if self._closed_loop_recorder is None:
            return
        telemetry_decision = SimpleNamespace(
            q_ref=np.asarray(q_ref, dtype=float),
            progress=decision.progress,
            speed_scale=decision.speed_scale,
            tracking_ratio=decision.tracking_ratio,
            torque_ratio=decision.torque_ratio,
        )
        self._record_closed_loop(
            sample,
            telemetry_decision,
            phase,
            elapsed,
            event=event,
            extra=extra,
        )

    def _deadline_sleep(self, next_tick):
        delay = next_tick - time.monotonic()
        if delay > 0.0:
            time.sleep(delay)
            self._consecutive_deadline_misses = 0
            return next_tick
        self._deadline_miss_count += 1
        self._consecutive_deadline_misses += 1
        if self._consecutive_deadline_misses >= MAX_CONSECUTIVE_DEADLINE_MISSES:
            raise base.ControlledReturnRequested(
                "500 Hz publication deadline missed {} consecutive times".format(
                    self._consecutive_deadline_misses
                )
            )
        if delay < -base.CONTROL_PERIOD_S:
            return time.monotonic()
        return next_tick

    def _wait_for_valid_contact_estimate(self, q_ref):
        print(
            "holding STANDARD at 500 Hz while validating four-wheel load estimation",
            flush=True,
        )
        gate = closed_loop.ContactValidityGate(0.5)
        start = time.monotonic()
        next_tick = start
        tick = 0
        latest_contact = None
        hold_governor = closed_loop.ReferenceGovernor(
            q_ref,
            q_ref,
            duration_s=1.0,
            timeout_s=CONTACT_GATE_TIMEOUT_S + 1.0,
        )
        while True:
            elapsed = time.monotonic() - start
            if elapsed > CONTACT_GATE_TIMEOUT_S:
                reason = (
                    latest_contact.reason
                    if latest_contact is not None
                    else "no contact estimate was produced"
                )
                raise base.ControlledReturnRequested(
                    "four-wheel load estimate was not valid for 0.5 s: {}".format(
                        reason
                    )
                )
            sample = super(adaptive.AdaptiveGestureController, self)._check_runtime(
                q_ref,
                motion_context="WBC contact validation",
                motion_elapsed_s=elapsed,
            )
            self._check_extended_runtime(sample)
            if tick % 5 == 0:
                hold_decision = hold_governor.step(
                    sample.pose,
                    sample.leg_velocity,
                    sample.tau_est[:12],
                    closed_loop.WBC_PERIOD_S,
                    elapsed,
                )
                if hold_decision.emergency:
                    raise RuntimeError(
                        hold_decision.reason or "contact-gate torque emergency"
                    )
                if hold_decision.request_return:
                    raise base.ControlledReturnRequested(
                        hold_decision.reason
                        or "contact-gate governor requested controlled return"
                    )
                latest_contact = self._estimate_contacts(sample)
                ready = gate.update(latest_contact.valid, closed_loop.WBC_PERIOD_S)
                self._record_wbc_tick(
                    sample,
                    q_ref,
                    hold_decision,
                    "WBC contact validation",
                    elapsed,
                    event=latest_contact.reason or "contact-valid",
                    extra=self._contact_extra(latest_contact),
                )
                if ready:
                    raw = closed_loop.estimate_task_space(
                        sample.pose,
                        sample.rpy,
                        latest_contact.forces,
                        baseline_height_m=0.0,
                    )
                    self._baseline_height_m = raw.raw_height_m
                    self._baseline_rpy = list(sample.rpy)
                    loads = ", ".join(
                        "{}={:.1f} N".format(
                            closed_loop.LEG_NAMES[index],
                            latest_contact.forces[index, 2],
                        )
                        for index in range(4)
                    )
                    print(
                        "four-wheel estimate valid for 0.5 s: {}; balance residual {:.1%}".format(
                            loads, latest_contact.balance_residual_ratio
                        ),
                        flush=True,
                    )
                    return latest_contact
            self._write_pose(q_ref)
            tick += 1
            next_tick += base.CONTROL_PERIOD_S
            next_tick = self._deadline_sleep(next_tick)

    def _hold_while_confirming_support(self, q_ref):
        if not self.require_support_confirmation:
            return
        if not sys.stdin.isatty():
            raise base.ControlledReturnRequested(
                "WBC support confirmation requires an interactive TTY"
            )
        print(
            "Visually verify belly clearance, four loaded wheels, blocked wheels, "
            "support/spotter, and E-stop readiness.",
            flush=True,
        )
        responses = queue.Queue(maxsize=1)

        def read_confirmation():
            try:
                entered = input("Type {!r} to start WBC motion: ".format(SUPPORT_CONFIRMATION))
                responses.put((entered.strip(), None))
            except BaseException as error:  # input failures must fail closed
                responses.put((None, error))

        input_thread = threading.Thread(target=read_confirmation, daemon=True)
        input_thread.start()
        start = time.monotonic()
        next_tick = start
        tick = 0
        latest_contact = self._latest_contact_estimate
        hold_governor = closed_loop.ReferenceGovernor(
            q_ref,
            q_ref,
            duration_s=1.0,
            timeout_s=SUPPORT_CONFIRMATION_TIMEOUT_S + 1.0,
        )
        while True:
            elapsed = time.monotonic() - start
            if elapsed > SUPPORT_CONFIRMATION_TIMEOUT_S:
                raise base.ControlledReturnRequested(
                    "timed out waiting for four-wheel and belly-clear confirmation"
                )
            sample = super(adaptive.AdaptiveGestureController, self)._check_runtime(
                q_ref,
                motion_context="WBC visual support confirmation",
                motion_elapsed_s=elapsed,
            )
            self._check_extended_runtime(sample)
            if tick % 5 == 0:
                hold_decision = hold_governor.step(
                    sample.pose,
                    sample.leg_velocity,
                    sample.tau_est[:12],
                    closed_loop.WBC_PERIOD_S,
                    elapsed,
                )
                if hold_decision.emergency:
                    raise RuntimeError(
                        hold_decision.reason or "support-hold torque emergency"
                    )
                if hold_decision.request_return:
                    raise base.ControlledReturnRequested(
                        hold_decision.reason
                        or "support-hold governor requested controlled return"
                    )
                latest_contact = self._estimate_contacts(sample)
                if not latest_contact.valid:
                    raise base.ControlledReturnRequested(
                        "contact estimate became invalid during visual confirmation: {}".format(
                            latest_contact.reason
                        )
                    )
                self._record_wbc_tick(
                    sample,
                    q_ref,
                    hold_decision,
                    "WBC visual support confirmation",
                    elapsed,
                    extra=self._contact_extra(latest_contact),
                )
            try:
                entered, input_error = responses.get_nowait()
            except queue.Empty:
                entered = input_error = None
            else:
                if input_error is not None:
                    raise base.ControlledReturnRequested(
                        "support confirmation input failed: {}".format(input_error)
                    )
                if entered != SUPPORT_CONFIRMATION:
                    raise base.ControlledReturnRequested(
                        "support confirmation did not match"
                    )
                # Re-anchor height and attitude after the operator's inspection.
                raw = closed_loop.estimate_task_space(
                    sample.pose,
                    sample.rpy,
                    latest_contact.forces,
                    baseline_height_m=0.0,
                )
                self._baseline_height_m = raw.raw_height_m
                self._baseline_rpy = list(sample.rpy)
                print("WBC support confirmation accepted", flush=True)
                return
            self._write_pose(q_ref)
            tick += 1
            next_tick += base.CONTROL_PERIOD_S
            next_tick = self._deadline_sleep(next_tick)

    def _run_wbc_phase(
        self,
        name,
        current_q_ref,
        posture_source,
        posture_target,
        task_source,
        task_target,
        duration_s,
        timeout_s,
    ):
        phase = "WBC {}".format(name)
        print(
            "{} (nominal {:g} s, wall timeout {:g} s)".format(
                phase, duration_s, timeout_s
            ),
            flush=True,
        )
        governor = closed_loop.TaskProgressGovernor(duration_s, timeout_s)
        q_ref = np.asarray(current_q_ref, dtype=float).copy()
        start = time.monotonic()
        next_tick = start
        tick = 0
        last_decision = None
        last_extra = dict(self._last_wbc_extra)
        invalid_contact_s = 0.0
        while True:
            elapsed = time.monotonic() - start
            sample = super(adaptive.AdaptiveGestureController, self)._check_runtime(
                q_ref.tolist(),
                motion_context=phase,
                motion_elapsed_s=elapsed,
            )
            self._check_extended_runtime(sample)
            if tick % 5 == 0:
                contact = self._estimate_contacts(sample)
                invalid_contact_s = (
                    0.0
                    if contact.valid
                    else invalid_contact_s + closed_loop.WBC_PERIOD_S
                )
                if invalid_contact_s >= 0.10:
                    raise base.ControlledReturnRequested(
                        "contact estimate invalid for 0.10 s during WBC: {}".format(
                            contact.reason
                        )
                    )
                if not contact.valid:
                    # Hold the last safe q_ref while allowing only the bounded
                    # 0.10 s validation grace period.
                    self._write_pose(q_ref.tolist())
                else:
                    estimate = self._estimate_task(sample, contact)
                    final_tolerance = self._task_is_within_tolerance(
                        estimate, task_target
                    )
                    decision = governor.step(
                        q_ref,
                        sample.pose,
                        sample.leg_velocity,
                        sample.tau_est[:12],
                        closed_loop.WBC_PERIOD_S,
                        elapsed,
                        task_within_tolerance=final_tolerance,
                    )
                    if decision.emergency:
                        raise RuntimeError(decision.reason or "WBC governor emergency")
                    if decision.request_return:
                        raise base.ControlledReturnRequested(
                            decision.reason or "WBC governor requested controlled return"
                        )
                    desired_task = closed_loop.interpolate_task_target(
                        task_source,
                        task_target,
                        decision.progress,
                    )
                    desired_posture = closed_loop.smooth_path(
                        posture_source,
                        posture_target,
                        decision.progress,
                    )
                    try:
                        qp_result = self._wbc_solver.solve(
                            sample.pose,
                            q_ref,
                            desired_posture,
                            estimate,
                            desired_task,
                            sample.gyro,
                        )
                    except Exception as error:
                        raise base.ControlledReturnRequested(
                            "WBC solver failed closed with {}: {}".format(
                                type(error).__name__, error
                            )
                        ) from error
                    if not qp_result.valid:
                        raise base.ControlledReturnRequested(
                            "WBC QP failed closed: {}".format(qp_result.reason)
                        )
                    q_ref = qp_result.q_ref.copy()
                    last_decision = decision
                    last_extra = self._contact_extra(contact)
                    last_extra.update(
                        self._task_extra(estimate, desired_task, qp_result)
                    )
                    self._last_wbc_extra = dict(last_extra)
                    if decision.warning and decision.warning != self._last_governor_warning:
                        print(
                            "WARNING: {}".format(decision.warning),
                            file=sys.stderr,
                            flush=True,
                        )
                        self._last_governor_warning = decision.warning
                    if decision.completed:
                        self._write_pose(q_ref.tolist())
                        self._record_wbc_tick(
                            sample,
                            q_ref,
                            decision,
                            phase,
                            elapsed,
                            event="phase-complete",
                            extra=last_extra,
                        )
                        return q_ref.tolist()
            self._write_pose(q_ref.tolist())
            if last_decision is None:
                last_decision = SimpleNamespace(
                    progress=0.0,
                    speed_scale=0.0,
                    tracking_ratio=0.0,
                    torque_ratio=0.0,
                )
            self._record_wbc_tick(
                sample,
                q_ref,
                last_decision,
                phase,
                elapsed,
                extra=last_extra,
            )
            tick += 1
            next_tick += base.CONTROL_PERIOD_S
            next_tick = self._deadline_sleep(next_tick)

    def _run_wbc_sequence(self):
        self._capture_extended_baseline()
        sample = self._latest_sample()
        current_q_ref = self._adaptive_transition(
            "standard",
            sample.pose,
            base.STANDARD,
            base.STANDARD_TRANSITION_S,
            closed_loop.STARTUP_TIMEOUT_S,
        )
        self._adaptive_hold("standard", base.STANDARD, base.STANDARD_HOLD_S)
        self._wait_for_valid_contact_estimate(current_q_ref)
        self._hold_while_confirming_support(current_q_ref)
        if self._baseline_rpy is None:
            raise RuntimeError("WBC attitude baseline was not established")

        standard_task = closed_loop.task_target_for_gesture(
            self.gesture, "standard", self._baseline_rpy
        )
        task_source = standard_task
        posture_source = np.asarray(base.STANDARD, dtype=float)
        if self.gesture == "height":
            side_specs = (
                ("low", base.LOW),
                ("high", base.HIGH),
            )
            cycles = base.HEIGHT_CYCLES
        elif self.gesture == "roll":
            side_specs = (
                ("right", base.ROLL_RIGHT),
                ("left", base.ROLL_LEFT),
            )
            cycles = base.ROLL_CYCLES
        else:
            raise RuntimeError("unsupported WBC gesture: {!r}".format(self.gesture))

        for cycle in range(1, cycles + 1):
            print("WBC {} cycle {}/{}".format(self.gesture, cycle, cycles), flush=True)
            for side, posture_target in side_specs:
                task_target = closed_loop.task_target_for_gesture(
                    self.gesture, side, self._baseline_rpy
                )
                current_q_ref = self._run_wbc_phase(
                    "transition -> {}".format(side),
                    current_q_ref,
                    posture_source,
                    posture_target,
                    task_source,
                    task_target,
                    self.timing.transition_s,
                    closed_loop.FAST_TRANSITION_TIMEOUT_S,
                )
                current_q_ref = self._run_wbc_phase(
                    "hold {}".format(side),
                    current_q_ref,
                    posture_target,
                    posture_target,
                    task_target,
                    task_target,
                    self.timing.hold_s,
                    self.timing.hold_s + closed_loop.HOLD_CONVERGENCE_TIMEOUT_S,
                )
                posture_source = np.asarray(posture_target, dtype=float)
                task_source = task_target

        sample = self._latest_sample()
        current_pose = self._adaptive_transition(
            "standard after WBC",
            sample.pose,
            base.STANDARD,
            base.STANDARD_TRANSITION_S,
            closed_loop.STARTUP_TIMEOUT_S,
        )
        self._adaptive_hold("standard", base.STANDARD, base.STANDARD_HOLD_S)
        self._finish_adaptive_at_prone(current_pose)

    def _run_selected_gesture(self):
        self._run_wbc_sequence()


def main(argv=None):
    return base.main(
        argv,
        timing=base.FAST_TIMING,
        tracking_stop_rad=base.RUN_MAX_TRACKING_ERROR_RAD,
        controller_class=WBCGestureController,
        sequence_printer=print_wbc_plan,
    )


if __name__ == "__main__":
    raise SystemExit(main())
