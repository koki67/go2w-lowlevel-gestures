#!/usr/bin/env python3
"""Tracking-adaptive Go2W joint-space height and roll gestures.

Physical motion remains opt-in through ``--live`` and the existing interactive
ownership confirmation.  This controller never falls back to a
no-tracking-stop profile.
"""

from __future__ import print_function

import csv
from datetime import datetime
import json
import math
import os
from pathlib import Path
import sys
import time

import numpy as np

import go2w_closed_loop_control as closed_loop
import go2w_gesture_real as base


CONTROLLER_TYPE = "adaptive-joint-space"


def print_adaptive_plan(gesture=None, timing=base.FAST_TIMING):
    base.print_sequence_plan(gesture, timing)
    print("Closed-loop controller: adaptive joint-space reference governor")
    print(
        "  q tracking envelopes: hip 0.18 rad, thigh 0.14 rad, calf 0.25 rad"
    )
    print("  scheduled speed through 50% envelope; linear slowdown to 90%")
    print("  progress stops at 90%; 100% for 0.10 s requests controlled return")
    print(
        "  completion gate: <=50% envelope and max |dq| <=0.20 rad/s for 0.30 s"
    )
    if gesture in (None, "roll"):
        print(
            "  loaded roll completion: <=70% envelope, PD/tau_est equilibrium, "
            "max |dq| <=0.02 rad/s, torque <60%, intended IMU direction for 0.30 s"
        )
    print("  wall timeouts: fast 8 s, startup 12 s, prone return 15 s")
    print(
        "  provisional tau_est policy: warn 60%, stop progress 75%, "
        "controlled return 85% for 0.10 s, immediate error 100%"
    )
    print("  no automatic retry and no no-tracking-stop fallback")


class ClosedLoopTelemetryRecorder:
    """Buffer expanded telemetry; all file writes happen after control stops."""

    def __init__(self, log_dir, gesture, controller_type):
        self.log_dir = Path(log_dir).expanduser()
        self.log_dir.mkdir(parents=True, exist_ok=True)
        if not self.log_dir.is_dir() or not os.access(str(self.log_dir), os.W_OK):
            raise RuntimeError(
                "closed-loop log directory is not writable: {}".format(self.log_dir)
            )
        self.gesture = str(gesture)
        self.controller_type = str(controller_type)
        self.created_at = datetime.now().astimezone()
        self.started_at = None
        self.rows = []
        self._finalized_paths = None

    def start(self):
        if self.started_at is None:
            self.started_at = time.monotonic()

    @staticmethod
    def _finite_or_blank(value):
        try:
            converted = float(value)
        except (TypeError, ValueError):
            return ""
        return converted if math.isfinite(converted) else ""

    def record(
        self,
        sample,
        target_q,
        *,
        phase,
        phase_elapsed_s,
        progress,
        speed_scale,
        tracking_ratio,
        torque_ratio,
        deadline_miss_count,
        consecutive_deadline_misses,
        event="",
        extra=None,
    ):
        if self.started_at is None:
            self.start()
        now = time.monotonic()
        target = [float(value) for value in target_q]
        row = {
            "controller_type": self.controller_type,
            "gesture": self.gesture,
            "run_elapsed_s": now - self.started_at,
            "phase": str(phase),
            "phase_elapsed_s": float(phase_elapsed_s),
            "trajectory_progress": float(progress),
            "speed_scale": float(speed_scale),
            "tracking_ratio": float(tracking_ratio),
            "tau_est_ratio": float(torque_ratio),
            "lowstate_age_s": max(0.0, now - sample.received_at),
            "deadline_miss_count": int(deadline_miss_count),
            "consecutive_deadline_misses": int(consecutive_deadline_misses),
            "event": str(event or ""),
            "roll_rad": float(sample.rpy[0]),
            "pitch_rad": float(sample.rpy[1]),
            "yaw_rad": float(sample.rpy[2]),
            "power_v": self._finite_or_blank(sample.power_v),
            "power_a": self._finite_or_blank(sample.power_a),
        }
        for axis, value in zip(("x", "y", "z"), sample.gyro):
            row["gyro_{}_rad_s".format(axis)] = float(value)
        for axis, value in zip(("x", "y", "z"), sample.acceleration):
            row["accel_{}_m_s2".format(axis)] = float(value)
        for index, joint_name in enumerate(base.LEG_JOINT_NAMES):
            measured = float(sample.pose[index])
            row["measured_{}_rad".format(joint_name)] = measured
            row["velocity_{}_rad_s".format(joint_name)] = float(
                sample.leg_velocity[index]
            )
            row["target_{}_rad".format(joint_name)] = target[index]
            row["error_target_minus_measured_{}_rad".format(joint_name)] = (
                target[index] - measured
            )
        for index in range(16):
            name = (
                base.LEG_JOINT_NAMES[index]
                if index < 12
                else "{}_wheel".format(base.LEG_JOINT_NAMES[(index - 12) * 3][:2])
            )
            row["tau_est_{}_nm".format(name)] = float(sample.tau_est[index])
            row["mode_{}".format(name)] = int(sample.motor_mode[index])
            row["lost_{}".format(name)] = int(sample.motor_lost[index])
            row["temperature_{}_raw".format(name)] = self._finite_or_blank(
                sample.temperature[index]
            )
        if extra:
            row.update(extra)
        self.rows.append(row)

    def _unique_paths(self):
        timestamp = self.created_at.strftime("%Y%m%dT%H%M%S_%f%z")
        safe_controller = self.controller_type.replace(" ", "-")
        stem = "{}_{}_{}_closed-loop".format(
            timestamp, self.gesture, safe_controller
        )
        suffix = 1
        while True:
            candidate = stem if suffix == 1 else "{}-{}".format(stem, suffix)
            csv_path = self.log_dir / "{}.csv".format(candidate)
            summary_path = self.log_dir / "{}.summary.json".format(candidate)
            if not csv_path.exists() and not summary_path.exists():
                return csv_path, summary_path
            suffix += 1

    def finalize(self, outcome, error_text=None):
        if self._finalized_paths is not None:
            return self._finalized_paths
        csv_path, summary_path = self._unique_paths()
        headers = []
        seen = set()
        for row in self.rows:
            for key in row:
                if key not in seen:
                    seen.add(key)
                    headers.append(key)
        with csv_path.open("w", newline="", encoding="utf-8") as output:
            writer = csv.DictWriter(output, fieldnames=headers, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(self.rows)

        phase_counts = {}
        for row in self.rows:
            phase = row["phase"]
            phase_counts[phase] = phase_counts.get(phase, 0) + 1

        def finite_values(key):
            values = []
            for row in self.rows:
                try:
                    value = float(row.get(key, ""))
                except (TypeError, ValueError):
                    continue
                if math.isfinite(value):
                    values.append(value)
            return values

        def range_summary(key):
            values = finite_values(key)
            if not values:
                return {"minimum": None, "maximum": None}
            return {"minimum": min(values), "maximum": max(values)}

        temperature_ranges = {}
        for index in range(16):
            name = (
                base.LEG_JOINT_NAMES[index]
                if index < 12
                else "{}_wheel".format(base.LEG_JOINT_NAMES[(index - 12) * 3][:2])
            )
            temperature_ranges[name] = range_summary(
                "temperature_{}_raw".format(name)
            )

        qp_solve_times = finite_values("wbc_qp_solve_time_s")
        task_height_errors = [
            abs(target - measured)
            for target, measured in zip(
                finite_values("task_target_relative_height_m"),
                finite_values("task_measured_relative_height_m"),
            )
        ]
        task_roll_errors = [
            abs(target - measured)
            for target, measured in zip(
                finite_values("task_target_roll_rad"),
                finite_values("task_measured_roll_rad"),
            )
        ]
        task_pitch_errors = [
            abs(target - measured)
            for target, measured in zip(
                finite_values("task_target_pitch_rad"),
                finite_values("task_measured_pitch_rad"),
            )
        ]
        qp_status_counts = {}
        for row in self.rows:
            status = str(row.get("wbc_qp_status", "")).strip()
            if status:
                qp_status_counts[status] = qp_status_counts.get(status, 0) + 1
        loaded_roll_rows = [
            row
            for row in self.rows
            if row.get("completion_gate") == "loaded-roll-static-pd"
        ]
        loaded_roll_completed_rows = [
            row
            for row in loaded_roll_rows
            if bool(row.get("loaded_roll_gate_completed", False))
        ]
        summary = {
            "schema_version": 1,
            "created_at": self.created_at.isoformat(),
            "controller_type": self.controller_type,
            "gesture": self.gesture,
            "outcome": str(outcome),
            "error": error_text,
            "sample_count": len(self.rows),
            "phase_sample_counts": phase_counts,
            "max_tracking_ratio": max(
                (float(row["tracking_ratio"]) for row in self.rows), default=None
            ),
            "max_tau_est_ratio": max(
                (float(row["tau_est_ratio"]) for row in self.rows), default=None
            ),
            "deadline_miss_count": max(
                (int(row["deadline_miss_count"]) for row in self.rows), default=0
            ),
            "max_consecutive_deadline_misses": max(
                (int(row["consecutive_deadline_misses"]) for row in self.rows),
                default=0,
            ),
            "temperature_raw_by_motor": temperature_ranges,
            "power_v": range_summary("power_v"),
            "power_a": range_summary("power_a"),
            "adaptive_roll_equilibrium": {
                "active": bool(loaded_roll_rows),
                "completion_sample_count": len(loaded_roll_completed_rows),
                "max_raw_tracking_ratio": max(
                    finite_values("loaded_roll_raw_tracking_ratio"),
                    default=None,
                ),
                "max_pd_residual_ratio": max(
                    finite_values("loaded_roll_pd_residual_ratio"),
                    default=None,
                ),
                "max_abs_dq_rad_s": max(
                    finite_values("loaded_roll_max_abs_dq_rad_s"),
                    default=None,
                ),
                "max_signed_body_roll_rad": max(
                    finite_values("loaded_roll_signed_body_roll_rad"),
                    default=None,
                ),
                "thresholds": {
                    "raw_tracking_ratio": (
                        closed_loop.LOADED_ROLL_COMPLETION_RATIO
                    ),
                    "pd_residual_ratio": (
                        closed_loop.LOADED_ROLL_PD_RESIDUAL_RATIO
                    ),
                    "max_abs_dq_rad_s": (
                        closed_loop.LOADED_ROLL_MAX_DQ_RAD_S
                    ),
                    "minimum_signed_body_roll_rad": (
                        closed_loop.LOADED_ROLL_MIN_SIGNED_BODY_ROLL_RAD
                    ),
                    "maximum_torque_ratio_exclusive": (
                        closed_loop.TORQUE_WARN_RATIO
                    ),
                    "dwell_s": closed_loop.CONVERGENCE_DWELL_S,
                },
                "physically_qualified": False,
            },
            "wbc": {
                "qp_solve_count": len(qp_solve_times),
                "qp_solve_p99_s": (
                    float(np.percentile(qp_solve_times, 99))
                    if qp_solve_times
                    else None
                ),
                "qp_solve_max_s": max(qp_solve_times) if qp_solve_times else None,
                "qp_status_counts": qp_status_counts,
                "max_qp_iterations": max(
                    finite_values("wbc_qp_iterations"), default=None
                ),
                "max_contact_torque_residual_ratio": max(
                    finite_values("contact_torque_residual_ratio"), default=None
                ),
                "max_contact_balance_residual_ratio": max(
                    finite_values("contact_balance_residual_ratio"), default=None
                ),
                "minimum_contact_normal_load_n": min(
                    finite_values("contact_minimum_normal_load_n"), default=None
                ),
                "max_contact_velocity_residual_m_s": max(
                    finite_values("wbc_contact_velocity_residual_m_s"), default=None
                ),
                "max_abs_task_height_error_m": max(
                    task_height_errors, default=None
                ),
                "max_abs_task_roll_error_rad": max(
                    task_roll_errors, default=None
                ),
                "max_abs_task_pitch_error_rad": max(
                    task_pitch_errors, default=None
                ),
            },
            "provisional_tau_policy": {
                "warning_ratio": closed_loop.TORQUE_WARN_RATIO,
                "progress_stop_ratio": closed_loop.TORQUE_STOP_RATIO,
                "controlled_return_ratio": closed_loop.TORQUE_RETURN_RATIO,
                "immediate_error_ratio": closed_loop.TORQUE_ERROR_RATIO,
                "physically_qualified": False,
            },
            "qualification": {
                "software_complete": outcome == "completed",
                "simulation_pass": False,
                "jetson_software_pass": False,
                "physical_pass": False,
            },
        }
        with summary_path.open("w", encoding="utf-8") as output:
            json.dump(summary, output, indent=2, sort_keys=True)
            output.write("\n")
        self._finalized_paths = (csv_path, summary_path)
        return self._finalized_paths


class AdaptiveGestureController(base.HardwareGestureController):
    def __init__(self, *args, controller_type=CONTROLLER_TYPE, **kwargs):
        super().__init__(*args, **kwargs)
        self.controller_type = controller_type
        self._closed_loop_recorder = None
        self._expected_motor_modes = None
        self._initial_motor_lost = None
        self._deadline_miss_count = 0
        self._consecutive_deadline_misses = 0
        self._last_governor_warning = None

    def _prepare_tracking_recording(self):
        super()._prepare_tracking_recording()
        if self.tracking_log_dir is not None:
            self._closed_loop_recorder = ClosedLoopTelemetryRecorder(
                self.tracking_log_dir,
                self.gesture,
                self.controller_type,
            )
            self._closed_loop_recorder.start()

    def finalize_tracking_log(self, outcome, error_text=None):
        existing_paths = super().finalize_tracking_log(outcome, error_text=error_text)
        expanded_paths = None
        if self._closed_loop_recorder is not None:
            expanded_paths = self._closed_loop_recorder.finalize(
                outcome, error_text=error_text
            )
            print(
                "closed-loop telemetry saved: {} and {}".format(
                    expanded_paths[0], expanded_paths[1]
                ),
                flush=True,
            )
        return existing_paths, expanded_paths

    @staticmethod
    def _extended_state_is_complete(sample):
        return (
            len(sample.tau_est) == 16
            and len(sample.motor_mode) == 16
            and len(sample.motor_lost) == 16
            and len(sample.temperature) == 16
            and len(sample.gyro) == 3
            and len(sample.acceleration) == 3
        )

    def _capture_extended_baseline(self):
        sample = self._latest_sample()
        if sample is None or not self._extended_state_is_complete(sample):
            raise RuntimeError(
                "extended LowState telemetry is unavailable for closed-loop control"
            )
        self._expected_motor_modes = list(sample.motor_mode)
        self._initial_motor_lost = list(sample.motor_lost)

    def _check_extended_runtime(self, sample):
        if not self._extended_state_is_complete(sample):
            raise RuntimeError("extended LowState telemetry became incomplete")
        required_finite = (
            sample.pose
            + sample.leg_velocity
            + sample.wheel_velocity
            + sample.rpy
            + sample.tau_est
            + sample.gyro
            + sample.acceleration
        )
        if not all(math.isfinite(float(value)) for value in required_finite):
            raise RuntimeError("extended LowState contains a non-finite control value")
        if self._expected_motor_modes is None or self._initial_motor_lost is None:
            self._capture_extended_baseline()
        changed_modes = [
            index
            for index in range(16)
            if sample.motor_mode[index] != self._expected_motor_modes[index]
        ]
        if changed_modes:
            raise base.ControlledReturnRequested(
                "motor mode changed from its startup value at indices {}".format(
                    changed_modes
                )
            )
        increased_lost = [
            index
            for index in range(16)
            if sample.motor_lost[index] > self._initial_motor_lost[index]
        ]
        if increased_lost:
            raise base.ControlledReturnRequested(
                "motor lost counter increased at indices {}".format(increased_lost)
            )
        torque_ratio = max(
            abs(float(sample.tau_est[index])) / closed_loop.TORQUE_LIMIT_NM[index]
            for index in range(12)
        )
        if torque_ratio >= closed_loop.TORQUE_ERROR_RATIO:
            raise RuntimeError(
                "tau_est reached 100% of the model torque range; immediate stop"
            )

    def _validate_preflight_state(self):
        """Fail read-only preflight if closed-loop inputs are unavailable."""

        sample = self._latest_sample()
        if sample is None or time.monotonic() - sample.received_at > base.LOWSTATE_TIMEOUT_S:
            raise RuntimeError(
                "extended LowState is stale during closed-loop preflight"
            )
        self._capture_extended_baseline()
        self._check_extended_runtime(sample)

    def _record_closed_loop(
        self,
        sample,
        decision,
        phase,
        phase_elapsed_s,
        event="",
        extra=None,
    ):
        if self._closed_loop_recorder is None:
            return
        self._closed_loop_recorder.record(
            sample,
            decision.q_ref,
            phase=phase,
            phase_elapsed_s=phase_elapsed_s,
            progress=decision.progress,
            speed_scale=decision.speed_scale,
            tracking_ratio=decision.tracking_ratio,
            torque_ratio=decision.torque_ratio,
            deadline_miss_count=self._deadline_miss_count,
            consecutive_deadline_misses=self._consecutive_deadline_misses,
            event=event,
            extra=extra,
        )

    def _adaptive_phase(
        self,
        phase,
        source,
        target,
        duration_s,
        timeout_s,
        *,
        ignore_first_stop=False,
        return_mode=False,
        loaded_roll_baseline_rad=None,
        loaded_roll_expected_sign=None,
    ):
        governor = closed_loop.ReferenceGovernor(
            source,
            target,
            duration_s=duration_s,
            timeout_s=timeout_s,
        )
        loaded_roll_requested = (
            loaded_roll_baseline_rad is not None
            or loaded_roll_expected_sign is not None
        )
        if loaded_roll_requested and (
            loaded_roll_baseline_rad is None
            or loaded_roll_expected_sign is None
        ):
            raise ValueError(
                "loaded roll completion requires both baseline and expected sign"
            )
        loaded_roll_gate = None
        if loaded_roll_requested:
            loaded_roll_gate = closed_loop.LoadedRollEquilibriumGate(
                loaded_roll_baseline_rad,
                loaded_roll_expected_sign,
                base.KP,
                base.KD,
            )
        start = time.monotonic()
        previous_step = start
        next_tick = start
        last_pose = list(source)
        while True:
            now = time.monotonic()
            elapsed = now - start
            dt_s = min(max(now - previous_step, 0.0), 2.0 * base.CONTROL_PERIOD_S)
            if dt_s <= 0.0:
                dt_s = base.CONTROL_PERIOD_S
            previous_step = now
            sample = super()._check_runtime(
                last_pose,
                ignore_first_stop=ignore_first_stop,
                motion_context=phase,
                motion_elapsed_s=elapsed,
            )
            self._check_extended_runtime(sample)
            decision = governor.step(
                sample.pose,
                sample.leg_velocity,
                sample.tau_est[:12],
                dt_s,
                elapsed,
                return_mode=return_mode,
            )
            event = decision.reason or decision.warning or ""
            phase_completed = decision.completed
            extra = None
            if loaded_roll_gate is not None:
                loaded_status = loaded_roll_gate.update(
                    decision.q_ref,
                    sample.pose,
                    sample.leg_velocity,
                    sample.tau_est[:12],
                    sample.rpy[0],
                    dt_s,
                    endpoint_reached=decision.progress >= 1.0,
                )
                phase_completed = loaded_status.completed
                extra = {
                    "completion_gate": "loaded-roll-static-pd",
                    "loaded_roll_gate_completed": loaded_status.completed,
                    "loaded_roll_gate_dwell_s": loaded_status.accumulated_s,
                    "loaded_roll_raw_tracking_ratio": (
                        loaded_status.raw_tracking_ratio
                    ),
                    "loaded_roll_pd_residual_ratio": (
                        loaded_status.pd_residual_ratio
                    ),
                    "loaded_roll_signed_body_roll_rad": (
                        loaded_status.signed_body_roll_rad
                    ),
                    "loaded_roll_max_abs_dq_rad_s": (
                        loaded_status.max_abs_dq_rad_s
                    ),
                    "loaded_roll_joint_bound_met": loaded_status.joint_bound_met,
                    "loaded_roll_pd_balance_met": loaded_status.pd_balance_met,
                    "loaded_roll_low_velocity_met": loaded_status.low_velocity_met,
                    "loaded_roll_torque_margin_met": (
                        loaded_status.torque_margin_met
                    ),
                    "loaded_roll_direction_met": loaded_status.roll_direction_met,
                }
                if phase_completed and not event:
                    event = "loaded roll static-PD equilibrium confirmed"
            self._record_closed_loop(
                sample,
                decision,
                phase,
                elapsed,
                event=event,
                extra=extra,
            )
            if decision.warning and decision.warning != self._last_governor_warning:
                print("WARNING: {}".format(decision.warning), file=sys.stderr, flush=True)
                self._last_governor_warning = decision.warning
            if decision.emergency:
                raise RuntimeError(decision.reason or "governor emergency stop")
            if decision.request_return:
                raise base.ControlledReturnRequested(
                    decision.reason or "reference governor requested controlled return"
                )
            last_pose = decision.q_ref.tolist()
            self._write_pose(last_pose)
            if phase_completed:
                return last_pose

            next_tick += base.CONTROL_PERIOD_S
            delay = next_tick - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
                self._consecutive_deadline_misses = 0
            else:
                self._deadline_miss_count += 1
                self._consecutive_deadline_misses += 1
                if delay < -base.CONTROL_PERIOD_S:
                    next_tick = time.monotonic()

    def _adaptive_transition(
        self,
        name,
        source,
        target,
        duration_s,
        timeout_s,
        *,
        ignore_first_stop=False,
        return_mode=False,
        loaded_roll_baseline_rad=None,
        loaded_roll_expected_sign=None,
    ):
        phase = "adaptive transition -> {}".format(name)
        print(
            "{} (nominal {:g} s, wall timeout {:g} s)".format(
                phase, duration_s, timeout_s
            ),
            flush=True,
        )
        return self._adaptive_phase(
            phase,
            source,
            target,
            duration_s,
            timeout_s,
            ignore_first_stop=ignore_first_stop,
            return_mode=return_mode,
            loaded_roll_baseline_rad=loaded_roll_baseline_rad,
            loaded_roll_expected_sign=loaded_roll_expected_sign,
        )

    def _adaptive_hold(
        self,
        name,
        pose,
        minimum_s,
        *,
        ignore_first_stop=False,
        return_mode=False,
        loaded_roll_baseline_rad=None,
        loaded_roll_expected_sign=None,
    ):
        phase = "adaptive hold {}".format(name)
        timeout_s = minimum_s + closed_loop.HOLD_CONVERGENCE_TIMEOUT_S
        print(
            "{} (minimum {:.1f} s, convergence timeout {:.1f} s)".format(
                phase, minimum_s, closed_loop.HOLD_CONVERGENCE_TIMEOUT_S
            ),
            flush=True,
        )
        self._adaptive_phase(
            phase,
            pose,
            pose,
            minimum_s,
            timeout_s,
            ignore_first_stop=ignore_first_stop,
            return_mode=return_mode,
            loaded_roll_baseline_rad=loaded_roll_baseline_rad,
            loaded_roll_expected_sign=loaded_roll_expected_sign,
        )

    def _run_height_sequence(self):
        self._capture_extended_baseline()
        sample = self._latest_sample()
        current_pose = list(sample.pose)
        current_pose = self._adaptive_transition(
            "standard",
            current_pose,
            base.STANDARD,
            base.STANDARD_TRANSITION_S,
            closed_loop.STARTUP_TIMEOUT_S,
        )
        self._adaptive_hold("standard", base.STANDARD, base.STANDARD_HOLD_S)
        for cycle in range(1, base.HEIGHT_CYCLES + 1):
            print("adaptive height cycle {}/{}".format(cycle, base.HEIGHT_CYCLES))
            current_pose = self._adaptive_transition(
                "low",
                current_pose,
                base.LOW,
                self.timing.transition_s,
                closed_loop.FAST_TRANSITION_TIMEOUT_S,
            )
            self._adaptive_hold("low", base.LOW, self.timing.hold_s)
            current_pose = self._adaptive_transition(
                "high",
                current_pose,
                base.HIGH,
                self.timing.transition_s,
                closed_loop.FAST_TRANSITION_TIMEOUT_S,
            )
            self._adaptive_hold("high", base.HIGH, self.timing.hold_s)
        current_pose = self._adaptive_transition(
            "standard",
            current_pose,
            base.STANDARD,
            base.STANDARD_TRANSITION_S,
            closed_loop.STARTUP_TIMEOUT_S,
        )
        self._adaptive_hold("standard", base.STANDARD, base.STANDARD_HOLD_S)
        self._finish_adaptive_at_prone(current_pose)

    def _run_roll_sequence(self):
        self._capture_extended_baseline()
        sample = self._latest_sample()
        current_pose = list(sample.pose)
        current_pose = self._adaptive_transition(
            "standard",
            current_pose,
            base.STANDARD,
            base.STANDARD_TRANSITION_S,
            closed_loop.STARTUP_TIMEOUT_S,
        )
        self._adaptive_hold("standard", base.STANDARD, base.STANDARD_HOLD_S)
        standard_sample = self._latest_sample()
        if standard_sample is None:
            raise RuntimeError("extended LowState missing after STANDARD hold")
        self._check_extended_runtime(standard_sample)
        standard_roll_rad = float(standard_sample.rpy[0])
        for cycle in range(1, base.ROLL_CYCLES + 1):
            print("adaptive roll cycle {}/{}".format(cycle, base.ROLL_CYCLES))
            current_pose = self._adaptive_transition(
                "right roll",
                current_pose,
                base.ROLL_RIGHT,
                self.timing.transition_s,
                closed_loop.FAST_TRANSITION_TIMEOUT_S,
                loaded_roll_baseline_rad=standard_roll_rad,
                loaded_roll_expected_sign=closed_loop.ADAPTIVE_ROLL_RIGHT_IMU_SIGN,
            )
            self._adaptive_hold(
                "right roll",
                base.ROLL_RIGHT,
                self.timing.hold_s,
                loaded_roll_baseline_rad=standard_roll_rad,
                loaded_roll_expected_sign=closed_loop.ADAPTIVE_ROLL_RIGHT_IMU_SIGN,
            )
            current_pose = self._adaptive_transition(
                "left roll",
                current_pose,
                base.ROLL_LEFT,
                self.timing.transition_s,
                closed_loop.FAST_TRANSITION_TIMEOUT_S,
                loaded_roll_baseline_rad=standard_roll_rad,
                loaded_roll_expected_sign=closed_loop.ADAPTIVE_ROLL_LEFT_IMU_SIGN,
            )
            self._adaptive_hold(
                "left roll",
                base.ROLL_LEFT,
                self.timing.hold_s,
                loaded_roll_baseline_rad=standard_roll_rad,
                loaded_roll_expected_sign=closed_loop.ADAPTIVE_ROLL_LEFT_IMU_SIGN,
            )
        current_pose = self._adaptive_transition(
            "standard",
            current_pose,
            base.STANDARD,
            base.STANDARD_TRANSITION_S,
            closed_loop.STARTUP_TIMEOUT_S,
        )
        self._adaptive_hold("standard", base.STANDARD, base.STANDARD_HOLD_S)
        self._finish_adaptive_at_prone(current_pose)

    def _finish_adaptive_at_prone(self, current_pose):
        self._adaptive_transition(
            "captured prone",
            current_pose,
            self._captured_prone,
            base.PRONE_TRANSITION_S,
            closed_loop.PRONE_RETURN_TIMEOUT_S,
            return_mode=True,
        )
        self._adaptive_hold(
            "captured prone",
            self._captured_prone,
            base.PRONE_HOLD_S,
            return_mode=True,
        )
        self._ended_prone = True
        self._neutralize(base.NEUTRAL_COMMAND_S)

    def _return_prone_after_interrupt(self):
        if self._publisher is None or self._captured_prone is None:
            return False
        sample = self._latest_sample()
        if sample is None or time.monotonic() - sample.received_at > base.LOWSTATE_TIMEOUT_S:
            print(
                "cannot perform adaptive return because LowState is stale",
                file=sys.stderr,
                flush=True,
            )
            return False
        try:
            self._adaptive_transition(
                "captured prone after stop",
                sample.pose,
                self._captured_prone,
                base.PRONE_TRANSITION_S,
                closed_loop.PRONE_RETURN_TIMEOUT_S,
                ignore_first_stop=True,
                return_mode=True,
            )
            self._adaptive_hold(
                "captured prone",
                self._captured_prone,
                base.PRONE_HOLD_S,
                ignore_first_stop=True,
                return_mode=True,
            )
            self._ended_prone = True
            return True
        except (base.HardStop, base.ControlledReturnRequested, RuntimeError) as error:
            print(
                "adaptive controlled return aborted: {}".format(error),
                file=sys.stderr,
                flush=True,
            )
            return False


def main(argv=None):
    return base.main(
        argv,
        timing=base.FAST_TIMING,
        tracking_stop_rad=base.RUN_MAX_TRACKING_ERROR_RAD,
        controller_class=AdaptiveGestureController,
        sequence_printer=print_adaptive_plan,
    )


if __name__ == "__main__":
    raise SystemExit(main())
