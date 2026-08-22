#!/usr/bin/env python3
"""MuJoCo qualification and GUI inspection for closed-loop Go2W control.

The controller consumes only hardware-equivalent joint, actuator-torque, and
IMU state.  MuJoCo ground-truth base pose and contacts are recorded strictly as
evaluation evidence and are never passed into the control kernel.  Qualification
is headless by default; ``--viewer`` attaches MuJoCo's passive GUI to the same
model, data, and controller execution path for one selected initial condition.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import shutil
import sys
import tempfile
import time

import go2w_adaptive_plot as adaptive_plot


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
UNITREE_MUJOCO_ROOT = Path(
    os.environ.get("UNITREE_MUJOCO_ROOT", WORKSPACE_ROOT.parent / "unitree_mujoco")
).expanduser().resolve()
VENV_PYTHON = Path(
    os.environ.get(
        "UNITREE_MUJOCO_PYTHON",
        UNITREE_MUJOCO_ROOT / "simulate_python" / ".venv" / "bin" / "python",
    )
).expanduser().absolute()
VENV_ROOT = VENV_PYTHON.parent.parent
MODEL_DIR = UNITREE_MUJOCO_ROOT / "unitree_robots" / "go2w"
MODEL_XML = MODEL_DIR / "go2w.xml"
MODEL_ASSETS = MODEL_DIR / "assets"
FLAT_SCENE = WORKSPACE_ROOT / "simulation" / "scenes" / "go2w_flat.xml"
DEFAULT_OUTPUT_DIR = WORKSPACE_ROOT / "runs" / "mujoco" / "closed-loop"

INITIAL_CONDITIONS = ("normal", "asymmetric-prone", "belly-loaded-prone")
CONTROLLERS = ("adaptive", "wbc")
GESTURES = ("height", "roll")
NOMINAL_PRONE = [0.0, 1.36, -2.65] * 4
ASYMMETRIC_PRONE = [
    0.08,
    1.48,
    -2.70,
    -0.03,
    1.24,
    -2.57,
    0.04,
    1.42,
    -2.68,
    -0.10,
    1.28,
    -2.60,
]
BELLY_LOADED_PRONE = [0.0, 1.52, -2.71] * 4

np = None
mujoco = None
mujoco_viewer = None
control = None
hardware = None


class SimulationFailure(RuntimeError):
    pass


class ViewerClosed(SimulationFailure):
    """Raised when the operator closes the passive viewer during a case."""


def reexec_with_simulator_python() -> None:
    if Path(sys.prefix).resolve() == VENV_ROOT.resolve():
        return
    if not VENV_PYTHON.is_file():
        raise RuntimeError("simulation Python not found: {}".format(VENV_PYTHON))
    os.execv(str(VENV_PYTHON), [str(VENV_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]])


def load_runtime(enable_viewer=False) -> None:
    global np, mujoco, mujoco_viewer, control, hardware
    if str(WORKSPACE_ROOT) not in sys.path:
        sys.path.insert(0, str(WORKSPACE_ROOT))
    try:
        import numpy as _np
        import mujoco as _mujoco
        if enable_viewer:
            import mujoco.viewer as _mujoco_viewer
        else:
            _mujoco_viewer = None
        import osqp  # noqa: F401
        import scipy  # noqa: F401
        import go2w_closed_loop_control as _control
        import go2w_gesture_real as _hardware
    except ImportError as error:
        raise RuntimeError(
            "closed-loop simulation dependency missing in {}: {}".format(
                Path(sys.executable).resolve(), error
            )
        ) from error
    np = _np
    mujoco = _mujoco
    mujoco_viewer = _mujoco_viewer
    control = _control
    hardware = _hardware


def describe(controller_name=None, gesture=None, initial="all") -> None:
    controllers = CONTROLLERS if controller_name is None else (controller_name,)
    gestures = GESTURES if gesture is None else (gesture,)
    initials = INITIAL_CONDITIONS if initial == "all" else (initial,)
    print("Go2W closed-loop MuJoCo qualification")
    print("  controllers: {}".format(", ".join(controllers)))
    print("  gestures: {}".format(", ".join(gestures)))
    print("  initial conditions: {}".format(", ".join(initials)))
    print("  cycles per case: 3")
    print("  adaptive transitions/holds: 1.0 s / 0.5 s nominal")
    print("  adaptive roll completion: load-aware static-PD equilibrium gate")
    print("  WBC QP / position-PD publication: 100 Hz / 500 Hz")
    print("  external MJCF/source are read-only; a temporary scene is used")
    print("  ground truth is evaluation-only, never a controller input")
    print("  --viewer: same controller path in a real-time MuJoCo GUI, one initial condition")
    print("  --save-plot: adaptive joint tracking and governor SVGs (opt-in)")
    print("  output is simulation qualification, not physical qualification")


def doctor() -> bool:
    checks = (
        ("unitree_mujoco root", UNITREE_MUJOCO_ROOT, "dir"),
        ("simulation Python", VENV_PYTHON, "exec"),
        ("Go2W MJCF", MODEL_XML, "file"),
        ("Go2W assets", MODEL_ASSETS, "dir"),
        ("flat scene", FLAT_SCENE, "file"),
    )
    ready = True
    for label, path, kind in checks:
        exists = path.is_dir() if kind == "dir" else path.is_file()
        if kind == "exec":
            exists = exists and os.access(path, os.X_OK)
        print("[{}] {}: {}".format("ok" if exists else "missing", label, path))
        ready = ready and exists
    try:
        load_runtime()
        print("[ok] mujoco/numpy/scipy/osqp and shared control kernel")
    except RuntimeError as error:
        print("[missing] {}".format(error), file=sys.stderr)
        ready = False
    return ready


def quaternion_to_rpy(quaternion):
    w, x, y, z = [float(value) for value in quaternion]
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sine_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sine_pitch)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return [roll, pitch, yaw]


class HeadlessHarness:
    def __init__(
        self,
        initial_condition,
        viewer_enabled=False,
        viewer_speed=1.0,
        record_adaptive_plot=False,
    ):
        self.initial_condition = initial_condition
        self._scene_workspace = tempfile.TemporaryDirectory(
            prefix="go2w-closed-loop-mujoco-"
        )
        scene_dir = Path(self._scene_workspace.name)
        shutil.copy2(FLAT_SCENE, scene_dir / "scene_flat.xml")
        (scene_dir / "go2w.xml").symlink_to(MODEL_XML)
        (scene_dir / "assets").symlink_to(MODEL_ASSETS, target_is_directory=True)
        self.model = mujoco.MjModel.from_xml_path(str(scene_dir / "scene_flat.xml"))
        self.data = mujoco.MjData(self.model)
        self.dt = float(self.model.opt.timestep)
        if abs(self.dt - 0.002) > 1.0e-9:
            raise SimulationFailure("MuJoCo timestep is not 0.002 s")
        self.base_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link"
        )
        self.viewer = None
        self.viewer_speed = float(viewer_speed)
        self._viewer_wall_start = None
        self._viewer_sim_start = None
        if viewer_enabled:
            self._start_viewer()
        self.q_ref = None
        self.phase = "initialization"
        self.max_tracking_ratio = 0.0
        self.max_tau_ratio = 0.0
        self.max_abs_tracking_error_rad = 0.0
        self.min_ground_truth_wheel_contacts = 4
        self.min_wbc_ground_truth_wheel_contacts = 4
        self.max_controller_tilt_rad = 0.0
        self.max_ground_truth_tilt_rad = 0.0
        self.max_contact_balance_ratio = 0.0
        self.max_contact_velocity_residual_m_s = 0.0
        self.qp_solve_times = []
        self.hold_endpoints = []
        self.phase_records = []
        self.cycles_completed = 0
        self.controlled_return_attempted = False
        self.controlled_return_succeeded = False
        self.controlled_return_error = None
        self._adaptive_plot_time_origin_s = None
        self._adaptive_plot_recorder = adaptive_plot.AdaptivePlotRecorder(
            record_adaptive_plot,
            control.JOINT_NAMES,
            control.TRACKING_ENVELOPE_RAD,
        )

    def close(self):
        viewer = self.viewer
        self.viewer = None
        if viewer is not None:
            try:
                viewer.close()
            except RuntimeError:
                pass
        self._scene_workspace.cleanup()

    def _start_viewer(self):
        if mujoco_viewer is None:
            raise RuntimeError("MuJoCo viewer runtime was not loaded")
        try:
            mujoco.mj_forward(self.model, self.data)
            self.viewer = mujoco_viewer.launch_passive(
                self.model,
                self.data,
                show_left_ui=True,
                show_right_ui=True,
            )
            self.viewer.cam.lookat[:] = self.data.xpos[self.base_body_id]
            self.viewer.cam.distance = 1.5
            self.viewer.cam.azimuth = 135.0
            self.viewer.cam.elevation = -20.0
            self._viewer_wall_start = time.monotonic()
            self._viewer_sim_start = float(self.data.time)
            self._sync_viewer(pace=False)
        except Exception:
            if self.viewer is not None:
                try:
                    self.viewer.close()
                except RuntimeError:
                    pass
                self.viewer = None
            self._scene_workspace.cleanup()
            raise

    def _sync_viewer(self, pace=True):
        if self.viewer is None:
            return
        if not self.viewer.is_running():
            raise ViewerClosed("MuJoCo viewer window was closed")
        self.viewer.sync()
        if not pace:
            return
        simulated_elapsed = float(self.data.time) - self._viewer_sim_start
        deadline = self._viewer_wall_start + simulated_elapsed / self.viewer_speed
        remaining = deadline - time.monotonic()
        if remaining > 0.0:
            time.sleep(remaining)

    def hold_viewer_until_closed(self):
        if self.viewer is None or not self.viewer.is_running():
            return
        print(
            "[viewer] case finished; close the MuJoCo window to write the summary",
            flush=True,
        )
        try:
            while self.viewer.is_running():
                self.viewer.sync()
                time.sleep(1.0 / 60.0)
        except KeyboardInterrupt:
            print("[viewer] interrupted; closing the MuJoCo window", flush=True)

    def set_phase(self, phase):
        self.phase = phase
        if self.viewer is not None:
            print("[viewer] phase: {}".format(phase), flush=True)

    def reset_evaluation_metrics(self):
        """Start qualification metrics after controller-independent setup."""

        self.q_ref = None
        self.set_phase("captured-prone")
        self.max_tracking_ratio = 0.0
        self.max_tau_ratio = 0.0
        self.max_abs_tracking_error_rad = 0.0
        self.min_ground_truth_wheel_contacts = 4
        self.min_wbc_ground_truth_wheel_contacts = 4
        self.max_controller_tilt_rad = 0.0
        self.max_ground_truth_tilt_rad = 0.0
        self.max_contact_balance_ratio = 0.0
        self.max_contact_velocity_residual_m_s = 0.0
        self.qp_solve_times = []
        self.hold_endpoints = []
        self.phase_records = []
        self.cycles_completed = 0
        self.controlled_return_attempted = False
        self.controlled_return_succeeded = False
        self.controlled_return_error = None
        self._adaptive_plot_time_origin_s = float(self.data.time)

    def _record_adaptive_plot_sample(
        self,
        phase,
        phase_elapsed_s,
        phase_duration_s,
        measured_q,
        decision,
    ):
        if not self._adaptive_plot_recorder.enabled:
            return
        if self._adaptive_plot_time_origin_s is None:
            raise RuntimeError("adaptive plot time origin was not initialized")
        self._adaptive_plot_recorder.record(
            time_s=float(self.data.time) - self._adaptive_plot_time_origin_s,
            phase=phase,
            phase_elapsed_s=phase_elapsed_s,
            phase_duration_s=phase_duration_s,
            progress=decision.progress,
            speed_scale=decision.speed_scale,
            tracking_ratio=decision.tracking_ratio,
            torque_ratio=decision.torque_ratio,
            q_ref=decision.q_ref,
            q_measured=measured_q,
            event=(
                decision.reason
                if decision.emergency or decision.request_return
                else None
            ),
        )

    def write_adaptive_plots(self, output_dir, stem):
        return self._adaptive_plot_recorder.write(Path(output_dir), stem)

    def state(self):
        sensor = self.data.sensordata
        q = np.asarray(sensor[:12], dtype=float).copy()
        dq = np.asarray(sensor[16:28], dtype=float).copy()
        tau = np.asarray(sensor[32:44], dtype=float).copy()
        # Controller attitude comes from the same IMU sensor channel available
        # on hardware.  xquat remains evaluation-only in ground_truth().
        rpy = quaternion_to_rpy(sensor[48:52])
        return q, dq, tau, rpy

    def imu_gyro(self):
        return np.asarray(self.data.sensordata[52:55], dtype=float).copy()

    def ground_truth(self):
        rpy = quaternion_to_rpy(self.data.xquat[self.base_body_id])
        wheel_bodies = set()
        for contact_index in range(int(self.data.ncon)):
            contact = self.data.contact[contact_index]
            for geom_id in (int(contact.geom1), int(contact.geom2)):
                body_id = int(self.model.geom_bodyid[geom_id])
                body_name = mujoco.mj_id2name(
                    self.model, mujoco.mjtObj.mjOBJ_BODY, body_id
                )
                if body_name and body_name.endswith("_wheel_link"):
                    wheel_bodies.add(body_name)
        return {
            "base_position_m": [float(value) for value in self.data.xpos[self.base_body_id]],
            "base_rpy_rad": rpy,
            "wheel_contact_count": len(wheel_bodies),
            "contact_count": int(self.data.ncon),
            "actuator_force_nm": [float(value) for value in self.data.actuator_force],
        }

    def _update_metrics(self, q_ref):
        q, _dq, tau, rpy = self.state()
        error = np.abs(np.asarray(q_ref) - q)
        tracking_ratio = float(np.max(error / control.TRACKING_ENVELOPE_RAD))
        torque_ratio = float(np.max(np.abs(tau) / control.TORQUE_LIMIT_NM))
        if torque_ratio >= control.TORQUE_ERROR_RATIO:
            raise SimulationFailure(
                "{}: tau_est reached 100% of the model torque range".format(
                    self.phase
                )
            )
        truth = self.ground_truth()
        self.max_tracking_ratio = max(self.max_tracking_ratio, tracking_ratio)
        self.max_tau_ratio = max(self.max_tau_ratio, torque_ratio)
        self.max_abs_tracking_error_rad = max(
            self.max_abs_tracking_error_rad, float(np.max(error))
        )
        self.min_ground_truth_wheel_contacts = min(
            self.min_ground_truth_wheel_contacts, truth["wheel_contact_count"]
        )
        if self.phase.startswith("WBC "):
            self.min_wbc_ground_truth_wheel_contacts = min(
                self.min_wbc_ground_truth_wheel_contacts,
                truth["wheel_contact_count"],
            )
        self.max_controller_tilt_rad = max(
            self.max_controller_tilt_rad, abs(rpy[0]), abs(rpy[1])
        )
        truth_rpy = truth["base_rpy_rad"]
        self.max_ground_truth_tilt_rad = max(
            self.max_ground_truth_tilt_rad,
            abs(truth_rpy[0]),
            abs(truth_rpy[1]),
        )

    def check_runtime_state(self, q_ref, phase):
        q, dq, tau, rpy = self.state()
        values = np.concatenate([q, dq, tau, np.asarray(rpy, dtype=float)])
        if not np.all(np.isfinite(values)):
            raise SimulationFailure("{}: non-finite controller state".format(phase))
        tracking_error = float(np.max(np.abs(np.asarray(q_ref) - q)))
        if tracking_error > hardware.RUN_MAX_TRACKING_ERROR_RAD:
            raise SimulationFailure(
                "{}: 0.55 rad tracking watchdog triggered ({:.6f} rad)".format(
                    phase, tracking_error
                )
            )
        tilt = max(abs(float(rpy[0])), abs(float(rpy[1])))
        if tilt > hardware.RUN_MAX_TILT_RAD:
            raise SimulationFailure(
                "{}: body tilt watchdog triggered ({:.6f} rad)".format(
                    phase, tilt
                )
            )
        return q, dq, tau, rpy

    def step(self, q_ref=None):
        if q_ref is None:
            self.data.ctrl[:] = 0.0
        else:
            q_ref_array = np.asarray(q_ref, dtype=float)
            q, dq, _tau, _rpy = self.state()
            leg_control = (
                np.asarray(hardware.KP) * (q_ref_array - q)
                - np.asarray(hardware.KD) * dq
            )
            control_values = np.zeros(self.model.nu, dtype=float)
            control_values[:12] = leg_control
            wheel_dq = np.asarray(self.data.sensordata[28:32], dtype=float)
            control_values[12:16] = -2.0 * wheel_dq
            control_values = np.minimum(
                np.maximum(control_values, self.model.actuator_ctrlrange[:, 0]),
                self.model.actuator_ctrlrange[:, 1],
            )
            self.data.ctrl[:] = control_values
            self.q_ref = q_ref_array.copy()
        mujoco.mj_step(self.model, self.data)
        self._sync_viewer()
        if q_ref is not None:
            self._update_metrics(q_ref)

    def run_for(self, duration_s, q_ref=None):
        steps = int(math.ceil(duration_s / self.dt))
        for _ in range(steps):
            self.step(q_ref)

    def interpolate_to(self, target, duration_s):
        source = self.state()[0]
        steps = int(math.ceil(duration_s / self.dt))
        for step_index in range(steps + 1):
            alpha = min(1.0, step_index * self.dt / duration_s)
            self.step(control.smooth_path(source, target, alpha))

    def prepare(self):
        # The audited MJCF naturally lands into a folded prone pose before any
        # command is issued.  The other two cases add a controller-independent
        # preparation phase and then capture the resulting measured state.
        self.set_phase("preparation-settle")
        self.run_for(1.2, None)
        if self.initial_condition == "asymmetric-prone":
            self.set_phase("preparation-asymmetric-prone")
            self.interpolate_to(ASYMMETRIC_PRONE, 1.5)
            self.run_for(0.8, ASYMMETRIC_PRONE)
        elif self.initial_condition == "belly-loaded-prone":
            self.set_phase("preparation-belly-loaded-prone")
            self.interpolate_to(BELLY_LOADED_PRONE, 1.5)
            self.run_for(0.8, BELLY_LOADED_PRONE)
        elif self.initial_condition != "normal":
            raise ValueError("unknown initial condition: {}".format(self.initial_condition))
        captured = self.state()[0].copy()
        self.reset_evaluation_metrics()
        return captured

    def adaptive_phase(
        self,
        name,
        source,
        target,
        duration_s,
        timeout_s,
        return_mode=False,
        *,
        loaded_roll_baseline_rad=None,
        loaded_roll_expected_sign=None,
    ):
        self.set_phase("adaptive {}".format(name))
        governor = control.ReferenceGovernor(source, target, duration_s, timeout_s)
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
            loaded_roll_gate = control.LoadedRollEquilibriumGate(
                loaded_roll_baseline_rad,
                loaded_roll_expected_sign,
                hardware.KP,
                hardware.KD,
            )
        elapsed = 0.0
        while True:
            q, dq, tau, rpy = self.check_runtime_state(
                governor.current_ref, self.phase
            )
            decision = governor.step(
                q,
                dq,
                tau,
                self.dt,
                elapsed,
                return_mode=return_mode,
            )
            phase_completed = decision.completed
            loaded_status = None
            if loaded_roll_gate is not None:
                loaded_status = loaded_roll_gate.update(
                    decision.q_ref,
                    q,
                    dq,
                    tau,
                    rpy[0],
                    self.dt,
                    endpoint_reached=decision.progress >= 1.0,
                )
                phase_completed = loaded_status.completed
            self._record_adaptive_plot_sample(
                name,
                elapsed,
                duration_s,
                q,
                decision,
            )
            if decision.emergency or decision.request_return:
                raise SimulationFailure(
                    "{}: {}".format(name, decision.reason or "governor stopped")
                )
            self.step(decision.q_ref)
            elapsed += self.dt
            if phase_completed:
                record = {
                    "phase": name,
                    "elapsed_s": elapsed,
                    "progress": decision.progress,
                    "tracking_ratio": decision.tracking_ratio,
                    "tau_ratio": decision.torque_ratio,
                    "completion_gate": (
                        "loaded-roll-static-pd"
                        if loaded_status is not None
                        else "default-50-percent"
                    ),
                }
                if loaded_status is not None:
                    record.update(
                        {
                            "loaded_roll_gate_dwell_s": (
                                loaded_status.accumulated_s
                            ),
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
                        }
                    )
                self.phase_records.append(record)
                return decision.q_ref.copy()

    def adaptive_sequence(self, gesture, captured_prone):
        current = self.adaptive_phase(
            "startup-standard",
            captured_prone,
            hardware.STANDARD,
            hardware.STANDARD_TRANSITION_S,
            control.STARTUP_TIMEOUT_S,
        )
        current = self.adaptive_phase(
            "hold-standard",
            current,
            current,
            hardware.STANDARD_HOLD_S,
            hardware.STANDARD_HOLD_S + control.HOLD_CONVERGENCE_TIMEOUT_S,
        )
        standard_roll_rad = float(self.state()[3][0])
        if gesture == "height":
            sides = (("low", hardware.LOW, None), ("high", hardware.HIGH, None))
        else:
            sides = (
                (
                    "right",
                    hardware.ROLL_RIGHT,
                    control.ADAPTIVE_ROLL_RIGHT_IMU_SIGN,
                ),
                (
                    "left",
                    hardware.ROLL_LEFT,
                    control.ADAPTIVE_ROLL_LEFT_IMU_SIGN,
                ),
            )
        for cycle in range(1, 4):
            for side, target, expected_roll_sign in sides:
                loaded_roll_options = {}
                if expected_roll_sign is not None:
                    loaded_roll_options = {
                        "loaded_roll_baseline_rad": standard_roll_rad,
                        "loaded_roll_expected_sign": expected_roll_sign,
                    }
                current = self.adaptive_phase(
                    "transition-{}-{}".format(cycle, side),
                    current,
                    target,
                    1.0,
                    control.FAST_TRANSITION_TIMEOUT_S,
                    **loaded_roll_options,
                )
                current = self.adaptive_phase(
                    "hold-{}-{}".format(cycle, side),
                    current,
                    target,
                    0.5,
                    0.5 + control.HOLD_CONVERGENCE_TIMEOUT_S,
                    **loaded_roll_options,
                )
            self.cycles_completed = cycle
        current = self.adaptive_phase(
            "return-standard",
            current,
            hardware.STANDARD,
            hardware.STANDARD_TRANSITION_S,
            control.STARTUP_TIMEOUT_S,
        )
        current = self.adaptive_phase(
            "hold-return-standard",
            current,
            current,
            hardware.STANDARD_HOLD_S,
            hardware.STANDARD_HOLD_S + control.HOLD_CONVERGENCE_TIMEOUT_S,
        )
        current = self.adaptive_phase(
            "return-captured-prone",
            current,
            captured_prone,
            hardware.PRONE_TRANSITION_S,
            control.PRONE_RETURN_TIMEOUT_S,
            return_mode=True,
        )
        self.adaptive_phase(
            "hold-captured-prone",
            current,
            current,
            hardware.PRONE_HOLD_S,
            hardware.PRONE_HOLD_S + control.HOLD_CONVERGENCE_TIMEOUT_S,
            return_mode=True,
        )
        self.controlled_return_attempted = True
        self.controlled_return_succeeded = True

    def valid_contact_gate(self, q_ref):
        self.set_phase("WBC contact validation")
        gate = control.ContactValidityGate(0.5)
        hold_governor = control.ReferenceGovernor(
            q_ref,
            q_ref,
            duration_s=1.0,
            timeout_s=6.0,
        )
        elapsed = 0.0
        last_contact = None
        while elapsed <= 5.0:
            q, dq, tau, rpy = self.check_runtime_state(q_ref, self.phase)
            if int(round(elapsed / self.dt)) % 5 == 0:
                decision = hold_governor.step(
                    q,
                    dq,
                    tau,
                    control.WBC_PERIOD_S,
                    elapsed,
                )
                if decision.emergency or decision.request_return:
                    raise SimulationFailure(
                        "contact-gate governor: {}".format(
                            decision.reason or "governor stopped"
                        )
                    )
                last_contact = control.estimate_contact_forces(q, tau, rpy)
                self.max_contact_balance_ratio = max(
                    self.max_contact_balance_ratio,
                    last_contact.balance_residual_ratio,
                )
                if gate.update(last_contact.valid, control.WBC_PERIOD_S):
                    raw = control.estimate_task_space(
                        q, rpy, last_contact.forces, baseline_height_m=0.0
                    )
                    return last_contact, raw.raw_height_m, list(rpy)
            self.step(q_ref)
            elapsed += self.dt
        reason = last_contact.reason if last_contact is not None else "no estimate"
        raise SimulationFailure("contact validity gate failed: {}".format(reason))

    @staticmethod
    def task_in_tolerance(estimate, target):
        return (
            abs(estimate.relative_height_m - target.relative_height_m) <= 0.015
            and abs(estimate.roll_rad - target.roll_rad) <= math.radians(2.0)
            and abs(estimate.pitch_rad - target.pitch_rad) <= math.radians(2.0)
        )

    def wbc_phase(
        self,
        name,
        q_ref,
        posture_source,
        posture_target,
        task_source,
        task_target,
        duration_s,
        timeout_s,
        baseline_height,
        solver,
    ):
        self.set_phase("WBC {}".format(name))
        governor = control.TaskProgressGovernor(duration_s, timeout_s)
        q_ref = np.asarray(q_ref, dtype=float)
        elapsed = 0.0
        tick = 0
        invalid_contact_s = 0.0
        last_estimate = None
        last_qp = None
        while True:
            q, dq, tau, rpy = self.check_runtime_state(q_ref, self.phase)
            if tick % 5 == 0:
                contact = control.estimate_contact_forces(q, tau, rpy)
                self.max_contact_balance_ratio = max(
                    self.max_contact_balance_ratio,
                    contact.balance_residual_ratio,
                )
                invalid_contact_s = (
                    0.0 if contact.valid else invalid_contact_s + control.WBC_PERIOD_S
                )
                if invalid_contact_s >= 0.10:
                    raise SimulationFailure(
                        "{} contact invalid: {}".format(name, contact.reason)
                    )
                if contact.valid:
                    estimate = control.estimate_task_space(
                        q, rpy, contact.forces, baseline_height_m=baseline_height
                    )
                    decision = governor.step(
                        q_ref,
                        q,
                        dq,
                        tau,
                        control.WBC_PERIOD_S,
                        elapsed,
                        task_within_tolerance=self.task_in_tolerance(
                            estimate, task_target
                        ),
                    )
                    if decision.emergency or decision.request_return:
                        raise SimulationFailure(
                            "{} governor: {}".format(name, decision.reason)
                        )
                    desired_task = control.interpolate_task_target(
                        task_source, task_target, decision.progress
                    )
                    desired_posture = control.smooth_path(
                        posture_source, posture_target, decision.progress
                    )
                    qp = solver.solve(
                        q,
                        q_ref,
                        desired_posture,
                        estimate,
                        desired_task,
                        self.imu_gyro(),
                    )
                    if not qp.valid:
                        raise SimulationFailure(
                            "{} QP: {}".format(name, qp.reason)
                        )
                    q_ref = qp.q_ref.copy()
                    self.qp_solve_times.append(qp.solve_time_s)
                    self.max_contact_velocity_residual_m_s = max(
                        self.max_contact_velocity_residual_m_s,
                        qp.contact_velocity_residual_m_s,
                    )
                    last_estimate = estimate
                    last_qp = qp
                    if decision.completed:
                        self.phase_records.append(
                            {
                                "phase": name,
                                "elapsed_s": elapsed,
                                "progress": decision.progress,
                                "height_error_m": (
                                    estimate.relative_height_m
                                    - task_target.relative_height_m
                                ),
                                "roll_error_rad": estimate.roll_rad - task_target.roll_rad,
                                "pitch_error_rad": (
                                    estimate.pitch_rad - task_target.pitch_rad
                                ),
                                "contact_balance_residual_ratio": (
                                    contact.balance_residual_ratio
                                ),
                                "contact_velocity_residual_m_s": (
                                    qp.contact_velocity_residual_m_s
                                ),
                                "qp_solve_time_s": qp.solve_time_s,
                            }
                        )
                        if "hold" in name:
                            self.hold_endpoints.append(dict(self.phase_records[-1]))
                        return q_ref
            self.step(q_ref)
            tick += 1
            elapsed += self.dt

    def wbc_sequence(self, gesture, captured_prone):
        current = self.adaptive_phase(
            "startup-standard",
            captured_prone,
            hardware.STANDARD,
            hardware.STANDARD_TRANSITION_S,
            control.STARTUP_TIMEOUT_S,
        )
        current = self.adaptive_phase(
            "hold-standard",
            current,
            current,
            hardware.STANDARD_HOLD_S,
            hardware.STANDARD_HOLD_S + control.HOLD_CONVERGENCE_TIMEOUT_S,
        )
        _contact, baseline_height, baseline_rpy = self.valid_contact_gate(current)
        standard_task = control.task_target_for_gesture(
            gesture, "standard", baseline_rpy
        )
        posture_source = np.asarray(hardware.STANDARD, dtype=float)
        task_source = standard_task
        if gesture == "height":
            sides = (("low", hardware.LOW), ("high", hardware.HIGH))
        else:
            sides = (("right", hardware.ROLL_RIGHT), ("left", hardware.ROLL_LEFT))
        solver = control.KinematicWBC()
        for cycle in range(1, 4):
            for side, posture_target in sides:
                task_target = control.task_target_for_gesture(
                    gesture, side, baseline_rpy
                )
                current = self.wbc_phase(
                    "transition-{}-{}".format(cycle, side),
                    current,
                    posture_source,
                    posture_target,
                    task_source,
                    task_target,
                    1.0,
                    control.FAST_TRANSITION_TIMEOUT_S,
                    baseline_height,
                    solver,
                )
                current = self.wbc_phase(
                    "hold-{}-{}".format(cycle, side),
                    current,
                    posture_target,
                    posture_target,
                    task_target,
                    task_target,
                    0.5,
                    0.5 + control.HOLD_CONVERGENCE_TIMEOUT_S,
                    baseline_height,
                    solver,
                )
                posture_source = np.asarray(posture_target, dtype=float)
                task_source = task_target
            self.cycles_completed = cycle
        current = self.adaptive_phase(
            "adaptive-standard-after-wbc",
            self.state()[0],
            hardware.STANDARD,
            hardware.STANDARD_TRANSITION_S,
            control.STARTUP_TIMEOUT_S,
        )
        current = self.adaptive_phase(
            "hold-return-standard",
            current,
            current,
            hardware.STANDARD_HOLD_S,
            hardware.STANDARD_HOLD_S + control.HOLD_CONVERGENCE_TIMEOUT_S,
        )
        current = self.adaptive_phase(
            "return-captured-prone",
            current,
            captured_prone,
            hardware.PRONE_TRANSITION_S,
            control.PRONE_RETURN_TIMEOUT_S,
            return_mode=True,
        )
        self.adaptive_phase(
            "hold-captured-prone",
            current,
            current,
            hardware.PRONE_HOLD_S,
            hardware.PRONE_HOLD_S + control.HOLD_CONVERGENCE_TIMEOUT_S,
            return_mode=True,
        )
        self.controlled_return_attempted = True
        self.controlled_return_succeeded = True

    def summary(self, controller_name, gesture, captured_prone, error=None):
        final_q, final_dq, _tau, _rpy = self.state()
        truth = self.ground_truth()
        qp_p99 = (
            float(np.percentile(self.qp_solve_times, 99)) if self.qp_solve_times else None
        )
        return_error = float(np.max(np.abs(final_q - captured_prone)))
        hold_ok = all(
            abs(item["height_error_m"]) <= 0.015
            and abs(item["roll_error_rad"]) <= math.radians(2.0)
            and abs(item["pitch_error_rad"]) <= math.radians(2.0)
            for item in self.hold_endpoints
        )
        base_pass = (
            error is None
            and self.cycles_completed == 3
            and self.controlled_return_succeeded
            and return_error <= 0.10
            and self.max_tracking_ratio <= 1.0
            and self.max_abs_tracking_error_rad < hardware.RUN_MAX_TRACKING_ERROR_RAD
            and self.max_tau_ratio < control.TORQUE_ERROR_RATIO
            and self.max_controller_tilt_rad <= hardware.RUN_MAX_TILT_RAD
        )
        if controller_name == "wbc":
            qualified = (
                base_pass
                and len(self.hold_endpoints) == 6
                and hold_ok
                and self.max_contact_balance_ratio <= 0.15
                and self.max_contact_velocity_residual_m_s <= 0.01
                and self.min_wbc_ground_truth_wheel_contacts == 4
                and qp_p99 is not None
                and qp_p99 < 0.005
            )
        else:
            qualified = base_pass
        return {
            "controller": controller_name,
            "gesture": gesture,
            "initial_condition": self.initial_condition,
            "simulation_pass": qualified,
            "physical_pass": False,
            "error": error,
            "cycles_completed": self.cycles_completed,
            "captured_prone_q_rad": [float(value) for value in captured_prone],
            "final_q_rad": [float(value) for value in final_q],
            "final_max_abs_dq_rad_s": float(np.max(np.abs(final_dq))),
            "return_max_abs_error_rad": return_error,
            "max_tracking_envelope_ratio": self.max_tracking_ratio,
            "max_abs_tracking_error_rad": self.max_abs_tracking_error_rad,
            "max_tau_est_model_ratio": self.max_tau_ratio,
            "max_controller_imu_tilt_rad": self.max_controller_tilt_rad,
            "max_ground_truth_tilt_rad": self.max_ground_truth_tilt_rad,
            "minimum_ground_truth_wheel_contacts": self.min_ground_truth_wheel_contacts,
            "minimum_wbc_ground_truth_wheel_contacts": (
                self.min_wbc_ground_truth_wheel_contacts
            ),
            "max_contact_balance_residual_ratio": self.max_contact_balance_ratio,
            "max_contact_velocity_residual_m_s": (
                self.max_contact_velocity_residual_m_s
            ),
            "qp_solve_count": len(self.qp_solve_times),
            "qp_solve_p99_s": qp_p99,
            "hold_endpoints": self.hold_endpoints,
            "phase_records": self.phase_records,
            "final_ground_truth": truth,
            "ground_truth_used_for_control": False,
            "qualification_scope": "simulation-only",
            "controlled_return_attempted": self.controlled_return_attempted,
            "controlled_return_succeeded": self.controlled_return_succeeded,
            "controlled_return_error": self.controlled_return_error,
        }


def run_case(
    controller_name,
    gesture,
    initial_condition,
    viewer_enabled=False,
    viewer_speed=1.0,
    viewer_hold=False,
    save_plot=False,
    plot_output_dir=None,
    plot_stem=None,
):
    harness = HeadlessHarness(
        initial_condition,
        viewer_enabled=viewer_enabled,
        viewer_speed=viewer_speed,
        record_adaptive_plot=save_plot,
    )
    captured = None
    error = None
    plot_artifacts = {
        "requested": bool(save_plot),
        "sample_count": 0,
        "joint_tracking_svg": None,
        "adaptive_governor_svg": None,
        "generation_error": None,
    }
    try:
        captured = harness.prepare()
        if controller_name == "adaptive":
            harness.adaptive_sequence(gesture, captured)
        else:
            harness.wbc_sequence(gesture, captured)
    except Exception as caught:
        error = "{}: {}".format(type(caught).__name__, caught)
        if viewer_enabled:
            print("[viewer] case stopped: {}".format(error), file=sys.stderr, flush=True)
        if captured is not None and not isinstance(caught, ViewerClosed):
            harness.controlled_return_attempted = True
            try:
                current = harness.state()[0].copy()
                current = harness.adaptive_phase(
                    "failure-return-captured-prone",
                    current,
                    captured,
                    hardware.PRONE_TRANSITION_S,
                    control.PRONE_RETURN_TIMEOUT_S,
                    return_mode=True,
                )
                harness.adaptive_phase(
                    "failure-hold-captured-prone",
                    current,
                    current,
                    hardware.PRONE_HOLD_S,
                    hardware.PRONE_HOLD_S + control.HOLD_CONVERGENCE_TIMEOUT_S,
                    return_mode=True,
                )
                harness.controlled_return_succeeded = True
            except Exception as return_error:
                harness.controlled_return_error = "{}: {}".format(
                    type(return_error).__name__, return_error
                )
    finally:
        if captured is None:
            captured = harness.state()[0].copy()
        if viewer_hold:
            harness.hold_viewer_until_closed()
        if save_plot:
            try:
                if plot_output_dir is None or plot_stem is None:
                    raise RuntimeError("plot output directory and stem are required")
                plot_artifacts.update(
                    harness.write_adaptive_plots(plot_output_dir, plot_stem)
                )
            except Exception as plot_error:
                plot_artifacts["generation_error"] = "{}: {}".format(
                    type(plot_error).__name__, plot_error
                )
        summary = harness.summary(controller_name, gesture, captured, error=error)
        summary["plot_artifacts"] = plot_artifacts
        harness.close()
    return summary


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="MuJoCo qualification and GUI inspection for Go2W closed-loop gestures"
    )
    parser.add_argument("--controller", choices=CONTROLLERS)
    parser.add_argument("--gesture", choices=GESTURES)
    parser.add_argument(
        "--initial",
        choices=("all",) + INITIAL_CONDITIONS,
        default="all",
    )
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR))
    parser.add_argument(
        "--viewer",
        action="store_true",
        help="open MuJoCo's passive GUI for one selected initial condition",
    )
    parser.add_argument(
        "--viewer-speed",
        type=float,
        default=1.0,
        help="simulated-time / wall-time factor used by --viewer (default: 1.0)",
    )
    parser.add_argument(
        "--viewer-hold",
        action="store_true",
        help="keep the final/failure state visible until the viewer window is closed",
    )
    parser.add_argument(
        "--save-plot",
        action="store_true",
        help="save adaptive joint-tracking and reference-governor SVGs after the run",
    )
    parser.add_argument("--describe", action="store_true")
    parser.add_argument("--doctor", action="store_true")
    return parser.parse_args(argv)


def argument_error(args):
    if args.viewer and args.initial == "all":
        return "--viewer requires one explicit --initial condition, not 'all'"
    if args.viewer_hold and not args.viewer:
        return "--viewer-hold requires --viewer"
    if not math.isfinite(args.viewer_speed) or args.viewer_speed <= 0.0:
        return "--viewer-speed must be a positive finite number"
    if args.save_plot and args.controller != "adaptive":
        return "--save-plot currently requires --controller adaptive"
    return None


def main(argv=None):
    args = parse_args(argv)
    if args.describe:
        describe(args.controller, args.gesture, args.initial)
        return 0
    invalid = argument_error(args)
    if invalid is not None:
        print("error: {}".format(invalid), file=sys.stderr)
        return 2
    reexec_with_simulator_python()
    load_runtime(enable_viewer=args.viewer)
    if args.doctor:
        return 0 if doctor() else 1
    if args.controller is None or args.gesture is None:
        print(
            "error: --controller and --gesture are required unless --describe/--doctor is used",
            file=sys.stderr,
        )
        return 2

    initial_conditions = (
        INITIAL_CONDITIONS if args.initial == "all" else (args.initial,)
    )
    created_at = datetime.now().astimezone()
    output_dir = Path(args.output_dir).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)
    timestamp = created_at.strftime("%Y%m%dT%H%M%S_%f%z")
    mode_suffix = "_viewer" if args.viewer else ""
    cases = []
    for initial_condition in initial_conditions:
        print(
            "running {} {} from {}".format(
                args.controller, args.gesture, initial_condition
            ),
            flush=True,
        )
        case = run_case(
            args.controller,
            args.gesture,
            initial_condition,
            viewer_enabled=args.viewer,
            viewer_speed=args.viewer_speed,
            viewer_hold=args.viewer_hold,
            save_plot=args.save_plot,
            plot_output_dir=output_dir,
            plot_stem="{}_{}_{}{}_{}".format(
                timestamp,
                args.controller,
                args.gesture,
                mode_suffix,
                initial_condition,
            ),
        )
        cases.append(case)
        print(
            "{}: {}".format(
                initial_condition,
                "SIMULATION PASS" if case["simulation_pass"] else "FAIL",
            ),
            flush=True,
        )
        if case["error"]:
            print(case["error"], file=sys.stderr, flush=True)
        artifacts = case["plot_artifacts"]
        if artifacts["requested"]:
            if artifacts["generation_error"] is not None:
                print(
                    "plot generation failed: {}".format(
                        artifacts["generation_error"]
                    ),
                    file=sys.stderr,
                    flush=True,
                )
            else:
                print(
                    "adaptive joint plot: {}".format(
                        artifacts["joint_tracking_svg"]
                    ),
                    flush=True,
                )
                print(
                    "adaptive governor plot: {}".format(
                        artifacts["adaptive_governor_svg"]
                    ),
                    flush=True,
                )

    output_path = output_dir / "{}_{}_{}{}_qualification.summary.json".format(
        timestamp, args.controller, args.gesture, mode_suffix
    )
    overall_pass = all(case["simulation_pass"] for case in cases)
    plot_generation_pass = not args.save_plot or all(
        case["plot_artifacts"]["generation_error"] is None for case in cases
    )
    document = {
        "schema_version": 1,
        "created_at": created_at.isoformat(),
        "controller": args.controller,
        "gesture": args.gesture,
        "execution_mode": "viewer-inspection" if args.viewer else "headless-qualification",
        "viewer_speed": args.viewer_speed if args.viewer else None,
        "plots_requested": args.save_plot,
        "plot_generation_pass": plot_generation_pass,
        "model_path": str(MODEL_XML),
        "model_sha256": control.MODEL_SOURCE_SHA256,
        "cases": cases,
        "simulation_pass": overall_pass,
        "jetson_software_pass": False,
        "physical_pass": False,
        "qualification_scope": "simulation-only",
    }
    with output_path.open("w", encoding="utf-8") as output:
        json.dump(document, output, indent=2, sort_keys=True)
        output.write("\n")
    print("summary: {}".format(output_path), flush=True)
    print(
        "SIMULATION {} — this does not qualify physical hardware".format(
            "PASS" if overall_pass else "FAIL"
        ),
        flush=True,
    )
    if args.save_plot:
        print(
            "PLOT GENERATION {}".format("PASS" if plot_generation_pass else "FAIL"),
            flush=True,
        )
    return 0 if overall_pass and plot_generation_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
