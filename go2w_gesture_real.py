#!/usr/bin/env python3
"""Run slow-profile Go2W low-level gestures through Unitree SDK2Py.

The default invocation is a read-only preflight. Physical motion requires the
explicit ``--live`` flag and a confirmation phrase entered on a TTY.

After a confirmed return to the captured prone pose, the live sequence sends a
neutral zero-gain command, closes its LowCmd writer, waits for the topic to
become quiet, and requests the Sport service that was active at startup. If the
script starts with Sport already released, there is no captured service name to
restore; the script may still run after the same quiet-topic ownership check.

Supported deployment targets are Python 3.8+ on the Jetson host or a host-
networked container with ``unitree_sdk2py`` installed. ROS 2 is not used by the
script; SDK2Py communicates directly over CycloneDDS.
"""

from __future__ import print_function

import argparse
from array import array
from collections import deque
import csv
from dataclasses import dataclass
from datetime import datetime
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import socket
import struct
import sys
import threading
import time
from typing import List, Optional


DDS_DOMAIN = 0
DEFAULT_INTERFACE = "eth0"
DEFAULT_EXPECTED_IP = "192.168.123.18"

CONTROL_PERIOD_S = 0.002  # 500 Hz
STANDARD_TRANSITION_S = 2.0
PRONE_TRANSITION_S = 3.0
STANDARD_HOLD_S = 2.0
PRONE_HOLD_S = 2.0
NEUTRAL_COMMAND_S = 1.0
HEIGHT_CYCLES = 3
ROLL_CYCLES = 3
ROLL_LIMIT_SCALE = 0.70

LOWSTATE_STARTUP_TIMEOUT_S = 10.0
LOWSTATE_TIMEOUT_S = 0.10
PRONE_STABILITY_WINDOW_S = 0.5
PRONE_STABILITY_TIMEOUT_S = 5.0
PRONE_MAX_JOINT_RANGE_RAD = 0.05
PRONE_MAX_LEG_SPEED_RAD_S = 0.25
PRONE_MAX_WHEEL_SPEED_RAD_S = 0.50
PRONE_MAX_TILT_RAD = 0.35
PRONE_MAX_CALF_ANGLE_RAD = -2.20
RUN_MAX_TILT_RAD = 0.55
# Provisional fail-closed heuristics. These values were not derived from
# measured hardware tracking distributions, actuator limits, or a qualified
# torque bound. The warning preserves visibility at the old stop threshold;
# the stop threshold allows a narrow additional diagnostic range.
RUN_TRACKING_WARNING_RAD = 0.45
RUN_MAX_TRACKING_ERROR_RAD = 0.55
TRACKING_WARNING_PRINT_INTERVAL_S = 0.5
DEFAULT_TRACKING_LOG_DIR = os.environ.get("GO2W_TRACKING_LOG_DIR", "runs")
LOWCMD_QUIET_S = 0.5
LOWCMD_QUIET_TIMEOUT_S = 3.0
MODE_RELEASE_TIMEOUT_S = 10.0
MODE_SELECT_TIMEOUT_S = 10.0

POS_STOP_F = 2.146e9
VEL_STOP_F = 16000.0
GESTURE_NAMES = ("height", "roll")
LIVE_CONFIRMATIONS = {
    "height": "RUN GO2W LOW LEVEL",
    "roll": "RUN GO2W ROLL LOW LEVEL",
}


@dataclass(frozen=True)
class GestureTimingProfile:
    name: str
    transition_s: float
    hold_s: float


SLOW_TIMING = GestureTimingProfile(
    name="slow",
    transition_s=2.0,
    hold_s=2.0,
)
FAST_TIMING = GestureTimingProfile(
    name="fast",
    transition_s=1.0,
    hold_s=0.5,
)

KP = [60.0, 80.0, 80.0] * 4
KD = [5.0, 4.0, 4.0] * 4
LEG_JOINT_NAMES = (
    "FR_hip",
    "FR_thigh",
    "FR_calf",
    "FL_hip",
    "FL_thigh",
    "FL_calf",
    "RR_hip",
    "RR_thigh",
    "RR_calf",
    "RL_hip",
    "RL_thigh",
    "RL_calf",
)


def symmetric_pose(hip, thigh, calf):
    # Joint order: FR, FL, RR, RL; each leg is hip, thigh, calf.
    return [
        hip,
        thigh,
        calf,
        -hip,
        thigh,
        calf,
        hip,
        thigh,
        calf,
        -hip,
        thigh,
        calf,
    ]


STANDARD = symmetric_pose(0.10, 0.90, -1.80)
LOW = symmetric_pose(0.10, 1.15, -2.30)
HIGH = symmetric_pose(0.10, 0.65, -1.30)
CALF_INDICES = (2, 5, 8, 11)
HIP_INDICES = (0, 3, 6, 9)

# Unitree Go2W URDF limits for FR/FL/RR/RL hip abduction.  These values were
# cross-checked against the MuJoCo go2w.xml model used to qualify this gesture.
HIP_LIMIT_LOWER_RAD = -1.0472
HIP_LIMIT_UPPER_RAD = 1.0472


def make_roll_targets():
    positive_margin = min(
        HIP_LIMIT_UPPER_RAD - STANDARD[index] for index in HIP_INDICES
    )
    negative_margin = min(
        STANDARD[index] - HIP_LIMIT_LOWER_RAD for index in HIP_INDICES
    )
    theoretical_limit = min(positive_margin, negative_margin)
    amplitude = theoretical_limit * ROLL_LIMIT_SCALE

    right = list(STANDARD)
    left = list(STANDARD)
    for index in HIP_INDICES:
        right[index] -= amplitude
        left[index] += amplitude
    return amplitude, right, left


ROLL_AMPLITUDE_RAD, ROLL_RIGHT, ROLL_LEFT = make_roll_targets()


# SDK symbols are loaded after argument parsing so --help and --describe work
# even on a development host that does not have unitree_sdk2py installed.
ChannelFactoryInitialize = None
ChannelPublisher = None
ChannelSubscriber = None
LowCmd_ = None
LowState_ = None
MotionSwitcherClient = None
SportClient = None
CRC = None
unitree_go_msg_dds__LowCmd_ = None


def load_unitree_sdk():
    global ChannelFactoryInitialize
    global ChannelPublisher
    global ChannelSubscriber
    global LowCmd_
    global LowState_
    global MotionSwitcherClient
    global SportClient
    global CRC
    global unitree_go_msg_dds__LowCmd_

    try:
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
            MotionSwitcherClient as _MotionSwitcherClient,
        )
        from unitree_sdk2py.core.channel import (
            ChannelFactoryInitialize as _ChannelFactoryInitialize,
            ChannelPublisher as _ChannelPublisher,
            ChannelSubscriber as _ChannelSubscriber,
        )
        from unitree_sdk2py.go2.sport.sport_client import SportClient as _SportClient
        from unitree_sdk2py.idl.default import (
            unitree_go_msg_dds__LowCmd_ as _new_lowcmd,
        )
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import (
            LowCmd_ as _LowCmd,
            LowState_ as _LowState,
        )
        from unitree_sdk2py.utils.crc import CRC as _CRC
    except ImportError as error:
        raise RuntimeError(
            "unitree_sdk2py is unavailable; install the pinned "
            "unitree_sdk2_python checkout and cyclonedds before deployment: {}".format(
                error
            )
        )

    ChannelFactoryInitialize = _ChannelFactoryInitialize
    ChannelPublisher = _ChannelPublisher
    ChannelSubscriber = _ChannelSubscriber
    LowCmd_ = _LowCmd
    LowState_ = _LowState
    MotionSwitcherClient = _MotionSwitcherClient
    SportClient = _SportClient
    CRC = _CRC
    unitree_go_msg_dds__LowCmd_ = _new_lowcmd


def interface_ipv4(interface):
    if not interface or len(interface.encode("utf-8")) > 15:
        raise RuntimeError("invalid network interface name: {!r}".format(interface))
    request = struct.pack("256s", interface.encode("utf-8"))
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        try:
            response = fcntl.ioctl(sock.fileno(), 0x8915, request)  # SIOCGIFADDR
        except OSError as error:
            raise RuntimeError(
                "cannot read IPv4 address for interface {}: {}".format(interface, error)
            )
    return socket.inet_ntoa(response[20:24])


def validate_pose_values(name, pose):
    if len(pose) != 12:
        raise RuntimeError("{} must contain exactly 12 leg joint angles".format(name))
    if not all(math.isfinite(float(value)) for value in pose):
        raise RuntimeError("{} contains a non-finite joint angle".format(name))


def interpolate(source, target, alpha):
    alpha = max(0.0, min(1.0, float(alpha)))
    blend = alpha * alpha * (3.0 - 2.0 * alpha)
    return [
        float(source[index])
        + (float(target[index]) - float(source[index])) * blend
        for index in range(12)
    ]


def print_common_shutdown_plan():
    print("  hold standard: {:.1f} s".format(STANDARD_HOLD_S))
    print("  standard -> captured prone: {:.1f} s".format(PRONE_TRANSITION_S))
    print("  hold prone: {:.1f} s".format(PRONE_HOLD_S))
    print(
        "  neutral zero-gain command: {:.1f} s, then close LowCmd".format(
            NEUTRAL_COMMAND_S
        )
    )
    print("  restore the captured Sport service when one was active at startup")


def print_height_plan(timing=SLOW_TIMING):
    print("Go2W real-robot gesture: height")
    print("  timing profile: {}".format(timing.name))
    print("  captured prone -> standard: {:.1f} s".format(STANDARD_TRANSITION_S))
    print("  hold standard: {:.1f} s".format(STANDARD_HOLD_S))
    print("  repeat {} times:".format(HEIGHT_CYCLES))
    print(
        "    -> low: {:.1f} s, hold {:.1f} s".format(
            timing.transition_s, timing.hold_s
        )
    )
    print(
        "    -> high: {:.1f} s, hold {:.1f} s".format(
            timing.transition_s, timing.hold_s
        )
    )
    print("  high -> standard: {:.1f} s".format(STANDARD_TRANSITION_S))
    print_common_shutdown_plan()


def print_roll_plan(timing=SLOW_TIMING):
    print("Go2W real-robot gesture: roll")
    print("  timing profile: {}".format(timing.name))
    print("  captured prone -> standard: {:.1f} s".format(STANDARD_TRANSITION_S))
    print("  hold standard: {:.1f} s".format(STANDARD_HOLD_S))
    print(
        "  common hip offset: +/-{:.5f} rad ({:.0%} of URDF-derived limit)".format(
            ROLL_AMPLITUDE_RAD, ROLL_LIMIT_SCALE
        )
    )
    print("  repeat {} times:".format(ROLL_CYCLES))
    print(
        "    -> right: {:g} s, hold {:.1f} s".format(
            timing.transition_s, timing.hold_s
        )
    )
    print(
        "    -> left: {:g} s, hold {:.1f} s".format(
            timing.transition_s, timing.hold_s
        )
    )
    print("  left -> standard: {:.1f} s".format(STANDARD_TRANSITION_S))
    print_common_shutdown_plan()


def print_sequence_plan(gesture=None, timing=SLOW_TIMING):
    if gesture is None:
        print_height_plan(timing)
        print("")
        print_roll_plan(timing)
    elif gesture == "height":
        print_height_plan(timing)
    elif gesture == "roll":
        print_roll_plan(timing)
    else:
        raise ValueError("unknown gesture: {!r}".format(gesture))


def print_tracking_policy(tracking_stop_rad):
    if tracking_stop_rad is None:
        print(
            "Joint tracking policy: warn and record above {:.2f} rad; "
            "tracking-error stop disabled".format(RUN_TRACKING_WARNING_RAD)
        )
    else:
        print(
            "Joint tracking policy: warn above {:.2f} rad; stop above {:.2f} rad".format(
                RUN_TRACKING_WARNING_RAD,
                tracking_stop_rad,
            )
        )


@dataclass
class StateSample:
    received_at: float
    pose: List[float]
    leg_velocity: List[float]
    wheel_velocity: List[float]
    rpy: List[float]


class TrackingRecorder:
    """Buffer live tracking samples without filesystem I/O in the 500 Hz loop."""

    _ROW_WIDTH = 44

    def __init__(
        self,
        log_dir,
        gesture,
        timing,
        tracking_stop_rad=RUN_MAX_TRACKING_ERROR_RAD,
    ):
        self.log_dir = Path(log_dir).expanduser()
        self.log_dir.mkdir(parents=True, exist_ok=True)
        if not self.log_dir.is_dir() or not os.access(str(self.log_dir), os.W_OK):
            raise RuntimeError(
                "tracking log directory is not writable: {}".format(self.log_dir)
            )

        self.gesture = gesture
        self.timing = timing
        self.tracking_stop_rad = tracking_stop_rad
        self.created_at = datetime.now().astimezone()
        self.started_at = None  # type: Optional[float]
        self._rows = array("d")
        self._phase_ids = array("H")
        self._phases = []
        self._phase_to_id = {}
        self._phase_sample_counts = {}
        self._per_joint_peak = [None] * 12
        self._global_peak = None
        self._warning_sample_count = 0
        self._stop_sample_count = 0
        self._max_lowstate_age_s = 0.0
        self._finalized_paths = None

    @property
    def sample_count(self):
        return len(self._phase_ids)

    def start(self):
        if self.started_at is None:
            self.started_at = time.monotonic()

    def record(self, sample, commanded_pose, motion_context, motion_elapsed_s):
        if self.started_at is None:
            self.start()
        phase = str(motion_context or "unspecified")
        phase_id = self._phase_to_id.get(phase)
        if phase_id is None:
            phase_id = len(self._phases)
            if phase_id > 65535:
                raise RuntimeError("too many tracking telemetry phases")
            self._phase_to_id[phase] = phase_id
            self._phases.append(phase)

        now = time.monotonic()
        run_elapsed_s = now - self.started_at
        lowstate_age_s = max(0.0, now - sample.received_at)
        phase_elapsed_s = (
            float("nan") if motion_elapsed_s is None else float(motion_elapsed_s)
        )
        measured = [float(value) for value in sample.pose]
        target = [float(value) for value in commanded_pose]
        errors = [target[index] - measured[index] for index in range(12)]
        max_index = max(range(12), key=lambda index: abs(errors[index]))
        max_abs_error = abs(errors[max_index])

        self._phase_ids.append(phase_id)
        self._rows.extend(
            [
                run_elapsed_s,
                phase_elapsed_s,
                lowstate_age_s,
                float(sample.rpy[0]),
                float(sample.rpy[1]),
                float(sample.rpy[2]),
            ]
            + measured
            + target
            + errors
            + [max_abs_error, float(max_index)]
        )
        self._phase_sample_counts[phase] = (
            self._phase_sample_counts.get(phase, 0) + 1
        )
        self._max_lowstate_age_s = max(self._max_lowstate_age_s, lowstate_age_s)
        if max_abs_error > RUN_TRACKING_WARNING_RAD:
            self._warning_sample_count += 1
        if (
            self.tracking_stop_rad is not None
            and max_abs_error > self.tracking_stop_rad
        ):
            self._stop_sample_count += 1

        for index in range(12):
            peak = self._per_joint_peak[index]
            if peak is None or abs(errors[index]) > peak["max_abs_error_rad"]:
                candidate = {
                    "index": index,
                    "name": LEG_JOINT_NAMES[index],
                    "max_abs_error_rad": abs(errors[index]),
                    "signed_error_rad": errors[index],
                    "measured_rad": measured[index],
                    "target_rad": target[index],
                    "run_elapsed_s": run_elapsed_s,
                    "phase": phase,
                    "phase_elapsed_s": (
                        None if math.isnan(phase_elapsed_s) else phase_elapsed_s
                    ),
                }
                self._per_joint_peak[index] = candidate
                if (
                    self._global_peak is None
                    or candidate["max_abs_error_rad"]
                    > self._global_peak["max_abs_error_rad"]
                ):
                    self._global_peak = dict(candidate)

        return errors, max_index

    @staticmethod
    def _csv_header():
        header = [
            "run_elapsed_s",
            "phase",
            "phase_elapsed_s",
            "lowstate_age_s",
            "roll_rad",
            "pitch_rad",
            "yaw_rad",
        ]
        for prefix in ("measured", "target", "error_target_minus_measured"):
            header.extend(
                "{}_{}_rad".format(prefix, name) for name in LEG_JOINT_NAMES
            )
        header.extend(
            ["max_abs_error_rad", "max_error_joint_index", "max_error_joint_name"]
        )
        return header

    def _unique_output_paths(self):
        timestamp = self.created_at.strftime("%Y%m%dT%H%M%S_%f%z")
        policy_name = (
            self.timing.name
            if self.tracking_stop_rad is not None
            else "{}_no-tracking-stop".format(self.timing.name)
        )
        stem = "{}_{}_{}_tracking".format(timestamp, self.gesture, policy_name)
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

        csv_path, summary_path = self._unique_output_paths()
        with csv_path.open("x", newline="", encoding="utf-8") as output:
            writer = csv.writer(output)
            writer.writerow(self._csv_header())
            for row_index, phase_id in enumerate(self._phase_ids):
                offset = row_index * self._ROW_WIDTH
                row = self._rows[offset : offset + self._ROW_WIDTH]
                phase_elapsed_s = row[1]
                max_index = int(row[43])
                writer.writerow(
                    [
                        "{:.9f}".format(row[0]),
                        self._phases[phase_id],
                        (
                            ""
                            if math.isnan(phase_elapsed_s)
                            else "{:.9f}".format(phase_elapsed_s)
                        ),
                    ]
                    + ["{:.9f}".format(value) for value in row[2:43]]
                    + [max_index, LEG_JOINT_NAMES[max_index]]
                )

        duration_s = 0.0
        if self.sample_count:
            duration_s = self._rows[(self.sample_count - 1) * self._ROW_WIDTH]
        summary = {
            "schema_version": 1,
            "created_at": self.created_at.isoformat(),
            "gesture": self.gesture,
            "timing_profile": self.timing.name,
            "transition_s": self.timing.transition_s,
            "hold_s": self.timing.hold_s,
            "control_period_s": CONTROL_PERIOD_S,
            "tracking_warning_rad": RUN_TRACKING_WARNING_RAD,
            "tracking_stop_enabled": self.tracking_stop_rad is not None,
            "tracking_stop_rad": self.tracking_stop_rad,
            "outcome": str(outcome),
            "error": None if error_text is None else str(error_text),
            "sample_count": self.sample_count,
            "duration_s": duration_s,
            "max_lowstate_age_s": self._max_lowstate_age_s,
            "warning_crossing_sample_count": self._warning_sample_count,
            "stop_crossing_sample_count": self._stop_sample_count,
            "samples_per_phase": self._phase_sample_counts,
            "global_peak": self._global_peak,
            "per_joint_peak": self._per_joint_peak,
            "csv_file": csv_path.name,
        }
        with summary_path.open("x", encoding="utf-8") as output:
            json.dump(summary, output, indent=2, sort_keys=True)
            output.write("\n")

        self._finalized_paths = (csv_path, summary_path)
        return self._finalized_paths


class InterruptedSequence(Exception):
    pass


class HardStop(Exception):
    pass


class HardwareGestureController:
    def __init__(
        self,
        interface,
        expected_ip,
        gesture,
        timing=SLOW_TIMING,
        tracking_log_dir=None,
        tracking_stop_rad=RUN_MAX_TRACKING_ERROR_RAD,
    ):
        if gesture not in GESTURE_NAMES:
            raise ValueError("unknown gesture: {!r}".format(gesture))
        if not isinstance(timing, GestureTimingProfile):
            raise ValueError("invalid timing profile: {!r}".format(timing))
        if tracking_stop_rad is not None and (
            not math.isfinite(tracking_stop_rad) or tracking_stop_rad <= 0.0
        ):
            raise ValueError(
                "tracking stop threshold must be a positive finite value or None"
            )
        self.interface = interface
        self.expected_ip = expected_ip
        self.gesture = gesture
        self.timing = timing
        self.tracking_log_dir = tracking_log_dir
        self.tracking_stop_rad = tracking_stop_rad
        self._lock = threading.Lock()
        self._samples = deque(maxlen=2000)
        self._last_lowcmd_time = None  # type: Optional[float]
        self._publishing = False
        self._stop_requested = threading.Event()
        self._hard_stop_requested = threading.Event()
        self._signal_count = 0

        self._publisher = None
        self._motion_switcher = None
        self._sport_client = None
        self._command = unitree_go_msg_dds__LowCmd_()
        self._crc = CRC()
        self._last_commanded_pose = None  # type: Optional[List[float]]
        self._mode_released = False
        self._mode_release_attempted = False
        self._restore_mode_name = None  # type: Optional[str]
        self._mode_restore_attempted = False
        self._mode_restored = False
        self._neutralized = False
        self._ended_prone = False
        self._captured_prone = None  # type: Optional[List[float]]
        self._tracking_recorder = None  # type: Optional[TrackingRecorder]
        self._last_tracking_warning_print = None  # type: Optional[float]
        self._run_outcome = "not-started"
        self._configure_neutral_command()

    def _prepare_tracking_recording(self):
        if self.tracking_log_dir is None:
            return
        recorder = TrackingRecorder(
            self.tracking_log_dir,
            self.gesture,
            self.timing,
            tracking_stop_rad=self.tracking_stop_rad,
        )
        self._tracking_recorder = recorder
        print(
            "tracking telemetry prepared: {} (buffered in memory during motion)".format(
                recorder.log_dir
            ),
            flush=True,
        )

    def finalize_tracking_log(self, outcome, error_text=None):
        if self._tracking_recorder is None:
            return None
        paths = self._tracking_recorder.finalize(outcome, error_text=error_text)
        print(
            "tracking telemetry saved: {} and {}".format(paths[0], paths[1]),
            flush=True,
        )
        return paths

    def request_stop(self, _signum=None, _frame=None):
        self._signal_count += 1
        if self._signal_count == 1:
            print(
                "\nstop requested; attempting a controlled return to the captured prone pose",
                file=sys.stderr,
                flush=True,
            )
            self._stop_requested.set()
        else:
            print(
                "\nsecond stop requested; abandoning controlled return and neutralizing",
                file=sys.stderr,
                flush=True,
            )
            self._hard_stop_requested.set()

    def on_low_state(self, message):
        now = time.monotonic()
        try:
            sample = StateSample(
                received_at=now,
                pose=[float(message.motor_state[index].q) for index in range(12)],
                leg_velocity=[
                    float(message.motor_state[index].dq) for index in range(12)
                ],
                wheel_velocity=[
                    float(message.motor_state[index].dq) for index in range(12, 16)
                ],
                rpy=[float(message.imu_state.rpy[index]) for index in range(3)],
            )
        except (AttributeError, IndexError, TypeError, ValueError):
            return
        with self._lock:
            self._samples.append(sample)

    def on_lowcmd(self, _message):
        # Before this process publishes, any observed traffic must become quiet
        # after ReleaseMode. Once publishing begins, DDS does not expose enough
        # source identity here to distinguish our samples from another writer.
        if not self._publishing:
            with self._lock:
                self._last_lowcmd_time = time.monotonic()

    def _configure_neutral_command(self):
        self._command.head[0] = 0xFE
        self._command.head[1] = 0xEF
        self._command.level_flag = 0xFF
        self._command.gpio = 0
        for motor in self._command.motor_cmd:
            motor.mode = 0x01
            motor.q = POS_STOP_F
            motor.kp = 0.0
            motor.dq = VEL_STOP_F
            motor.kd = 0.0
            motor.tau = 0.0

    def _set_pose_command(self, pose):
        validate_pose_values("commanded pose", pose)
        for index in range(12):
            motor = self._command.motor_cmd[index]
            motor.mode = 0x01
            motor.q = float(pose[index])
            motor.kp = KP[index]
            motor.dq = 0.0
            motor.kd = KD[index]
            motor.tau = 0.0

        # Go2W wheel motors are held at zero velocity, never by position.
        for index in range(12, 16):
            motor = self._command.motor_cmd[index]
            motor.mode = 0x01
            motor.q = POS_STOP_F
            motor.kp = 0.0
            motor.dq = 0.0
            motor.kd = 2.0
            motor.tau = 0.0

        for index in range(16, len(self._command.motor_cmd)):
            motor = self._command.motor_cmd[index]
            motor.mode = 0x01
            motor.q = POS_STOP_F
            motor.kp = 0.0
            motor.dq = VEL_STOP_F
            motor.kd = 0.0
            motor.tau = 0.0

    def _write_pose(self, pose):
        self._set_pose_command(pose)
        self._command.crc = self._crc.Crc(self._command)
        if self._publisher.Write(self._command) is False:
            raise RuntimeError("DDS write failed for rt/lowcmd pose command")
        self._last_commanded_pose = list(pose)

    def _write_neutral(self):
        self._configure_neutral_command()
        self._command.crc = self._crc.Crc(self._command)
        if self._publisher.Write(self._command) is False:
            raise RuntimeError("DDS write failed for rt/lowcmd neutral command")

    def _latest_sample(self):
        with self._lock:
            return self._samples[-1] if self._samples else None

    def _wait_for_first_state(self):
        deadline = time.monotonic() + LOWSTATE_STARTUP_TIMEOUT_S
        while time.monotonic() < deadline:
            sample = self._latest_sample()
            if sample is not None:
                return sample
            if self._stop_requested.is_set():
                raise InterruptedSequence()
            time.sleep(0.01)
        raise RuntimeError("timed out waiting for rt/lowstate")

    def _capture_stable_prone(self):
        deadline = time.monotonic() + PRONE_STABILITY_TIMEOUT_S
        last_reason = "not enough LowState history"
        while time.monotonic() < deadline:
            if self._stop_requested.is_set():
                raise InterruptedSequence()
            now = time.monotonic()
            with self._lock:
                samples = [
                    sample
                    for sample in self._samples
                    if now - sample.received_at <= PRONE_STABILITY_WINDOW_S
                ]
            if (
                len(samples) >= 2
                and samples[-1].received_at - samples[0].received_at
                >= PRONE_STABILITY_WINDOW_S * 0.9
            ):
                pose_ranges = [
                    max(sample.pose[index] for sample in samples)
                    - min(sample.pose[index] for sample in samples)
                    for index in range(12)
                ]
                max_pose_range = max(pose_ranges)
                max_leg_speed = max(
                    abs(value)
                    for sample in samples
                    for value in sample.leg_velocity
                )
                max_wheel_speed = max(
                    abs(value)
                    for sample in samples
                    for value in sample.wheel_velocity
                )
                latest = samples[-1]
                max_tilt = max(abs(latest.rpy[0]), abs(latest.rpy[1]))
                max_calf_angle = max(latest.pose[index] for index in CALF_INDICES)

                if not all(
                    math.isfinite(value)
                    for sample in samples
                    for value in (
                        sample.pose
                        + sample.leg_velocity
                        + sample.wheel_velocity
                        + sample.rpy
                    )
                ):
                    last_reason = "LowState contains non-finite values"
                elif max_pose_range > PRONE_MAX_JOINT_RANGE_RAD:
                    last_reason = "joint range {:.3f} rad exceeds {:.3f}".format(
                        max_pose_range, PRONE_MAX_JOINT_RANGE_RAD
                    )
                elif max_leg_speed > PRONE_MAX_LEG_SPEED_RAD_S:
                    last_reason = "leg speed {:.3f} rad/s exceeds {:.3f}".format(
                        max_leg_speed, PRONE_MAX_LEG_SPEED_RAD_S
                    )
                elif max_wheel_speed > PRONE_MAX_WHEEL_SPEED_RAD_S:
                    last_reason = "wheel speed {:.3f} rad/s exceeds {:.3f}".format(
                        max_wheel_speed, PRONE_MAX_WHEEL_SPEED_RAD_S
                    )
                elif max_tilt > PRONE_MAX_TILT_RAD:
                    last_reason = "body tilt {:.3f} rad exceeds {:.3f}".format(
                        max_tilt, PRONE_MAX_TILT_RAD
                    )
                elif max_calf_angle > PRONE_MAX_CALF_ANGLE_RAD:
                    last_reason = (
                        "calf angle {:.3f} rad does not look like the expected prone pose"
                    ).format(max_calf_angle)
                else:
                    averaged_pose = [
                        sum(sample.pose[index] for sample in samples) / len(samples)
                        for index in range(12)
                    ]
                    validate_pose_values("captured prone pose", averaged_pose)
                    return averaged_pose, latest.rpy
            time.sleep(0.02)
        raise RuntimeError("initial prone-state check failed: {}".format(last_reason))

    def _check_mode(self):
        code, result = self._motion_switcher.CheckMode()
        if code != 0 or result is None or not isinstance(result, dict):
            raise RuntimeError(
                "MotionSwitcher CheckMode failed: code={}, result={!r}".format(
                    code, result
                )
            )
        return str(result.get("form", "")), str(result.get("name", ""))

    def _release_mode(self, expected_name):
        deadline = time.monotonic() + MODE_RELEASE_TIMEOUT_S
        attempt = 0
        while time.monotonic() < deadline:
            if self._stop_requested.is_set():
                raise InterruptedSequence()
            attempt += 1
            self._mode_release_attempted = True
            code, _ = self._motion_switcher.ReleaseMode()
            if code != 0:
                raise RuntimeError(
                    "MotionSwitcher ReleaseMode failed on attempt {}: code={}".format(
                        attempt, code
                    )
                )
            time.sleep(0.25)
            _form, active_name = self._check_mode()
            if not active_name:
                self._mode_released = True
                print(
                    "Sport service {!r} released after {} attempt(s)".format(
                        expected_name, attempt
                    ),
                    flush=True,
                )
                return
            time.sleep(0.50)
        raise RuntimeError(
            "Sport service {!r} remained active after ReleaseMode".format(expected_name)
        )

    def _wait_for_lowcmd_quiet(self, ignore_stop=False, handoff="LowCmd ownership"):
        deadline = time.monotonic() + LOWCMD_QUIET_TIMEOUT_S
        while time.monotonic() < deadline:
            if self._stop_requested.is_set() and not ignore_stop:
                raise InterruptedSequence()
            with self._lock:
                last_lowcmd_time = self._last_lowcmd_time
            if (
                last_lowcmd_time is None
                or time.monotonic() - last_lowcmd_time >= LOWCMD_QUIET_S
            ):
                return
            time.sleep(0.02)
        raise RuntimeError(
            "rt/lowcmd did not become quiet before {}; another command writer may be active".format(
                handoff
            )
        )

    def _close_lowcmd_publisher(self):
        self._publishing = False
        publisher = self._publisher
        self._publisher = None
        if publisher is None:
            return
        publisher.Close()
        # Enforce a full quiet interval after our final sample. The subscriber
        # extends this timestamp if any other LowCmd writer is still active.
        with self._lock:
            self._last_lowcmd_time = time.monotonic()

    def _restore_mode(self, expected_name):
        if not expected_name:
            raise RuntimeError(
                "cannot restore Sport Mode without a captured service name"
            )

        self._mode_restore_attempted = True
        code, _ = self._motion_switcher.SelectMode(expected_name)
        if code != 0:
            raise RuntimeError(
                "MotionSwitcher SelectMode({!r}) failed: code={}".format(
                    expected_name, code
                )
            )

        deadline = time.monotonic() + MODE_SELECT_TIMEOUT_S
        while time.monotonic() < deadline:
            if self._hard_stop_requested.is_set():
                raise RuntimeError(
                    "hard stop requested while waiting for Sport Mode restoration"
                )
            _form, active_name = self._check_mode()
            if active_name == expected_name:
                self._mode_released = False
                self._mode_restored = True
                print(
                    "Sport service {!r} restored and confirmed by CheckMode()".format(
                        expected_name
                    ),
                    flush=True,
                )
                return
            if active_name:
                raise RuntimeError(
                    "SelectMode({!r}) activated unexpected service {!r}".format(
                        expected_name, active_name
                    )
                )
            time.sleep(0.25)

        raise RuntimeError(
            "Sport service {!r} was not confirmed after SelectMode".format(
                expected_name
            )
        )

    def _check_runtime(
        self,
        commanded_pose,
        ignore_first_stop=False,
        motion_context=None,
        motion_elapsed_s=None,
    ):
        if self._hard_stop_requested.is_set():
            raise HardStop()
        if self._stop_requested.is_set() and not ignore_first_stop:
            raise InterruptedSequence()

        sample = self._latest_sample()
        if sample is None or time.monotonic() - sample.received_at > LOWSTATE_TIMEOUT_S:
            raise RuntimeError("rt/lowstate watchdog expired")
        if not all(
            math.isfinite(value)
            for value in sample.pose + sample.leg_velocity + sample.rpy
        ):
            raise RuntimeError("rt/lowstate contains non-finite values")

        if commanded_pose is not None:
            if self._tracking_recorder is not None:
                signed_errors, max_index = self._tracking_recorder.record(
                    sample,
                    commanded_pose,
                    motion_context,
                    motion_elapsed_s,
                )
            else:
                signed_errors = [
                    commanded_pose[index] - sample.pose[index] for index in range(12)
                ]
                max_index = max(
                    range(12), key=lambda index: abs(signed_errors[index])
                )

            warning_indices = [
                index
                for index, error in enumerate(signed_errors)
                if abs(error) > RUN_TRACKING_WARNING_RAD
            ]
            exceeded_indices = (
                []
                if self.tracking_stop_rad is None
                else [
                    index
                    for index, error in enumerate(signed_errors)
                    if abs(error) > self.tracking_stop_rad
                ]
            )
            if warning_indices and not exceeded_indices:
                now = time.monotonic()
                if (
                    self._last_tracking_warning_print is None
                    or now - self._last_tracking_warning_print
                    >= TRACKING_WARNING_PRINT_INTERVAL_S
                ):
                    context_text = ""
                    if motion_context is not None:
                        context_text = " during {}".format(motion_context)
                        if motion_elapsed_s is not None:
                            context_text += " at {:.3f} s".format(motion_elapsed_s)
                    stop_text = (
                        "stop disabled"
                        if self.tracking_stop_rad is None
                        else "stop={:.3f} rad".format(self.tracking_stop_rad)
                    )
                    print(
                        (
                            "WARNING: joint tracking error: {}/12 leg joints "
                            "exceeded warning={:.3f} rad{}; max_abs_error={:.9f} "
                            "rad at motor[{}]/q[{}] {} ({})"
                        ).format(
                            len(warning_indices),
                            RUN_TRACKING_WARNING_RAD,
                            context_text,
                            abs(signed_errors[max_index]),
                            max_index,
                            max_index,
                            LEG_JOINT_NAMES[max_index],
                            stop_text,
                        ),
                        file=sys.stderr,
                        flush=True,
                    )
                    self._last_tracking_warning_print = now
            elif not warning_indices:
                self._last_tracking_warning_print = None

            if exceeded_indices:
                context_text = ""
                if motion_context is not None:
                    context_text = " during {}".format(motion_context)
                    if motion_elapsed_s is not None:
                        context_text += " at {:.3f} s".format(motion_elapsed_s)
                lines = [
                    (
                        "joint tracking watchdog triggered: {}/12 leg joints "
                        "exceeded the limit{}; max_abs_error={:.9f} rad at "
                        "motor[{}]/q[{}] {} > limit={:.9f} rad"
                    ).format(
                        len(exceeded_indices),
                        context_text,
                        abs(signed_errors[max_index]),
                        max_index,
                        max_index,
                        LEG_JOINT_NAMES[max_index],
                        self.tracking_stop_rad,
                    )
                ]
                for index in exceeded_indices:
                    lines.append(
                        (
                            "  motor[{0}]/q[{0}] {1}: measured={2:+.6f} rad, "
                            "commanded={3:+.6f} rad, "
                            "commanded-measured={4:+.9f} rad, "
                            "abs_error={5:.9f} rad"
                        ).format(
                            index,
                            LEG_JOINT_NAMES[index],
                            sample.pose[index],
                            commanded_pose[index],
                            signed_errors[index],
                            abs(signed_errors[index]),
                        )
                    )
                raise RuntimeError("\n".join(lines))

        max_tilt = max(abs(sample.rpy[0]), abs(sample.rpy[1]))
        if max_tilt > RUN_MAX_TILT_RAD:
            raise RuntimeError(
                "body tilt watchdog triggered: {:.3f} rad > {:.3f} rad".format(
                    max_tilt, RUN_MAX_TILT_RAD
                )
            )
        return sample

    def _run_for(
        self,
        duration_s,
        pose_at,
        ignore_first_stop=False,
        motion_context=None,
    ):
        start = time.monotonic()
        next_tick = start
        last_pose = list(pose_at(0.0))
        while True:
            elapsed = time.monotonic() - start
            self._check_runtime(
                last_pose,
                ignore_first_stop=ignore_first_stop,
                motion_context=motion_context,
                motion_elapsed_s=elapsed,
            )
            alpha = min(1.0, elapsed / duration_s)
            last_pose = list(pose_at(alpha))
            self._write_pose(last_pose)
            if elapsed >= duration_s:
                return last_pose

            next_tick += CONTROL_PERIOD_S
            delay = next_tick - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
            elif delay < -CONTROL_PERIOD_S:
                # Do not emit catch-up bursts after an OS scheduling delay.
                next_tick = time.monotonic()

    def _transition(
        self,
        name,
        source,
        target,
        duration_s,
        ignore_first_stop=False,
    ):
        print("transition -> {} ({:g} s)".format(name, duration_s), flush=True)
        return self._run_for(
            duration_s,
            lambda alpha: interpolate(source, target, alpha),
            ignore_first_stop=ignore_first_stop,
            motion_context="transition -> {}".format(name),
        )

    def _hold(self, name, pose, duration_s, ignore_first_stop=False):
        print("hold {} ({:.1f} s)".format(name, duration_s), flush=True)
        self._run_for(
            duration_s,
            lambda _alpha: pose,
            ignore_first_stop=ignore_first_stop,
            motion_context="hold {}".format(name),
        )

    def _neutralize(self, duration_s):
        if self._publisher is None:
            return
        print(
            "neutral zero-gain command ({:.1f} s)".format(duration_s),
            flush=True,
        )
        start = time.monotonic()
        next_tick = start
        while time.monotonic() - start < duration_s:
            self._write_neutral()
            if self._hard_stop_requested.is_set():
                break
            next_tick += CONTROL_PERIOD_S
            delay = next_tick - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
            elif delay < -CONTROL_PERIOD_S:
                next_tick = time.monotonic()
        self._neutralized = True

    def _run_height_sequence(self):
        current_pose = list(self._latest_sample().pose)
        current_pose = self._transition(
            "standard",
            current_pose,
            STANDARD,
            STANDARD_TRANSITION_S,
        )
        self._hold("standard", STANDARD, STANDARD_HOLD_S)

        for cycle in range(1, HEIGHT_CYCLES + 1):
            print("height cycle {}/{}".format(cycle, HEIGHT_CYCLES), flush=True)
            current_pose = self._transition(
                "low", current_pose, LOW, self.timing.transition_s
            )
            self._hold("low", LOW, self.timing.hold_s)
            current_pose = self._transition(
                "high", current_pose, HIGH, self.timing.transition_s
            )
            self._hold("high", HIGH, self.timing.hold_s)

        current_pose = self._transition(
            "standard",
            current_pose,
            STANDARD,
            STANDARD_TRANSITION_S,
        )
        self._hold("standard", STANDARD, STANDARD_HOLD_S)
        self._finish_at_captured_prone(current_pose)

    def _run_roll_sequence(self):
        current_pose = list(self._latest_sample().pose)
        current_pose = self._transition(
            "standard",
            current_pose,
            STANDARD,
            STANDARD_TRANSITION_S,
        )
        self._hold("standard", STANDARD, STANDARD_HOLD_S)

        for cycle in range(1, ROLL_CYCLES + 1):
            print("roll cycle {}/{}".format(cycle, ROLL_CYCLES), flush=True)
            current_pose = self._transition(
                "right roll",
                current_pose,
                ROLL_RIGHT,
                self.timing.transition_s,
            )
            self._hold("right roll", ROLL_RIGHT, self.timing.hold_s)
            current_pose = self._transition(
                "left roll",
                current_pose,
                ROLL_LEFT,
                self.timing.transition_s,
            )
            self._hold("left roll", ROLL_LEFT, self.timing.hold_s)

        current_pose = self._transition(
            "standard",
            current_pose,
            STANDARD,
            STANDARD_TRANSITION_S,
        )
        self._hold("standard", STANDARD, STANDARD_HOLD_S)
        self._finish_at_captured_prone(current_pose)

    def _finish_at_captured_prone(self, current_pose):
        self._transition(
            "captured prone",
            current_pose,
            self._captured_prone,
            PRONE_TRANSITION_S,
        )
        self._hold("captured prone", self._captured_prone, PRONE_HOLD_S)
        self._ended_prone = True
        self._neutralize(NEUTRAL_COMMAND_S)

    def _run_selected_gesture(self):
        if self.gesture == "height":
            self._run_height_sequence()
        elif self.gesture == "roll":
            self._run_roll_sequence()
        else:
            raise RuntimeError("unsupported gesture: {!r}".format(self.gesture))

    def _return_prone_after_interrupt(self):
        if self._publisher is None or self._captured_prone is None:
            return False
        sample = self._latest_sample()
        if sample is None or time.monotonic() - sample.received_at > LOWSTATE_TIMEOUT_S:
            print(
                "cannot perform controlled return because LowState is stale",
                file=sys.stderr,
                flush=True,
            )
            return False
        try:
            self._transition(
                "captured prone after interrupt",
                sample.pose,
                self._captured_prone,
                PRONE_TRANSITION_S,
                ignore_first_stop=True,
            )
            self._hold(
                "captured prone",
                self._captured_prone,
                PRONE_HOLD_S,
                ignore_first_stop=True,
            )
            self._ended_prone = True
            return True
        except (HardStop, RuntimeError) as error:
            print(
                "controlled return aborted: {}".format(error),
                file=sys.stderr,
                flush=True,
            )
            return False

    def _initialize_dds(self):
        ChannelFactoryInitialize(DDS_DOMAIN, self.interface)
        lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        lowcmd_subscriber = ChannelSubscriber("rt/lowcmd", LowCmd_)
        lowstate_subscriber.Init(self.on_low_state, 10)
        lowcmd_subscriber.Init(self.on_lowcmd, 10)

        self._motion_switcher = MotionSwitcherClient()
        self._motion_switcher.SetTimeout(5.0)
        self._motion_switcher.Init()
        self._sport_client = SportClient()
        self._sport_client.SetTimeout(5.0)
        self._sport_client.Init()

        # Retain subscribers for the lifetime of this controller.
        self._lowstate_subscriber = lowstate_subscriber
        self._lowcmd_subscriber = lowcmd_subscriber

    def _confirm_live(self, mode_form, mode_name, prone_pose, rpy):
        if not sys.stdin.isatty():
            raise RuntimeError("--live requires an interactive TTY confirmation")
        print("\nLIVE HARDWARE PRECHECK PASSED", flush=True)
        print("  selected gesture: {}".format(self.gesture), flush=True)
        print("  timing profile: {}".format(self.timing.name), flush=True)
        print("  NIC: {} = {}".format(self.interface, self.expected_ip), flush=True)
        print(
            "  active motion service: form={!r}, name={!r}".format(
                mode_form, mode_name
            ),
            flush=True,
        )
        if mode_name:
            print(
                "  ownership handoff: release {!r}, run LowCmd, then restore {!r}".format(
                    mode_name, mode_name
                ),
                flush=True,
            )
        else:
            print(
                "  ownership handoff: Sport is already released; proceed only if "
                "rt/lowcmd is quiet, and leave Sport released at the end",
                flush=True,
            )
        print(
            "  captured prone q[0:12]: [{}]".format(
                ", ".join("{:.4f}".format(value) for value in prone_pose)
            ),
            flush=True,
        )
        print(
            "  IMU rpy: [{}] rad".format(
                ", ".join("{:.4f}".format(value) for value in rpy)
            ),
            flush=True,
        )
        tracking_stop_text = (
            "disabled"
            if self.tracking_stop_rad is None
            else "{:.2f} rad".format(self.tracking_stop_rad)
        )
        print(
            "  joint tracking: warning {:.2f} rad, stop {}; log directory {}".format(
                RUN_TRACKING_WARNING_RAD,
                tracking_stop_text,
                self.tracking_log_dir,
            ),
            flush=True,
        )
        if self.tracking_stop_rad is None:
            print(
                "  WARNING: joint tracking error is recorded but cannot stop this run.",
                file=sys.stderr,
                flush=True,
            )
        print(
            "Ensure the robot is belly-down on a flat floor, wheels are blocked, "
            "a support/spotter is present, and the hardware E-stop is held ready.",
            flush=True,
        )
        confirmation = LIVE_CONFIRMATIONS[self.gesture]
        entered = input(
            "Type {!r} to take LowCmd ownership and move: ".format(confirmation)
        )
        if entered.strip() != confirmation:
            raise RuntimeError(
                "live confirmation did not match; LowCmd ownership was not changed"
            )
        if self._stop_requested.is_set():
            raise InterruptedSequence()

    def run(self, live):
        actual_ip = interface_ipv4(self.interface)
        if actual_ip != self.expected_ip:
            raise RuntimeError(
                "{} has IPv4 {}, expected {}; refusing DDS initialization".format(
                    self.interface, actual_ip, self.expected_ip
                )
            )

        for name, pose in (
            ("STANDARD", STANDARD),
            ("LOW", LOW),
            ("HIGH", HIGH),
            ("ROLL_RIGHT", ROLL_RIGHT),
            ("ROLL_LEFT", ROLL_LEFT),
        ):
            validate_pose_values(name, pose)
        for name, pose in (("ROLL_RIGHT", ROLL_RIGHT), ("ROLL_LEFT", ROLL_LEFT)):
            for index in HIP_INDICES:
                if not HIP_LIMIT_LOWER_RAD <= pose[index] <= HIP_LIMIT_UPPER_RAD:
                    raise RuntimeError(
                        "{} hip target exceeds URDF limit at motor {}".format(
                            name, index
                        )
                    )

        print(
            "network preflight passed: {} = {}, DDS domain {}".format(
                self.interface, actual_ip, DDS_DOMAIN
            ),
            flush=True,
        )
        self._initialize_dds()
        self._wait_for_first_state()
        mode_form, mode_name = self._check_mode()
        prone_pose, rpy = self._capture_stable_prone()

        print_sequence_plan(self.gesture, self.timing)
        print_tracking_policy(self.tracking_stop_rad)
        print(
            "read-only preflight: form={!r}, active mode={!r}, stable prone pose confirmed".format(
                mode_form, mode_name
            ),
            flush=True,
        )
        if not live:
            self._run_outcome = "preflight-completed"
            print(
                "DRY RUN COMPLETE: no StopMove, ReleaseMode, SelectMode, or LowCmd write was issued",
                flush=True,
            )
            return

        self._confirm_live(mode_form, mode_name, prone_pose, rpy)
        # Validate the host-persistent destination before changing Sport
        # ownership. No telemetry file is written in the 500 Hz control loop.
        self._prepare_tracking_recording()
        self._run_outcome = "preparing-lowcmd"

        if mode_name:
            self._restore_mode_name = mode_name
            stop_code = self._sport_client.StopMove()
            if stop_code != 0:
                raise RuntimeError(
                    "SportClient StopMove failed: code={}".format(stop_code)
                )
            time.sleep(0.5)
        else:
            self._mode_released = True
            print(
                "no active Sport service was reported; treating the robot as already "
                "released and requiring rt/lowcmd to be quiet before continuing",
                flush=True,
            )

        # Re-measure after confirmation (and after StopMove when Sport was
        # active) so shutdown uses the actual stable pose immediately before
        # this process takes LowCmd ownership.
        self._captured_prone, _rpy = self._capture_stable_prone()
        if mode_name:
            self._release_mode(mode_name)
        self._wait_for_lowcmd_quiet(handoff="starting this LowCmd publisher")

        self._publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self._publisher.Init()
        self._publishing = True
        if self._tracking_recorder is not None:
            self._tracking_recorder.start()
        self._run_outcome = "running"

        publisher_close_error = None
        try:
            self._run_selected_gesture()
        except InterruptedSequence:
            self._run_outcome = "interrupted-controlled-return"
            self._return_prone_after_interrupt()
            self._neutralize(NEUTRAL_COMMAND_S)
        except HardStop:
            self._run_outcome = "hard-stop"
            self._neutralize(0.25)
        finally:
            if not self._neutralized:
                try:
                    self._neutralize(0.5)
                except Exception as error:
                    print(
                        "best-effort neutral command failed: {}".format(error),
                        file=sys.stderr,
                        flush=True,
                    )
            try:
                self._close_lowcmd_publisher()
            except Exception as error:
                publisher_close_error = error
                print(
                    "failed to close the LowCmd publisher: {}".format(error),
                    file=sys.stderr,
                    flush=True,
                )

        if publisher_close_error is not None:
            raise RuntimeError(
                "LowCmd publisher close failed; refusing Sport Mode restoration: {}".format(
                    publisher_close_error
                )
            )

        if self._ended_prone:
            if self._restore_mode_name:
                self._wait_for_lowcmd_quiet(
                    ignore_stop=True,
                    handoff="Sport Mode restoration",
                )
                self._restore_mode(self._restore_mode_name)
                print(
                    "LowCmd publication stopped after the captured prone pose was held; "
                    "the startup Sport service is active again.",
                    flush=True,
                )
            else:
                print(
                    "LowCmd publication stopped after the captured prone pose was held. "
                    "Sport was already released at startup, so no service name was "
                    "available to restore; Sport remains released.",
                    flush=True,
                )
        else:
            print(
                "WARNING: LowCmd publication stopped without confirming the captured "
                "prone pose. Keep the robot supported and use the hardware E-stop. "
                "Sport Mode was not restored automatically.",
                file=sys.stderr,
                flush=True,
            )
        if self._run_outcome == "running":
            self._run_outcome = "completed"


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description=(
            "Go2W real-hardware low-level gestures. Default execution is a "
            "read-only preflight; physical motion requires --live plus an "
            "interactive gesture-specific confirmation."
        )
    )
    parser.add_argument(
        "--gesture",
        choices=GESTURE_NAMES,
        help="gesture to preflight or execute (required unless --describe is used)",
    )
    parser.add_argument(
        "--interface",
        default=DEFAULT_INTERFACE,
        help="robot-facing NIC (default: %(default)s)",
    )
    parser.add_argument(
        "--expected-ip",
        default=DEFAULT_EXPECTED_IP,
        help="required IPv4 address on the robot-facing NIC (default: %(default)s)",
    )
    parser.add_argument(
        "--live",
        action="store_true",
        help="allow Sport release and physical LowCmd motion after TTY confirmation",
    )
    parser.add_argument(
        "--tracking-log-dir",
        default=DEFAULT_TRACKING_LOG_DIR,
        help=(
            "host-persistent directory for live CSV/JSON telemetry "
            "(default: %(default)s; env: GO2W_TRACKING_LOG_DIR)"
        ),
    )
    parser.add_argument(
        "--describe",
        action="store_true",
        help="print the sequence without importing SDK2Py or opening DDS",
    )
    return parser.parse_args(argv)


def main(
    argv=None,
    timing=SLOW_TIMING,
    tracking_stop_rad=RUN_MAX_TRACKING_ERROR_RAD,
):
    if not isinstance(timing, GestureTimingProfile):
        raise ValueError("invalid timing profile: {!r}".format(timing))
    if tracking_stop_rad is not None and (
        not math.isfinite(tracking_stop_rad) or tracking_stop_rad <= 0.0
    ):
        raise ValueError(
            "tracking stop threshold must be a positive finite value or None"
        )
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.describe:
        print_sequence_plan(args.gesture, timing)
        print_tracking_policy(tracking_stop_rad)
        return 0
    if args.gesture is None:
        print(
            "error: --gesture is required for preflight or live execution",
            file=sys.stderr,
            flush=True,
        )
        return 2

    controller = None
    exit_code = 0
    outcome = "completed"
    error_text = None
    try:
        load_unitree_sdk()
        controller = HardwareGestureController(
            args.interface,
            args.expected_ip,
            args.gesture,
            timing=timing,
            tracking_log_dir=args.tracking_log_dir,
            tracking_stop_rad=tracking_stop_rad,
        )
        signal.signal(signal.SIGINT, controller.request_stop)
        signal.signal(signal.SIGTERM, controller.request_stop)
        controller.run(live=args.live)
        outcome = controller._run_outcome
    except InterruptedSequence as error:
        exit_code = 130
        outcome = "interrupted-before-control"
        error_text = str(error) or "stop requested before control ownership changed"
        print("stopped before control ownership changed", file=sys.stderr, flush=True)
    except (RuntimeError, ValueError) as error:
        exit_code = 1
        outcome = "error"
        error_text = str(error)
        print("error: {}".format(error), file=sys.stderr, flush=True)
        if controller is not None and (
            controller._mode_released
            or controller._mode_release_attempted
            or controller._mode_restore_attempted
        ):
            print(
                "WARNING: Sport Mode is released or restoration was not confirmed. "
                "Keep the robot supported; do not assume Sport stabilization is active.",
                file=sys.stderr,
                flush=True,
            )
    finally:
        if controller is not None:
            try:
                controller.finalize_tracking_log(outcome, error_text=error_text)
            except Exception as log_error:
                print(
                    "error: failed to save tracking telemetry: {}".format(log_error),
                    file=sys.stderr,
                    flush=True,
                )
                exit_code = 1
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
