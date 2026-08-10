#!/usr/bin/env python3
"""Run selected Go2W low-level gestures on hardware through Unitree SDK2Py.

The default invocation is a read-only preflight. Physical motion requires the
explicit ``--live`` flag and a confirmation phrase entered on a TTY.

This program intentionally does not reactivate Sport Mode after LowCmd. The
safe overlap-free handoff from a user LowCmd publisher back to the Go2W Sport
controller has not been qualified here. Instead, the live sequence records the
robot's stable initial prone joint pose, returns to that measured pose, sends a
neutral zero-gain command briefly, and then stops publishing.

Supported deployment targets are Python 3.8+ on the Jetson host or a host-
networked container with ``unitree_sdk2py`` installed. ROS 2 is not used by the
script; SDK2Py communicates directly over CycloneDDS.
"""

from __future__ import print_function

import argparse
from collections import deque
from dataclasses import dataclass
import fcntl
import math
import signal
import socket
import struct
import sys
import threading
import time
from typing import List


DDS_DOMAIN = 0
DEFAULT_INTERFACE = "eth0"
DEFAULT_EXPECTED_IP = "192.168.123.18"

CONTROL_PERIOD_S = 0.002  # 500 Hz
STANDARD_TRANSITION_S = 2.0
HEIGHT_TRANSITION_S = 1.0
PRONE_TRANSITION_S = 3.0
STANDARD_HOLD_S = 2.0
HEIGHT_HOLD_S = 0.5
PRONE_HOLD_S = 2.0
NEUTRAL_COMMAND_S = 1.0
HEIGHT_CYCLES = 3
ROLL_TRANSITION_S = 0.75
ROLL_HOLD_S = 0.5
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
RUN_MAX_TRACKING_ERROR_RAD = 0.45
LOWCMD_QUIET_S = 0.5
LOWCMD_QUIET_TIMEOUT_S = 3.0
MODE_RELEASE_TIMEOUT_S = 10.0

POS_STOP_F = 2.146e9
VEL_STOP_F = 16000.0
GESTURE_NAMES = ("height", "roll")
LIVE_CONFIRMATIONS = {
    "height": "RUN GO2W LOW LEVEL",
    "roll": "RUN GO2W ROLL LOW LEVEL",
}

KP = [60.0, 80.0, 80.0] * 4
KD = [5.0, 4.0, 4.0] * 4


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
    print("  neutral zero-gain command: {:.1f} s, then stop LowCmd".format(NEUTRAL_COMMAND_S))
    print("  Sport Mode is not reactivated automatically")


def print_height_plan():
    print("Go2W real-robot gesture: height")
    print("  captured prone -> standard: {:.1f} s".format(STANDARD_TRANSITION_S))
    print("  hold standard: {:.1f} s".format(STANDARD_HOLD_S))
    print("  repeat {} times:".format(HEIGHT_CYCLES))
    print("    -> low: {:.1f} s, hold {:.1f} s".format(HEIGHT_TRANSITION_S, HEIGHT_HOLD_S))
    print("    -> high: {:.1f} s, hold {:.1f} s".format(HEIGHT_TRANSITION_S, HEIGHT_HOLD_S))
    print("  high -> standard: {:.1f} s".format(STANDARD_TRANSITION_S))
    print_common_shutdown_plan()


def print_roll_plan():
    print("Go2W real-robot gesture: roll")
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
            ROLL_TRANSITION_S, ROLL_HOLD_S
        )
    )
    print(
        "    -> left: {:g} s, hold {:.1f} s".format(
            ROLL_TRANSITION_S, ROLL_HOLD_S
        )
    )
    print("  left -> standard: {:.1f} s".format(STANDARD_TRANSITION_S))
    print_common_shutdown_plan()


def print_sequence_plan(gesture=None):
    if gesture is None:
        print_height_plan()
        print("")
        print_roll_plan()
    elif gesture == "height":
        print_height_plan()
    elif gesture == "roll":
        print_roll_plan()
    else:
        raise ValueError("unknown gesture: {!r}".format(gesture))


@dataclass
class StateSample:
    received_at: float
    pose: List[float]
    leg_velocity: List[float]
    wheel_velocity: List[float]
    rpy: List[float]


class InterruptedSequence(Exception):
    pass


class HardStop(Exception):
    pass


class HardwareGestureController:
    def __init__(self, interface, expected_ip, gesture):
        if gesture not in GESTURE_NAMES:
            raise ValueError("unknown gesture: {!r}".format(gesture))
        self.interface = interface
        self.expected_ip = expected_ip
        self.gesture = gesture
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
        self._neutralized = False
        self._ended_prone = False
        self._captured_prone = None  # type: Optional[List[float]]
        self._configure_neutral_command()

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

    def _wait_for_lowcmd_quiet(self):
        deadline = time.monotonic() + LOWCMD_QUIET_TIMEOUT_S
        while time.monotonic() < deadline:
            if self._stop_requested.is_set():
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
            "rt/lowcmd did not become quiet after ReleaseMode; another command writer may be active"
        )

    def _check_runtime(self, commanded_pose, ignore_first_stop=False):
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

        max_tilt = max(abs(sample.rpy[0]), abs(sample.rpy[1]))
        if max_tilt > RUN_MAX_TILT_RAD:
            raise RuntimeError(
                "body tilt watchdog triggered: {:.3f} rad > {:.3f} rad".format(
                    max_tilt, RUN_MAX_TILT_RAD
                )
            )

        if commanded_pose is not None:
            tracking_error = max(
                abs(sample.pose[index] - commanded_pose[index]) for index in range(12)
            )
            if tracking_error > RUN_MAX_TRACKING_ERROR_RAD:
                raise RuntimeError(
                    "joint tracking watchdog triggered: {:.3f} rad > {:.3f} rad".format(
                        tracking_error, RUN_MAX_TRACKING_ERROR_RAD
                    )
                )
        return sample

    def _run_for(self, duration_s, pose_at, ignore_first_stop=False):
        start = time.monotonic()
        next_tick = start
        last_pose = list(pose_at(0.0))
        while True:
            self._check_runtime(last_pose, ignore_first_stop=ignore_first_stop)
            elapsed = time.monotonic() - start
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
        )

    def _hold(self, name, pose, duration_s, ignore_first_stop=False):
        print("hold {} ({:.1f} s)".format(name, duration_s), flush=True)
        self._run_for(
            duration_s,
            lambda _alpha: pose,
            ignore_first_stop=ignore_first_stop,
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
                "low", current_pose, LOW, HEIGHT_TRANSITION_S
            )
            self._hold("low", LOW, HEIGHT_HOLD_S)
            current_pose = self._transition(
                "high", current_pose, HIGH, HEIGHT_TRANSITION_S
            )
            self._hold("high", HIGH, HEIGHT_HOLD_S)

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
                ROLL_TRANSITION_S,
            )
            self._hold("right roll", ROLL_RIGHT, ROLL_HOLD_S)
            current_pose = self._transition(
                "left roll",
                current_pose,
                ROLL_LEFT,
                ROLL_TRANSITION_S,
            )
            self._hold("left roll", ROLL_LEFT, ROLL_HOLD_S)

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
        print("  NIC: {} = {}".format(self.interface, self.expected_ip), flush=True)
        print(
            "  active motion service: form={!r}, name={!r}".format(
                mode_form, mode_name
            ),
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
        print(
            "Ensure the robot is belly-down on a flat floor, wheels are blocked, "
            "a support/spotter is present, and the hardware E-stop is held ready.",
            flush=True,
        )
        confirmation = LIVE_CONFIRMATIONS[self.gesture]
        entered = input(
            "Type {!r} to release Sport Mode and move: ".format(confirmation)
        )
        if entered.strip() != confirmation:
            raise RuntimeError("live confirmation did not match; no mode was released")
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

        print_sequence_plan(self.gesture)
        print(
            "read-only preflight: form={!r}, active mode={!r}, stable prone pose confirmed".format(
                mode_form, mode_name
            ),
            flush=True,
        )
        if not live:
            print(
                "DRY RUN COMPLETE: no StopMove, ReleaseMode, SelectMode, or LowCmd write was issued",
                flush=True,
            )
            return

        if not mode_name:
            raise RuntimeError(
                "no active Sport service was reported; refusing to assume LowCmd ownership"
            )

        self._confirm_live(mode_form, mode_name, prone_pose, rpy)

        stop_code = self._sport_client.StopMove()
        if stop_code != 0:
            raise RuntimeError("SportClient StopMove failed: code={}".format(stop_code))
        time.sleep(0.5)

        # Re-measure after StopMove so the shutdown target is the actual stable
        # hardware pose immediately before releasing the motion service.
        self._captured_prone, _rpy = self._capture_stable_prone()
        self._release_mode(mode_name)
        self._wait_for_lowcmd_quiet()

        self._publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self._publisher.Init()
        self._publishing = True

        try:
            self._run_selected_gesture()
        except InterruptedSequence:
            self._return_prone_after_interrupt()
            self._neutralize(NEUTRAL_COMMAND_S)
        except HardStop:
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
            self._publishing = False

        if self._ended_prone:
            print(
                "LowCmd publication stopped after the captured prone pose was held. "
                "Sport Mode remains released; reactivate it only through a separately "
                "qualified procedure while the robot is safely supported.",
                flush=True,
            )
        else:
            print(
                "WARNING: LowCmd publication stopped without confirming the captured "
                "prone pose. Keep the robot supported and use the hardware E-stop. "
                "Sport Mode remains released.",
                file=sys.stderr,
                flush=True,
            )


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
        "--describe",
        action="store_true",
        help="print the sequence without importing SDK2Py or opening DDS",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.describe:
        print_sequence_plan(args.gesture)
        return 0
    if args.gesture is None:
        print(
            "error: --gesture is required for preflight or live execution",
            file=sys.stderr,
            flush=True,
        )
        return 2

    controller = None
    try:
        load_unitree_sdk()
        controller = HardwareGestureController(
            args.interface,
            args.expected_ip,
            args.gesture,
        )
        signal.signal(signal.SIGINT, controller.request_stop)
        signal.signal(signal.SIGTERM, controller.request_stop)
        controller.run(live=args.live)
    except InterruptedSequence:
        print("stopped before control ownership changed", file=sys.stderr, flush=True)
        return 130
    except (RuntimeError, ValueError) as error:
        print("error: {}".format(error), file=sys.stderr, flush=True)
        if controller is not None and controller._mode_released:
            print(
                "WARNING: Sport Mode was released and was not reactivated. Keep the "
                "robot supported; do not assume Sport stabilization is active.",
                file=sys.stderr,
                flush=True,
            )
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
