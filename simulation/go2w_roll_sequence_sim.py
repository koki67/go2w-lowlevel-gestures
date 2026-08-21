#!/usr/bin/env python3
"""Run a URDF-limit-derived Go2W body-roll sequence in MuJoCo.

This is simulation-only.  It starts the flat Go2W MuJoCo scene, owns the sole
LowCmd publisher on DDS domain 0/loopback, and continuously holds STANDARD
after the requested sequence until Ctrl+C.

The URDF does not define a base/body roll limit because the base is floating.
Instead, this script derives the largest symmetric common hip-abduction offset
that keeps all four hip commands inside their URDF limits, then uses 70% of
that offset.  The achieved body roll is measured from the simulated IMU and
reported separately.
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import signal
import sys
import time
import xml.etree.ElementTree as ET

import go2w_height_sequence_sim as base  # noqa: E402


ROLL_TRANSITION_S = 0.75
FINAL_STANDARD_TRANSITION_S = 2.0
ROLL_HOLD_S = 0.5
ROLL_CYCLES = 3
ROLL_LIMIT_SCALE = base.hardware_gesture.ROLL_LIMIT_SCALE

HIP_INDICES = (0, 3, 6, 9)
HIP_JOINT_NAMES = (
    "FR_hip_joint",
    "FL_hip_joint",
    "RR_hip_joint",
    "RL_hip_joint",
)
MODEL_XML = base.MODEL_XML

# unitree_mujoco has the simulation MJCF but no Go2W URDF.  Prefer the local
# Go2W description workspaces used by this machine, with GO2W_URDF available as
# an explicit override.  The selected URDF limits are cross-checked against the
# MJCF that MuJoCo actually loads.
DEFAULT_URDF_CANDIDATES = (
    base.UNITREE_MUJOCO_ROOT.parent
    / "fastlio-go2w"
    / "humble_ws"
    / "src"
    / "go2w_description"
    / "urdf"
    / "go2w_description.urdf",
    base.UNITREE_MUJOCO_ROOT.parent
    / "frontier-fw-go2w"
    / "humble_ws"
    / "src"
    / "go2w_description"
    / "urdf"
    / "go2w_description.urdf",
)


def parse_urdf_hip_limits(path: Path) -> tuple[float, float]:
    root = ET.parse(path).getroot()
    limits: list[tuple[float, float]] = []
    for name in HIP_JOINT_NAMES:
        joint = root.find(f".//joint[@name='{name}']")
        if joint is None:
            raise RuntimeError(f"URDF is missing {name}: {path}")
        limit = joint.find("limit")
        if limit is None or "lower" not in limit.attrib or "upper" not in limit.attrib:
            raise RuntimeError(f"URDF joint has no finite limits: {name}")
        limits.append((float(limit.attrib["lower"]), float(limit.attrib["upper"])))

    first = limits[0]
    if any(
        abs(lower - first[0]) > 1e-9 or abs(upper - first[1]) > 1e-9
        for lower, upper in limits[1:]
    ):
        raise RuntimeError(f"URDF hip limits are not identical: {limits}")
    return first


def parse_mjcf_hip_limits(path: Path) -> tuple[float, float]:
    root = ET.parse(path).getroot()
    joint = root.find(".//default[@class='abduction']/joint")
    if joint is None or "range" not in joint.attrib:
        raise RuntimeError(f"MJCF abduction range not found: {path}")
    values = [float(value) for value in joint.attrib["range"].split()]
    if len(values) != 2:
        raise RuntimeError(f"invalid MJCF abduction range: {joint.attrib['range']!r}")
    return values[0], values[1]


def resolve_hip_limits() -> tuple[float, float, str]:
    override = os.environ.get("GO2W_URDF")
    if override:
        urdf_path = Path(override).expanduser().resolve()
        if not urdf_path.is_file():
            raise RuntimeError(f"GO2W_URDF does not exist: {urdf_path}")
    else:
        urdf_path = next((path for path in DEFAULT_URDF_CANDIDATES if path.is_file()), None)

    mjcf_limits = parse_mjcf_hip_limits(MODEL_XML)
    if urdf_path is None:
        return (*mjcf_limits, f"{MODEL_XML} (MJCF fallback; no Go2W URDF found)")

    urdf_limits = parse_urdf_hip_limits(urdf_path)
    if any(abs(a - b) > 1e-9 for a, b in zip(urdf_limits, mjcf_limits)):
        raise RuntimeError(
            f"URDF hip limits {urdf_limits} disagree with MuJoCo {mjcf_limits}"
        )
    return (*urdf_limits, str(urdf_path))


def make_roll_targets(
    lower: float,
    upper: float,
) -> tuple[float, list[float], list[float]]:
    positive_margin = min(upper - base.STANDARD[index] for index in HIP_INDICES)
    negative_margin = min(base.STANDARD[index] - lower for index in HIP_INDICES)
    theoretical_limit = min(positive_margin, negative_margin)
    if theoretical_limit <= 0.0:
        raise RuntimeError("STANDARD lies outside the common hip-offset range")
    amplitude = theoretical_limit * ROLL_LIMIT_SCALE

    # A negative common hip offset produces positive base roll (right side
    # down) when the wheels remain on the floor; the simulated IMU is treated
    # as the authority and printed after each hold.
    right = list(base.STANDARD)
    left = list(base.STANDARD)
    for index in HIP_INDICES:
        right[index] -= amplitude
        left[index] += amplitude

    for name, pose in (("right", right), ("left", left)):
        for index in HIP_INDICES:
            if pose[index] < lower - 1e-9 or pose[index] > upper + 1e-9:
                raise RuntimeError(
                    f"{name} hip target {pose[index]} exceeds [{lower}, {upper}]"
                )
    return amplitude, right, left


def quaternion_to_rpy(quaternion: list[float]) -> tuple[float, float, float]:
    w, x, y, z = quaternion
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm < 1e-9:
        raise ValueError("zero-length quaternion")
    w, x, y, z = (value / norm for value in (w, x, y, z))

    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_term = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(pitch_term)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def print_plan(
    lower: float,
    upper: float,
    source: str,
    amplitude: float,
    save_plot: bool = False,
    *,
    sequence_name: str = "roll",
    transition_s: float = ROLL_TRANSITION_S,
    hold_s: float = ROLL_HOLD_S,
    cycles: int = ROLL_CYCLES,
    final_standard_transition_s: float = FINAL_STANDARD_TRANSITION_S,
) -> None:
    print(f"hip limits: [{lower:.4f}, {upper:.4f}] rad, source={source}")
    print(
        "applied common hip offset: "
        f"±{amplitude:.5f} rad (±{math.degrees(amplitude):.2f} deg), "
        f"{ROLL_LIMIT_SCALE:.0%} of the URDF-derived limit"
    )
    print(f"Go2W simulated {sequence_name} sequence:")
    print(f"  startup -> standard: {base.STANDARD_TRANSITION_S:.1f} s")
    print(f"  hold standard: {base.STANDARD_HOLD_S:.1f} s")
    print(f"  repeat {cycles} times:")
    print(f"    -> right: {transition_s:g} s, hold {hold_s:g} s")
    print(f"    -> left: {transition_s:g} s, hold {hold_s:g} s")
    print(f"  left -> standard: {final_standard_transition_s:.1f} s")
    print("  hold standard at 500 Hz until Ctrl+C")
    print(
        "  joint tracking plot: "
        + ("save after final hold" if save_plot else "disabled (use --save-plot)")
    )


class RollSequenceController(base.SequenceController):
    def __init__(
        self,
        right: list[float],
        left: list[float],
        *,
        plot_stem: str = "go2w_roll_sequence_sim",
        transition_s: float = ROLL_TRANSITION_S,
        hold_s: float = ROLL_HOLD_S,
        cycles: int = ROLL_CYCLES,
        cycle_label: str = "roll",
        final_standard_transition_s: float = FINAL_STANDARD_TRANSITION_S,
        save_plot: bool = False,
    ) -> None:
        if transition_s <= 0.0 or hold_s <= 0.0:
            raise ValueError("roll transition and hold durations must be positive")
        if cycles <= 0:
            raise ValueError("roll cycles must be positive")
        if final_standard_transition_s <= 0.0:
            raise ValueError("final standard transition duration must be positive")

        super().__init__(plot_stem, save_plot=save_plot)
        self._right = right
        self._left = left
        self._transition_s = transition_s
        self._hold_s = hold_s
        self._cycles = cycles
        self._cycle_label = cycle_label
        self._final_standard_transition_s = final_standard_transition_s
        self._rpy: tuple[float, float, float] | None = None
        self._peak_abs_roll = 0.0

    def on_low_state(self, message: base.LowState_) -> None:
        super().on_low_state(message)
        try:
            quaternion = [float(message.imu_state.quaternion[i]) for i in range(4)]
            rpy = quaternion_to_rpy(quaternion)
        except (AttributeError, IndexError, TypeError, ValueError):
            return
        with self._lock:
            self._rpy = rpy
            self._peak_abs_roll = max(self._peak_abs_roll, abs(rpy[0]))

    def _report_orientation(self, label: str) -> None:
        with self._lock:
            rpy = self._rpy
            base_height = self._base_height
        if rpy is None:
            print(f"{label}: IMU orientation unavailable", flush=True)
            return
        height_text = "unknown" if base_height is None else f"{base_height:.3f} m"
        print(
            f"{label}: measured body rpy="
            f"[{math.degrees(rpy[0]):.2f}, {math.degrees(rpy[1]):.2f}, "
            f"{math.degrees(rpy[2]):.2f}] deg, base_z={height_text}",
            flush=True,
        )

    def _transition(
        self,
        publisher: base.ChannelPublisher,
        name: str,
        source: list[float],
        target: list[float],
        duration_s: float,
    ) -> list[float]:
        # Preserve sub-second timing exactly in the runtime log; the reusable
        # height-sequence helper displays durations with only one decimal.
        print(f"transition -> {name} ({duration_s:g} s)", flush=True)
        return self._run_for(
            publisher,
            duration_s,
            lambda alpha: self._interpolate(source, target, alpha),
        )

    def _hold(
        self,
        publisher: base.ChannelPublisher,
        name: str,
        pose: list[float],
        duration_s: float,
    ) -> None:
        # Keep very short shake-off holds visible instead of rounding 0.03 s
        # down to 0.0 s in the runtime log.
        with self._lock:
            base_height = self._base_height
        height_text = "unknown" if base_height is None else f"{base_height:.3f} m"
        print(f"hold {name} ({duration_s:g} s), base_z={height_text}", flush=True)
        self._run_for(publisher, duration_s, lambda _alpha: pose)

    def run(self) -> None:
        flat_scene = self._prepare_flat_scene()

        base.ChannelFactoryInitialize(base.DDS_DOMAIN, base.DDS_INTERFACE)
        low_state_subscriber = base.ChannelSubscriber("rt/lowstate", base.LowState_)
        sport_state_subscriber = base.ChannelSubscriber(
            "rt/sportmodestate", base.SportModeState_
        )
        lowcmd_subscriber = base.ChannelSubscriber("rt/lowcmd", base.LowCmd_)
        low_state_subscriber.Init(self.on_low_state, 10)
        sport_state_subscriber.Init(self.on_sport_state, 10)
        lowcmd_subscriber.Init(self.on_lowcmd, 10)

        time.sleep(0.25)
        with self._lock:
            simulator_already_present = self._last_low_state_time is not None
        if self._lowcmd_seen_before_start or simulator_already_present:
            raise RuntimeError(
                "DDS activity already exists on domain 0/lo; stop the existing "
                "controller and simulator before running this script"
            )

        simulator_command = [
            str(base.SIMULATOR),
            "-i",
            str(base.DDS_DOMAIN),
            "-n",
            base.DDS_INTERFACE,
            "-r",
            "go2w",
            "-s",
            str(flat_scene),
        ]
        print("starting flat Go2W MuJoCo GUI", flush=True)
        self._simulator = base.subprocess.Popen(
            simulator_command,
            cwd=base.UNITREE_MUJOCO_ROOT,
            stdout=base.subprocess.DEVNULL,
            stderr=base.subprocess.STDOUT,
        )

        publisher = base.ChannelPublisher("rt/lowcmd", base.LowCmd_)
        publisher.Init()

        try:
            current_pose = self._wait_for_first_state()
            self._publishing = True

            current_pose = self._transition(
                publisher,
                "standard",
                current_pose,
                base.STANDARD,
                base.STANDARD_TRANSITION_S,
            )
            self._hold(publisher, "standard", base.STANDARD, base.STANDARD_HOLD_S)
            self._report_orientation("standard hold complete")

            for cycle in range(1, self._cycles + 1):
                print(
                    f"{self._cycle_label} cycle {cycle}/{self._cycles}", flush=True
                )
                current_pose = self._transition(
                    publisher,
                    "right roll limit",
                    current_pose,
                    self._right,
                    self._transition_s,
                )
                self._hold(
                    publisher,
                    "right roll limit",
                    self._right,
                    self._hold_s,
                )
                self._report_orientation(f"cycle {cycle} right hold complete")

                current_pose = self._transition(
                    publisher,
                    "left roll limit",
                    current_pose,
                    self._left,
                    self._transition_s,
                )
                self._hold(
                    publisher,
                    "left roll limit",
                    self._left,
                    self._hold_s,
                )
                self._report_orientation(f"cycle {cycle} left hold complete")

            self._transition(
                publisher,
                "standard",
                current_pose,
                base.STANDARD,
                self._final_standard_transition_s,
            )
            final_hold_start = time.monotonic()
            self._report_orientation("returned to standard")

            with self._lock:
                peak_roll = self._peak_abs_roll
            if self._save_plot:
                plot_status = (
                    f"The joint plot will be written after "
                    f"{base.FINAL_HOLD_PLOT_S:.1f} s of final hold; press Ctrl+C "
                    "after that to close MuJoCo."
                )
            else:
                plot_status = (
                    "Joint plot recording is disabled; press Ctrl+C to close MuJoCo."
                )
            print(
                "sequence complete; holding standard pose at 500 Hz. "
                f"Peak measured |roll|={math.degrees(peak_roll):.2f} deg. "
                f"{plot_status}",
                flush=True,
            )
            self._run_final_hold(publisher, base.STANDARD, final_hold_start)
        except InterruptedError:
            print("stop requested", flush=True)
        finally:
            self._publishing = False
            self._stop_simulator()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--describe",
        action="store_true",
        help="print limits and sequence without starting DDS or MuJoCo",
    )
    parser.add_argument(
        "--save-plot",
        action="store_true",
        help="record target/actual joint angles and save an SVG after final hold",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.describe:
        lower = base.hardware_gesture.HIP_LIMIT_LOWER_RAD
        upper = base.hardware_gesture.HIP_LIMIT_UPPER_RAD
        source = "hardware gesture constants; runtime cross-checks MuJoCo MJCF"
        amplitude, _right, _left = make_roll_targets(lower, upper)
        print_plan(lower, upper, source, amplitude, args.save_plot)
        return 0

    try:
        base.reexec_with_simulator_python(__file__)
        base.load_unitree_sdk()
        lower, upper, source = resolve_hip_limits()
    except Exception as error:
        print(f"error: {error}", file=sys.stderr, flush=True)
        return 1

    amplitude, right, left = make_roll_targets(lower, upper)
    print_plan(lower, upper, source, amplitude, args.save_plot)
    controller = RollSequenceController(right, left, save_plot=args.save_plot)
    signal.signal(signal.SIGINT, controller.request_stop)
    signal.signal(signal.SIGTERM, controller.request_stop)
    try:
        controller.run()
    except Exception as error:
        print(f"error: {error}", file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
