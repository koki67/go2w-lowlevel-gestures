#!/usr/bin/env python3
"""Run a single Go2W low-to-high joint-pose sequence in MuJoCo.

This script is simulation-only. It starts and owns the flat Go2W MuJoCo
process, commands the same STANDARD, LOW, and HIGH joint angles as
go2w_height_sequence_sim.py, and continuously holds HIGH after the sequence
until Ctrl+C.
"""

from __future__ import annotations

import argparse
import signal
import sys
import time

import go2w_height_sequence_sim as base  # noqa: E402


LOW_TRANSITION_S = 2.0
LOW_HOLD_S = 2.0
HIGH_TRANSITION_S = 0.5


def format_pose(pose: list[float]) -> str:
    return "[" + ", ".join(f"{angle:.2f}" for angle in pose) + "] rad"


def print_plan() -> None:
    print("joint order: FR, FL, RR, RL; each leg is hip, thigh, calf")
    print(f"STANDARD = {format_pose(base.STANDARD)}")
    print(f"LOW      = {format_pose(base.LOW)}")
    print(f"HIGH     = {format_pose(base.HIGH)}")
    print("Go2W simulated low-to-high sequence:")
    print(f"  startup -> standard: {base.STANDARD_TRANSITION_S:.1f} s")
    print(f"  hold standard: {base.STANDARD_HOLD_S:.1f} s")
    print(f"  standard -> low: {LOW_TRANSITION_S:.1f} s")
    print(f"  hold low: {LOW_HOLD_S:.1f} s")
    print(f"  low -> high: {HIGH_TRANSITION_S:.1f} s")
    print("  hold high at 500 Hz until Ctrl+C")


class LowToHighSequenceController(base.SequenceController):
    def __init__(self) -> None:
        super().__init__("go2w_low_to_high_sequence_sim")

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

        # Refuse to create a second command owner or a second simulator on
        # DDS domain 0/loopback.
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
            self._hold(
                publisher,
                "standard",
                base.STANDARD,
                base.STANDARD_HOLD_S,
            )

            current_pose = self._transition(
                publisher,
                "low",
                current_pose,
                base.LOW,
                LOW_TRANSITION_S,
            )
            self._hold(publisher, "low", base.LOW, LOW_HOLD_S)

            self._transition(
                publisher,
                "high",
                current_pose,
                base.HIGH,
                HIGH_TRANSITION_S,
            )
            final_hold_start = time.monotonic()

            with self._lock:
                base_height = self._base_height
            height_text = "unknown" if base_height is None else f"{base_height:.3f} m"
            print(
                "sequence complete; holding high pose at 500 Hz "
                f"(base_z={height_text}). The joint plot is written after "
                f"{base.FINAL_HOLD_PLOT_S:.1f} s of final hold; press Ctrl+C "
                "after that to close MuJoCo.",
                flush=True,
            )
            self._run_final_hold(publisher, base.HIGH, final_hold_start)
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
        help="print joint targets and sequence without starting DDS or MuJoCo",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.describe:
        print_plan()
        return 0

    try:
        base.reexec_with_simulator_python(__file__)
        base.load_unitree_sdk()
    except Exception as error:
        print(f"error: {error}", file=sys.stderr, flush=True)
        return 1

    print_plan()
    controller = LowToHighSequenceController()
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
