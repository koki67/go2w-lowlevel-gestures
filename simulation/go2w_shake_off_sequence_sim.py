#!/usr/bin/env python3
"""Run a fast wet-dog-style Go2W shake-off sequence in MuJoCo.

This is an intentionally aggressive simulation-only experiment. It reuses the
same URDF-limited right/left roll targets as go2w_roll_sequence_sim.py, but
switches between them much faster to approximate a dog shaking off water.
After the sequence, it returns to STANDARD and holds until Ctrl+C.
"""

from __future__ import annotations

import argparse
import signal
import sys

import go2w_roll_sequence_sim as roll  # noqa: E402


base = roll.base

SHAKE_TRANSITION_S = 0.10
SHAKE_HOLD_S = 0.03
SHAKE_CYCLES = 8


def print_plan(
    lower: float,
    upper: float,
    source: str,
    amplitude: float,
    save_plot: bool = False,
) -> None:
    roll.print_plan(
        lower,
        upper,
        source,
        amplitude,
        save_plot,
        sequence_name="shake-off",
        transition_s=SHAKE_TRANSITION_S,
        hold_s=SHAKE_HOLD_S,
        cycles=SHAKE_CYCLES,
    )
    print("  intent: experimental wet-dog-style rapid body shake")
    print("  safety scope: simulation only; not qualified for hardware")


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
        amplitude, _right, _left = roll.make_roll_targets(lower, upper)
        print_plan(lower, upper, source, amplitude, args.save_plot)
        return 0

    try:
        base.reexec_with_simulator_python(__file__)
        base.load_unitree_sdk()
        lower, upper, source = roll.resolve_hip_limits()
    except Exception as error:
        print(f"error: {error}", file=sys.stderr, flush=True)
        return 1

    amplitude, right, left = roll.make_roll_targets(lower, upper)
    print_plan(lower, upper, source, amplitude, args.save_plot)
    controller = roll.RollSequenceController(
        right,
        left,
        plot_stem="go2w_shake_off_sequence_sim",
        transition_s=SHAKE_TRANSITION_S,
        hold_s=SHAKE_HOLD_S,
        cycles=SHAKE_CYCLES,
        cycle_label="shake-off",
        save_plot=args.save_plot,
    )
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
