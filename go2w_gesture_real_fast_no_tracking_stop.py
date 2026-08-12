#!/usr/bin/env python3
"""Run fast hardware gestures without a joint-tracking-error stop threshold.

The repeated gesture transitions take 1.0 s and holds take 0.5 s. The 0.45 rad
warning and complete tracking telemetry remain enabled. All other runtime
watchdogs and the normal LowCmd/Sport Mode ownership handling are unchanged.
"""

from __future__ import print_function

from go2w_gesture_real import FAST_TIMING
from go2w_gesture_real import main as controller_main


def main(argv=None):
    return controller_main(
        argv,
        timing=FAST_TIMING,
        tracking_stop_rad=None,
    )


if __name__ == "__main__":
    raise SystemExit(main())
