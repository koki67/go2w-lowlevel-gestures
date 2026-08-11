#!/usr/bin/env python3
"""Run the legacy 1.0 s transition / 0.5 s hold hardware profile."""

from __future__ import print_function

from go2w_gesture_real import LEGACY_TIMING
from go2w_gesture_real import main as controller_main


def main(argv=None):
    return controller_main(argv, timing=LEGACY_TIMING)


if __name__ == "__main__":
    raise SystemExit(main())
