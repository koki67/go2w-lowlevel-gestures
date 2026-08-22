#!/usr/bin/env python3
"""Leave Go2W motion services released for manual initial-pose setup.

This utility never creates a ``rt/lowcmd`` publisher.  It checks that the
robot-facing network interface is the expected one, requires the topic to be
quiet, stops the active Sport service, and confirms ``ReleaseMode()`` through
``CheckMode()``.  The released state intentionally remains active after the
process exits so an operator can move the supported robot by hand.
"""

from __future__ import print_function

import argparse
import signal
import sys
import threading
import time

import go2w_gesture_real as base


RELEASE_CONFIRMATION = "RELEASE GO2W FOR MANUAL POSITIONING"


def print_sequence_plan():
    print("Go2W manual initial-pose preparation:")
    print("  1. verify the robot-facing NIC and require rt/lowcmd to be quiet")
    print("  2. report the active motion service and require a typed confirmation")
    print("  3. call Sport StopMove() when a service is active")
    print("  4. call ReleaseMode() until CheckMode() reports no active service")
    print("  5. publish no LowCmd and leave the robot released after exit")


class ReleaseModeController:
    def __init__(self, interface, expected_ip):
        self.interface = interface
        self.expected_ip = expected_ip
        self._lock = threading.Lock()
        self._stop_requested = threading.Event()
        self._last_lowcmd_time = None
        self._motion_switcher = None
        self._sport_client = None
        self._lowcmd_subscriber = None
        self.mode_release_attempted = False
        self.mode_released = False

    def request_stop(self, _signum=None, _frame=None):
        self._stop_requested.set()

    def on_lowcmd(self, _message):
        with self._lock:
            self._last_lowcmd_time = time.monotonic()

    def _initialize_dds(self):
        base.ChannelFactoryInitialize(base.DDS_DOMAIN, self.interface)
        lowcmd_subscriber = base.ChannelSubscriber("rt/lowcmd", base.LowCmd_)
        lowcmd_subscriber.Init(self.on_lowcmd, 10)

        self._motion_switcher = base.MotionSwitcherClient()
        self._motion_switcher.SetTimeout(5.0)
        self._motion_switcher.Init()
        self._sport_client = base.SportClient()
        self._sport_client.SetTimeout(5.0)
        self._sport_client.Init()

        # Retain the subscriber and require a complete quiet observation window
        # after DDS initialization before changing controller ownership.
        self._lowcmd_subscriber = lowcmd_subscriber

    def _check_mode(self):
        code, result = self._motion_switcher.CheckMode()
        if code != 0 or result is None or not isinstance(result, dict):
            raise RuntimeError(
                "MotionSwitcher CheckMode failed: code={}, result={!r}".format(
                    code, result
                )
            )
        return str(result.get("form", "")), str(result.get("name", ""))

    def _wait_for_lowcmd_quiet(self, handoff):
        deadline = time.monotonic() + base.LOWCMD_QUIET_TIMEOUT_S
        quiet_since = time.monotonic()
        while time.monotonic() < deadline:
            if self._stop_requested.is_set():
                raise InterruptedError("release interrupted before completion")
            with self._lock:
                last_lowcmd_time = self._last_lowcmd_time
            if last_lowcmd_time is not None and last_lowcmd_time > quiet_since:
                quiet_since = last_lowcmd_time
            if time.monotonic() - quiet_since >= base.LOWCMD_QUIET_S:
                return
            time.sleep(0.02)
        raise RuntimeError(
            "rt/lowcmd did not stay quiet before {}; another command writer may "
            "be active".format(handoff)
        )

    def _confirm_release(self, mode_form, mode_name):
        if not sys.stdin.isatty():
            raise RuntimeError("ReleaseMode requires an interactive TTY confirmation")
        print("\nRELEASE MODE HARDWARE PRECHECK", flush=True)
        print("  NIC: {} = {}".format(self.interface, self.expected_ip), flush=True)
        print(
            "  active motion service: form={!r}, name={!r}".format(
                mode_form, mode_name
            ),
            flush=True,
        )
        print("  rt/lowcmd: quiet", flush=True)
        print(
            "  REQUIRED: support the body against dropping or pinching, block all "
            "wheels, keep hands clear until release completes, and hold the "
            "hardware E-stop ready.",
            flush=True,
        )
        print(
            "  After success there is no posture or balance controller; move the "
            "joints manually only while the robot remains supported.",
            flush=True,
        )
        entered = input("Type {!r} to continue: ".format(RELEASE_CONFIRMATION))
        if entered.strip() != RELEASE_CONFIRMATION:
            raise RuntimeError(
                "release confirmation did not match; controller ownership was not changed"
            )
        if self._stop_requested.is_set():
            raise InterruptedError("release interrupted before ownership changed")

    def _release_mode(self, expected_name):
        deadline = time.monotonic() + base.MODE_RELEASE_TIMEOUT_S
        attempt = 0
        while time.monotonic() < deadline:
            if self._stop_requested.is_set():
                raise InterruptedError("release interrupted before completion")
            attempt += 1
            self.mode_release_attempted = True
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
                self.mode_released = True
                print(
                    "Sport service {!r} released after {} attempt(s)".format(
                        expected_name, attempt
                    ),
                    flush=True,
                )
                return
            time.sleep(0.50)
        raise RuntimeError(
            "Sport service {!r} remained active after ReleaseMode".format(
                expected_name
            )
        )

    def run(self):
        actual_ip = base.interface_ipv4(self.interface)
        if actual_ip != self.expected_ip:
            raise RuntimeError(
                "{} has IPv4 {}, expected {}; refusing DDS initialization".format(
                    self.interface, actual_ip, self.expected_ip
                )
            )

        print(
            "network preflight passed: {} = {}, DDS domain {}".format(
                self.interface, actual_ip, base.DDS_DOMAIN
            ),
            flush=True,
        )
        self._initialize_dds()
        mode_form, mode_name = self._check_mode()
        self._wait_for_lowcmd_quiet("manual-positioning release")

        if not mode_name:
            self.mode_released = True
            print(
                "RELEASE MODE READY: CheckMode() already reports no active motion "
                "service; no StopMove(), ReleaseMode(), or LowCmd write was issued.",
                flush=True,
            )
            return

        self._confirm_release(mode_form, mode_name)
        confirmed_form, confirmed_name = self._check_mode()
        if (confirmed_form, confirmed_name) != (mode_form, mode_name):
            raise RuntimeError(
                "active motion service changed during confirmation: "
                "form={!r}, name={!r}; refusing release".format(
                    confirmed_form, confirmed_name
                )
            )

        stop_code = self._sport_client.StopMove()
        if stop_code != 0:
            raise RuntimeError("SportClient StopMove failed: code={}".format(stop_code))
        time.sleep(0.5)
        self._release_mode(mode_name)
        self._wait_for_lowcmd_quiet("manual joint positioning")
        final_form, final_name = self._check_mode()
        if final_name:
            raise RuntimeError(
                "motion service became active after release: form={!r}, name={!r}".format(
                    final_form, final_name
                )
            )

        print(
            "RELEASE MODE READY: no active motion service and no LowCmd publisher "
            "from this process. Keep the robot supported while setting the known "
            "initial pose by hand.",
            flush=True,
        )


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description=(
            "Release Go2W motion services for supported manual initial-pose setup. "
            "This command never publishes LowCmd."
        )
    )
    parser.add_argument(
        "--interface",
        default=base.DEFAULT_INTERFACE,
        help="robot-facing NIC (default: %(default)s)",
    )
    parser.add_argument(
        "--expected-ip",
        default=base.DEFAULT_EXPECTED_IP,
        help="required IPv4 address on the robot-facing NIC (default: %(default)s)",
    )
    parser.add_argument(
        "--describe",
        action="store_true",
        help="print the release sequence without importing SDK2Py or opening DDS",
    )
    return parser.parse_args(argv)


def main(argv=None, controller_class=ReleaseModeController):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.describe:
        print_sequence_plan()
        return 0

    controller = None
    try:
        base.load_unitree_sdk()
        controller = controller_class(args.interface, args.expected_ip)
        signal.signal(signal.SIGINT, controller.request_stop)
        signal.signal(signal.SIGTERM, controller.request_stop)
        controller.run()
        return 0
    except (InterruptedError, KeyboardInterrupt) as error:
        print("stopped: {}".format(error), file=sys.stderr, flush=True)
        if controller is not None and (
            controller.mode_released or controller.mode_release_attempted
        ):
            print(
                "WARNING: Sport Mode may be released. Keep the robot supported; "
                "do not assume posture stabilization is active.",
                file=sys.stderr,
                flush=True,
            )
        return 130
    except (RuntimeError, ValueError) as error:
        print("error: {}".format(error), file=sys.stderr, flush=True)
        if controller is not None and (
            controller.mode_released or controller.mode_release_attempted
        ):
            print(
                "WARNING: Sport Mode may be released. Keep the robot supported; "
                "do not assume posture stabilization is active.",
                file=sys.stderr,
                flush=True,
            )
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
