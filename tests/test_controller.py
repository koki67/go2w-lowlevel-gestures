import csv
from contextlib import redirect_stderr
import io
import json
import tempfile
import threading
import time
import unittest
from unittest import mock

import go2w_gesture_real as controller_module
import go2w_gesture_real_fast as fast_module
import go2w_gesture_real_fast_no_tracking_stop as fast_no_tracking_stop_module
import go2w_gesture_real_no_tracking_stop as no_tracking_stop_module


PRONE = [0.0, 1.36, -2.65] * 4


class FakePublisher:
    def __init__(self):
        self.commands = []
        self.fail = False
        self.initialized = False
        self.closed = False

    def Init(self):
        self.initialized = True

    def Close(self):
        self.closed = True

    def Write(self, command):
        if self.fail:
            return False
        self.commands.append(
            {
                "q": [float(command.motor_cmd[index].q) for index in range(16)],
                "dq": [float(command.motor_cmd[index].dq) for index in range(16)],
                "kp": [float(command.motor_cmd[index].kp) for index in range(16)],
                "kd": [float(command.motor_cmd[index].kd) for index in range(16)],
                "crc": int(command.crc),
            }
        )
        return True


class ControllerTests(unittest.TestCase):
    def setUp(self):
        controller_module.load_unitree_sdk()

    def test_interpolation_endpoints(self):
        self.assertEqual(
            controller_module.interpolate(
                controller_module.LOW, controller_module.HIGH, 0.0
            ),
            controller_module.LOW,
        )
        self.assertEqual(
            controller_module.interpolate(
                controller_module.LOW, controller_module.HIGH, 1.0
            ),
            controller_module.HIGH,
        )

    def test_original_controller_keeps_core_lowstate_when_extended_fields_are_absent(self):
        controller = controller_module.HardwareGestureController(
            "eth0", "192.168.123.18", "height"
        )
        message = mock.Mock(
            motor_state=[
                mock.Mock(q=0.01 * index, dq=-0.02 * index, spec=("q", "dq"))
                for index in range(16)
            ],
            imu_state=mock.Mock(rpy=[0.1, -0.2, 0.3], spec=("rpy",)),
            spec=("motor_state", "imu_state"),
        )

        controller.on_low_state(message)

        sample = controller._latest_sample()
        self.assertIsNotNone(sample)
        self.assertEqual(sample.pose[3], 0.03)
        self.assertEqual(sample.wheel_velocity, [-0.24, -0.26, -0.28, -0.3])
        self.assertEqual(sample.rpy, [0.1, -0.2, 0.3])
        self.assertEqual(sample.tau_est, [])
        self.assertEqual(sample.motor_mode, [])
        self.assertEqual(sample.gyro, [])

    def test_dry_run_never_creates_publisher_or_changes_mode(self):
        controller = controller_module.HardwareGestureController(
            "eth0", "192.168.123.18", "height"
        )
        controller._initialize_dds = mock.Mock()
        controller._wait_for_first_state = mock.Mock()
        controller._check_mode = mock.Mock(return_value=("1", "ai-w"))
        controller._capture_stable_prone = mock.Mock(
            return_value=(list(PRONE), [0.0, 0.0, 0.0])
        )
        controller._validate_preflight_state = mock.Mock()

        with mock.patch.object(
            controller_module, "interface_ipv4", return_value="192.168.123.18"
        ):
            controller.run(live=False)

        self.assertIsNone(controller._publisher)
        self.assertFalse(controller._mode_released)
        controller._initialize_dds.assert_called_once_with()
        controller._validate_preflight_state.assert_called_once_with()

    def test_roll_dry_run_never_creates_publisher_or_changes_mode(self):
        controller = controller_module.HardwareGestureController(
            "eth0", "192.168.123.18", "roll"
        )
        controller._initialize_dds = mock.Mock()
        controller._wait_for_first_state = mock.Mock()
        controller._check_mode = mock.Mock(return_value=("1", "ai-w"))
        controller._capture_stable_prone = mock.Mock(
            return_value=(list(PRONE), [0.0, 0.0, 0.0])
        )

        with mock.patch.object(
            controller_module, "interface_ipv4", return_value="192.168.123.18"
        ):
            controller.run(live=False)

        self.assertIsNone(controller._publisher)
        self.assertFalse(controller._mode_released)
        controller._initialize_dds.assert_called_once_with()

    def test_execution_without_gesture_fails_before_sdk_load(self):
        with mock.patch.object(controller_module, "load_unitree_sdk") as sdk_load:
            self.assertEqual(controller_module.main([]), 2)
        sdk_load.assert_not_called()

    def test_ip_mismatch_fails_before_dds(self):
        controller = controller_module.HardwareGestureController(
            "eth0", "192.168.123.18", "height"
        )
        controller._initialize_dds = mock.Mock()

        with mock.patch.object(
            controller_module, "interface_ipv4", return_value="192.168.123.99"
        ):
            with self.assertRaisesRegex(RuntimeError, "refusing DDS initialization"):
                controller.run(live=False)

        controller._initialize_dds.assert_not_called()

    def test_pose_and_neutral_command_fields(self):
        controller = controller_module.HardwareGestureController(
            "eth0", "192.168.123.18", "height"
        )
        publisher = FakePublisher()
        controller._publisher = publisher

        controller._write_pose(controller_module.STANDARD)
        pose_command = publisher.commands[-1]
        self.assertEqual(pose_command["q"][:12], controller_module.STANDARD)
        self.assertEqual(pose_command["dq"][12:16], [0.0] * 4)
        self.assertEqual(pose_command["kp"][12:16], [0.0] * 4)
        self.assertEqual(pose_command["kd"][12:16], [2.0] * 4)
        self.assertNotEqual(pose_command["crc"], 0)

        controller._write_neutral()
        neutral_command = publisher.commands[-1]
        self.assertEqual(neutral_command["q"], [controller_module.POS_STOP_F] * 16)
        self.assertEqual(neutral_command["dq"], [controller_module.VEL_STOP_F] * 16)
        self.assertEqual(neutral_command["kp"], [0.0] * 16)
        self.assertEqual(neutral_command["kd"], [0.0] * 16)

    def test_tracking_watchdog_reports_all_exceeded_leg_joints(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            controller = controller_module.HardwareGestureController(
                "eth0", "192.168.123.18", "height"
            )
            controller._tracking_recorder = controller_module.TrackingRecorder(
                temporary_directory,
                "height",
                controller_module.SLOW_TIMING,
            )
            controller._tracking_recorder.start()
            commanded = list(controller_module.STANDARD)
            measured = list(commanded)
            measured[2] += 0.5501
            measured[6] -= 0.62
            measured[10] += 0.54
            controller._latest_sample = mock.Mock(
                return_value=controller_module.StateSample(
                    received_at=time.monotonic(),
                    pose=measured,
                    leg_velocity=[0.0] * 12,
                    wheel_velocity=[0.0] * 4,
                    rpy=[0.01, -0.02, 0.03],
                )
            )

            with self.assertRaises(RuntimeError) as raised:
                controller._check_runtime(
                    commanded,
                    motion_context="transition -> high",
                    motion_elapsed_s=1.25,
                )

            message = str(raised.exception)
            self.assertIn("2/12 leg joints", message)
            self.assertIn("during transition -> high at 1.250 s", message)
            self.assertIn("max_abs_error=0.620000000 rad", message)
            self.assertIn("limit=0.550000000 rad", message)
            self.assertIn("motor[2]/q[2] FR_calf", message)
            self.assertIn("motor[6]/q[6] RR_hip", message)
            self.assertNotIn("motor[10]/q[10] RL_thigh", message)
            self.assertIn("commanded-measured=-0.550100000 rad", message)
            self.assertIn("commanded-measured=+0.620000000 rad", message)

            # The row that triggers the stop is buffered before the exception.
            self.assertEqual(controller._tracking_recorder.sample_count, 1)
            csv_path, summary_path = controller.finalize_tracking_log(
                "error", error_text=message
            )
            with csv_path.open(newline="", encoding="utf-8") as input_file:
                rows = list(csv.DictReader(input_file))
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["phase"], "transition -> high")
            self.assertEqual(float(rows[0]["phase_elapsed_s"]), 1.25)
            self.assertGreaterEqual(float(rows[0]["lowstate_age_s"]), 0.0)
            self.assertAlmostEqual(
                float(rows[0]["measured_FR_calf_rad"]), measured[2]
            )
            self.assertAlmostEqual(
                float(rows[0]["target_FR_calf_rad"]), commanded[2]
            )
            self.assertAlmostEqual(
                float(rows[0]["error_target_minus_measured_FR_calf_rad"]),
                -0.5501,
            )
            for joint_name in controller_module.LEG_JOINT_NAMES:
                self.assertIn("measured_{}_rad".format(joint_name), rows[0])
                self.assertIn("target_{}_rad".format(joint_name), rows[0])
                self.assertIn(
                    "error_target_minus_measured_{}_rad".format(joint_name),
                    rows[0],
                )

            with summary_path.open(encoding="utf-8") as input_file:
                summary = json.load(input_file)
            self.assertEqual(summary["sample_count"], 1)
            self.assertEqual(summary["stop_crossing_sample_count"], 1)
            self.assertGreaterEqual(summary["max_lowstate_age_s"], 0.0)
            self.assertEqual(summary["global_peak"]["name"], "RR_hip")
            self.assertAlmostEqual(
                summary["global_peak"]["max_abs_error_rad"], 0.62
            )

    def test_old_stop_threshold_is_now_a_throttled_warning(self):
        controller = controller_module.HardwareGestureController(
            "eth0", "192.168.123.18", "height"
        )
        commanded = list(controller_module.STANDARD)
        measured = list(commanded)
        measured[8] += 0.46
        controller._latest_sample = mock.Mock(
            return_value=controller_module.StateSample(
                received_at=time.monotonic(),
                pose=measured,
                leg_velocity=[0.0] * 12,
                wheel_velocity=[0.0] * 4,
                rpy=[0.0, 0.0, 0.0],
            )
        )

        stderr = io.StringIO()
        with redirect_stderr(stderr):
            controller._check_runtime(
                commanded,
                motion_context="hold low",
                motion_elapsed_s=0.25,
            )
            controller._check_runtime(
                commanded,
                motion_context="hold low",
                motion_elapsed_s=0.252,
            )

        warning_output = stderr.getvalue()
        self.assertEqual(warning_output.count("WARNING: joint tracking error"), 1)
        self.assertIn("warning=0.450 rad", warning_output)
        self.assertIn("stop=0.550 rad", warning_output)
        self.assertIn("RR_calf", warning_output)

    def test_no_tracking_stop_records_large_error_without_stopping(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            controller = controller_module.HardwareGestureController(
                "eth0",
                "192.168.123.18",
                "height",
                tracking_log_dir=temporary_directory,
                tracking_stop_rad=None,
            )
            controller._prepare_tracking_recording()
            controller._tracking_recorder.start()
            commanded = list(controller_module.STANDARD)
            measured = list(commanded)
            measured[5] += 0.80
            controller._latest_sample = mock.Mock(
                return_value=controller_module.StateSample(
                    received_at=time.monotonic(),
                    pose=measured,
                    leg_velocity=[0.0] * 12,
                    wheel_velocity=[0.0] * 4,
                    rpy=[0.0, 0.0, 0.0],
                )
            )

            stderr = io.StringIO()
            with redirect_stderr(stderr):
                controller._check_runtime(
                    commanded,
                    motion_context="transition -> high",
                    motion_elapsed_s=0.75,
                )

            self.assertIn("stop disabled", stderr.getvalue())
            csv_path, summary_path = controller.finalize_tracking_log("completed")
            self.assertIn("no-tracking-stop", csv_path.name)
            with summary_path.open(encoding="utf-8") as input_file:
                summary = json.load(input_file)
            self.assertFalse(summary["tracking_stop_enabled"])
            self.assertIsNone(summary["tracking_stop_rad"])
            self.assertEqual(summary["stop_crossing_sample_count"], 0)
            self.assertEqual(summary["warning_crossing_sample_count"], 1)
            self.assertAlmostEqual(
                summary["global_peak"]["max_abs_error_rad"], 0.80
            )

    def test_dry_run_does_not_create_tracking_log_directory(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            log_dir = controller_module.Path(temporary_directory) / "live-only"
            controller = controller_module.HardwareGestureController(
                "eth0",
                "192.168.123.18",
                "height",
                tracking_log_dir=str(log_dir),
            )
            controller._initialize_dds = mock.Mock()
            controller._wait_for_first_state = mock.Mock()
            controller._check_mode = mock.Mock(return_value=("1", "ai-w"))
            controller._capture_stable_prone = mock.Mock(
                return_value=(list(PRONE), [0.0, 0.0, 0.0])
            )

            with mock.patch.object(
                controller_module, "interface_ipv4", return_value="192.168.123.18"
            ):
                controller.run(live=False)

            self.assertFalse(log_dir.exists())

    def test_main_finalizes_tracking_telemetry_after_runtime_error(self):
        controller = mock.Mock()
        controller.run.side_effect = RuntimeError("simulated watchdog stop")
        controller._mode_released = True
        controller._mode_release_attempted = True
        controller._mode_restore_attempted = False

        with mock.patch.object(controller_module, "load_unitree_sdk"), mock.patch.object(
            controller_module,
            "HardwareGestureController",
            return_value=controller,
        ), mock.patch.object(controller_module.signal, "signal"):
            exit_code = controller_module.main(
                [
                    "--gesture",
                    "height",
                    "--live",
                    "--tracking-log-dir",
                    "/tmp/tracking-test",
                ]
            )

        self.assertEqual(exit_code, 1)
        controller.run.assert_called_once_with(live=True)
        controller.finalize_tracking_log.assert_called_once_with(
            "error", error_text="simulated watchdog stop"
        )

    def test_dds_write_failure_is_not_ignored(self):
        controller = controller_module.HardwareGestureController(
            "eth0", "192.168.123.18", "height"
        )
        publisher = FakePublisher()
        publisher.fail = True
        controller._publisher = publisher

        with self.assertRaisesRegex(RuntimeError, "DDS write failed"):
            controller._write_pose(controller_module.STANDARD)

    def test_release_mode_failure_stops_before_lowcmd(self):
        controller = controller_module.HardwareGestureController(
            "eth0", "192.168.123.18", "height"
        )
        controller._motion_switcher = mock.Mock()
        controller._motion_switcher.ReleaseMode.return_value = (3101, None)

        with self.assertRaisesRegex(
            RuntimeError, "ReleaseMode failed on attempt 1: code=3101"
        ):
            controller._release_mode("ai-w")

        self.assertFalse(controller._mode_released)
        self.assertTrue(controller._mode_release_attempted)
        controller._motion_switcher.CheckMode.assert_not_called()

    def test_restore_mode_selects_captured_service_and_confirms_it(self):
        controller = controller_module.HardwareGestureController(
            "eth0", "192.168.123.18", "height"
        )
        controller._mode_released = True
        controller._motion_switcher = mock.Mock()
        controller._motion_switcher.SelectMode.return_value = (0, None)
        controller._check_mode = mock.Mock(
            side_effect=[("", ""), ("1", "ai-w")]
        )

        with mock.patch.object(controller_module.time, "sleep"):
            controller._restore_mode("ai-w")

        controller._motion_switcher.SelectMode.assert_called_once_with("ai-w")
        self.assertTrue(controller._mode_restore_attempted)
        self.assertTrue(controller._mode_restored)
        self.assertFalse(controller._mode_released)

    def test_restore_mode_rpc_failure_remains_fail_closed(self):
        controller = controller_module.HardwareGestureController(
            "eth0", "192.168.123.18", "height"
        )
        controller._mode_released = True
        controller._motion_switcher = mock.Mock()
        controller._motion_switcher.SelectMode.return_value = (3102, None)
        controller._check_mode = mock.Mock()

        with self.assertRaisesRegex(
            RuntimeError, r"SelectMode\('ai-w'\) failed: code=3102"
        ):
            controller._restore_mode("ai-w")

        self.assertTrue(controller._mode_restore_attempted)
        self.assertFalse(controller._mode_restored)
        self.assertTrue(controller._mode_released)
        controller._check_mode.assert_not_called()

    def _configure_live_run(self, controller, mode):
        fake_publisher = FakePublisher()
        controller._initialize_dds = mock.Mock()
        controller._wait_for_first_state = mock.Mock()
        controller._check_mode = mock.Mock(return_value=mode)
        controller._capture_stable_prone = mock.Mock(
            return_value=(list(PRONE), [0.0, 0.0, 0.0])
        )
        controller._confirm_live = mock.Mock()
        controller._wait_for_lowcmd_quiet = mock.Mock()

        def complete_gesture():
            controller._ended_prone = True
            controller._neutralized = True

        controller._run_selected_gesture = mock.Mock(side_effect=complete_gesture)
        return fake_publisher

    def test_live_run_accepts_already_released_mode_when_lowcmd_is_quiet(self):
        controller = controller_module.HardwareGestureController(
            "eth0", "192.168.123.18", "height"
        )
        fake_publisher = self._configure_live_run(controller, ("", ""))
        controller._sport_client = mock.Mock()

        with mock.patch.object(
            controller_module, "interface_ipv4", return_value="192.168.123.18"
        ), mock.patch.object(
            controller_module, "ChannelPublisher", return_value=fake_publisher
        ):
            controller.run(live=True)

        controller._sport_client.StopMove.assert_not_called()
        controller._wait_for_lowcmd_quiet.assert_called_once_with(
            handoff="starting this LowCmd publisher"
        )
        self.assertEqual(controller._capture_stable_prone.call_count, 2)
        self.assertTrue(controller._mode_released)
        self.assertIsNone(controller._restore_mode_name)
        self.assertTrue(fake_publisher.initialized)
        self.assertTrue(fake_publisher.closed)
        self.assertIsNone(controller._publisher)

    def test_live_run_restores_startup_mode_only_after_closing_lowcmd(self):
        controller = controller_module.HardwareGestureController(
            "eth0", "192.168.123.18", "height"
        )
        fake_publisher = self._configure_live_run(controller, ("1", "ai-w"))
        controller._sport_client = mock.Mock()
        controller._sport_client.StopMove.return_value = 0

        def release_mode(name):
            self.assertEqual(name, "ai-w")
            controller._mode_released = True

        controller._release_mode = mock.Mock(side_effect=release_mode)

        def restore_mode(name):
            self.assertTrue(fake_publisher.closed)
            self.assertIsNone(controller._publisher)
            self.assertEqual(name, "ai-w")

        controller._restore_mode = mock.Mock(side_effect=restore_mode)

        with mock.patch.object(
            controller_module, "interface_ipv4", return_value="192.168.123.18"
        ), mock.patch.object(
            controller_module, "ChannelPublisher", return_value=fake_publisher
        ), mock.patch.object(controller_module.time, "sleep"):
            controller.run(live=True)

        controller._sport_client.StopMove.assert_called_once_with()
        controller._release_mode.assert_called_once_with("ai-w")
        controller._restore_mode.assert_called_once_with("ai-w")
        self.assertEqual(
            controller._wait_for_lowcmd_quiet.call_args_list,
            [
                mock.call(handoff="starting this LowCmd publisher"),
                mock.call(ignore_stop=True, handoff="Sport Mode restoration"),
            ],
        )

    def test_watchdog_failure_closes_lowcmd_without_restoring_sport(self):
        controller = controller_module.HardwareGestureController(
            "eth0", "192.168.123.18", "height"
        )
        fake_publisher = self._configure_live_run(controller, ("1", "ai-w"))
        controller._sport_client = mock.Mock()
        controller._sport_client.StopMove.return_value = 0
        controller._release_mode = mock.Mock(
            side_effect=lambda _name: setattr(controller, "_mode_released", True)
        )
        controller._run_selected_gesture = mock.Mock(
            side_effect=RuntimeError("joint tracking watchdog triggered")
        )
        controller._neutralize = mock.Mock(
            side_effect=lambda _duration: setattr(controller, "_neutralized", True)
        )
        controller._restore_mode = mock.Mock()

        with mock.patch.object(
            controller_module, "interface_ipv4", return_value="192.168.123.18"
        ), mock.patch.object(
            controller_module, "ChannelPublisher", return_value=fake_publisher
        ), mock.patch.object(controller_module.time, "sleep"):
            with self.assertRaisesRegex(RuntimeError, "tracking watchdog"):
                controller.run(live=True)

        self.assertTrue(fake_publisher.closed)
        controller._restore_mode.assert_not_called()
        self.assertTrue(controller._mode_released)

    def run_accelerated_sequence(self, gesture):
        accelerated_timing = controller_module.GestureTimingProfile(
            name="accelerated-test",
            transition_s=0.01,
            hold_s=0.01,
        )
        controller = controller_module.HardwareGestureController(
            "eth0",
            "192.168.123.18",
            gesture,
            timing=accelerated_timing,
            # This millisecond-scale command-generation harness intentionally
            # runs faster than its feeder thread can deterministically publish
            # every intermediate sample.  Watchdog behavior is covered by
            # dedicated tests; disabling it here removes scheduler flakiness
            # without changing any production wrapper policy.
            tracking_stop_rad=None,
        )
        publisher = FakePublisher()
        controller._publisher = publisher
        controller._publishing = True
        controller._captured_prone = list(PRONE)

        finished = threading.Event()

        def feed_state():
            while not finished.is_set():
                pose = controller._last_commanded_pose or PRONE
                sample = controller_module.StateSample(
                    received_at=time.monotonic(),
                    pose=list(pose),
                    leg_velocity=[0.0] * 12,
                    wheel_velocity=[0.0] * 4,
                    rpy=[0.0, 0.0, 0.0],
                )
                with controller._lock:
                    controller._samples.append(sample)
                time.sleep(0.0005)

        timing_overrides = {
            "CONTROL_PERIOD_S": 0.001,
            "STANDARD_TRANSITION_S": 0.02,
            "PRONE_TRANSITION_S": 0.02,
            "STANDARD_HOLD_S": 0.01,
            "PRONE_HOLD_S": 0.01,
            "NEUTRAL_COMMAND_S": 0.01,
            "HEIGHT_CYCLES": 1,
            "ROLL_CYCLES": 1,
        }

        thread = threading.Thread(target=feed_state, daemon=True)
        thread.start()
        try:
            with mock.patch.multiple(controller_module, **timing_overrides):
                controller._run_selected_gesture()
        finally:
            finished.set()
            thread.join(timeout=1.0)

        def pose_was_commanded(target):
            return any(
                max(
                    abs(command["q"][index] - target[index])
                    for index in range(12)
                )
                < 1e-6
                for command in publisher.commands
            )

        return controller, pose_was_commanded

    def test_accelerated_height_sequence(self):
        controller, pose_was_commanded = self.run_accelerated_sequence("height")

        self.assertTrue(pose_was_commanded(controller_module.STANDARD))
        self.assertTrue(pose_was_commanded(controller_module.LOW))
        self.assertTrue(pose_was_commanded(controller_module.HIGH))
        self.assertTrue(pose_was_commanded(PRONE))
        self.assertTrue(controller._ended_prone)
        self.assertTrue(controller._neutralized)

    def test_fast_script_selects_fast_timing_profile(self):
        with mock.patch.object(
            fast_module, "controller_main", return_value=0
        ) as controller_main:
            self.assertEqual(fast_module.main(["--describe"]), 0)

        controller_main.assert_called_once_with(
            ["--describe"], timing=controller_module.FAST_TIMING
        )

    def test_no_tracking_stop_script_selects_slow_profile_without_stop(self):
        with mock.patch.object(
            no_tracking_stop_module, "controller_main", return_value=0
        ) as controller_main:
            self.assertEqual(no_tracking_stop_module.main(["--describe"]), 0)

        controller_main.assert_called_once_with(
            ["--describe"],
            timing=controller_module.SLOW_TIMING,
            tracking_stop_rad=None,
        )

    def test_fast_no_tracking_stop_script_selects_fast_profile_without_stop(self):
        with mock.patch.object(
            fast_no_tracking_stop_module, "controller_main", return_value=0
        ) as controller_main:
            self.assertEqual(
                fast_no_tracking_stop_module.main(["--describe"]),
                0,
            )

        controller_main.assert_called_once_with(
            ["--describe"],
            timing=controller_module.FAST_TIMING,
            tracking_stop_rad=None,
        )

    def test_fast_profile_applies_to_height_and_roll_cycles(self):
        for gesture in ("height", "roll"):
            controller = controller_module.HardwareGestureController(
                "eth0",
                "192.168.123.18",
                gesture,
                timing=controller_module.FAST_TIMING,
            )
            controller._captured_prone = list(PRONE)
            controller._latest_sample = mock.Mock(
                return_value=controller_module.StateSample(
                    received_at=time.monotonic(),
                    pose=list(PRONE),
                    leg_velocity=[0.0] * 12,
                    wheel_velocity=[0.0] * 4,
                    rpy=[0.0, 0.0, 0.0],
                )
            )

            transitions = []
            holds = []

            def transition(name, _source, target, duration, **_kwargs):
                transitions.append((name, duration))
                return list(target)

            def hold(name, _pose, duration, **_kwargs):
                holds.append((name, duration))

            controller._transition = transition
            controller._hold = hold
            controller._neutralize = mock.Mock()
            controller._run_selected_gesture()

            cycle_names = (
                ("low", "high")
                if gesture == "height"
                else ("right roll", "left roll")
            )
            cycle_transitions = [
                duration
                for name, duration in transitions
                if name in cycle_names
            ]
            cycle_holds = [
                duration for name, duration in holds if name in cycle_names
            ]
            self.assertEqual(cycle_transitions, [1.0] * 6)
            self.assertEqual(cycle_holds, [0.5] * 6)

    def test_accelerated_roll_sequence(self):
        controller, pose_was_commanded = self.run_accelerated_sequence("roll")

        self.assertTrue(pose_was_commanded(controller_module.STANDARD))
        self.assertTrue(pose_was_commanded(controller_module.ROLL_RIGHT))
        self.assertTrue(pose_was_commanded(controller_module.ROLL_LEFT))
        self.assertTrue(pose_was_commanded(PRONE))
        self.assertTrue(controller._ended_prone)
        self.assertTrue(controller._neutralized)

    def test_height_sequence_order_and_timing(self):
        controller = controller_module.HardwareGestureController(
            "eth0", "192.168.123.18", "height"
        )
        controller._captured_prone = list(PRONE)
        controller._latest_sample = mock.Mock(
            return_value=controller_module.StateSample(
                received_at=time.monotonic(),
                pose=list(PRONE),
                leg_velocity=[0.0] * 12,
                wheel_velocity=[0.0] * 4,
                rpy=[0.0, 0.0, 0.0],
            )
        )

        transitions = []
        holds = []

        def transition(name, _source, target, duration, **_kwargs):
            transitions.append((name, duration))
            return list(target)

        def hold(name, _pose, duration, **_kwargs):
            holds.append((name, duration))

        controller._transition = transition
        controller._hold = hold
        controller._neutralize = mock.Mock()
        controller._run_height_sequence()

        expected_transitions = [("standard", 2.0)]
        for _cycle in range(3):
            expected_transitions.extend([("low", 2.0), ("high", 2.0)])
        expected_transitions.extend(
            [("standard", 2.0), ("captured prone", 3.0)]
        )
        self.assertEqual(transitions, expected_transitions)

        expected_holds = [("standard", 2.0)]
        for _cycle in range(3):
            expected_holds.extend([("low", 2.0), ("high", 2.0)])
        expected_holds.extend([("standard", 2.0), ("captured prone", 2.0)])
        self.assertEqual(holds, expected_holds)
        controller._neutralize.assert_called_once_with(1.0)

    def test_roll_targets_are_70_percent_and_inside_urdf_limits(self):
        self.assertAlmostEqual(controller_module.ROLL_AMPLITUDE_RAD, 0.66304)
        for pose in (controller_module.ROLL_RIGHT, controller_module.ROLL_LEFT):
            for index in controller_module.HIP_INDICES:
                self.assertGreaterEqual(
                    pose[index], controller_module.HIP_LIMIT_LOWER_RAD
                )
                self.assertLessEqual(
                    pose[index], controller_module.HIP_LIMIT_UPPER_RAD
                )

    def test_roll_sequence_order_and_timing(self):
        controller = controller_module.HardwareGestureController(
            "eth0", "192.168.123.18", "roll"
        )
        controller._captured_prone = list(PRONE)
        controller._latest_sample = mock.Mock(
            return_value=controller_module.StateSample(
                received_at=time.monotonic(),
                pose=list(PRONE),
                leg_velocity=[0.0] * 12,
                wheel_velocity=[0.0] * 4,
                rpy=[0.0, 0.0, 0.0],
            )
        )

        transitions = []
        holds = []

        def transition(name, _source, target, duration, **_kwargs):
            transitions.append((name, duration))
            return list(target)

        def hold(name, _pose, duration, **_kwargs):
            holds.append((name, duration))

        controller._transition = transition
        controller._hold = hold
        controller._neutralize = mock.Mock()
        controller._run_roll_sequence()

        expected_transitions = [("standard", 2.0)]
        for _cycle in range(3):
            expected_transitions.extend(
                [("right roll", 2.0), ("left roll", 2.0)]
            )
        expected_transitions.extend(
            [("standard", 2.0), ("captured prone", 3.0)]
        )
        self.assertEqual(transitions, expected_transitions)

        expected_holds = [("standard", 2.0)]
        for _cycle in range(3):
            expected_holds.extend(
                [("right roll", 2.0), ("left roll", 2.0)]
            )
        expected_holds.extend([("standard", 2.0), ("captured prone", 2.0)])
        self.assertEqual(holds, expected_holds)
        controller._neutralize.assert_called_once_with(1.0)


if __name__ == "__main__":
    unittest.main()
