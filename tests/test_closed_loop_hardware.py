import inspect
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import time
import unittest
from unittest import mock

import go2w_gesture_real as base
import go2w_gesture_real_adaptive as adaptive
import go2w_gesture_real_wbc as wbc


PRONE = [0.0, 1.36, -2.65] * 4


def extended_sample(pose=None):
    return base.StateSample(
        received_at=time.monotonic(),
        pose=list(base.STANDARD if pose is None else pose),
        leg_velocity=[0.0] * 12,
        wheel_velocity=[0.0] * 4,
        rpy=[0.0, 0.0, 0.0],
        tau_est=[0.0] * 16,
        motor_mode=[1] * 16,
        motor_lost=[0] * 16,
        temperature=[30.0] * 16,
        gyro=[0.0] * 3,
        acceleration=[0.0, 0.0, 9.81],
        power_v=30.0,
        power_a=1.0,
    )


class AdaptiveHardwareTests(unittest.TestCase):
    def make_controller(self, gesture):
        controller = object.__new__(adaptive.AdaptiveGestureController)
        controller.gesture = gesture
        controller.timing = base.FAST_TIMING
        controller._captured_prone = list(PRONE)
        controller._ended_prone = False
        controller._capture_extended_baseline = mock.Mock()
        controller._latest_sample = mock.Mock(return_value=extended_sample(PRONE))
        transitions = []
        holds = []

        def transition(name, _source, target, duration, timeout, **kwargs):
            transitions.append((name, duration, timeout, dict(kwargs)))
            return list(target)

        def hold(name, _pose, duration, **kwargs):
            holds.append((name, duration, dict(kwargs)))

        controller._adaptive_transition = mock.Mock(side_effect=transition)
        controller._adaptive_hold = mock.Mock(side_effect=hold)
        controller._finish_adaptive_at_prone = mock.Mock()
        return controller, transitions, holds

    def test_height_and_roll_each_schedule_three_fast_cycles_then_prone(self):
        for gesture, runner, side_names in (
            ("height", "_run_height_sequence", ("low", "high")),
            ("roll", "_run_roll_sequence", ("right roll", "left roll")),
        ):
            with self.subTest(gesture=gesture):
                controller, transitions, holds = self.make_controller(gesture)
                getattr(controller, runner)()
                repeated_transitions = [
                    item for item in transitions if item[0] in side_names
                ]
                repeated_holds = [item for item in holds if item[0] in side_names]
                self.assertEqual(len(repeated_transitions), 6)
                self.assertEqual(len(repeated_holds), 6)
                self.assertTrue(all(item[1] == 1.0 for item in repeated_transitions))
                self.assertTrue(all(item[1] == 0.5 for item in repeated_holds))
                controller._finish_adaptive_at_prone.assert_called_once()

    def test_mode_change_and_lost_increment_fail_closed(self):
        controller = object.__new__(adaptive.AdaptiveGestureController)
        controller._expected_motor_modes = [1] * 16
        controller._initial_motor_lost = [0] * 16
        changed = extended_sample()
        changed.motor_mode[7] = 2
        with self.assertRaisesRegex(base.ControlledReturnRequested, "mode changed"):
            controller._check_extended_runtime(changed)

        lost = extended_sample()
        lost.motor_lost[4] = 1
        with self.assertRaisesRegex(base.ControlledReturnRequested, "lost counter"):
            controller._check_extended_runtime(lost)

    def test_closed_loop_preflight_rejects_missing_extended_lowstate(self):
        controller = object.__new__(adaptive.AdaptiveGestureController)
        controller._latest_sample = mock.Mock(
            return_value=base.StateSample(
                received_at=time.monotonic(),
                pose=list(base.STANDARD),
                leg_velocity=[0.0] * 12,
                wheel_velocity=[0.0] * 4,
                rpy=[0.0, 0.0, 0.0],
            )
        )
        with self.assertRaisesRegex(RuntimeError, "extended LowState telemetry"):
            controller._validate_preflight_state()

    def test_telemetry_record_only_buffers_and_tau_command_remains_zero(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = adaptive.ClosedLoopTelemetryRecorder(
                directory, "height", adaptive.CONTROLLER_TYPE
            )
            sample = extended_sample()
            with mock.patch("builtins.open", side_effect=AssertionError("file I/O")):
                recorder.record(
                    sample,
                    base.STANDARD,
                    phase="test",
                    phase_elapsed_s=0.1,
                    progress=0.2,
                    speed_scale=1.0,
                    tracking_ratio=0.1,
                    torque_ratio=0.2,
                    deadline_miss_count=0,
                    consecutive_deadline_misses=0,
                )
            self.assertEqual(len(recorder.rows), 1)
            _csv_path, summary_path = recorder.finalize("completed")
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(summary["power_v"]["minimum"], 30.0)
            self.assertEqual(summary["power_a"]["maximum"], 1.0)
            self.assertEqual(
                summary["temperature_raw_by_motor"]["FR_hip"]["maximum"],
                30.0,
            )
            self.assertEqual(summary["wbc"]["qp_solve_count"], 0)

        base.load_unitree_sdk()
        controller = adaptive.AdaptiveGestureController(
            "eth0", "192.168.123.18", "height"
        )
        publisher = mock.Mock()
        publisher.Write.return_value = True
        controller._publisher = publisher
        controller._write_pose(base.STANDARD)
        command = publisher.Write.call_args.args[0]
        self.assertEqual([command.motor_cmd[index].tau for index in range(16)], [0.0] * 16)

    def test_500_hz_control_functions_have_no_filesystem_writes(self):
        functions = (
            adaptive.AdaptiveGestureController._adaptive_phase,
            wbc.WBCGestureController._run_wbc_phase,
            wbc.WBCGestureController._wait_for_valid_contact_estimate,
            wbc.WBCGestureController._hold_while_confirming_support,
        )
        forbidden = (".open(", "write_text(", "json.dump(", "csv.writer(")
        for function in functions:
            source = inspect.getsource(function)
            with self.subTest(function=function.__name__):
                for token in forbidden:
                    self.assertNotIn(token, source)


class WBCHardwareTests(unittest.TestCase):
    def make_controller(self, gesture):
        controller = object.__new__(wbc.WBCGestureController)
        controller.gesture = gesture
        controller.timing = base.FAST_TIMING
        controller._captured_prone = list(PRONE)
        controller._baseline_height_m = None
        controller._baseline_rpy = None
        controller._capture_extended_baseline = mock.Mock()
        controller._latest_sample = mock.Mock(return_value=extended_sample(PRONE))
        controller._adaptive_transition = mock.Mock(
            side_effect=lambda _name, _source, target, _duration, _timeout, **_kwargs: list(target)
        )
        controller._adaptive_hold = mock.Mock()

        def contact_gate(_q_ref):
            controller._baseline_height_m = 0.25
            controller._baseline_rpy = [0.0, 0.0, 0.0]

        controller._wait_for_valid_contact_estimate = mock.Mock(side_effect=contact_gate)
        controller._hold_while_confirming_support = mock.Mock()
        controller._run_wbc_phase = mock.Mock(
            side_effect=lambda _name, _current, _posture_source, posture_target, *_args: list(posture_target)
        )
        controller._finish_adaptive_at_prone = mock.Mock()
        return controller

    def test_height_and_roll_wbc_each_schedule_three_cycles_and_adaptive_return(self):
        for gesture in ("height", "roll"):
            with self.subTest(gesture=gesture):
                controller = self.make_controller(gesture)
                controller._run_wbc_sequence()
                self.assertEqual(controller._run_wbc_phase.call_count, 12)
                phase_names = [
                    call.args[0] for call in controller._run_wbc_phase.call_args_list
                ]
                self.assertEqual(
                    sum(name.startswith("transition") for name in phase_names), 6
                )
                self.assertEqual(sum(name.startswith("hold") for name in phase_names), 6)
                controller._finish_adaptive_at_prone.assert_called_once()
                controller._wait_for_valid_contact_estimate.assert_called_once()
                controller._hold_while_confirming_support.assert_called_once()

    def test_contact_estimator_receives_joint_torque_and_imu_attitude(self):
        controller = object.__new__(wbc.WBCGestureController)
        controller._latest_contact_estimate = None
        sample = extended_sample()
        expected = mock.sentinel.contact
        with mock.patch.object(
            wbc.closed_loop, "estimate_contact_forces", return_value=expected
        ) as estimator:
            result = controller._estimate_contacts(sample)
        self.assertIs(result, expected)
        estimator.assert_called_once_with(sample.pose, sample.tau_est[:12], sample.rpy)

    def test_wbc_description_is_explicitly_kinematic_position_pd(self):
        with mock.patch("builtins.print") as printer:
            wbc.print_wbc_plan("height")
        text = "\n".join(" ".join(str(value) for value in call.args) for call in printer.call_args_list)
        self.assertIn("position PD", text)
        self.assertIn("not a direct-torque dynamic WBC", text)
        self.assertIn(wbc.SUPPORT_CONFIRMATION, text)

    def test_five_consecutive_500_hz_deadline_misses_request_return(self):
        controller = object.__new__(wbc.WBCGestureController)
        controller._deadline_miss_count = 0
        controller._consecutive_deadline_misses = 0
        with mock.patch.object(wbc.time, "monotonic", return_value=10.0):
            for _index in range(wbc.MAX_CONSECUTIVE_DEADLINE_MISSES - 1):
                controller._deadline_sleep(9.0)
            with self.assertRaisesRegex(
                base.ControlledReturnRequested, "deadline missed"
            ):
                controller._deadline_sleep(9.0)


if __name__ == "__main__":
    unittest.main()
