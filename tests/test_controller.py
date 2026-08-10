import threading
import time
import unittest
from unittest import mock

import go2w_height_sequence_real as controller_module


PRONE = [0.0, 1.36, -2.65] * 4


class FakePublisher:
    def __init__(self):
        self.commands = []
        self.fail = False

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

    def test_dry_run_never_creates_publisher_or_changes_mode(self):
        controller = controller_module.HardwareSequenceController(
            "eth0", "192.168.123.18"
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

    def test_ip_mismatch_fails_before_dds(self):
        controller = controller_module.HardwareSequenceController(
            "eth0", "192.168.123.18"
        )
        controller._initialize_dds = mock.Mock()

        with mock.patch.object(
            controller_module, "interface_ipv4", return_value="192.168.123.99"
        ):
            with self.assertRaisesRegex(RuntimeError, "refusing DDS initialization"):
                controller.run(live=False)

        controller._initialize_dds.assert_not_called()

    def test_pose_and_neutral_command_fields(self):
        controller = controller_module.HardwareSequenceController(
            "eth0", "192.168.123.18"
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

    def test_dds_write_failure_is_not_ignored(self):
        controller = controller_module.HardwareSequenceController(
            "eth0", "192.168.123.18"
        )
        publisher = FakePublisher()
        publisher.fail = True
        controller._publisher = publisher

        with self.assertRaisesRegex(RuntimeError, "DDS write failed"):
            controller._write_pose(controller_module.STANDARD)

    def test_accelerated_full_command_sequence(self):
        controller = controller_module.HardwareSequenceController(
            "eth0", "192.168.123.18"
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
            "HEIGHT_TRANSITION_S": 0.01,
            "PRONE_TRANSITION_S": 0.02,
            "STANDARD_HOLD_S": 0.01,
            "HEIGHT_HOLD_S": 0.01,
            "PRONE_HOLD_S": 0.01,
            "NEUTRAL_COMMAND_S": 0.01,
            "HEIGHT_CYCLES": 1,
        }

        thread = threading.Thread(target=feed_state, daemon=True)
        thread.start()
        try:
            with mock.patch.multiple(controller_module, **timing_overrides):
                controller._run_height_sequence()
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

        self.assertTrue(pose_was_commanded(controller_module.STANDARD))
        self.assertTrue(pose_was_commanded(controller_module.LOW))
        self.assertTrue(pose_was_commanded(controller_module.HIGH))
        self.assertTrue(pose_was_commanded(PRONE))
        self.assertTrue(controller._ended_prone)
        self.assertTrue(controller._neutralized)


if __name__ == "__main__":
    unittest.main()
