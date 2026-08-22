from contextlib import redirect_stderr
import io
import unittest
from unittest import mock

import go2w_enter_release_mode as release_module


class ReleaseModeTests(unittest.TestCase):
    def _controller(self):
        controller = release_module.ReleaseModeController(
            "eth0", "192.168.123.18"
        )
        controller._motion_switcher = mock.Mock()
        controller._sport_client = mock.Mock()
        controller._initialize_dds = mock.Mock()
        controller._wait_for_lowcmd_quiet = mock.Mock()
        controller._confirm_release = mock.Mock()
        return controller

    def test_describe_does_not_load_sdk(self):
        with mock.patch.object(release_module.base, "load_unitree_sdk") as sdk_load:
            self.assertEqual(release_module.main(["--describe"]), 0)
        sdk_load.assert_not_called()

    def test_ip_mismatch_fails_before_dds(self):
        controller = self._controller()
        with mock.patch.object(
            release_module.base,
            "interface_ipv4",
            return_value="192.168.123.99",
        ):
            with self.assertRaisesRegex(RuntimeError, "refusing DDS initialization"):
                controller.run()
        controller._initialize_dds.assert_not_called()

    def test_active_sport_is_stopped_released_and_left_without_lowcmd(self):
        controller = self._controller()
        controller._motion_switcher.CheckMode.side_effect = [
            (0, {"form": "1", "name": "ai-w"}),
            (0, {"form": "1", "name": "ai-w"}),
            (0, {"form": "", "name": ""}),
            (0, {"form": "", "name": ""}),
        ]
        controller._motion_switcher.ReleaseMode.return_value = (0, None)
        controller._sport_client.StopMove.return_value = 0

        with mock.patch.object(
            release_module.base,
            "interface_ipv4",
            return_value="192.168.123.18",
        ), mock.patch.object(release_module.time, "sleep"), mock.patch.object(
            release_module.base, "ChannelPublisher"
        ) as publisher:
            controller.run()

        controller._confirm_release.assert_called_once_with("1", "ai-w")
        controller._sport_client.StopMove.assert_called_once_with()
        controller._motion_switcher.ReleaseMode.assert_called_once_with()
        self.assertEqual(controller._wait_for_lowcmd_quiet.call_count, 2)
        self.assertTrue(controller.mode_release_attempted)
        self.assertTrue(controller.mode_released)
        publisher.assert_not_called()

    def test_already_released_state_changes_nothing(self):
        controller = self._controller()
        controller._motion_switcher.CheckMode.return_value = (
            0,
            {"form": "", "name": ""},
        )

        with mock.patch.object(
            release_module.base,
            "interface_ipv4",
            return_value="192.168.123.18",
        ):
            controller.run()

        controller._confirm_release.assert_not_called()
        controller._sport_client.StopMove.assert_not_called()
        controller._motion_switcher.ReleaseMode.assert_not_called()
        controller._wait_for_lowcmd_quiet.assert_called_once_with(
            "manual-positioning release"
        )
        self.assertTrue(controller.mode_released)

    def test_confirmation_mismatch_does_not_change_ownership(self):
        controller = release_module.ReleaseModeController(
            "eth0", "192.168.123.18"
        )
        with mock.patch.object(
            release_module.sys.stdin, "isatty", return_value=True
        ), mock.patch("builtins.input", return_value="NO"):
            with self.assertRaisesRegex(RuntimeError, "confirmation did not match"):
                controller._confirm_release("1", "ai-w")
        self.assertFalse(controller.mode_release_attempted)
        self.assertFalse(controller.mode_released)

    def test_lowcmd_sample_restarts_the_full_quiet_window(self):
        controller = release_module.ReleaseModeController(
            "eth0", "192.168.123.18"
        )
        controller._last_lowcmd_time = 0.25

        with mock.patch.object(
            release_module.time,
            "monotonic",
            side_effect=[0.0, 0.0, 0.30, 0.30, 0.76, 0.76],
        ), mock.patch.object(release_module.time, "sleep") as sleep:
            controller._wait_for_lowcmd_quiet("test handoff")

        sleep.assert_called_once_with(0.02)

    def test_release_rpc_failure_is_reported(self):
        controller = self._controller()
        controller._motion_switcher.ReleaseMode.return_value = (3101, None)

        with self.assertRaisesRegex(
            RuntimeError, "ReleaseMode failed on attempt 1: code=3101"
        ):
            controller._release_mode("ai-w")

        self.assertTrue(controller.mode_release_attempted)
        self.assertFalse(controller.mode_released)
        controller._motion_switcher.CheckMode.assert_not_called()

    def test_main_warns_when_failure_occurs_after_release_attempt(self):
        controller = mock.Mock()
        controller.mode_released = False
        controller.mode_release_attempted = True
        controller.run.side_effect = RuntimeError("simulated release failure")
        stderr = io.StringIO()

        with mock.patch.object(
            release_module.base, "load_unitree_sdk"
        ), mock.patch.object(release_module.signal, "signal"), redirect_stderr(stderr):
            exit_code = release_module.main([], controller_class=lambda *_: controller)

        self.assertEqual(exit_code, 1)
        self.assertIn("Sport Mode may be released", stderr.getvalue())

    def test_main_warns_when_interrupted_after_release(self):
        controller = mock.Mock()
        controller.mode_released = True
        controller.mode_release_attempted = True
        controller.run.side_effect = InterruptedError("operator stop")
        stderr = io.StringIO()

        with mock.patch.object(
            release_module.base, "load_unitree_sdk"
        ), mock.patch.object(release_module.signal, "signal"), redirect_stderr(stderr):
            exit_code = release_module.main([], controller_class=lambda *_: controller)

        self.assertEqual(exit_code, 130)
        self.assertIn("Sport Mode may be released", stderr.getvalue())


if __name__ == "__main__":
    unittest.main()
