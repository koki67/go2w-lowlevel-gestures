import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from qualification import go2w_qualify_live as qualifier


SHA = "a" * 40


class QualificationRunnerTests(unittest.TestCase):
    def test_wbc_checklist_limits_live_scope_to_one_belly_clear_pose(self):
        run = mock.Mock(controller="wbc")

        qualifier.physical_checklist(run)

        emitted = "\n".join(call.args[0] for call in run.emit.call_args_list)
        self.assertIn("もっとも安定した単一の初期姿勢", emitted)
        self.assertIn("腹部は床にも支持具にも接触していない", emitted)
        self.assertIn("腹部荷重姿勢は今回の実機試験対象に含めない", emitted)

    def test_published_main_is_the_default_qualification_branch(self):
        args = qualifier.parse_args(
            ["--controller", "wbc", "--gesture", "height"]
        )

        self.assertEqual(qualifier.EXPECTED_BRANCH, "main")
        self.assertEqual(args.expected_branch, "main")
        self.assertFalse(hasattr(args, "expected_sha"))

    def common_patches(self):
        return (
            mock.patch.object(
                qualifier,
                "check_repository",
                return_value=(qualifier.EXPECTED_BRANCH, SHA),
            ),
            mock.patch.object(
                qualifier,
                "check_platform_and_network",
                return_value=qualifier.DEFAULT_EXPECTED_IP,
            ),
        )

    def latest_summary(self, output_root):
        paths = list(Path(output_root).glob("*/qualification.summary.json"))
        self.assertEqual(len(paths), 1)
        return json.loads(paths[0].read_text(encoding="utf-8"))

    def test_default_mode_runs_read_only_stages_and_never_invokes_live(self):
        with tempfile.TemporaryDirectory() as output_root:
            seen = []

            def run_stage(_self, stage, command, **_kwargs):
                seen.append((stage, tuple(command)))

            repository_patch, platform_patch = self.common_patches()
            with (
                repository_patch,
                platform_patch,
                mock.patch.object(
                    qualifier.QualificationRun,
                    "run_command",
                    autospec=True,
                    side_effect=run_stage,
                ),
            ):
                exit_code = qualifier.main(
                    [
                        "--controller",
                        "adaptive",
                        "--gesture",
                        "height",
                        "--output-root",
                        output_root,
                    ]
                )

            self.assertEqual(exit_code, 0)
            self.assertEqual(
                [stage for stage, _command in seen],
                ["build", "tests", "pip-check", "describe", "read-only-preflight"],
            )
            self.assertFalse(any("live-adaptive-height" in command for _, command in seen))
            summary = self.latest_summary(output_root)
            self.assertEqual(summary["outcome"], "software-preflight-passed-no-live")
            self.assertFalse(summary["live_requested"])
            self.assertFalse(summary["physical_pass"])

    def test_failed_stage_short_circuits_all_following_steps(self):
        with tempfile.TemporaryDirectory() as output_root:
            seen = []

            def run_stage(_self, stage, _command, **_kwargs):
                seen.append(stage)
                if stage == "tests":
                    raise qualifier.QualificationFailure("tests failed")

            repository_patch, platform_patch = self.common_patches()
            with (
                repository_patch,
                platform_patch,
                mock.patch.object(
                    qualifier.QualificationRun,
                    "run_command",
                    autospec=True,
                    side_effect=run_stage,
                ),
            ):
                exit_code = qualifier.main(
                    [
                        "--controller",
                        "wbc",
                        "--gesture",
                        "roll",
                        "--output-root",
                        output_root,
                    ]
                )
            self.assertEqual(exit_code, 1)
            self.assertEqual(seen, ["build", "tests"])
            self.assertIn("tests failed", self.latest_summary(output_root)["error"])

    def test_wrong_confirmation_never_invokes_live_target(self):
        with tempfile.TemporaryDirectory() as output_root:
            seen = []

            def run_stage(_self, stage, command, **_kwargs):
                seen.append((stage, tuple(command)))

            repository_patch, platform_patch = self.common_patches()
            with (
                repository_patch,
                platform_patch,
                mock.patch.object(qualifier.sys.stdin, "isatty", return_value=True),
                mock.patch("builtins.input", return_value="NO"),
                mock.patch.object(
                    qualifier.QualificationRun,
                    "run_command",
                    autospec=True,
                    side_effect=run_stage,
                ),
            ):
                exit_code = qualifier.main(
                    [
                        "--controller",
                        "wbc",
                        "--gesture",
                        "height",
                        "--output-root",
                        output_root,
                        "--live",
                    ]
                )
            self.assertEqual(exit_code, 1)
            self.assertNotIn("live", [stage for stage, _command in seen])
            self.assertIn("did not match", self.latest_summary(output_root)["error"])

    def test_matching_confirmation_invokes_exactly_one_matching_live_target(self):
        with tempfile.TemporaryDirectory() as output_root:
            seen = []

            def run_stage(_self, stage, command, **kwargs):
                seen.append((stage, tuple(command), kwargs))

            repository_patch, platform_patch = self.common_patches()
            phrase = qualifier.LIVE_CONFIRMATIONS[("wbc", "roll")]
            with (
                repository_patch,
                platform_patch,
                mock.patch.object(qualifier.sys.stdin, "isatty", return_value=True),
                mock.patch("builtins.input", return_value=phrase),
                mock.patch.object(
                    qualifier,
                    "relative_controller_log_dir",
                    return_value="runs/qualification/test/controller",
                ),
                mock.patch.object(
                    qualifier.QualificationRun,
                    "run_command",
                    autospec=True,
                    side_effect=run_stage,
                ),
            ):
                exit_code = qualifier.main(
                    [
                        "--controller",
                        "wbc",
                        "--gesture",
                        "roll",
                        "--output-root",
                        output_root,
                        "--live",
                    ]
                )
            self.assertEqual(exit_code, 0)
            live_commands = [
                (command, kwargs)
                for stage, command, kwargs in seen
                if stage == "live"
            ]
            self.assertEqual(len(live_commands), 1)
            command, kwargs = live_commands[0]
            self.assertIn("live-wbc-roll", command)
            self.assertTrue(kwargs["interactive_output"])
            self.assertFalse(any("no-tracking-stop" in part for part in command))
            summary = self.latest_summary(output_root)
            self.assertNotIn("expected_sha", summary)
            self.assertEqual(
                summary["remote_tracking_ref"], "refs/remotes/origin/main"
            )
            self.assertEqual(
                summary["physical_initial_pose_scope"],
                "single-proven-stable-four-wheel-loaded-belly-clear",
            )
            self.assertFalse(summary["alternate_initial_pose_cases_planned"])

    def test_dirty_repository_is_rejected_without_mutation(self):
        with mock.patch.object(
            qualifier, "capture", return_value=" M important.py"
        ) as capture:
            with self.assertRaisesRegex(qualifier.QualificationFailure, "dirty"):
                qualifier.check_repository(qualifier.EXPECTED_BRANCH)
        capture.assert_called_once_with(
            ("git", "status", "--porcelain", "--untracked-files=normal")
        )

    def test_remote_tracking_sha_must_match_current_head(self):
        with mock.patch.object(
            qualifier,
            "capture",
            side_effect=("", qualifier.EXPECTED_BRANCH, SHA, "b" * 40),
        ):
            with self.assertRaisesRegex(
                qualifier.QualificationFailure, "remote-tracking SHA mismatch"
            ):
                qualifier.check_repository(qualifier.EXPECTED_BRANCH)


if __name__ == "__main__":
    unittest.main()
