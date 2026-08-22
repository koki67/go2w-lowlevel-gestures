from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock
import xml.etree.ElementTree as ET

import numpy as np


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SIMULATION_DIR = WORKSPACE_ROOT / "simulation"
if str(SIMULATION_DIR) not in sys.path:
    sys.path.insert(0, str(SIMULATION_DIR))

import go2w_gesture_real as hardware
import go2w_height_sequence_sim as height
import go2w_quick_stand_sequence_sim as quick_stand
import go2w_roll_sequence_sim as roll
import go2w_shake_off_sequence_sim as shake_off
import go2w_adaptive_plot as adaptive_plot
import go2w_closed_loop_sequence_sim as closed_loop
import go2w_wbc_plot as wbc_plot


class SimulationContractTests(unittest.TestCase):
    def test_plot_saving_is_cli_opt_in_for_every_sequence(self):
        for module in (height, roll, quick_stand, shake_off):
            with self.subTest(module=module.__name__, mode="default"):
                with mock.patch.object(sys, "argv", [module.__name__]):
                    self.assertFalse(module.parse_args().save_plot)
            with self.subTest(module=module.__name__, mode="save"):
                with mock.patch.object(
                    sys, "argv", [module.__name__, "--save-plot"]
                ):
                    self.assertTrue(module.parse_args().save_plot)

        with mock.patch.object(sys, "argv", ["closed-loop"]):
            self.assertFalse(closed_loop.parse_args().save_plot)
        with mock.patch.object(
            sys,
            "argv",
            [
                "closed-loop",
                "--controller",
                "adaptive",
                "--gesture",
                "height",
                "--save-plot",
            ],
        ):
            args = closed_loop.parse_args()
        self.assertTrue(args.save_plot)
        self.assertIsNone(closed_loop.argument_error(args))

    def test_closed_loop_plot_accepts_wbc_controller_diagnostics(self):
        with mock.patch.object(
            sys,
            "argv",
            [
                "closed-loop",
                "--controller",
                "wbc",
                "--gesture",
                "height",
                "--save-plot",
            ],
        ):
            args = closed_loop.parse_args()
        self.assertTrue(args.save_plot)
        self.assertIsNone(closed_loop.argument_error(args))

    def test_tracking_samples_are_only_buffered_when_plot_saving_is_enabled(self):
        def make_controller(save_plot):
            controller = object.__new__(height.SequenceController)
            controller._save_plot = save_plot
            controller._tracking_complete = False
            controller._tracking_start_time = None
            controller._tracking_times = []
            controller._tracking_targets = []
            controller._tracking_actuals = []
            controller._low_state = mock.Mock(
                motor_state=[mock.Mock(q=0.0) for _ in range(12)]
            )
            controller._lock = mock.MagicMock()
            return controller

        default_controller = make_controller(False)
        default_controller._record_tracking_sample(height.STANDARD)
        self.assertEqual(default_controller._tracking_times, [])
        self.assertEqual(default_controller._tracking_targets, [])
        self.assertEqual(default_controller._tracking_actuals, [])

        saving_controller = make_controller(True)
        with mock.patch.object(height.time, "monotonic", return_value=100.0):
            saving_controller._record_tracking_sample(height.STANDARD)
        self.assertEqual(saving_controller._tracking_times, [0.0])
        self.assertEqual(saving_controller._tracking_targets, [height.STANDARD])
        self.assertEqual(len(saving_controller._tracking_actuals), 1)

    def test_adaptive_plot_recording_is_opt_in_and_writes_two_valid_svgs(self):
        names = tuple("joint_{}".format(index) for index in range(12))
        envelopes = (0.2,) * 12
        disabled = adaptive_plot.AdaptivePlotRecorder(False, names, envelopes)
        disabled.record(
            time_s=0.0,
            phase="startup-standard",
            phase_elapsed_s=0.0,
            phase_duration_s=2.0,
            progress=0.0,
            speed_scale=1.0,
            tracking_ratio=0.0,
            torque_ratio=0.0,
            q_ref=(0.0,) * 12,
            q_measured=(0.0,) * 12,
        )
        self.assertEqual(disabled.samples, [])

        recorder = adaptive_plot.AdaptivePlotRecorder(True, names, envelopes)
        for index in range(8):
            phase = "startup-standard" if index < 4 else "transition-1-low"
            local_index = index if index < 4 else index - 4
            reference = tuple(0.1 * index for _ in range(12))
            measured = tuple(
                value - (0.02 if joint == index % 12 else 0.0)
                for joint, value in enumerate(reference)
            )
            recorder.record(
                time_s=0.1 * index,
                phase=phase,
                phase_elapsed_s=0.1 * local_index,
                phase_duration_s=0.3,
                progress=min(1.0, 0.25 * local_index),
                speed_scale=1.0 if index < 2 else 0.5,
                tracking_ratio=0.1 * index,
                torque_ratio=0.05 * index,
                q_ref=reference,
                q_measured=measured,
                event=(
                    "phase wall timeout exceeded 8.000 s" if index == 7 else None
                ),
            )

        with tempfile.TemporaryDirectory() as temporary_directory:
            artifacts = recorder.write(Path(temporary_directory), "adaptive-test")
            joint_path = Path(artifacts["joint_tracking_svg"])
            governor_path = Path(artifacts["adaptive_governor_svg"])
            self.assertTrue(joint_path.is_file())
            self.assertTrue(governor_path.is_file())
            self.assertEqual(artifacts["sample_count"], 8)
            ET.parse(joint_path)
            ET.parse(governor_path)
            joint_svg = joint_path.read_text(encoding="utf-8")
            governor_svg = governor_path.read_text(encoding="utf-8")
            self.assertIn("adaptive target", joint_svg)
            self.assertIn("joint_11", joint_svg)
            self.assertIn("Phase progress", governor_svg)
            self.assertIn("time-only nominal", governor_svg)
            self.assertIn("progress stop", governor_svg)
            self.assertIn("move 1-low", governor_svg)
            self.assertIn("phase wall timeout exceeded 8.000 s", governor_svg)

    def test_run_case_attaches_requested_adaptive_plot_artifacts(self):
        class FakeHarness:
            def __init__(self, initial_condition, **kwargs):
                self.initial_condition = initial_condition
                self.recording_requested = kwargs["record_adaptive_plot"]

            def prepare(self):
                return [0.0] * 12

            def adaptive_sequence(self, _gesture, _captured):
                pass

            def write_adaptive_plots(self, output_dir, stem):
                self.plot_call = (Path(output_dir), stem)
                return {
                    "sample_count": 123,
                    "joint_tracking_svg": "/tmp/joints.svg",
                    "adaptive_governor_svg": "/tmp/governor.svg",
                }

            def summary(self, _controller, _gesture, _captured, error=None):
                return {"simulation_pass": error is None, "error": error}

            def close(self):
                pass

        with mock.patch.object(closed_loop, "HeadlessHarness", FakeHarness):
            summary = closed_loop.run_case(
                "adaptive",
                "height",
                "normal",
                save_plot=True,
                plot_output_dir="/tmp/output",
                plot_stem="run-normal",
            )
        artifacts = summary["plot_artifacts"]
        self.assertTrue(artifacts["requested"])
        self.assertEqual(artifacts["sample_count"], 123)
        self.assertEqual(artifacts["generation_error"], None)
        self.assertEqual(artifacts["adaptive_governor_svg"], "/tmp/governor.svg")

    def test_wbc_plot_recording_is_opt_in_and_writes_five_valid_svgs(self):
        names = tuple("joint_{}".format(index) for index in range(12))
        envelopes = (0.2,) * 12
        torque_limits = (23.7,) * 8 + (45.43,) * 4

        def make_recorder(enabled):
            return wbc_plot.WBCPlotRecorder(
                enabled,
                names,
                envelopes,
                torque_limits,
                body_weight_n=187.63,
                tilt_limit_rad=0.55,
                max_commanded_dq_rad_s=1.0,
                max_commanded_ddq_rad_s2=4.0,
                wbc_period_s=0.01,
            )

        contact = SimpleNamespace(
            forces=(
                (1.0, 0.0, 45.0),
                (-1.0, 0.0, 49.0),
                (0.5, 0.0, 44.0),
                (-0.5, 0.0, 49.63),
            ),
            valid=True,
            reason=None,
            total_vertical_load_n=187.63,
            minimum_normal_load_n=44.0,
            torque_residual_ratio=0.08,
            balance_residual_ratio=0.06,
            max_jacobian_condition=15.0,
            solve_time_s=0.001,
            iterations=75,
        )
        task_target = SimpleNamespace(
            relative_height_m=0.02,
            roll_rad=0.20,
            pitch_rad=0.0,
            yaw_rad=0.0,
        )
        task_estimate = SimpleNamespace(
            relative_height_m=0.018,
            roll_rad=0.19,
            pitch_rad=0.01,
            yaw_rad=0.005,
        )
        qp = SimpleNamespace(
            generalized_velocity=(0.0,) * 6 + (0.1,) * 12,
            valid=True,
            reason=None,
            solve_time_s=0.003,
            iterations=100,
            primal_residual=1.0e-5,
            dual_residual=2.0e-5,
            contact_velocity_residual_m_s=0.002,
        )
        record_arguments = {
            "time_s": 0.0,
            "phase": "transition-1-right",
            "q_ref": (0.1,) * 12,
            "q_measured": (0.09,) * 12,
            "measured_dq": (0.02,) * 12,
            "tau_est": (2.0,) * 12,
            "body_rpy": (0.19, 0.01, 0.005),
            "progress": 0.5,
            "speed_scale": 0.8,
            "task_target": task_target,
            "task_estimate": task_estimate,
            "contact": contact,
            "qp_result": qp,
            "support_y_m": (-0.15, 0.15, -0.15, 0.15),
        }
        disabled = make_recorder(False)
        disabled.record(**record_arguments)
        self.assertEqual(disabled.samples, [])

        recorder = make_recorder(True)
        for index in range(8):
            arguments = dict(record_arguments)
            arguments["time_s"] = 0.01 * index
            if index >= 4:
                arguments["phase"] = "hold-1-right"
            if index == 7:
                arguments["event"] = "contact balance residual exceeded limit"
            recorder.record(**arguments)

        with tempfile.TemporaryDirectory() as temporary_directory:
            artifacts = recorder.write(Path(temporary_directory), "wbc-test")
            expected_keys = (
                "joint_tracking_svg",
                "wbc_task_tracking_svg",
                "wbc_contact_support_svg",
                "wbc_contact_qp_health_svg",
                "wbc_solver_safety_svg",
            )
            self.assertEqual(artifacts["sample_count"], 8)
            for key in expected_keys:
                path = Path(artifacts[key])
                self.assertTrue(path.is_file())
                ET.parse(path)
            self.assertIn(
                "WBC target",
                Path(artifacts["joint_tracking_svg"]).read_text(encoding="utf-8"),
            )
            self.assertIn(
                "WBC task-space tracking",
                Path(artifacts["wbc_task_tracking_svg"]).read_text(
                    encoding="utf-8"
                ),
            )
            self.assertIn(
                "Estimated lateral CoP",
                Path(artifacts["wbc_contact_support_svg"]).read_text(
                    encoding="utf-8"
                ),
            )
            self.assertIn(
                "25% limit",
                Path(artifacts["wbc_contact_qp_health_svg"]).read_text(
                    encoding="utf-8"
                ),
            )
            self.assertIn(
                "runtime stop",
                Path(artifacts["wbc_solver_safety_svg"]).read_text(
                    encoding="utf-8"
                ),
            )

    def test_wbc_runtime_plot_samples_are_limited_to_the_100_hz_control_rate(self):
        harness = object.__new__(closed_loop.HeadlessHarness)
        harness._wbc_plot_time_origin_s = 0.0
        harness._last_wbc_plot_sample_time_s = None
        harness.data = SimpleNamespace(time=0.0)
        recorder = mock.Mock(enabled=True)
        harness._wbc_plot_recorder = recorder
        fake_control = SimpleNamespace(
            WBC_PERIOD_S=0.01,
            wheel_positions=lambda _q: np.zeros((4, 3)),
            rpy_rotation=lambda _rpy: np.eye(3),
        )
        state = (
            [0.0] * 12,
            [0.0] * 12,
            [0.0] * 12,
            [0.0] * 12,
            [0.0] * 3,
        )
        with mock.patch.object(closed_loop, "control", fake_control):
            harness._record_wbc_plot_sample("phase", *state)
            harness.data.time = 0.002
            harness._record_wbc_plot_sample("phase", *state)
            harness.data.time = 0.010
            harness._record_wbc_plot_sample("phase", *state)
            harness.data.time = 0.012
            harness._record_wbc_plot_sample("phase", *state, event="failure")
        self.assertEqual(recorder.record.call_count, 3)

    def test_run_case_attaches_requested_wbc_plot_artifacts(self):
        class FakeHarness:
            def __init__(self, initial_condition, **kwargs):
                self.initial_condition = initial_condition
                self.wbc_recording_requested = kwargs["record_wbc_plot"]
                self.adaptive_recording_requested = kwargs[
                    "record_adaptive_plot"
                ]

            def prepare(self):
                return [0.0] * 12

            def wbc_sequence(self, _gesture, _captured):
                pass

            def write_wbc_plots(self, output_dir, stem):
                self.plot_call = (Path(output_dir), stem)
                return {
                    "sample_count": 456,
                    "joint_tracking_svg": "/tmp/wbc-joints.svg",
                    "wbc_task_tracking_svg": "/tmp/wbc-task.svg",
                    "wbc_contact_support_svg": "/tmp/wbc-support.svg",
                    "wbc_contact_qp_health_svg": "/tmp/wbc-contact.svg",
                    "wbc_solver_safety_svg": "/tmp/wbc-solver.svg",
                }

            def summary(self, _controller, _gesture, _captured, error=None):
                return {"simulation_pass": error is None, "error": error}

            def close(self):
                pass

        with mock.patch.object(closed_loop, "HeadlessHarness", FakeHarness):
            summary = closed_loop.run_case(
                "wbc",
                "roll",
                "normal",
                save_plot=True,
                plot_output_dir="/tmp/output",
                plot_stem="run-normal",
            )
        artifacts = summary["plot_artifacts"]
        self.assertTrue(artifacts["requested"])
        self.assertEqual(artifacts["sample_count"], 456)
        self.assertEqual(artifacts["generation_error"], None)
        self.assertEqual(artifacts["wbc_task_tracking_svg"], "/tmp/wbc-task.svg")

    def test_height_targets_are_owned_by_hardware_controller(self):
        self.assertEqual(height.STANDARD, hardware.STANDARD)
        self.assertEqual(height.LOW, hardware.LOW)
        self.assertEqual(height.HIGH, hardware.HIGH)
        self.assertIs(quick_stand.base, height)

    def test_quick_stand_transitions_from_low_to_high_in_point_one_seconds(self):
        self.assertEqual(quick_stand.QUICK_STAND_TRANSITION_S, 0.1)

    def test_roll_targets_match_hardware_controller(self):
        amplitude, right, left = roll.make_roll_targets(
            hardware.HIP_LIMIT_LOWER_RAD,
            hardware.HIP_LIMIT_UPPER_RAD,
        )
        self.assertAlmostEqual(amplitude, hardware.ROLL_AMPLITUDE_RAD)
        self.assertEqual(right, hardware.ROLL_RIGHT)
        self.assertEqual(left, hardware.ROLL_LEFT)

    def test_shake_off_reuses_roll_targets_with_faster_timing(self):
        self.assertIs(shake_off.roll, roll)
        self.assertEqual(shake_off.SHAKE_TRANSITION_S, 0.10)
        self.assertEqual(shake_off.SHAKE_HOLD_S, 0.03)
        self.assertEqual(shake_off.SHAKE_CYCLES, 8)
        self.assertLess(shake_off.SHAKE_TRANSITION_S, roll.ROLL_TRANSITION_S)
        self.assertLess(shake_off.SHAKE_HOLD_S, roll.ROLL_HOLD_S)

    def test_roll_controller_logs_short_holds_without_rounding_to_zero(self):
        controller = object.__new__(roll.RollSequenceController)
        controller._lock = mock.MagicMock()
        controller._base_height = None
        controller._run_for = mock.Mock()
        publisher = mock.sentinel.publisher
        pose = [0.0] * 12

        with mock.patch("builtins.print") as print_mock:
            controller._hold(publisher, "right roll limit", pose, 0.03)

        print_mock.assert_called_once_with(
            "hold right roll limit (0.03 s), base_z=unknown", flush=True
        )
        run_args = controller._run_for.call_args.args
        self.assertEqual(run_args[:2], (publisher, 0.03))
        self.assertIs(run_args[2](0.5), pose)

    def test_flat_scene_is_a_floor_only_go2w_scene(self):
        root = ET.parse(height.FLAT_SCENE).getroot()
        include = root.find("include")
        self.assertIsNotNone(include)
        self.assertEqual(include.attrib["file"], "go2w.xml")
        geoms = root.findall("./worldbody/geom")
        self.assertEqual(len(geoms), 1)
        self.assertEqual(geoms[0].attrib["name"], "floor")
        self.assertEqual(geoms[0].attrib["type"], "plane")

    def test_runtime_scene_bundle_links_external_model_without_mutating_it(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary_root = Path(temporary_directory)
            simulator = Path("/bin/true")
            python = Path(sys.executable)
            model_dir = temporary_root / "unitree_robots" / "go2w"
            model_xml = model_dir / "go2w.xml"
            assets = model_dir / "assets"
            assets.mkdir(parents=True, exist_ok=True)
            model_xml.write_text("<mujoco/>", encoding="utf-8")

            controller = object.__new__(height.SequenceController)
            controller._scene_workspace = None
            with (
                mock.patch.object(height, "UNITREE_MUJOCO_ROOT", temporary_root),
                mock.patch.object(height, "SIMULATOR", simulator),
                mock.patch.object(height, "VENV_PYTHON", python),
                mock.patch.object(height, "MODEL_XML", model_xml),
                mock.patch.object(height, "MODEL_ASSETS", assets),
            ):
                scene = controller._prepare_flat_scene()
                scene_dir = scene.parent
                self.assertEqual(scene.read_bytes(), height.FLAT_SCENE.read_bytes())
                self.assertEqual((scene_dir / "go2w.xml").resolve(), model_xml)
                self.assertEqual((scene_dir / "assets").resolve(), assets)
                self.assertFalse((model_dir / "scene_flat.xml").exists())
                controller._scene_workspace.cleanup()

    def test_closed_loop_matrix_covers_both_controllers_gestures_and_three_initials(self):
        self.assertEqual(closed_loop.CONTROLLERS, ("adaptive", "wbc"))
        self.assertEqual(closed_loop.GESTURES, ("height", "roll"))
        self.assertEqual(
            closed_loop.INITIAL_CONDITIONS,
            ("normal", "asymmetric-prone", "belly-loaded-prone"),
        )
        with mock.patch.object(
            sys,
            "argv",
            [
                "closed-loop",
                "--controller",
                "wbc",
                "--gesture",
                "roll",
                "--initial",
                "normal",
                "--viewer",
                "--viewer-speed",
                "0.5",
                "--viewer-hold",
            ],
        ):
            args = closed_loop.parse_args()
        self.assertEqual(args.controller, "wbc")
        self.assertEqual(args.gesture, "roll")
        self.assertEqual(args.initial, "normal")
        self.assertTrue(args.viewer)
        self.assertEqual(args.viewer_speed, 0.5)
        self.assertTrue(args.viewer_hold)
        self.assertIsNone(closed_loop.argument_error(args))

    def test_closed_loop_viewer_rejects_multi_case_and_invalid_pacing(self):
        with mock.patch.object(
            sys,
            "argv",
            ["closed-loop", "--viewer", "--initial", "all"],
        ):
            args = closed_loop.parse_args()
        self.assertIn("one explicit", closed_loop.argument_error(args))

        with mock.patch.object(
            sys,
            "argv",
            [
                "closed-loop",
                "--viewer",
                "--initial",
                "normal",
                "--viewer-speed",
                "0",
            ],
        ):
            args = closed_loop.parse_args()
        self.assertIn("positive finite", closed_loop.argument_error(args))

        with mock.patch.object(sys, "argv", ["closed-loop", "--viewer-hold"]):
            args = closed_loop.parse_args()
        self.assertIn("requires --viewer", closed_loop.argument_error(args))

    def test_closed_loop_viewer_syncs_and_paces_against_simulated_time(self):
        harness = object.__new__(closed_loop.HeadlessHarness)
        harness.viewer = mock.Mock()
        harness.viewer.is_running.return_value = True
        harness.viewer_speed = 2.0
        harness._viewer_wall_start = 10.0
        harness._viewer_sim_start = 0.0
        harness.data = mock.Mock(time=0.2)

        with (
            mock.patch.object(closed_loop.time, "monotonic", return_value=10.05),
            mock.patch.object(closed_loop.time, "sleep") as sleep_mock,
        ):
            harness._sync_viewer()

        harness.viewer.sync.assert_called_once_with()
        sleep_mock.assert_called_once()
        self.assertAlmostEqual(sleep_mock.call_args.args[0], 0.05)

        harness.viewer.is_running.return_value = False
        with self.assertRaises(closed_loop.ViewerClosed):
            harness._sync_viewer()

    def test_closed_loop_viewer_hold_handles_terminal_interrupt(self):
        harness = object.__new__(closed_loop.HeadlessHarness)
        harness.viewer = mock.Mock()
        harness.viewer.is_running.return_value = True
        harness.viewer.sync.side_effect = KeyboardInterrupt

        with mock.patch("builtins.print") as print_mock:
            harness.hold_viewer_until_closed()

        self.assertIn(
            "interrupted",
            print_mock.call_args_list[-1].args[0],
        )

    def test_closed_loop_controller_state_uses_imu_sensor_not_ground_truth_pose(self):
        source = Path(closed_loop.__file__).read_text(encoding="utf-8")
        state_source = source[
            source.index("    def state(self):") : source.index(
                "    def imu_gyro(self):"
            )
        ]
        truth_source = source[
            source.index("    def ground_truth(self):") : source.index(
                "    def _update_metrics(self, q_ref):"
            )
        ]
        self.assertIn("sensor[48:52]", state_source)
        self.assertNotIn("self.data.xquat", state_source)
        self.assertIn("xquat", truth_source)

    def test_closed_loop_summary_never_claims_physical_qualification(self):
        harness = object.__new__(closed_loop.HeadlessHarness)
        harness.initial_condition = "normal"
        harness.cycles_completed = 0
        harness.max_tracking_ratio = 0.0
        harness.max_abs_tracking_error_rad = 0.0
        harness.max_tau_ratio = 0.0
        harness.max_controller_tilt_rad = 0.0
        harness.max_ground_truth_tilt_rad = 0.0
        harness.min_ground_truth_wheel_contacts = 4
        harness.min_wbc_ground_truth_wheel_contacts = 4
        harness.max_contact_balance_ratio = 0.0
        harness.max_contact_velocity_residual_m_s = 0.0
        harness.qp_solve_times = []
        harness.hold_endpoints = []
        harness.phase_records = []
        harness.controlled_return_attempted = False
        harness.controlled_return_succeeded = False
        harness.controlled_return_error = None
        harness.state = mock.Mock(
            return_value=(
                __import__("numpy").zeros(12),
                __import__("numpy").zeros(12),
                __import__("numpy").zeros(12),
                [0.0, 0.0, 0.0],
            )
        )
        harness.ground_truth = mock.Mock(return_value={})
        numpy = __import__("numpy")
        with mock.patch.object(closed_loop, "np", numpy):
            summary = harness.summary(
                "adaptive", "height", [0.0] * 12, error="not run"
            )
        self.assertFalse(summary["simulation_pass"])
        self.assertFalse(summary["physical_pass"])
        self.assertEqual(summary["qualification_scope"], "simulation-only")

    def test_closed_loop_failure_attempts_adaptive_captured_prone_return(self):
        class FakeHarness:
            def __init__(self, initial_condition, **_kwargs):
                self.initial_condition = initial_condition
                self.controlled_return_attempted = False
                self.controlled_return_succeeded = False
                self.controlled_return_error = None
                self.return_phases = []

            def prepare(self):
                return [0.0] * 12

            def adaptive_sequence(self, _gesture, _captured):
                raise closed_loop.SimulationFailure("test stop")

            def state(self):
                return (__import__("numpy").zeros(12),) * 3 + ([0.0] * 3,)

            def adaptive_phase(
                self, name, _source, target, _duration, _timeout, **_kwargs
            ):
                self.return_phases.append(name)
                return __import__("numpy").asarray(target, dtype=float)

            def summary(self, _controller, _gesture, _captured, error=None):
                return {
                    "error": error,
                    "controlled_return_attempted": self.controlled_return_attempted,
                    "controlled_return_succeeded": self.controlled_return_succeeded,
                    "return_phases": self.return_phases,
                }

            def close(self):
                pass

        import go2w_closed_loop_control as control_kernel

        with (
            mock.patch.object(closed_loop, "HeadlessHarness", FakeHarness),
            mock.patch.object(closed_loop, "control", control_kernel),
            mock.patch.object(closed_loop, "hardware", hardware),
        ):
            summary = closed_loop.run_case("adaptive", "height", "normal")
        self.assertIn("test stop", summary["error"])
        self.assertTrue(summary["controlled_return_attempted"])
        self.assertTrue(summary["controlled_return_succeeded"])
        self.assertEqual(
            summary["return_phases"],
            ["failure-return-captured-prone", "failure-hold-captured-prone"],
        )


if __name__ == "__main__":
    unittest.main()
