from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock
import xml.etree.ElementTree as ET


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SIMULATION_DIR = WORKSPACE_ROOT / "simulation"
if str(SIMULATION_DIR) not in sys.path:
    sys.path.insert(0, str(SIMULATION_DIR))

import go2w_gesture_real as hardware
import go2w_height_sequence_sim as height
import go2w_quick_stand_sequence_sim as quick_stand
import go2w_roll_sequence_sim as roll
import go2w_shake_off_sequence_sim as shake_off
import go2w_closed_loop_sequence_sim as closed_loop


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
                "all",
            ],
        ):
            args = closed_loop.parse_args()
        self.assertEqual(args.controller, "wbc")
        self.assertEqual(args.gesture, "roll")
        self.assertEqual(args.initial, "all")

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
            def __init__(self, initial_condition):
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
