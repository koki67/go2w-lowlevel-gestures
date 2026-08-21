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


if __name__ == "__main__":
    unittest.main()
