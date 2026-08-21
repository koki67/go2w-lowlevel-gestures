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
import go2w_low_to_high_sequence_sim as low_to_high
import go2w_roll_sequence_sim as roll


class SimulationContractTests(unittest.TestCase):
    def test_height_targets_are_owned_by_hardware_controller(self):
        self.assertEqual(height.STANDARD, hardware.STANDARD)
        self.assertEqual(height.LOW, hardware.LOW)
        self.assertEqual(height.HIGH, hardware.HIGH)
        self.assertIs(low_to_high.base, height)

    def test_roll_targets_match_hardware_controller(self):
        amplitude, right, left = roll.make_roll_targets(
            hardware.HIP_LIMIT_LOWER_RAD,
            hardware.HIP_LIMIT_UPPER_RAD,
        )
        self.assertAlmostEqual(amplitude, hardware.ROLL_AMPLITUDE_RAD)
        self.assertEqual(right, hardware.ROLL_RIGHT)
        self.assertEqual(left, hardware.ROLL_LEFT)

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
