import importlib.util
from pathlib import Path
import unittest


MODULE_PATH = Path(__file__).resolve().parents[1] / "src" / "for_test" / "ee_parquet_pose_player.py"
spec = importlib.util.spec_from_file_location("ee_parquet_pose_player", MODULE_PATH)
player = importlib.util.module_from_spec(spec)
spec.loader.exec_module(player)


class EeParquetPosePlayerTest(unittest.TestCase):
    def test_quaternion_row_uses_action_columns(self):
        row = {
            "action.left_arm": [1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0, 0.25],
            "action.right_arm": [4.0, 5.0, 6.0, 0.0, 0.0, 0.70710678, 0.70710678, 0.75],
        }

        left, right, triggers = player.parse_arm_pose_row(row, "action")

        self.assertEqual(left.position, [1.0, 2.0, 3.0])
        self.assertEqual(left.quaternion_xyzw, [0.0, 0.0, 0.0, 1.0])
        self.assertEqual(right.position, [4.0, 5.0, 6.0])
        self.assertEqual(right.quaternion_xyzw, [0.0, 0.0, 0.70710678, 0.70710678])
        self.assertEqual(triggers, (0.25, 0.75))

    def test_six_d_payload_contains_position_and_two_rotation_columns(self):
        pose = player.ArmPose(
            position=[1.0, 2.0, 3.0],
            quaternion_xyzw=[0.0, 0.0, 0.70710678, 0.70710678],
            trigger=0.5,
        )

        payload = player.arm_pose_to_six_d_payload(pose)

        self.assertAlmostEqual(payload[0], 1.0)
        self.assertAlmostEqual(payload[1], 2.0)
        self.assertAlmostEqual(payload[2], 3.0)
        self.assertAlmostEqual(payload[3], 0.0, places=7)
        self.assertAlmostEqual(payload[4], 1.0, places=7)
        self.assertAlmostEqual(payload[5], 0.0, places=7)
        self.assertAlmostEqual(payload[6], -1.0, places=7)
        self.assertAlmostEqual(payload[7], 0.0, places=7)
        self.assertAlmostEqual(payload[8], 0.0, places=7)

    def test_rotation_vector_payload_contains_position_and_rot_vec(self):
        pose = player.ArmPose(
            position=[1.0, 2.0, 3.0],
            quaternion_xyzw=[0.0, 0.0, 0.70710678, 0.70710678],
            trigger=0.5,
        )

        payload = player.arm_pose_to_rot_vec_payload(pose)

        self.assertEqual(payload[:3], [1.0, 2.0, 3.0])
        self.assertAlmostEqual(payload[3], 0.0, places=7)
        self.assertAlmostEqual(payload[4], 0.0, places=7)
        self.assertAlmostEqual(payload[5], 1.5707963267948966, places=6)


if __name__ == "__main__":
    unittest.main()
