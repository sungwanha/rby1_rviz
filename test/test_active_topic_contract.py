from pathlib import Path
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
ACTIVE_FILES = [
    REPO_ROOT / "src" / "ik_solver" / "teleop_ik.cpp",
    REPO_ROOT / "src" / "ik_solver" / "proprio_pose_node.cpp",
]
README = REPO_ROOT / "README.md"
DISPLAY_LAUNCH = REPO_ROOT / "display.launch.py"


class ActiveTopicContractTest(unittest.TestCase):
    def test_active_nodes_use_split_proprio_joint_states(self):
        for path in ACTIVE_FILES:
            with self.subTest(path=path.name):
                text = path.read_text()
                self.assertIn('"/sensors/proprio/body/joint_states"', text)
                self.assertIn('"/sensors/proprio/hand_left/joint_states"', text)
                self.assertIn('"/sensors/proprio/hand_right/joint_states"', text)
                self.assertNotIn('"/joint_states"', text)

    def test_active_ik_node_publishes_controller_trajectories(self):
        text = (REPO_ROOT / "src" / "ik_solver" / "teleop_ik.cpp").read_text()

        self.assertIn('"/control/body/arm_left_controller/joint_trajectory"', text)
        self.assertIn('"/control/body/arm_right_controller/joint_trajectory"', text)
        self.assertIn('"/control/hand_left/hand_left_controller/joint_trajectory"', text)
        self.assertIn('"/control/hand_right/hand_right_controller/joint_trajectory"', text)
        self.assertNotIn('"/leader/master_arm_joint_left/joint_trajectory"', text)
        self.assertNotIn('"/leader/master_arm_joint_right/joint_trajectory"', text)

    def test_active_ik_trajectory_points_use_timer_period_time_from_start(self):
        text = (REPO_ROOT / "src" / "ik_solver" / "teleop_ik.cpp").read_text()

        self.assertIn("time_from_start.sec = 0", text)
        self.assertIn("time_from_start.nanosec = 20000000", text)

    def test_readme_documents_active_topic_contract(self):
        text = README.read_text()

        self.assertIn("/sensors/proprio/body/joint_states", text)
        self.assertIn("/sensors/proprio/hand_left/joint_states", text)
        self.assertIn("/sensors/proprio/hand_right/joint_states", text)
        self.assertIn("/control/body/arm_left_controller/joint_trajectory", text)
        self.assertIn("/control/body/arm_right_controller/joint_trajectory", text)
        self.assertIn("/control/hand_left/hand_left_controller/joint_trajectory", text)
        self.assertIn("/control/hand_right/hand_right_controller/joint_trajectory", text)
        self.assertNotIn("/leader/master_arm_joint_left/joint_trajectory", text)
        self.assertNotIn("/leader/master_arm_joint_right/joint_trajectory", text)

    def test_robot_state_publisher_uses_body_proprio_joint_states(self):
        text = DISPLAY_LAUNCH.read_text()

        self.assertIn("('/joint_states', '/sensors/proprio/body/joint_states')", text)


if __name__ == "__main__":
    unittest.main()
