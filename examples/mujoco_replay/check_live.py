"""Read-only assertions for the already running combined scene."""
import time
import rclpy
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import ListControllers

rclpy.init()
node = rclpy.create_node('combined_scene_verifier')
states = {}
subscriptions = [node.create_subscription(
    JointState, topic, lambda message, topic=topic: states.__setitem__(topic, message), 10)
    for topic in ('/joint_states', '/left_gripper/joint_states', '/right_gripper/joint_states')]
manager = node.create_client(ListControllers, '/controller_manager/list_controllers')
deadline = time.monotonic() + 30
while len(states) != 3 and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
assert len(states) == 3, list(states)
assert node.count_publishers('/clock') == 1, node.count_publishers('/clock')
assert len(states['/joint_states'].name) == 24
assert not {'left_trigger', 'right_trigger'} & set(states['/joint_states'].name)
for side in ('left', 'right'):
    state = states[f'/{side}_gripper/joint_states']
    assert state.name == [f'{side}_trigger'], state.name
    assert -0.03 <= state.position[0] <= 1.03, state.position
assert manager.wait_for_service(timeout_sec=5)
expected = {'joint_state_broadcaster', 'base_controller'} | {
    part + suffix for part in ('left_arm', 'right_arm', 'torso', 'head', 'left_gripper', 'right_gripper')
    for suffix in ('_controller', '_state_broadcaster')}
active = set()
deadline = time.monotonic() + 60
while not expected <= active and time.monotonic() < deadline:
    future = manager.call_async(ListControllers.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=2)
    assert future.done(), 'Controller manager did not respond'
    active = {controller.name for controller in future.result().controller if controller.state == 'active'}
    rclpy.spin_once(node, timeout_sec=0.1)
assert expected <= active, expected - active
print('PASS: one /clock publisher; 24 body-only joint states; independent left/right trigger states; 14 active controllers.')
print({topic: dict(zip(state.name, state.position)) for topic, state in states.items() if 'gripper' in topic})
node.destroy_node()
rclpy.shutdown()
