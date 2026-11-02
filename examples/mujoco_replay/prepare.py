"""Generate the a_1.2 demo scene from installed rby1_ros2 model assets."""
from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET

import PyKDL
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory


def expand(path):
    root = ET.parse(path).getroot()
    for parent in root.iter():
        for child in list(parent):
            if child.tag == 'include':
                index = list(parent).index(child)
                parent.remove(child)
                for element in expand(path.parent / child.get('file')):
                    parent.insert(index, element)
                    index += 1
            elif child.get('file'):
                child.set('file', str((path.parent / child.get('file')).resolve()))
    return root


def frame(joint):
    origin = joint.find('origin')
    xyz = [float(value) for value in origin.get('xyz', '0 0 0').split()]
    rpy = [float(value) for value in origin.get('rpy', '0 0 0').split()]
    return PyKDL.Frame(PyKDL.Rotation.RPY(*rpy), PyKDL.Vector(*xyz))


def set_frame(element, transform):
    element.set('pos', ' '.join(str(transform.p[index]) for index in range(3)))
    x, y, z, w = transform.M.GetQuaternion()
    element.set('quat', f'{w} {x} {y} {z}')


directory = Path(__file__).resolve().parent / 'generated'
directory.mkdir(exist_ok=True)
description = Path(get_package_share_directory('rby1_description'))
gripper = Path(get_package_share_directory('rby1_gripper_description'))
bringup = Path(get_package_share_directory('rby1_bringup'))
body = ET.fromstring(xacro.process_file(
    str(description / 'urdf/rby1.urdf.xacro'),
    mappings={'hardware_type': 'mujoco', 'robot_model': 'a', 'robot_version': 'v1.2',
              'headless': 'false', 'power_grippers': 'false'},
).toxml())
ET.ElementTree(body).write(directory / 'body.urdf', encoding='unicode')
scene = expand(description / 'mjcf/rby1a/model_act_v1.2.xml')
hands = ET.parse(gripper / 'mjcf/hand_both.xml').getroot()
assets = scene.find('asset')
for asset in hands.find('asset'):
    asset = deepcopy(asset)
    if asset.tag == 'material':
        asset.set('name', 'gripper_' + asset.get('name'))
    if asset.get('file'):
        asset.set('file', str(gripper / 'mjcf' / asset.get('file')))
    assets.append(asset)
defaults = ET.SubElement(scene, 'default')
for name in ('visual', 'collision'):
    element = deepcopy(hands.find(f'default/default[@class="{name}"]'))
    element.set('class', 'gripper_' + name)
    defaults.append(element)
control = body.find('ros2_control')
control.find('hardware/param[@name="mujoco_model"]').text = str(directory / 'combined.xml')
initial_positions = {
    'torso_1': -0.39793506945470714, 'torso_3': 1.0524335389525805,
    'left_arm_3': -2.2689280275926285, 'right_arm_3': -2.2689280275926285,
    'left_arm_6': -1.5707963267948966, 'right_arm_6': 1.5707963267948966,
}
for joint in control.findall('joint'):
    if joint.get('name') in initial_positions:
        joint.find('state_interface[@name="position"]/param[@name="initial_value"]').text = str(
            initial_positions[joint.get('name')])

for side, short in (('left', 'l'), ('right', 'r')):
    hand = ET.fromstring(xacro.process_file(
        str(gripper / 'urdf/hand.urdf.xacro'), mappings={'side': side},
    ).toxml())
    ET.ElementTree(hand).write(directory / f'{side}.urdf', encoding='unicode')
    mounted = deepcopy(hands.find(f'worldbody/body/body[@name="gripper_{short}_base"]'))
    # Use the public Xacro transforms for both attachment and finger geometry.
    mount = (frame(body.find(f'joint[@name="{side}_flange_joint"]'))
             * frame(hand.find('joint[@name="mount_frame_joint"]'))
             * frame(hand.find(f'joint[@name="gripper_{short}_mount_joint"]')))
    set_frame(mounted, mount)
    placeholder = scene.find(f'.//body[@name="EE_BODY_{short.upper()}"]')
    tool = frame(body.find(f'joint[@name="tool_{side}"]'))
    assert all(abs(float(value) - tool.p[index]) < 1e-10
               for index, value in enumerate(placeholder.get('pos').split()))
    for index in (1, 2):
        set_frame(mounted.find(f'body[@name="gripper_{short}_finger_{index}"]'),
                  frame(hand.find(f'joint[@name="gripper_{short}_finger_{index}_joint"]')))
    for geom in mounted.iter('geom'):
        geom.set('class', 'gripper_' + geom.get('class'))
        if geom.get('material'):
            geom.set('material', 'gripper_' + geom.get('material'))
    for joint in mounted.iter('joint'):
        joint.set('armature', '0')
        joint.set('damping', '0')
    placeholder.append(mounted)
    trigger = f'{side}_trigger'
    ET.SubElement(body, 'link', name=f'{side}_trigger_control_link')
    joint = deepcopy(hand.find(f'joint[@name="{trigger}"]'))
    joint.find('parent').set('link', 'base_link')
    joint.find('child').set('link', f'{side}_trigger_control_link')
    body.append(joint)
    controlled = ET.SubElement(control, 'joint', name=trigger)
    ET.SubElement(controlled, 'command_interface', name='position')
    state = ET.SubElement(controlled, 'state_interface', name='position')
    ET.SubElement(state, 'param', name='initial_value').text = '0.0'
for equality in hands.find('equality'):
    scene.find('equality').append(deepcopy(equality))
for actuator in hands.find('actuator'):
    actuator = deepcopy(actuator)
    # Preserve the source gripper gains; this demo does not tune out recorded jitter.
    actuator.set('kp', '1')
    actuator.set('kv', '0')
    scene.find('actuator').append(actuator)
ET.ElementTree(scene).write(directory / 'combined.xml', encoding='unicode')
ET.ElementTree(body).write(directory / 'control.urdf', encoding='unicode')

params = yaml.safe_load((bringup / 'config/rby1a_controllers.yaml').read_text())['/**']
manager = params['controller_manager']['ros__parameters']
body_joints = [joint.get('name') for joint in control.findall('joint')
               if joint.get('name') not in ('left_trigger', 'right_trigger')]
assert len(body_joints) == 24
params['joint_state_broadcaster'] = {'ros__parameters': {
    'joints': body_joints, 'interfaces': ['position', 'velocity', 'effort']}}
for part in ('left_arm', 'right_arm', 'torso', 'head'):
    name = part + '_state_broadcaster'
    manager[name] = {'type': 'joint_state_broadcaster/JointStateBroadcaster'}
    params[name] = {'ros__parameters': {
        'joints': list(params[part + '_controller']['ros__parameters']['joints']),
        'interfaces': ['position', 'velocity', 'effort']}}
gripper_params = yaml.safe_load((bringup / 'config/gripper_controllers.yaml').read_text())['/**']
manager.update({name: value for name, value in gripper_params['controller_manager']['ros__parameters'].items()
                if name != 'update_rate'})
params.update({name: value for name, value in gripper_params.items() if name != 'controller_manager'})
(directory / 'controllers.yaml').write_text(yaml.safe_dump({'/**': params}))
print(f'Generated runtime scene, descriptions, and controllers in {directory}')
