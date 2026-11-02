from pathlib import Path
import ctypes
import glob
import xml.etree.ElementTree as ET

directory = Path(__file__).resolve().parent / 'generated'
assert (directory / 'combined.xml').is_file(), 'Combined runtime scene is not generated'
robot = ET.parse(directory / 'control.urdf').getroot()
joint_names = [joint.get('name') for joint in robot.findall('ros2_control/joint')]
assert len(joint_names) == len(set(joint_names)) == 26, joint_names
assert {'left_trigger', 'right_trigger'} <= set(joint_names)
scene = ET.parse(directory / 'combined.xml').getroot()
for side in ('left', 'right'):
    assert scene.find(f'.//joint[@name="{side}_trigger"]').get('range') == '0 1'
    assert scene.find(f'actuator/position[@joint="{side}_trigger"]').get('ctrlrange') == '0 1'
library = ctypes.CDLL(glob.glob('/opt/ros/jazzy/opt/mujoco_vendor/lib/libmujoco.so*')[0])
library.mj_loadXML.argtypes = [ctypes.c_char_p, ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
library.mj_loadXML.restype = ctypes.c_void_p
library.mj_name2id.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_char_p]
library.mj_name2id.restype = ctypes.c_int
library.mj_deleteModel.argtypes = [ctypes.c_void_p]
error = ctypes.create_string_buffer(4096)
model = library.mj_loadXML(str(directory / 'combined.xml').encode(), None, error, len(error))
assert model, error.value.decode()
for name in joint_names:
    assert library.mj_name2id(model, 3, name.encode()) >= 0, name
library.mj_deleteModel(model)
print('PASS: MuJoCo compiled scene; 24 body joints + 2 triggers; limits and actuator mappings valid.')
