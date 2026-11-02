"""Single-scene MuJoCo body, grippers and IK for the bundled replay sample."""
from pathlib import Path
import xml.etree.ElementTree as ET
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    directory = Path(__file__).resolve().parent / 'generated'
    headless = LaunchConfiguration('headless').perform(context)
    control = ET.parse(directory / 'control.urdf').getroot()
    control.find('ros2_control/hardware/param[@name="headless"]').text = headless
    clock = {'use_sim_time': True}

    def rsp(name, xml, namespace='', prefix='', private=False):
        return Node(package='robot_state_publisher', executable='robot_state_publisher',
                    name=name, namespace=namespace, output='screen',
                    parameters=[clock, {'robot_description': xml, 'frame_prefix': prefix}],
                    remappings=[('/tf', '/internal/gripper_demo/tf'),
                                ('/tf_static', '/internal/gripper_demo/tf_static')] if private else [])

    def spawn(names, remaps=''):
        return Node(package='controller_manager', executable='spawner', output='screen',
                    arguments=[*names, '-c', '/controller_manager', '--controller-manager-timeout', '60',
                               '--controller-ros-args', '--ros-args ' + remaps])

    nodes = [
        rsp('control_description_publisher', ET.tostring(control, encoding='unicode'),
            '/internal/gripper_demo', private=True),
        rsp('robot_state_publisher', (directory / 'body.urdf').read_text()),
        Node(package='mujoco_ros2_control', executable='ros2_control_node', output='screen',
             parameters=[str(directory / 'controllers.yaml'), clock],
             remappings=[('robot_description', '/internal/gripper_demo/robot_description')],
             on_exit=[Shutdown(reason='Combined MuJoCo manager exited')]),
        spawn(['joint_state_broadcaster']),
        spawn(['left_arm_controller', 'right_arm_controller', 'torso_controller',
               'head_controller', 'base_controller', 'left_gripper_controller', 'right_gripper_controller'],
              '-r ~/cmd_vel:=/base_controller/cmd_vel -r ~/reference:=/base_controller/cmd_vel '
              '-r ~/odom:=/odom -r ~/odometry:=/odom -r ~/tf_odometry:=/tf'),
    ]
    for part in ('left_arm', 'right_arm', 'torso', 'head', 'left_gripper', 'right_gripper'):
        nodes.append(spawn([part + '_state_broadcaster'],
                           f'-r joint_states:=/{part}/joint_states '
                           f'-r dynamic_joint_states:=/{part}/dynamic_joint_states'))
    for side in ('left', 'right'):
        nodes += [
            rsp('robot_state_publisher', (directory / f'{side}.urdf').read_text(),
                '/' + side + '_gripper', side + '_gripper/'),
            Node(package='tf2_ros', executable='static_transform_publisher',
                 name=side + '_gripper_mount', parameters=[clock], output='screen',
                 arguments=['--frame-id', side + '_flange', '--child-frame-id', side + '_gripper/base_link']),
        ]
    nodes += [
        Node(package='rby1_teleop_cpp', executable='teleop_ik_node', output='screen',
             parameters=[clock], remappings=[
                 ('/sensors/proprio/body/joint_states', '/joint_states'),
                 ('/control/body/arm_left_controller/joint_trajectory', '/left_arm_controller/joint_trajectory'),
                 ('/control/body/arm_right_controller/joint_trajectory', '/right_arm_controller/joint_trajectory'),
                 ('/control/hand_left/hand_left_controller/joint_trajectory', '/left_gripper_controller/joint_trajectory'),
                 ('/control/hand_right/hand_right_controller/joint_trajectory', '/right_gripper_controller/joint_trajectory'),
             ]),
        Node(package='rby1_teleop_cpp', executable='proprio_pose_node', output='screen',
             parameters=[clock], remappings=[('/sensors/proprio/body/joint_states', '/joint_states')]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='ik_base_frame',
             parameters=[clock], arguments=['--frame-id', 'base_link', '--child-frame-id', 'base']),
    ]
    return nodes


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('headless', default_value='false',
                                                    choices=['true', 'false']),
                              OpaqueFunction(function=start)])
