import os
from launch import LaunchDescription
from launch_ros.actions import Node
# 🔥 파이썬 스크립트 실행을 위해 ExecuteProcess 추가
from launch.actions import ExecuteProcess 

def generate_launch_description():
    # 경로 설정
    urdf_file_path = '/home/hsw/rby1_rviz/model/model_v1.2.urdf'
    rviz_config_path = '/home/hsw/rby1_rviz/rby1.rviz'
    
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    # 경로 자동 변환
    robot_desc = robot_desc.replace('./meshes/', 'file:///home/hsw/rby1_rviz/model/meshes/')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher_gui',
        #     output='screen'
        # ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        ),
        # ExecuteProcess(
        #     cmd=['python3', '/home/hsw/rby1_rviz/teleop_ik.py'],
        #     output='screen'
        # )
        Node(
            package='rby1_teleop_cpp',
            executable='teleop_ik_node',
            name='teleop_ik_node',
            output='screen',
            arguments=[urdf_file_path]
        ),
        # Node(
        #     package='rosbridge_server',
        #     executable='rosbridge_websocket',
        #     name='rosbridge_server',
        #     output='screen',
        #     parameters=[{'port': 9091}]
        # )
    ])