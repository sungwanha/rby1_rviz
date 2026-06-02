import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 경로 설정
    current_dir = os.path.dirname(os.path.realpath(__file__))
    
    # 도커 내부라면 '/app/model/model_v1.2.urdf'가 됩니다.
    urdf_file_path = os.path.join(current_dir, 'model', 'model_v1.2.urdf')
    rviz_config_path = os.path.join(current_dir, 'rby1.rviz')
    
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    # 경로 자동 변환
    mesh_base_path = f'file://{os.path.join(current_dir, "model", "meshes")}/'
    robot_desc = robot_desc.replace('./meshes/', mesh_base_path)
    robot_desc = robot_desc.replace('file:///home/hsw/rby1_rviz/model/meshes/', mesh_base_path)

    return LaunchDescription([
        DeclareLaunchArgument(
            'pose_input_mode',
            default_value='quaternion',
            description='Pose input mode for teleop_ik_node: quaternion, six_d, or rot_vec'
        ),
        DeclareLaunchArgument(
            'sim_test',
            default_value='false',
            description='Start a local test pose publisher and auto-enable teleop'
        ),
        DeclareLaunchArgument(
            'data_replay',
            default_value='false',
            description='Replay EE pose data from a parquet file and auto-enable teleop'
        ),
        DeclareLaunchArgument(
            'parquet_path',
            default_value=os.path.join(os.getcwd(), 'data', 'chunk-000_ee', 'file-000.parquet'),
            description='Parquet path used when data_replay is true'
        ),
        DeclareLaunchArgument(
            'replay_column_prefix',
            default_value='action',
            description='Parquet column prefix to replay: action or observation'
        ),
        DeclareLaunchArgument(
            'replay_rate_hz',
            default_value='30.0',
            description='Data replay publish rate in Hz'
        ),
        DeclareLaunchArgument(
            'publish_proprio_poses',
            default_value='true',
            description='Publish ZED-frame arm proprioception poses from /joint_states'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Start RViz with rby1.rviz'
        ),
        DeclareLaunchArgument(
            'use_rosbridge',
            default_value='false',
            description='Start rosbridge websocket server on port 9091'
        ),
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
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            arguments=['-d', rviz_config_path]
        ),
        Node(
            package='rby1_teleop_cpp',
            executable='teleop_ik_node',
            name='teleop_ik_node',
            output='screen',
            parameters=[{'pose_input_mode': LaunchConfiguration('pose_input_mode')}],
            # arguments=[urdf_file_path]
        ),
        Node(
            package='rby1_teleop_cpp',
            executable='proprio_pose_node',
            name='proprio_pose_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('publish_proprio_poses'))
        ),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_server',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rosbridge')),
            parameters=[{'port': 9091}]
        ),
        Node(
            package='rby1_teleop_cpp',
            executable='sim_pose_publisher.py',
            name='sim_pose_publisher',
            output='screen',
            condition=IfCondition(LaunchConfiguration('sim_test')),
            parameters=[{
                'pose_input_mode': LaunchConfiguration('pose_input_mode'),
                'auto_enable_teleop': True
            }]
        ),
        Node(
            package='rby1_teleop_cpp',
            executable='ee_parquet_pose_player.py',
            name='ee_parquet_pose_player',
            output='screen',
            condition=IfCondition(LaunchConfiguration('data_replay')),
            parameters=[{
                'pose_input_mode': LaunchConfiguration('pose_input_mode'),
                'parquet_path': LaunchConfiguration('parquet_path'),
                'column_prefix': LaunchConfiguration('replay_column_prefix'),
                'replay_rate_hz': LaunchConfiguration('replay_rate_hz'),
                'loop': True,
                'auto_enable_teleop': True
            }]
        )
    ])
