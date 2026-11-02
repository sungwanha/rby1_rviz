#!/usr/bin/env python3
"""Replay episode zero once through the running simulated robot's teleop IK."""

import argparse
import json
import math
from pathlib import Path
import signal
import sys
import time

import numpy as np
import pandas as pd
import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import SetBool, Trigger

SOURCE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SOURCE / 'src/for_test'))
from ee_parquet_pose_player import ArmPose, EeParquetPosePlayer, parse_arm_pose_row

SIDES = ('left', 'right')
ARM_NAMES = {f'{side}_arm_{index}' for side in SIDES for index in range(7)}
FPS = 30


def load_episode(path):
    columns = ['episode_index', 'frame_index', 'timestamp', 'action.left_arm', 'action.right_arm']
    data = pd.read_parquet(path, columns=columns)
    data = data[data.episode_index == 0].sort_values('frame_index')
    assert len(data) == 456, f'Expected 456 episode-zero frames, got {len(data)}'
    assert data.frame_index.tolist() == list(range(456)), 'Missing or duplicate frame indices'
    timestamps = data.timestamp.to_numpy(dtype=float)
    assert np.isfinite(timestamps).all(), 'Invalid timestamps'
    assert np.allclose(np.diff(timestamps), 1 / FPS, atol=1e-6), 'Expected 30 Hz timestamps'
    frames = []
    for index, row in enumerate(data.to_dict('records')):
        left, right, _ = parse_arm_pose_row(row, 'action')
        for side, pose in zip(SIDES, (left, right)):
            values = np.array(pose.position + pose.quaternion_xyzw + [pose.trigger])
            assert np.isfinite(values).all(), f'Non-finite {side} data in frame {index}'
            norm = np.linalg.norm(pose.quaternion_xyzw)
            assert abs(norm - 1) < 1e-3, f'Invalid {side} quaternion in frame {index}: {norm}'
            assert 0 <= pose.trigger <= 1, f'Invalid {side} trigger in frame {index}'
            pose.quaternion_xyzw = (np.array(pose.quaternion_xyzw) / norm).tolist()
        frames.append((left, right))
    return frames


def pose_values(message):
    return np.array([message.position.x, message.position.y, message.position.z,
                     message.orientation.x, message.orientation.y,
                     message.orientation.z, message.orientation.w], dtype=float)


def blend_pose(start, target, fraction):
    quaternion = np.array(target.quaternion_xyzw)
    if np.dot(start[3:], quaternion) < 0:
        quaternion = -quaternion
    quaternion = start[3:] * (1 - fraction) + quaternion * fraction
    quaternion /= np.linalg.norm(quaternion)
    position = start[:3] * (1 - fraction) + np.array(target.position) * fraction
    return ArmPose(position.tolist(), quaternion.tolist(), target.trigger)


class Replay(Node):
    def __init__(self):
        super().__init__('single_episode_gripper_replay', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.clock_ns = 0
        self.last_clock_wall = time.monotonic()
        self.latest = {}
        self.received_at = {}
        self.ranges = {}
        self.counts = {}
        self.initial_poses = {}
        self.pose_max_motion = {side: 0.0 for side in SIDES}
        self.recording = False
        self.frames_sent = 0
        self.blend_frames = 0
        self.max_lateness_ms = 0.0
        self.command_trigger_ranges = {}
        self.pose_pubs = {}
        self.trigger_pubs = {}
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states',
                                 lambda msg: self.on_joints('body', msg), qos_profile_sensor_data)
        for side in SIDES:
            self.pose_pubs[side] = self.create_publisher(Pose, f'/control/body/arm_{side}_controller/pose', 10)
            self.trigger_pubs[side] = self.create_publisher(Float64, f'/control/hand_{side}/hand_{side}_controller/trigger', 10)
            self.create_subscription(JointState, f'/{side}_gripper/joint_states',
                                     lambda msg, side=side: self.on_joints(side, msg), qos_profile_sensor_data)
            self.create_subscription(Pose, f'/sensors/proprio/arm_{side}/pose',
                                     lambda msg, side=side: self.on_pose(side, msg), qos_profile_sensor_data)
        self.toggle = self.create_client(SetBool, '/toggle_teleop')
        self.nullspace = self.create_client(Trigger, '/update_nullspace_posture')

    def on_clock(self, message):
        value = message.clock.sec * 10**9 + message.clock.nanosec
        if value > self.clock_ns:
            self.last_clock_wall = time.monotonic()
        if value < self.clock_ns:
            raise RuntimeError('Simulation clock moved backwards')
        self.clock_ns = value

    def on_joints(self, key, message):
        values = dict(zip(message.name, message.position))
        if not values or not all(math.isfinite(value) for value in values.values()):
            raise ValueError(f'Invalid measured {key} joints')
        self.latest[key] = values
        self.received_at[key] = self.clock_ns
        if self.recording:
            self.counts[key] = self.counts.get(key, 0) + 1
            bounds = self.ranges.setdefault(key, {})
            for name, value in values.items():
                lower, upper = bounds.get(name, (value, value))
                bounds[name] = (min(lower, value), max(upper, value))

    def on_pose(self, side, message):
        values = pose_values(message)
        assert np.isfinite(values).all(), f'Invalid measured {side} pose'
        norm = np.linalg.norm(values[3:])
        assert abs(norm - 1) < 1e-3, f'Invalid measured {side} quaternion'
        values[3:] /= norm
        self.latest[side + '_pose'] = values
        self.received_at[side + '_pose'] = self.clock_ns
        if self.recording:
            self.counts[side + '_pose'] = self.counts.get(side + '_pose', 0) + 1
            motion = float(np.linalg.norm(values[:3] - self.initial_poses[side][:3]))
            self.pose_max_motion[side] = max(self.pose_max_motion[side], motion)

    def wait_until(self, predicate, timeout, label):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= deadline:
                raise TimeoutError(f'Timed out waiting for {label}')
            rclpy.spin_once(self, timeout_sec=0.01)

    def call_service(self, client, request, timeout=10):
        self.wait_until(client.service_is_ready, timeout, client.srv_name)
        future = client.call_async(request)
        self.wait_until(future.done, timeout, client.srv_name + ' response')
        response = future.result()
        if not response.success:
            raise RuntimeError(f'{client.srv_name}: {response.message}')
        return response.message

    def wait_for_sim_time(self, target_ns):
        while self.clock_ns < target_ns:
            if time.monotonic() - self.last_clock_wall > 10:
                raise TimeoutError('Simulation clock stopped for 10 wall seconds')
            rclpy.spin_once(self, timeout_sec=0.005)
        for key in ('body', 'left', 'right', 'left_pose', 'right_pose'):
            if self.clock_ns - self.received_at[key] > 2 * 10**9:
                raise TimeoutError(f'Stale measured {key} feedback')

    def publish(self, poses):
        for side, pose in zip(SIDES, poses):
            self.pose_pubs[side].publish(EeParquetPosePlayer.make_pose_msg(pose))
            self.trigger_pubs[side].publish(Float64(data=float(pose.trigger)))
            lower, upper = self.command_trigger_ranges.get(side, (pose.trigger, pose.trigger))
            self.command_trigger_ranges[side] = (min(lower, pose.trigger), max(upper, pose.trigger))

    def run(self, frames):
        required = ('body', 'left', 'right', 'left_pose', 'right_pose')
        self.wait_until(lambda: self.clock_ns > 0 and all(key in self.latest for key in required)
                        and ARM_NAMES <= self.latest['body'].keys()
                        and all(pub.get_subscription_count() > 0 for pub in self.pose_pubs.values())
                        and all(pub.get_subscription_count() > 0 for pub in self.trigger_pubs.values()),
                        30, 'simulation clock, measured states and control subscribers')
        self.call_service(self.toggle, SetBool.Request(data=False))
        self.wait_for_sim_time(self.clock_ns + 200_000_000)
        self.call_service(self.nullspace, Trigger.Request())
        self.initial_poses = {side: self.latest[side + '_pose'].copy() for side in SIDES}
        initial_triggers = {}
        for side in SIDES:
            measured = self.latest[side]
            assert side + '_trigger' in measured, f'Missing normalized measured {side}_trigger'
            initial_triggers[side] = measured[side + '_trigger']
            assert -1e-3 <= initial_triggers[side] <= 1.001, f'Invalid measured {side} trigger'
            self.trigger_pubs[side].publish(Float64(data=float(np.clip(initial_triggers[side], 0, 1))))
        self.wait_for_sim_time(self.clock_ns + 100_000_000)
        self.call_service(self.toggle, SetBool.Request(data=True))
        self.recording = True
        self.get_logger().info('Replaying episode 0: 456 frames at 30 Hz, 3 s blend, simulation clock')
        start_ns = self.clock_ns
        for step in range(91):
            self.wait_for_sim_time(start_ns + round(step * 10**9 / FPS))
            fraction = step / 90
            poses = [blend_pose(self.initial_poses[side], target, fraction)
                     for side, target in zip(SIDES, frames[0])]
            for side, pose in zip(SIDES, poses):
                pose.trigger = float(np.clip(initial_triggers[side] * (1 - fraction) + pose.trigger * fraction, 0, 1))
            self.publish(poses)
            self.blend_frames += 1
        start_ns = self.clock_ns + round(10**9 / FPS)
        for index, frame in enumerate(frames):
            target_ns = start_ns + round(index * 10**9 / FPS)
            self.wait_for_sim_time(target_ns)
            self.max_lateness_ms = max(self.max_lateness_ms, (self.clock_ns - target_ns) / 1e6)
            self.publish(frame)
            self.frames_sent += 1
            if self.frames_sent % 150 == 0:
                self.get_logger().info(f'Replayed {self.frames_sent}/456 frames')
        self.wait_for_sim_time(self.clock_ns + 2 * 10**9)
        assert self.frames_sent == len(frames), 'Replay frame count mismatch'
        for side in SIDES:
            lower, upper = self.ranges[side][side + '_trigger']
            assert upper - lower >= 0.03, f'{side} gripper did not move >= 0.03 normalized travel'
            arm_range = max(upper - lower for name, (lower, upper) in self.ranges['body'].items()
                            if name.startswith(side + '_arm_'))
            assert arm_range > 0.05, f'{side} arm did not move > 0.05 rad'

    def report(self, frames, error, disabled):
        endpoint = {}
        for side, target in zip(SIDES, frames[-1]):
            actual = self.latest.get(side + '_pose')
            if actual is not None:
                dot = np.clip(abs(np.dot(actual[3:], target.quaternion_xyzw)), 0, 1)
                endpoint[side] = {'position_error_m': float(np.linalg.norm(actual[:3] - target.position)),
                                  'orientation_error_rad': float(2 * np.arccos(dot)),
                                  'target_xyz': target.position, 'actual_xyz': actual[:3].tolist()}
        return {'status': 'completed' if error is None and disabled else 'failed',
                'episode_index': 0, 'expected_frames': len(frames), 'frames_sent': self.frames_sent,
                'blend_frames': self.blend_frames, 'fps': FPS, 'use_sim_time': True,
                'measurement_window': 'blend, episode replay and 2 s settling',
                'received_counts': self.counts, 'measured_joint_ranges': self.ranges,
                'published_trigger_ranges': self.command_trigger_ranges,
                'max_pose_position_motion_m': self.pose_max_motion,
                'max_frame_lateness_sim_ms': self.max_lateness_ms,
                'endpoint_tracking': endpoint, 'teleop_disabled': disabled, 'error': error}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--validate-only', action='store_true')
    args = parser.parse_args()
    frames = load_episode(SOURCE / 'samples/episode_000.parquet')
    if args.validate_only:
        print(json.dumps({'validation': 'passed', 'episode': 0, 'frames': len(frames), 'fps': FPS}))
        return 0
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Replay()
    def interrupted(signum, frame):
        raise KeyboardInterrupt(f'Signal {signum}')
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    error = None
    disabled = False
    try:
        node.run(frames)
    except (Exception, KeyboardInterrupt) as exc:
        error = f'{type(exc).__name__}: {exc}'
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        try:
            node.call_service(node.toggle, SetBool.Request(data=False), timeout=3)
            disabled = True
        except Exception as exc:
            error = f'{error or ""}; cleanup: {type(exc).__name__}: {exc}'
        print(json.dumps(node.report(frames, error, disabled), sort_keys=True), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if error is None and disabled else 1


if __name__ == '__main__':
    sys.exit(main())
