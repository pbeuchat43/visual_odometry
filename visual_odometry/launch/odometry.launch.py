"""Start the pinned estimator and small identity adapter from a frozen JSON file."""
# Copyright (c) 2026 Paul N. Beuchat. MIT License.
import hashlib
import json
from pathlib import Path
import re

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def read_configuration(path):
    path = Path(path)
    if not path.is_absolute() or path.stat().st_size > 16384:
        raise ValueError('config_file must be an absolute bounded prepared JSON file')
    config = json.loads(path.read_text())
    fields = {'schema_version', 'mode', 'camera_backend', 'namespace', 'camera_node',
              'frames', 'executor', 'preset_sha256', 'backend_identity'}
    if not isinstance(config, dict) or set(config) != fields or config['schema_version'] != 1:
        raise ValueError('unsupported odometry configuration schema or fields')
    if config['mode'] not in ('vo', 'vio') or config['camera_backend'] not in ('oak', 'realsense'):
        raise ValueError('unsupported mode or camera backend')
    expected_executor = 'single' if config['camera_backend'] == 'realsense' else 'multi'
    if config['executor'] != expected_executor:
        raise ValueError('executor must match the admitted camera preset')
    for key in ('namespace', 'camera_node'):
        if not isinstance(config[key], str) or not re.fullmatch(r'/[A-Za-z][A-Za-z0-9_/]{0,126}', config[key]) or config[key].endswith('/') or '//' in config[key]:
            raise ValueError('invalid absolute ROS namespace/node path')
    frames = config['frames']
    if not isinstance(frames, dict) or set(frames) != {'body', 'odom', 'left', 'right', 'imu'} or len(set(frames.values())) != 5:
        raise ValueError('distinct body/odom/left/right/imu frames are required')
    if any(not isinstance(v, str) or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_/]{0,127}', v) for v in frames.values()):
        raise ValueError('invalid frame name')
    if not isinstance(config['backend_identity'], str) or not re.fullmatch(r'sha256:[a-f0-9]{64}', config['backend_identity']):
        raise ValueError('an exact image or native-prefix manifest SHA256 is required')
    preset_path = Path(get_package_share_directory('visual_odometry')) / 'config/estimator.yaml'
    preset = preset_path.read_bytes()
    if hashlib.sha256(preset).hexdigest() != config['preset_sha256']:
        raise ValueError('prepared estimator preset differs from installed package')
    configuration_id = hashlib.sha256(json.dumps(config, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    return config, yaml.safe_load(preset), configuration_id


def launch(context):
    path = LaunchConfiguration('config_file').perform(context)
    config, parameters, identity = read_configuration(path)
    generation = LaunchConfiguration('camera_generation').perform(context)
    if not generation:
        supervisor = Node(package='visual_odometry', executable='odometry_supervisor',
            name='odometry_supervisor', namespace=config['namespace'], output='screen', arguments=['--config', path])
        return [RegisterEventHandler(OnProcessExit(target_action=supervisor,
            on_exit=[EmitEvent(event=Shutdown(reason='odometry supervisor exited'))])), supervisor]
    if not re.fullmatch(r'[A-Za-z0-9_-]{1,128}', generation):
        raise ValueError('invalid camera timing generation')
    frames = config['frames']; namespace = config['namespace']; camera = config['camera_node']
    parameters.update(tracking_mode=1 if config['mode'] == 'vio' else 0,
                      rectified_images=config['camera_backend'] == 'realsense',
                      base_frame=frames['body'], odom_frame=frames['odom'], map_frame=frames['odom'] + '_unused_map',
                      camera_optical_frames=[frames['left'], frames['right']], imu_frame=frames['imu'], verbosity=1)
    remaps = [('visual_slam/image_0', camera + '/stereo/left/image'),
              ('visual_slam/image_1', camera + '/stereo/right/image'),
              ('visual_slam/camera_info_0', camera + '/stereo/left/camera_info'),
              ('visual_slam/camera_info_1', camera + '/stereo/right/camera_info'),
              ('visual_slam/imu', camera + '/imu/combined'),
              ('visual_slam/tracking/odometry', namespace + '/vslam/upstream_odometry'),
              ('visual_slam/status', namespace + '/vslam/upstream_status'),
              ('visual_slam/reset', namespace + '/vslam/internal_reset')]
    container = ComposableNodeContainer(
        name='odometry_container', namespace=namespace, package='rclcpp_components',
        executable='component_container' if config['executor'] == 'single' else 'component_container_mt',
        output='screen', composable_node_descriptions=[ComposableNode(
            package='isaac_ros_visual_slam', plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
            name='vslam', namespace=namespace, parameters=[parameters], remappings=remaps)])
    adapter = Node(package='visual_odometry', executable='odometry_adapter', name='odometry', namespace=namespace,
        output='screen', parameters=[dict(mode=config['mode'], configuration_id=identity,
            camera_generation=generation,
            rectified_images=parameters['rectified_images'], body_frame=frames['body'], odom_frame=frames['odom'])],
        remappings=[('stereo_capture', camera+'/stereo/capture'), ('camera_status', camera+'/status'),
                    ('~/status', namespace+'/odometry/internal_status'),
                    ('tracking_odometry', namespace+'/vslam/upstream_odometry'), ('tracking_status', namespace+'/vslam/upstream_status')])
    # Keep adapter work outside the estimator's measured callback executor.
    def exited(event, context):
        if context.is_shutdown:
            return []
        return [EmitEvent(event=Shutdown(reason='required odometry process exited'))]
    return [RegisterEventHandler(OnProcessExit(target_action=container, on_exit=exited)),
            RegisterEventHandler(OnProcessExit(target_action=adapter, on_exit=exited)), container, adapter]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('config_file'),
        DeclareLaunchArgument('camera_generation', default_value=''), OpaqueFunction(function=launch)])
