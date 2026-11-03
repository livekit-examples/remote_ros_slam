# Copyright 2026 LiveKit
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# ScaRF-SLAM demo, SLAM host side. See README.md.
#
# Reads the SBS stereo stream that ROS Portal publishes (ROS Portal runs in
# its own container, see docker-compose.yml), rectifies it, runs RTAB-Map
# stereo odometry and SLAM, and records the ScaRF-SLAM input bag to
# <output_dir>/slam_bag. Run ScaRF-SLAM on that bag after the run, or pass
# scarf_live:=true to run it live and save the map on Ctrl+C.

from datetime import datetime
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PACKAGE = 'livekit_scarf_slam'
RECT_NAMESPACE = '/stereo'
# The Docker image sets SLAM_RUNS_DIR to the bind-mounted runs/ folder.
RUNS_DIR = os.environ.get('SLAM_RUNS_DIR', '~/scarf_slam_runs')


def _output_dir(context):
    value = LaunchConfiguration('output_dir').perform(context).strip()
    if value:
        return Path(value).expanduser()
    return Path(RUNS_DIR).expanduser() / datetime.now().strftime('%Y%m%d_%H%M%S')


def _launch_setup(context):
    output_dir = _output_dir(context)
    if (output_dir / 'slam_bag').exists():
        raise RuntimeError(f'{output_dir / "slam_bag"} already exists. Pick a new output_dir.')
    output_dir.mkdir(parents=True, exist_ok=True)

    calibration_path = LaunchConfiguration('calibration_path')
    sbs_topic = LaunchConfiguration('sbs_topic')
    rtabmap_params = PathJoinSubstitution([FindPackageShare(PACKAGE), 'config', 'rtabmap',
                                           'stereo.yaml'])
    rtabmap_remappings = [
        (f'{side}/{name}', f'{RECT_NAMESPACE}/{side}/{name}')
        for side in ('left', 'right') for name in ('image_rect', 'camera_info')]

    recorder_params = {
        'calibration_path': calibration_path,
        'sbs_topic': sbs_topic,
        'map_data_topic': '/rtabmap/mapData',
        'output_path': str(output_dir / 'slam_bag'),
    }
    if LaunchConfiguration('scarf_live').perform(context).lower() == 'true':
        live_params = [recorder_params, {
            'scarf_config': LaunchConfiguration('scarf_config'),
            'slam_folder': str(output_dir),
        }]
        # Optional ROS parameter file, for example the submap_* settings of the
        # map chunks. It must use the node name scarf_live_mapping or /**.
        params_file = LaunchConfiguration('scarf_live_params').perform(context).strip()
        if params_file:
            live_params.append(str(Path(params_file).expanduser()))
        # Live mapping: the node records the same bag and runs ScaRF-SLAM on the
        # keyframes. After Ctrl+C, it finishes the mapping and saves the map,
        # so it gets more time before SIGTERM and SIGKILL.
        recorder = Node(
            package=PACKAGE,
            executable='scarf_live_mapping_node',
            name='scarf_live_mapping',
            output='screen',
            emulate_tty=True,
            parameters=live_params,
            sigterm_timeout='120',
            sigkill_timeout='30',
        )
    else:
        recorder = Node(
            package=PACKAGE,
            executable='slam_bag_recorder_node',
            name='slam_bag_recorder',
            output='screen',
            parameters=[recorder_params],
        )

    return [
        LogInfo(msg=f'Writing the run output to {output_dir}'),
        Node(
            package=PACKAGE,
            executable='stereo_rectify_node',
            name='stereo_rectify',
            output='screen',
            parameters=[{
                'calibration_path': calibration_path,
                'input_topic': sbs_topic,
                'output_namespace': RECT_NAMESPACE,
                'decode_reduction': ParameterValue(
                    LaunchConfiguration('decode_reduction'), value_type=int),
                'rect_scale': ParameterValue(LaunchConfiguration('rect_scale'), value_type=float),
            }],
        ),
        Node(
            package='rtabmap_odom',
            executable='stereo_odometry',
            namespace='rtabmap',
            output='screen',
            parameters=[rtabmap_params],
            remappings=rtabmap_remappings,
        ),
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            namespace='rtabmap',
            output='screen',
            parameters=[rtabmap_params, {'database_path': str(output_dir / 'rtabmap.db')}],
            remappings=rtabmap_remappings,
            arguments=['-d'],
        ),
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            namespace='rtabmap',
            output='screen',
            parameters=[rtabmap_params],
            remappings=rtabmap_remappings,
            condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
        ),
        recorder,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'calibration_path',
            default_value='/remote_ros_slam/calibration/stereo_calibration.json',
            description='stereo_calibration.json of the SBS fisheye camera.'),
        DeclareLaunchArgument(
            'sbs_topic', default_value='/stereo_camera',
            description='SBS sensor_msgs/CompressedImage topic.'),
        DeclareLaunchArgument(
            'output_dir', default_value='',
            description=f'Run output folder. Default: {RUNS_DIR}/<date>_<time>.'),
        DeclareLaunchArgument(
            'decode_reduction', default_value='2',
            description='Decode the JPEG at 1/n size: 1, 2, 4, or 8.'),
        DeclareLaunchArgument(
            'rect_scale', default_value='0.5',
            description='Size of the rectified images relative to the calibration.'),
        DeclareLaunchArgument(
            'rtabmap_viz', default_value='false', description='Start the RTAB-Map GUI.'),
        DeclareLaunchArgument(
            'scarf_live', default_value='false',
            description='Run ScaRF-SLAM live on the keyframes, and save the map on Ctrl+C.'),
        DeclareLaunchArgument(
            'scarf_config',
            default_value=PathJoinSubstitution([FindPackageShare(PACKAGE), 'config', 'scarf_slam',
                                                'cobra_stereo.yaml']),
            description='ScaRF-SLAM configuration of the live mapping.'),
        DeclareLaunchArgument(
            'scarf_live_params', default_value='',
            description='Optional ROS parameter file for scarf_live_mapping_node, for '
                        'example the submap_* settings of the map chunks.'),
        OpaqueFunction(function=_launch_setup),
    ])
