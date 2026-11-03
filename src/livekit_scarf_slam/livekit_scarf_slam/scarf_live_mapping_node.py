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

"""
Run ScaRF-SLAM live on the RTAB-Map keyframes.

The node is the slam_bag recorder with a second output: each message that it
writes to the bag also goes to a LiveSlamInput, and ScaRF-SLAM maps from that
input while the robot drives. ROS runs in a background thread. ScaRF-SLAM runs
in the main thread. On SIGINT or SIGTERM, the node closes the bag and the
input, ScaRF-SLAM processes the rest, and it saves the map to
<slam_folder>/recon. Replay the bag with run_mapping.py and
<slam_folder>/scarf_config.yaml to map the same run again.
"""

from pathlib import Path
import shutil
import signal
import threading

from livekit_scarf_slam.live_input import LiveSlamInput
from livekit_scarf_slam.slam_bag_recorder_node import SlamBagRecorderNode
from livekit_scarf_slam.submap_cloud import SubmapPublishOptions
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions


class ScarfLiveMappingNode(SlamBagRecorderNode):
    """slam_bag recorder that also feeds the live mapper."""

    def __init__(self, live_input):
        self._live_input = live_input
        super().__init__()
        self.scarf_config = self.declare_parameter('scarf_config', '').value
        self.slam_folder = self.declare_parameter('slam_folder', '').value
        if not self.scarf_config or not self.slam_folder:
            raise ValueError('the scarf_config and slam_folder parameters are required')
        defaults = SubmapPublishOptions()
        self.submap_options = SubmapPublishOptions(
            voxel_m=self.declare_parameter('submap_voxel_m', defaults.voxel_m).value,
            max_bytes=self.declare_parameter('submap_max_bytes', defaults.max_bytes).value,
            scale_tolerance=self.declare_parameter(
                'submap_scale_tolerance', defaults.scale_tolerance).value,
            translation_tolerance_m=self.declare_parameter(
                'submap_translation_tolerance_m', defaults.translation_tolerance_m).value,
            rotation_tolerance_deg=self.declare_parameter(
                'submap_rotation_tolerance_deg', defaults.rotation_tolerance_deg).value,
            refresh_count=self.declare_parameter(
                'submap_refresh_count', defaults.refresh_count).value,
            colors=self.declare_parameter('submap_colors', defaults.colors).value,
        )
        self.publish_global_cloud = self.declare_parameter('publish_global_cloud', True).value
        self.submap_send_gap_s = self.declare_parameter('submap_send_gap_ms', 5.0).value / 1000.0
        self.submap_warmup_s = self.declare_parameter('submap_warmup_s', 2.0).value
        self.submap_refresh_interval_s = self.declare_parameter(
            'submap_refresh_interval_s', 1.0).value
        self.submap_max_rate_kbps = self.declare_parameter('submap_max_rate_kbps', 2000.0).value
        # Keep the config with the run, so that a replay maps with the same one.
        self.run_config = Path(self.slam_folder) / 'scarf_config.yaml'
        self.run_config.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(self.scarf_config, self.run_config)
        self.get_logger().info(f'Copied {self.scarf_config} to {self.run_config}')

    def _emit(self, topic, msg, stamp_ns):
        super()._emit(topic, msg, stamp_ns)
        self._live_input.add(topic, msg)


def _spin_until(node, executor, live_input, stop):
    """Spin until stop is set, then close the bag and the live input."""
    try:
        while not stop.is_set() and rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
    finally:
        node.close()
        live_input.close()


def main(args=None):
    # Ctrl+C must not interrupt the GPU work, so the handlers only set stop.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: stop.set())

    live_input = LiveSlamInput()
    node = ScarfLiveMappingNode(live_input)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=_spin_until, args=(node, executor, live_input, stop))
    ros_thread.start()

    app = None
    try:
        # Import here: the import loads torch and the ScaRF-SLAM modules.
        from livekit_scarf_slam.live_mapper import LiveInputClosed
        from livekit_scarf_slam.live_mapper import LiveScaRFSLAM
        app = LiveScaRFSLAM(node.slam_folder, live_input, node.submap_options,
                            node.publish_global_cloud, node.submap_send_gap_s,
                            node.submap_warmup_s, node.submap_refresh_interval_s,
                            node.submap_max_rate_kbps * 1000.0 / 8.0)
        app.app_configure(node.run_config)
        node.get_logger().info('ScaRF-SLAM is mapping live')
        app.do_processing_outer()
        node.get_logger().info(f'Saved the ScaRF-SLAM map to {node.slam_folder}/recon')
    except LiveInputClosed:
        node.get_logger().warn('Stopped before the first keyframe, so there is no map to save')
    finally:
        stop.set()
        ros_thread.join()
        if app is not None:
            app.close()
        if app is not None and app.ros2_node is not None:
            app.ros2_node.destroy_node()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
