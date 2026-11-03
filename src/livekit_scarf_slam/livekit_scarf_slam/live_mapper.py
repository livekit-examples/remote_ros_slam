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
ScaRF-SLAM mapper that reads a LiveSlamInput instead of a bag.

It uses the two hooks of patches/scarf_slam_live_hooks.patch. The rest of the
mapping is the upstream code, so a live run and a replay of its bag process
the same keyframes.

The map also goes out as voxelized, size-capped Draco chunks of each submap
(see submap_cloud.py), which ROS Portal can send over unreliable DataTracks.
A PacedSender spaces the chunk messages out, because ROS Portal loses most
messages that are published back to back (see paced_sender.py).
"""

from pathlib import Path
import time

from builtin_interfaces.msg import Time
from foxglove_msgs.msg import CompressedPointCloud
from livekit_scarf_slam import live_input as _scarf_path  # noqa: F401  Puts ScaRF on sys.path.
from livekit_scarf_slam.paced_sender import PacedSender
from livekit_scarf_slam.submap_cloud import SubmapCloudPublisher
from livekit_scarf_slam.submap_cloud import SubmapInput
from livekit_scarf_slam.submap_cloud import SubmapPublishOptions
import rclpy
from scarf_slam.integrations.ros2_publishing import create_qos_profile
from scarf_slam.integrations.slam_bag import SlamBagData
from scarf_slam.mapping_app import ScaRFSLAM


class LiveInputClosed(RuntimeError):
    """The live input closed before the first keyframe."""


class LiveScaRFSLAM(ScaRFSLAM):
    """ScaRF-SLAM with live input. The session ends when the input is closed."""

    def __init__(self, slam_folder, live_input, submap_options=SubmapPublishOptions(),
                 publish_global_cloud=True, send_gap_s=0.005, warmup_s=2.0,
                 refresh_interval_s=1.0, max_rate_bytes_per_s=250_000.0):
        super().__init__(slam_folder)
        self._live_input = live_input
        self._submap_cloud = SubmapCloudPublisher(submap_options)
        self._chunk_publishers = {}
        self._publish_global_cloud = publish_global_cloud
        self._chunk_sender = PacedSender(
            self._send_chunk, send_gap_s, warmup_s, refresh_interval_s,
            max_rate_bytes_per_s, size=lambda msg: len(msg.data))

    def close(self):
        """Send the queued chunk messages and stop the sender thread."""
        self._chunk_sender.close()

    def _load_input_data(self):
        if not self.config.get('use_slam'):
            raise ValueError('live mapping needs use_slam: true in the config')
        print('Waiting for the first live keyframes...')
        if not self._live_input.wait():
            raise LiveInputClosed('the live input closed before the first keyframe')
        update = self._live_input.take()
        self.slam_bag_data = SlamBagData(
            bag_path=Path('<live>'),
            trajectory_snapshots=update.trajectory_snapshots,
            odometry=update.odometry,
            compressed_images=update.compressed_images,
            image_formats=update.image_formats,
        )
        self.session_start_time = min(update.compressed_images)
        self.ref_traj_snapshot_timestamps = self.slam_bag_data.trajectory_snapshot_timestamps
        self.odom_ref_poses_dict = self.slam_bag_data.odometry.copy()
        first_traj_timestamp = self.ref_traj_snapshot_timestamps[0]
        self.in_ref_poses_dict = self._load_traj_snapshot(first_traj_timestamp)
        self.current_traj_timestamp = first_traj_timestamp
        self.ref_timestamps = update.keyframe_timestamps
        print(f'session start time: {self.session_start_time}, {self.ref_timestamps[0]}')

    def _wait_for_more_input(self):
        has_keyframes = self._live_input.wait()
        # Also merge a snapshot without keyframes, for example the final trajectory.
        update = self._live_input.take()
        data = self.slam_bag_data
        data.trajectory_snapshots.update(update.trajectory_snapshots)
        data.compressed_images.update(update.compressed_images)
        data.image_formats.update(update.image_formats)
        data.odometry.update(update.odometry)
        odometry = update.odometry
        if self.fixed_mono_trajectory_scale is not None:
            odometry = self._scale_pose_dict(odometry, self.fixed_mono_trajectory_scale)
        self.odom_ref_poses_dict.update(odometry)
        self.ref_traj_snapshot_timestamps = data.trajectory_snapshot_timestamps
        last = self.ref_timestamps[-1]
        self.ref_timestamps.extend(ts for ts in update.keyframe_timestamps if ts > last)
        return has_keyframes

    def _publish_sampled_global_pointcloud(self, header_timestamp=None):
        # ScaRF-SLAM calls this after each batch and at the end. The PointCloud2
        # of the whole map stays local (RViz). ROS Portal sends the chunks.
        if self._publish_global_cloud:
            super()._publish_sampled_global_pointcloud(header_timestamp=header_timestamp)
        if not self.publish_ros2_pointcloud or self.ros2_node is None:
            return
        start = time.perf_counter()
        submaps = {
            key: SubmapInput(
                local_points=submap.local_points,
                colors=submap.colors,
                scale=submap.scale,
                world_from_anchor=self.transforms.pose_to_matrix(
                    self.out_ph_poses_dict[submap.anchor_key]))
            for key, submap in self.submaps.items()
            if submap.anchor_key in self.out_ph_poses_dict}
        chunks = self._submap_cloud.update(submaps)
        if header_timestamp is None:
            stamp = self.ros2_node.get_clock().now().to_msg()
        else:
            stamp = Time(sec=int(header_timestamp.sec), nanosec=int(header_timestamp.nsec))
        for chunk in chunks:
            msg = CompressedPointCloud()
            msg.timestamp = stamp
            msg.frame_id = self.ros2_pointcloud_frame_id
            msg.pose.orientation.w = 1.0
            msg.data = chunk.data
            msg.format = 'draco'
            self._chunk_publisher(chunk.topic)
            self._chunk_sender.put(chunk.topic, msg)
        rclpy.spin_once(self.ros2_node, timeout_sec=0.0)
        if chunks:
            print(f'Submap cloud: {len(chunks)} chunks of {len(submaps)} submaps, '
                  f'{sum(len(c.data) for c in chunks) / 1000.0:.0f} KB, '
                  f'largest {max(len(c.data) for c in chunks) / 1000.0:.1f} KB, '
                  f'{(time.perf_counter() - start) * 1000.0:.0f} ms')

    def _chunk_publisher(self, topic):
        if topic not in self._chunk_publishers:
            self._chunk_publishers[topic] = self.ros2_node.create_publisher(
                CompressedPointCloud, topic, create_qos_profile())
        return self._chunk_publishers[topic]

    def _send_chunk(self, topic, msg):
        # Runs in the PacedSender thread. The main thread creates the publishers.
        self._chunk_publishers[topic].publish(msg)
