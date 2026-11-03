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
Thread-safe buffer of live ScaRF-SLAM input.

The recorder node adds the same messages that it writes to the bag. The
buffer converts them with the helpers of the ScaRF-SLAM bag loader, so the
live mapper sees the same keys and poses as a replay of the bag.

A replay picks, for each batch, the first trajectory snapshot at or after the
batch. The final trajectory has the key of the last snapshot, and the bag
loader lets it replace that snapshot. So the keyframes of a snapshot become
available only when a later snapshot arrives, or when the input is closed
after the final trajectory. Then no batch uses a snapshot that the final
trajectory replaces later, and a live run maps the same as its replay.
"""

from dataclasses import dataclass
from dataclasses import field
import os
import sys
import threading

from livekit_scarf_slam.slam_bag import FINAL_TRAJECTORY_TOPIC
from livekit_scarf_slam.slam_bag import IMAGE_TOPIC
from livekit_scarf_slam.slam_bag import ODOMETRY_TOPIC
from livekit_scarf_slam.slam_bag import TRAJECTORY_TOPIC

# ScaRF-SLAM is a plain source folder. Append it, so that its top-level folders
# (for example launch/) never shadow ROS packages.
SCARF_ROOT = os.environ.get('SCARF_SLAM_ROOT', '/opt/scarf/ScaRF-SLAM')
if SCARF_ROOT not in sys.path:
    sys.path.append(SCARF_ROOT)

from scarf_slam.integrations.slam_bag import _path_msg_snapshot_key  # noqa: E402, I100
from scarf_slam.integrations.slam_bag import _path_msg_to_pose_dict  # noqa: E402
from scarf_slam.integrations.slam_bag import _pose_to_mapping_pose  # noqa: E402
from scarf_slam.integrations.slam_bag import _stamp_key  # noqa: E402


@dataclass
class LiveInputUpdate:
    """New input since the last take(), in the shape of SlamBagData fields."""

    odometry: dict = field(default_factory=dict)
    compressed_images: dict = field(default_factory=dict)
    image_formats: dict = field(default_factory=dict)
    trajectory_snapshots: dict = field(default_factory=dict)

    @property
    def keyframe_timestamps(self):
        """Return the sorted keys that have both an odometry pose and an image."""
        return sorted(key for key in self.odometry if key in self.compressed_images)


def _merge(target, source):
    """Add the data of the LiveInputUpdate source to target."""
    target.odometry.update(source.odometry)
    target.compressed_images.update(source.compressed_images)
    target.image_formats.update(source.image_formats)
    target.trajectory_snapshots.update(source.trajectory_snapshots)


class LiveSlamInput:
    """Collect ScaRF-SLAM input messages from one thread for a mapper in another thread."""

    def __init__(self):
        self._condition = threading.Condition()
        self._staged = LiveInputUpdate()
        self._pending = LiveInputUpdate()
        self._ready = LiveInputUpdate()
        self._closed = False

    def add(self, topic, msg):
        """Add one message of a ScaRF-SLAM input topic."""
        with self._condition:
            if topic == ODOMETRY_TOPIC:
                self._staged.odometry[_stamp_key(msg.header.stamp)] = (
                    _pose_to_mapping_pose(msg.pose.pose))
            elif topic == IMAGE_TOPIC:
                key = _stamp_key(msg.header.stamp)
                self._staged.compressed_images[key] = bytes(msg.data)
                self._staged.image_formats[key] = str(msg.format)
            elif topic == TRAJECTORY_TOPIC:
                # A later snapshot exists now, so the pending one is final.
                _merge(self._ready, self._pending)
                self._pending, self._staged = self._staged, LiveInputUpdate()
                self._pending.trajectory_snapshots[_path_msg_snapshot_key(msg)] = (
                    _path_msg_to_pose_dict(msg))
                self._condition.notify_all()
            elif topic == FINAL_TRAJECTORY_TOPIC:
                # As in the bag loader, the final trajectory replaces the
                # snapshot with the same key. close() makes it ready.
                _merge(self._pending, self._staged)
                self._staged = LiveInputUpdate()
                self._pending.trajectory_snapshots[_path_msg_snapshot_key(msg)] = (
                    _path_msg_to_pose_dict(msg))
            else:
                raise ValueError(f'unknown ScaRF-SLAM input topic: {topic}')

    def close(self):
        """End the input. wait() returns False after the ready data is taken."""
        with self._condition:
            _merge(self._ready, self._pending)
            self._pending = LiveInputUpdate()
            self._closed = True
            self._condition.notify_all()

    def wait(self, timeout=None):
        """
        Block until a new keyframe is ready or the input is closed.

        Return True if a keyframe is ready, and False if the input is closed and
        nothing is left, or the timeout expired.
        """
        with self._condition:
            self._condition.wait_for(
                lambda: self._ready.keyframe_timestamps or self._closed, timeout)
            return bool(self._ready.keyframe_timestamps)

    def take(self):
        """Return the ready input and clear it."""
        with self._condition:
            update, self._ready = self._ready, LiveInputUpdate()
            return update
