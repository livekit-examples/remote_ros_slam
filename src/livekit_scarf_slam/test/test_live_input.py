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

import threading

from livekit_scarf_slam.bag_writer import SlamBagWriter
from livekit_scarf_slam.ros_messages import ns_from_time
from livekit_scarf_slam.ros_messages import odometry_msg
from livekit_scarf_slam.ros_messages import path_msg
from livekit_scarf_slam.ros_messages import time_from_ns
from livekit_scarf_slam.slam_bag import FINAL_TRAJECTORY_TOPIC
from livekit_scarf_slam.slam_bag import IMAGE_TOPIC
from livekit_scarf_slam.slam_bag import ODOMETRY_TOPIC
from livekit_scarf_slam.slam_bag import TRAJECTORY_TOPIC
from livekit_scarf_slam.transforms import make_transform
import pytest
from sensor_msgs.msg import CompressedImage

# The live input needs the ScaRF-SLAM sources of the slam image.
live_input = pytest.importorskip('livekit_scarf_slam.live_input')
slam_bag = pytest.importorskip('scarf_slam.integrations.slam_bag')

T0 = 1_760_000_000_000_000_000
STEP_NS = 333_000_000


def _pose(x, y=0.0):
    return make_transform(translation=[x, y, 0.5])


def _image_msg(stamp_ns):
    msg = CompressedImage()
    msg.header.stamp = time_from_ns(stamp_ns)
    msg.format = 'jpeg'
    msg.data.frombytes(f'jpeg@{stamp_ns}'.encode())
    return msg


def _session_messages():
    """Return (topic, msg, stamp_ns) in the order of the recorder for 3 updates."""
    messages = []
    trajectory = []
    for update in range(3):
        for index in range(2):
            stamp_ns = T0 + (2 * update + index) * STEP_NS
            odom = _pose(0.2 * (2 * update + index))
            odom_msg = odometry_msg(odom, stamp_ns, 'odom', 'cam0')
            messages.append((ODOMETRY_TOPIC, odom_msg, stamp_ns))
            messages.append((IMAGE_TOPIC, _image_msg(stamp_ns), stamp_ns))
            trajectory.append((stamp_ns, odom))
        # A loop closure shifts the whole trajectory a little in each update.
        trajectory = [(stamp, pose @ _pose(0.0, 0.01)) for stamp, pose in trajectory]
        snapshot = path_msg(trajectory, 'map')
        messages.append((TRAJECTORY_TOPIC, snapshot, ns_from_time(snapshot.header.stamp)))
    # RTAB-Map can still correct the graph after the last keyframe, so the final
    # trajectory differs from the last snapshot, and has the same key.
    final = path_msg([(stamp, pose @ _pose(0.0, 0.01)) for stamp, pose in trajectory], 'map')
    messages.append((FINAL_TRAJECTORY_TOPIC, final, ns_from_time(final.header.stamp)))
    return messages


def _feed(messages):
    buffer = live_input.LiveSlamInput()
    for topic, msg, _ in messages:
        buffer.add(topic, msg)
    return buffer


def _snapshot_key(path):
    return slam_bag._path_msg_snapshot_key(path)


def _poses_equal(left, right):
    assert left.keys() == right.keys()
    for key in left:
        assert left[key].pos == pytest.approx(right[key].pos)
        assert left[key].quat == pytest.approx(right[key].quat)


def test_live_input_matches_the_bag_loader(tmp_path):
    messages = _session_messages()
    writer = SlamBagWriter(tmp_path / 'slam_bag')
    for topic, msg, stamp_ns in messages:
        writer.write(topic, msg, stamp_ns)
    writer.close()
    loaded = slam_bag.load_slam_bag(tmp_path / 'slam_bag')

    buffer = _feed(messages)
    buffer.close()
    assert buffer.wait(timeout=0.0)
    live = buffer.take()

    assert live.compressed_images == loaded.compressed_images
    assert live.image_formats == loaded.image_formats
    _poses_equal(live.odometry, loaded.odometry)
    assert live.trajectory_snapshots.keys() == loaded.trajectory_snapshots.keys()
    for key in live.trajectory_snapshots:
        _poses_equal(live.trajectory_snapshots[key], loaded.trajectory_snapshots[key])
    assert live.keyframe_timestamps == sorted(loaded.odometry)


def test_keyframes_are_ready_only_after_the_next_snapshot():
    messages = _session_messages()
    buffer = _feed(messages[:5])  # Two keyframes and their snapshot.
    assert not buffer.wait(timeout=0.0)
    assert buffer.take().keyframe_timestamps == []

    buffer.add(*messages[9][:2])  # The snapshot of the second update.
    assert buffer.wait(timeout=0.0)
    update = buffer.take()
    assert update.keyframe_timestamps == sorted(update.odometry)
    assert len(update.keyframe_timestamps) == 2
    assert list(update.trajectory_snapshots) == [_snapshot_key(messages[4][1])]
    assert not buffer.wait(timeout=0.0)


def test_the_final_trajectory_replaces_the_last_snapshot_on_close():
    messages = _session_messages()
    buffer = _feed(messages[:-1])
    assert len(buffer.take().keyframe_timestamps) == 4
    buffer.add(*messages[-1][:2])
    assert not buffer.wait(timeout=0.0)  # The last keyframes wait for close().
    buffer.close()

    assert buffer.wait()
    update = buffer.take()
    assert len(update.keyframe_timestamps) == 2
    final = messages[-1][1]
    assert list(update.trajectory_snapshots) == [_snapshot_key(final)]
    _poses_equal(update.trajectory_snapshots[_snapshot_key(final)],
                 slam_bag._path_msg_to_pose_dict(final))
    assert not buffer.wait()  # Closed and empty, so this returns at once.


def test_wait_wakes_up_when_another_thread_adds_a_snapshot():
    messages = _session_messages()
    buffer = _feed(messages[:9])
    adder = threading.Timer(0.05, buffer.add, args=messages[9][:2])
    adder.start()
    try:
        assert buffer.wait(timeout=5.0)
    finally:
        adder.join()


def test_unknown_topic_is_an_error():
    with pytest.raises(ValueError):
        live_input.LiveSlamInput().add('/other', _image_msg(T0))
