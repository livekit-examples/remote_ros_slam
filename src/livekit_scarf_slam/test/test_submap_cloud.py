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

import math
import re

from livekit_scarf_slam.transforms import make_transform
import numpy as np
import pytest

DracoPy = pytest.importorskip('DracoPy')
submap_cloud = pytest.importorskip('livekit_scarf_slam.submap_cloud')

SubmapInput = submap_cloud.SubmapInput
SubmapPublishOptions = submap_cloud.SubmapPublishOptions

# A ROS 2 topic name: tokens of letters, digits, and underscores that do not
# start with a digit.
ROS_TOPIC = re.compile(r'^(/[A-Za-z_][A-Za-z0-9_]*)+$')


def _surface(count=20000, size=2.0, seed=0):
    """Return points on a wavy surface and their colors, in the anchor frame."""
    rng = np.random.default_rng(seed)
    xy = rng.uniform(0.0, size, (count, 2))
    z = 0.1 * np.sin(3.0 * xy[:, 0]) * np.cos(2.0 * xy[:, 1])
    points = np.column_stack([xy, z, np.ones(count)]).astype(np.float32)
    colors = rng.integers(0, 256, (count, 3), dtype=np.uint8)
    return points, colors


def _submap(scale=1.0, translation=(0.0, 0.0, 0.0), yaw_deg=0.0, seed=0, count=20000):
    points, colors = _surface(count=count, seed=seed)
    yaw = math.radians(yaw_deg)
    rotation = np.array([[math.cos(yaw), -math.sin(yaw), 0.0],
                         [math.sin(yaw), math.cos(yaw), 0.0],
                         [0.0, 0.0, 1.0]])
    return SubmapInput(points, colors, scale, make_transform(rotation, translation))


def _topics(chunks):
    return [chunk.topic for chunk in chunks]


def _submap_keys(chunks):
    return {chunk.submap_key for chunk in chunks}


def _world(submap, voxel_m):
    return submap_cloud.submap_world_cloud(submap, submap_cloud.submap_voxels(submap, voxel_m))


def test_chunk_topic_is_a_ros_name():
    assert submap_cloud.chunk_topic(7) == '/scarf_slam/submaps/chunk_007'
    assert submap_cloud.chunk_topic(1234) == '/scarf_slam/submaps/chunk_1234'
    assert ROS_TOPIC.match(submap_cloud.chunk_topic(7))


def test_chunks_take_slots_in_order_and_keep_them():
    options = SubmapPublishOptions(max_bytes=8000, refresh_count=0)
    publisher = submap_cloud.SubmapCloudPublisher(options)
    first = publisher.update({'1_0': _submap(seed=1)})
    slots = {(c.submap_key, c.chunk_index): c.topic for c in first}
    assert _topics(first) == [submap_cloud.chunk_topic(i) for i in range(len(first))]
    second = publisher.update({'1_0': _submap(seed=1, scale=1.5), '2_0': _submap(seed=2)})
    assert _submap_keys(second) == {'1_0', '2_0'}
    # Old chunks keep their slots. New chunks, of either submap, take the
    # next free slots in order.
    new_topics = []
    for chunk in second:
        if (chunk.submap_key, chunk.chunk_index) in slots:
            assert chunk.topic == slots[(chunk.submap_key, chunk.chunk_index)]
        else:
            new_topics.append(chunk.topic)
    assert new_topics == [submap_cloud.chunk_topic(len(first) + i)
                          for i in range(len(new_topics))]
    assert len(_topics(second)) == len(set(_topics(second)))


def test_voxel_average_means_positions_and_colors():
    points = np.array([[0.001, 0.0, 0.0, 1.0], [0.003, 0.002, 0.0, 1.0],
                       [0.05, 0.0, 0.0, 1.0], [0.051, 0.002, 0.004, 1.0]], dtype=np.float32)
    colors = np.array([[10, 20, 30], [20, 40, 61], [0, 0, 0], [255, 255, 255]], dtype=np.uint8)
    submap = SubmapInput(points, colors, 1.0, np.eye(4))
    world, mean_colors = _world(submap, 0.03)
    np.testing.assert_allclose(world, [[0.002, 0.001, 0.0], [0.0505, 0.001, 0.002]], atol=1e-6)
    np.testing.assert_array_equal(mean_colors, [[15, 30, 46], [128, 128, 128]])


def test_submap_points_skip_points_that_scarf_marks_invalid():
    points = np.array([[0.0, 0.0, 0.0, 1.0], [0.01, 0.0, 0.0, 0.0],
                       [0.02, 0.0, 0.0, np.nan], [np.nan, 0.0, 0.0, 1.0],
                       [0.03, 0.0, 0.0, 0.5]], dtype=np.float32)
    colors = np.arange(15, dtype=np.uint8).reshape(5, 3)
    cells = submap_cloud.submap_points(SubmapInput(points, colors, 1.0, np.eye(4)))
    np.testing.assert_array_equal(cells.means[:, 0], np.float32([0.0, 0.03]))
    np.testing.assert_array_equal(cells.color_sums, colors[[0, 4]])


def test_voxels_of_a_collapsed_submap_are_its_centroid():
    points, colors = _surface(count=100)
    world, _ = _world(SubmapInput(points, colors, 0.0, make_transform(translation=[1, 2, 3])),
                      0.03)
    np.testing.assert_allclose(world, [[1.0, 2.0, 3.0]], atol=1e-6)


def test_fine_cells_average_to_the_same_voxels():
    submap = _submap(scale=1.3)
    direct = submap_cloud.submap_voxels(submap, 0.03)
    fine = submap_cloud.submap_voxels(submap, 0.03 / 4)
    from_fine = submap_cloud.submap_voxels(submap, 0.03, fine)
    # Every point is in one fine cell, and the weighted means keep the centroid.
    assert from_fine.counts.sum() == direct.counts.sum() == len(submap.local_points)
    np.testing.assert_allclose(
        (from_fine.means * from_fine.counts[:, None]).sum(axis=0),
        (direct.means * direct.counts[:, None]).sum(axis=0), rtol=1e-6)
    assert abs(len(from_fine) - len(direct)) <= 0.05 * len(direct)


def test_submap_world_cloud_applies_scale_then_pose():
    voxels = submap_cloud.VoxelCells(
        means=np.array([[1.0, 0.0, 0.0], [0.0, 2.0, 0.0]]),
        color_sums=np.array([[30.0, 60.0, 90.0], [0.0, 0.0, 0.0]]),
        counts=np.array([3.0, 1.0]))
    submap = SubmapInput(np.zeros((2, 4), np.float32), np.zeros((2, 3), np.uint8), 0.5,
                         make_transform(np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0],
                                                  [0.0, 0.0, 1.0]]), [10.0, 0.0, 0.0]))
    points, colors = submap_cloud.submap_world_cloud(submap, voxels)
    np.testing.assert_allclose(points, [[10.0, 0.5, 0.0], [9.0, 0.0, 0.0]], atol=1e-6)
    np.testing.assert_array_equal(colors, [[10, 20, 30], [0, 0, 0]])


def test_submap_world_cloud_voxelizes_in_meters():
    # A scale of 0.1 puts the 2 m surface into 0.2 m, so a 3 cm voxel keeps
    # far fewer points than at scale 1.
    full, _ = _world(_submap(scale=1.0), 0.03)
    small, _ = _world(_submap(scale=0.1), 0.03)
    assert len(small) < len(full) / 10


def test_split_chunks_respects_max_bytes_and_keeps_points():
    options = SubmapPublishOptions(max_bytes=8000)
    points, colors = _world(_submap(), 0.01)
    chunks = submap_cloud.split_chunks(points, colors, options)
    assert len(chunks) > 1
    assert all(len(data) <= options.max_bytes for data, _ in chunks)
    assert sum(count for _, count in chunks) == len(points)
    decoded = [np.asarray(DracoPy.decode(data).points) for data, _ in chunks]
    assert sum(len(part) for part in decoded) == len(points)
    # The chunks are x slabs, so their x ranges barely overlap.
    ranges = sorted((part[:, 0].min(), part[:, 0].max()) for part in decoded)
    for (_, end), (start, _) in zip(ranges, ranges[1:]):
        assert start >= end - 0.05


def test_split_chunks_keeps_min_chunks():
    options = SubmapPublishOptions()
    points, colors = _world(_submap(count=500), 0.03)
    chunks = submap_cloud.split_chunks(points, colors, options, min_chunks=3)
    assert len(chunks) == 3
    assert sum(count for _, count in chunks) == len(points)


def test_update_sends_new_submaps_then_only_refreshes():
    publisher = submap_cloud.SubmapCloudPublisher(SubmapPublishOptions(refresh_count=1))
    submaps = {'1_0': _submap(seed=1), '2_0': _submap(seed=2), '3_0': _submap(seed=3)}
    first = publisher.update(submaps)
    assert _submap_keys(first) == {'1_0', '2_0', '3_0'}
    assert all(len(chunk.data) <= 32000 for chunk in first)

    # Nothing changed: each update refreshes the least recently sent submap.
    refreshed = [_submap_keys(publisher.update(submaps)) for _ in range(3)]
    assert refreshed == [{'1_0'}, {'2_0'}, {'3_0'}]


@pytest.mark.parametrize('change, sent', [
    ({'scale': 1.01}, False),
    ({'scale': 1.03}, True),
    ({'translation': (0.005, 0.0, 0.0)}, False),
    ({'translation': (0.02, 0.0, 0.0)}, True),
    ({'yaw_deg': 0.2}, False),
    ({'yaw_deg': 1.0}, True),
])
def test_update_resends_changed_submaps(change, sent):
    publisher = submap_cloud.SubmapCloudPublisher(SubmapPublishOptions(refresh_count=0))
    publisher.update({'1_0': _submap()})
    chunks = publisher.update({'1_0': _submap(**change)})
    assert bool(chunks) == sent


def test_update_never_shrinks_the_chunk_count():
    options = SubmapPublishOptions(voxel_m=0.03, max_bytes=2000, refresh_count=0)
    publisher = submap_cloud.SubmapCloudPublisher(options)
    large = _topics(publisher.update({'1_0': _submap(scale=1.0)}))
    # At scale 0.2 the submap fits one chunk, but every old chunk topic is
    # sent again, so none of them keeps the old points.
    small = _topics(publisher.update({'1_0': _submap(scale=0.2)}))
    assert len(large) > 1
    assert small == large


def test_update_skips_submaps_that_left():
    publisher = submap_cloud.SubmapCloudPublisher(SubmapPublishOptions(refresh_count=5))
    publisher.update({'1_0': _submap(seed=1), '2_0': _submap(seed=2)})
    chunks = publisher.update({'2_0': _submap(seed=2)})
    assert chunks and _submap_keys(chunks) == {'2_0'}


def test_options_reject_bad_values():
    with pytest.raises(ValueError):
        SubmapPublishOptions(voxel_m=0.0)
    with pytest.raises(ValueError):
        SubmapPublishOptions(max_bytes=0)
    with pytest.raises(ValueError):
        SubmapPublishOptions(refresh_count=-1)


def test_update_reuses_the_voxels_until_the_scale_drifts(monkeypatch):
    calls = []
    voxelize = submap_cloud.submap_voxels
    monkeypatch.setattr(
        submap_cloud, 'submap_voxels',
        lambda submap, voxel_m, cells=None: calls.append(
            (submap.scale, cells is None)) or voxelize(submap, voxel_m, cells))
    publisher = submap_cloud.SubmapCloudPublisher(SubmapPublishOptions(refresh_count=0))
    for scale in (1.0, 1.05, 1.09, 1.12, 2.5):
        assert publisher.update({'1_0': _submap(scale=scale)})
    # The first update builds the fine cells from the points, then the voxels
    # from the fine cells. 1.05 and 1.09 are within 10% of 1.0, so they reuse
    # the voxels. 1.12 averages the fine cells again. 2.5 is more than twice
    # the scale of the fine cells, so it builds them again.
    assert calls == [(1.0, True), (1.0, False), (1.12, False), (2.5, True), (2.5, False)]


@pytest.mark.parametrize('colors', [False, True])
def test_update_sends_colors_only_when_enabled(colors):
    publisher = submap_cloud.SubmapCloudPublisher(SubmapPublishOptions(colors=colors))
    chunks = publisher.update({'1_0': _submap()})
    decoded = DracoPy.decode(chunks[0].data)
    assert len(np.asarray(decoded.points)) == chunks[0].num_points
    assert (decoded.colors is not None) == colors


def test_colors_default_to_on():
    assert SubmapPublishOptions().colors
