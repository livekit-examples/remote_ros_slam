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

import numpy as np
import pytest

DracoPy = pytest.importorskip('DracoPy')
draco_cloud = pytest.importorskip('livekit_scarf_slam.draco_cloud')

# The layout of the ScaRF-SLAM cloud: x, y, z float32 and a packed rgb uint32.
OFFSETS = {'x': 0, 'y': 4, 'z': 8, 'rgb': 12}
POINT_STEP = 16


def _cloud(count=2000, seed=0):
    rng = np.random.default_rng(seed)
    points = rng.uniform([-3.0, -3.0, 0.0], [3.0, 3.0, 2.0], (count, 3)).astype(np.float32)
    # Distinct colors, so that sorting by color recovers the point order.
    rgb = (np.arange(count, dtype=np.uint32) * 7919) % (1 << 24)
    colors = np.stack([rgb >> 16, rgb >> 8, rgb], axis=1).astype(np.uint8)
    return points, colors


def _pack(points, colors):
    cloud = np.zeros(len(points), dtype=np.dtype(
        [('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')]))
    cloud['x'], cloud['y'], cloud['z'] = points.T
    cloud['rgb'] = (colors[:, 0].astype(np.uint32) << 16
                    | colors[:, 1].astype(np.uint32) << 8 | colors[:, 2])
    return cloud.tobytes()


def _sorted_rows(array):
    return array[np.lexsort(array.T[::-1])]


def test_unpack_xyzrgb():
    points, colors = _cloud()
    out_points, out_colors = draco_cloud.unpack_xyzrgb(
        _pack(points, colors), POINT_STEP, OFFSETS)
    np.testing.assert_array_equal(out_points, points)
    np.testing.assert_array_equal(out_colors, colors)


def test_unpack_drops_non_finite_points():
    points, colors = _cloud(10)
    points[3] = np.nan
    points[7, 2] = np.inf
    out_points, out_colors = draco_cloud.unpack_xyzrgb(
        _pack(points, colors), POINT_STEP, OFFSETS)
    keep = [i for i in range(10) if i not in (3, 7)]
    np.testing.assert_array_equal(out_points, points[keep])
    np.testing.assert_array_equal(out_colors, colors[keep])


@pytest.mark.parametrize('extent, step, max_bits, expected', [
    (6.7, 0.004, 16, 11),   # the ScaRF-SLAM map: 1676 steps
    (50.0, 0.004, 16, 14),  # 12501 steps
    (500.0, 0.004, 16, 16),  # clamped to max_bits
    (0.1, 0.004, 16, 8),    # never fewer than 8 bits
])
def test_quantization_bits(extent, step, max_bits, expected):
    points = np.array([[0.0, 0.0, 0.0], [extent, 0.5, 0.5]], dtype=np.float32)
    assert draco_cloud.quantization_bits(points, step, max_bits) == expected


@pytest.mark.parametrize('kd_tree', [True, False])
@pytest.mark.parametrize('color_bits', [8, 5])
def test_encode_round_trip(kd_tree, color_bits):
    points, colors = _cloud()
    bits = draco_cloud.quantization_bits(points, 0.004, 16)
    decoded = DracoPy.decode(
        draco_cloud.encode(points, colors, bits, color_bits=color_bits, kd_tree=kd_tree))
    out_points = np.asarray(decoded.points)
    out_colors = np.asarray(decoded.colors)
    assert out_points.shape == points.shape
    # Foxglove needs the alpha field for its RGBA color mode.
    assert out_colors.shape == (len(colors), 4)
    np.testing.assert_array_equal(out_colors[:, 3], 255)
    out_colors = np.ascontiguousarray(out_colors[:, :3])

    # The KD-tree encoder reorders the points, so compare them sorted by color
    # after the color reduction, which stays exact.
    mask = np.uint8((0xFF << (8 - color_bits)) & 0xFF)
    expected_colors = colors & mask
    if kd_tree:
        order, out_order = np.lexsort(expected_colors.T), np.lexsort(out_colors.T)
        if color_bits < 8:
            # Reduced colors repeat, so the point order is not recoverable.
            np.testing.assert_array_equal(
                _sorted_rows(out_colors), _sorted_rows(expected_colors))
            return
        points, expected_colors = points[order], expected_colors[order]
        out_points, out_colors = out_points[out_order], out_colors[out_order]
    np.testing.assert_array_equal(out_colors, expected_colors)
    step = (points.max(axis=0) - points.min(axis=0)).max() / (2 ** bits - 1)
    assert np.abs(out_points - points).max() <= step


def test_encode_rejects_bad_color_bits():
    points, colors = _cloud(10)
    with pytest.raises(ValueError):
        draco_cloud.encode(points, colors, 11, color_bits=0)


def test_encode_without_colors_has_positions_only():
    points, _ = _cloud()
    decoded = DracoPy.decode(draco_cloud.encode(points, None, 11))
    assert np.asarray(decoded.points).shape == points.shape
    assert decoded.colors is None
