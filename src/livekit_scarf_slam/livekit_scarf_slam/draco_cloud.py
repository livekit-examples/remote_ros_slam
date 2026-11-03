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
Draco compression of colored point clouds for foxglove_msgs/CompressedPointCloud.

Foxglove reads the Draco POSITION attribute as x, y, z and a COLOR attribute
of 4 components as red, green, blue, alpha. Its 3D panel colors points by
their own colors only in the "RGBA (separate fields)" color mode, which needs
all four fields, so the colors go out as RGBA with an opaque alpha. With RGB
only, the fields appear, but only as Color map inputs. The constant alpha adds
about 20 bytes per message. DracoPy writes both attribute types.

Draco quantizes the positions over the bounding box of the cloud, so the
number of quantization bits comes from the extent of the cloud and a target
step size, and the step stays the same while the map grows.

The KD-tree encoder (preserve_order off) is about twice as compact as the
sequential encoder on the ScaRF-SLAM map, but with 8-bit colors it slows down
faster than linearly with the point count (3.8 s for 858k points). Dropping
the colors to 5 bits per channel brings it to about 0.5 s with no visible
change in Foxglove.
"""

import math

import DracoPy
import numpy as np

# Draco accepts 1 to 30 bits. Fewer than 8 bits is never useful for a map.
_MIN_QUANTIZATION_BITS = 8
_MAX_QUANTIZATION_BITS = 30


def unpack_xyzrgb(data, point_step, offsets):
    """
    Return the finite (N, 3) float32 points and (N, 3) uint8 colors of a cloud.

    data is the PointCloud2 data, point_step its point stride, and offsets maps
    x, y, z (float32) and rgb (packed 0x00RRGGBB) to their byte offsets.
    """
    dtype = np.dtype({
        'names': ['x', 'y', 'z', 'rgb'],
        'formats': ['<f4', '<f4', '<f4', '<u4'],
        'offsets': [offsets[name] for name in ('x', 'y', 'z', 'rgb')],
        'itemsize': point_step,
    })
    cloud = np.frombuffer(data, dtype=dtype)
    points = np.stack([cloud['x'], cloud['y'], cloud['z']], axis=1)
    rgb = cloud['rgb']
    colors = np.stack([rgb >> 16, rgb >> 8, rgb], axis=1).astype(np.uint8)
    finite = np.isfinite(points).all(axis=1)
    return np.ascontiguousarray(points[finite]), np.ascontiguousarray(colors[finite])


def quantization_bits(points, step_m, max_bits):
    """Return the fewest bits that quantize the extent of points at step_m."""
    extent = float((points.max(axis=0) - points.min(axis=0)).max()) if len(points) else 0.0
    bits = math.ceil(math.log2(extent / step_m + 1.0)) if extent > 0.0 else 1
    return min(max(bits, _MIN_QUANTIZATION_BITS), max_bits, _MAX_QUANTIZATION_BITS)


def encode(points, colors, bits, color_bits=8, kd_tree=True):
    """
    Return the Draco bytes of a cloud, colors reduced to color_bits.

    colors are (N, 3) RGB or (N, 4) RGBA uint8. RGB gets an opaque alpha. With
    colors None, the cloud has positions only.
    """
    if not 1 <= color_bits <= 8:
        raise ValueError(f'color_bits must be between 1 and 8, got {color_bits}')
    if colors is not None:
        mask = np.uint8((0xFF << (8 - color_bits)) & 0xFF)
        colors = np.asarray(colors, dtype=np.uint8) & mask
        if colors.shape[1] == 3:
            colors = np.column_stack([colors, np.full(len(colors), 255, np.uint8)])
        colors = np.ascontiguousarray(colors)
    return bytes(DracoPy.encode(
        points, colors=colors, quantization_bits=bits, compression_level=7,
        preserve_order=not kd_tree, create_metadata=False))
