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
Voxelized, size-capped Draco chunks of the ScaRF-SLAM submaps.

LiveKit DataTracks are not reliable, and a frame is lost when one of its
packets is lost, so the map goes out as many small messages instead of one
large cloud. Each submap is voxelized from its full-resolution points (the
mean position and color of the valid points in each voxel), moved to the map
frame, and split into spatially coherent Draco chunks of at most max_bytes.
Each chunk has its own topic, so Foxglove keeps the latest message of every
chunk and a resend replaces it in place. The topics are numbered slots,
/scarf_slam/submaps/chunk_000 and up, in the order the chunks first appear, so
every run uses the same names and a saved Foxglove layout keeps its settings.

A submap is sent again when its scale or its anchor pose changes by more than
a tolerance. With refresh_count, every update also sends the least recently
sent submaps again. The live mapper leaves that off, because its PacedSender
sends the last message of every chunk topic again on a timer, so that a lost
message or a late viewer catches up without acknowledgements. A submap never
shrinks to fewer chunks than it had, so no chunk topic keeps stale points.

The module has no ROS dependency. SubmapCloudPublisher returns the chunks to
send, and the caller publishes them.
"""

from dataclasses import dataclass
import math

from livekit_scarf_slam.draco_cloud import encode
from livekit_scarf_slam.draco_cloud import quantization_bits
import numpy as np

TOPIC_PREFIX = '/scarf_slam/submaps'

# Chunk splitting stops here. With the default 32 KB budget, it allows 2 MB
# per submap, which no voxelized submap reaches.
_MAX_CHUNKS = 64

# Voxelizing a full-resolution submap (about 800k points) takes tens of ms,
# and the scale optimization changes most scales after every batch, often by
# more than 10%. So each submap keeps fine cells of voxel_m / _FINE_DIVISOR
# (mean, color sum, and count), and the voxels average those, about 5x fewer.
# A count-weighted mean of cell means is the mean of their points, so only the
# assignment of the cells to voxels is approximate. The fine cells stay fine
# enough until the scale grows by _FINE_DIVISOR / 2, and the voxels stay valid
# until the scale changes by _REVOXEL_SCALE_TOLERANCE, which changes the voxel
# size by as much.
_FINE_DIVISOR = 4
_REVOXEL_SCALE_TOLERANCE = 0.1


@dataclass(frozen=True)
class SubmapPublishOptions:
    """Settings of the submap cloud publisher."""

    voxel_m: float = 0.05
    max_bytes: int = 32_000
    position_step_m: float = 0.004
    max_quantization_bits: int = 16
    color_bits: int = 5
    # The mean voxel colors, at color_bits per channel. They about double the
    # bytes. Off, Draco sends positions only, and Foxglove colors the points.
    colors: bool = True
    scale_tolerance: float = 0.02
    translation_tolerance_m: float = 0.01
    rotation_tolerance_deg: float = 0.5
    # The PacedSender of the live mapper sends every chunk again on a timer,
    # so the batches do not need to.
    refresh_count: int = 0

    def __post_init__(self):
        if self.voxel_m <= 0.0:
            raise ValueError('voxel_m must be positive')
        if self.max_bytes <= 0:
            raise ValueError('max_bytes must be positive')
        if self.refresh_count < 0:
            raise ValueError('refresh_count must not be negative')


@dataclass(frozen=True)
class SubmapInput:
    """One submap: points and colors in its anchor frame, scale, and map pose."""

    local_points: np.ndarray
    colors: np.ndarray
    scale: float
    world_from_anchor: np.ndarray


@dataclass(frozen=True)
class Chunk:
    """One Draco message to publish on topic, chunk_index of submap_key."""

    topic: str
    submap_key: str
    chunk_index: int
    data: bytes
    num_points: int


@dataclass
class _SentState:
    scale: float
    world_from_anchor: np.ndarray
    num_chunks: int
    sent_update: int


@dataclass(frozen=True)
class VoxelCells:
    """Voxel means in the local frame of a submap, in x-major voxel order."""

    means: np.ndarray
    color_sums: np.ndarray
    counts: np.ndarray

    def __len__(self):
        return len(self.counts)


@dataclass
class _VoxelState:
    num_points: int
    fine_scale: float
    fine: VoxelCells
    scale: float
    voxels: VoxelCells


def chunk_topic(slot):
    """Return the topic of one chunk slot."""
    return f'{TOPIC_PREFIX}/chunk_{slot:03d}'


def voxel_average(cells, voxel_size):
    """Return the count-weighted mean of cells in each occupied voxel_size voxel."""
    if len(cells) == 0:
        return cells
    means = cells.means
    if np.isfinite(voxel_size):
        grid = np.floor((means - means.min(axis=0)) / voxel_size).astype(np.int64)
        dims = grid.max(axis=0) + 1
        keys = (grid[:, 0] * dims[1] + grid[:, 1]) * dims[2] + grid[:, 2]
    else:
        keys = np.zeros(len(cells), dtype=np.int64)
    # np.unique sorts the keys, so the voxels are in x-major order, and
    # contiguous ranges of them are compact slabs of the submap.
    _, inverse = np.unique(keys, return_inverse=True)
    num_voxels = inverse.max() + 1
    counts = np.bincount(inverse, cells.counts, num_voxels)
    weighted = means * cells.counts[:, None]
    voxel_means = np.column_stack(
        [np.bincount(inverse, weighted[:, axis], num_voxels) for axis in range(3)])
    color_sums = np.column_stack(
        [np.bincount(inverse, cells.color_sums[:, axis], num_voxels) for axis in range(3)])
    return VoxelCells(voxel_means / counts[:, None], color_sums, counts)


def submap_points(submap):
    """Return the valid local points of a submap as one-point cells."""
    local = np.asarray(submap.local_points)
    # As in the ScaRF-SLAM publish sample, a point is valid when its
    # confidence (column 3) is finite and not 0.
    valid = np.isfinite(local[:, :3]).all(axis=1) & np.isfinite(local[:, 3]) & (local[:, 3] != 0.0)
    return VoxelCells(
        means=local[valid, :3].astype(np.float64),
        color_sums=np.asarray(submap.colors)[valid].astype(np.float64),
        counts=np.ones(int(valid.sum())))


def submap_voxels(submap, voxel_m, cells=None):
    """Return the voxel_m voxels of a submap, from its points or from finer cells."""
    if cells is None:
        cells = submap_points(submap)
    # The scale is uniform: a voxel_m grid over the metric points is a
    # voxel_m / scale grid over the local points.
    scale = abs(float(submap.scale))
    return voxel_average(cells, voxel_m / scale if scale > 0.0 else np.inf)


def submap_world_cloud(submap, voxels):
    """Return the map-frame points and the mean colors of submap voxels."""
    transform = np.asarray(submap.world_from_anchor, dtype=np.float64)
    world = (voxels.means * float(submap.scale)) @ transform[:3, :3].T + transform[:3, 3]
    colors = np.clip(np.round(voxels.color_sums / voxels.counts[:, None]), 0, 255)
    return (np.ascontiguousarray(world, dtype=np.float32),
            np.ascontiguousarray(colors, dtype=np.uint8))


def split_chunks(points, colors, options, min_chunks=1):
    """
    Return the Draco bytes and point counts of at least min_chunks chunks.

    Each chunk is at most options.max_bytes. The points must be in spatial
    order, so that each contiguous range is a compact part of the submap.
    """
    def encode_range(start, end):
        part = points[start:end]
        if len(part) == 0:
            # Draco needs a point. A chunk that a submap no longer fills keeps
            # its topic with one point, so that its old points disappear.
            part, part_colors = points[:1], colors[:1]
        else:
            part_colors = colors[start:end]
        bits = quantization_bits(part, options.position_step_m, options.max_quantization_bits)
        # The colors stay in every step before this one, so options.colors
        # only decides whether the message carries them.
        part_colors = np.ascontiguousarray(part_colors) if options.colors else None
        return encode(np.ascontiguousarray(part), part_colors, bits,
                      options.color_bits), len(part)

    whole, _ = encode_range(0, len(points))
    num_chunks = max(min_chunks, math.ceil(len(whole) / options.max_bytes))
    while True:
        bounds = np.linspace(0, len(points), num_chunks + 1).round().astype(int)
        chunks = [encode_range(start, end) for start, end in zip(bounds[:-1], bounds[1:])]
        if (all(len(data) <= options.max_bytes for data, _ in chunks)
                or num_chunks >= min(_MAX_CHUNKS, max(len(points), 1))):
            return chunks
        num_chunks += 1


def _rotation_angle_deg(a, b):
    relative = a[:3, :3].T @ b[:3, :3]
    cos_angle = np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0)
    return math.degrees(math.acos(cos_angle))


class SubmapCloudPublisher:
    """Decide which submaps to send, and build their chunks."""

    def __init__(self, options=SubmapPublishOptions()):
        self.options = options
        self._sent = {}
        self._voxels = {}
        self._slots = {}
        self._update = 0

    def _slot(self, key, index):
        # A chunk keeps its slot for the run, so a resend replaces it in place.
        return self._slots.setdefault((key, index), len(self._slots))

    def _voxels_of(self, key, submap):
        voxel_m = self.options.voxel_m
        scale = abs(float(submap.scale))
        state = self._voxels.get(key)
        if (state is None or state.num_points != len(submap.local_points)
                or scale > state.fine_scale * _FINE_DIVISOR / 2):
            fine = submap_voxels(submap, voxel_m / _FINE_DIVISOR)
            state = _VoxelState(
                num_points=len(submap.local_points), fine_scale=scale, fine=fine,
                scale=scale, voxels=submap_voxels(submap, voxel_m, fine))
            self._voxels[key] = state
        elif abs(scale - state.scale) > _REVOXEL_SCALE_TOLERANCE * state.scale:
            state.scale = scale
            state.voxels = submap_voxels(submap, voxel_m, state.fine)
        return state.voxels

    def _changed(self, state, submap):
        options = self.options
        if abs(submap.scale - state.scale) > options.scale_tolerance * abs(state.scale):
            return True
        old, new = state.world_from_anchor, np.asarray(submap.world_from_anchor)
        if np.linalg.norm(new[:3, 3] - old[:3, 3]) > options.translation_tolerance_m:
            return True
        return _rotation_angle_deg(old, new) > options.rotation_tolerance_deg

    def update(self, submaps):
        """
        Return the chunks to publish for submaps, a dict of key to SubmapInput.

        New and changed submaps are sent, and then the refresh_count submaps
        that were sent least recently.
        """
        self._update += 1
        due = [key for key, submap in submaps.items()
               if key not in self._sent or self._changed(self._sent[key], submap)]
        stale = sorted((state.sent_update, key) for key, state in self._sent.items()
                       if key in submaps and key not in due)
        due += [key for _, key in stale[:self.options.refresh_count]]

        chunks = []
        for key in due:
            submap = submaps[key]
            points, colors = submap_world_cloud(submap, self._voxels_of(key, submap))
            if len(points) == 0:
                continue
            previous = self._sent.get(key)
            encoded = split_chunks(points, colors, self.options,
                                   min_chunks=previous.num_chunks if previous else 1)
            chunks += [Chunk(chunk_topic(self._slot(key, index)), key, index, data, num_points)
                       for index, (data, num_points) in enumerate(encoded)]
            self._sent[key] = _SentState(
                scale=submap.scale,
                world_from_anchor=np.array(submap.world_from_anchor, dtype=np.float64),
                num_chunks=len(encoded),
                sent_update=self._update)
        return chunks
