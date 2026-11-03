# livekit_scarf_slam

This package runs a
[ScaRF-SLAM](https://github.com/ori-drs/ScaRF-SLAM) demo through `ros_portal`.
The robot (a Jetson) sends one stereo camera stream through LiveKit. A SLAM
host does the localization and mapping in the `slam` container of this
repository.

The robot has no IMU, so the demo does not use OpenVINS, the default
ScaRF-SLAM pose source. RTAB-Map stereo SLAM (stereo odometry, loop closure,
and pose graph optimization) gives the poses instead.

The data flows in this order:

1. The camera driver on the robot publishes one side-by-side (SBS) JPEG frame
   on `/stereo_camera`. An SBS frame holds the left and right images next
   to each other, so both eyes have one stamp.
2. `ros_portal` sends the topic as the LiveKit video track `stereo_camera`.
3. `ros_portal` on the SLAM host publishes the track on `/stereo_camera`.
4. `stereo_rectify_node` splits each frame and rectifies both fisheye images
   for RTAB-Map. Rectified images have matching points on the same row.
5. RTAB-Map calculates the poses.
6. `slam_bag_recorder_node` writes the ScaRF-SLAM input bag. For each RTAB-Map
   keyframe, the bag has the left fisheye image, the odometry pose, and the
   loop-corrected trajectory.
7. ScaRF-SLAM builds the dense map, either live from the same keyframes or
   after the run from the bag. See [Replay Or Live](#replay-or-live).

## Python Environment

The `slam` container of this repository has all dependencies. See the
[repository README](../../README.md) to build and start it. You run all
commands in this page in the `slam` container, as `root`.

The upstream page uses a conda environment with Python 3.11. The container
does not use conda. ROS 2 Jazzy, `rclpy`, and colcon use the system Python
3.12 at `/usr/bin/python3`, and a different Python version breaks them.

Instead, the image has a venv (an isolated Python package folder) at
`/opt/slam_venv`. The image makes the venv from the system Python with
`--system-site-packages`. With this flag, the venv sees the ROS Python
packages and colcon. The ScaRF-SLAM packages are in the venv only. The image
sets `VIRTUAL_ENV` and `PATH`, so the venv is active in every shell.

The image has these Python packages in the venv:

- PyTorch, and `xformers` on `x86_64` only.
- Depth Anything 3, in editable mode from `/opt/scarf/Depth-Anything-3`.
- `rosbags`, `open3d`, `gtsam`, and `vismatch` 1.3.1 for ScaRF-SLAM. Later
  `vismatch` versions need `setuptools>=83`, and colcon cannot use it.

ScaRF-SLAM is in `/opt/scarf/ScaRF-SLAM`. `scarf_slam.repos` in the
repository root pins the commits of both clones.

Depth Anything 3 requires `numpy<2`, and the Jazzy apt packages, for example
`cv_bridge`, use numpy 1.x. torch needs `setuptools>=77.0.3`, and
`colcon-core` needs `setuptools<80`. The image sets `PIP_CONSTRAINT` to
`/opt/slam_venv/constraints.txt`, so each `pip install` keeps these limits.
Do not upgrade numpy to 2.x in the venv.

To make sure that the packages import, run this command:

```bash
python -c "import torch, depth_anything_3.api, open3d, gtsam, rosbags, vismatch, numpy, setuptools; print(torch.__version__, torch.cuda.is_available(), numpy.__version__, setuptools.__version__)"
```

The warning about `gsplat` is normal. ScaRF-SLAM does not need 3D Gaussian
splatting rendering. If you start the container with `ENABLE_GPU=true` on a
host with an NVIDIA GPU, `torch.cuda.is_available()` is `True`.

On an `aarch64` host, for example a Mac with Apple silicon, PyTorch runs on
the CPU only. `pycolmap` has no Linux `aarch64` wheel, so the image installs
Depth Anything 3 without it. The image applies
`patches/depth_anything_3_optional_pycolmap.patch`, so that
`depth_anything_3.api` imports without `pycolmap`. COLMAP export does not work
on this host. On `aarch64`, `pip check` reports
`depth-anything-3 0.0.0 requires pycolmap, which is not installed`. This
message is expected.

`vismatch` needs `poselib`, which also has no Linux `aarch64` wheel, so the
image builds `poselib` from source on `aarch64`. For the real-time demo, use
an `x86_64` host with an NVIDIA GPU.

Optional: to add sky masks to the Depth Anything 3 output, edit `da3.py` in
`/opt/scarf/Depth-Anything-3`. In the `forward` function of
`NestedDepthAnything3Net`, add this line just before the `return` statement:

```python
output.sky = metric_output.sky
```

The edit is lost when you recreate the container. The upstream page also
points to [pull request #7](https://github.com/ori-drs/ScaRF-SLAM/pull/7). It
is a faster implementation for large environments, but nobody reviewed it
independently.

## Build The Workspace

The image builds this package at build time. After you change the sources,
build the package again in the `slam` container. ScaRF-SLAM and Depth
Anything 3 are plain Python projects, so colcon does not build them.

1. Build the package:

   ```bash
   cd /remote_ros_slam
   colcon build --symlink-install --packages-up-to livekit_scarf_slam
   ```

2. Source the workspace:

   ```bash
   source install/setup.bash
   ```

3. Run the tests:

   ```bash
   colcon test --packages-select livekit_scarf_slam
   colcon test-result --verbose
   ```

## Configuration

Before the first run, calibrate the stereo camera. See
[calibration.md](calibration.md). The launch file reads the calibration from
`/remote_ros_slam/calibration/stereo_calibration.json` by default. The
`calibration/` folder of the repository is mounted there.

| File | Purpose |
| --- | --- |
| `config/ros_portal/edge_robot.yaml` | ROS Portal on the robot. Sends `/stereo_camera` as a video track at 8000 kbps. |
| `config/ros_portal/slam_host.yaml` | ROS Portal on the SLAM host. Receives the `stereo_camera` track. Sends `/tf`, the RTAB-Map odometry, trajectory, and map, and the ScaRF-SLAM output as data tracks, and no video: viewers watch the robot's `stereo_camera` track. This file is the default `LIVEKIT_CONFIG`. |
| `config/rtabmap/stereo.yaml` | RTAB-Map parameters. `frame_id` is `stereo_link`. RTAB-Map adds at most 3 keyframes per second. |
| `config/scarf_slam/cobra_stereo.yaml` | ScaRF-SLAM configuration. The camera blocks come from the calibration. ScaRF-SLAM publishes `/scarf_slam/clouds`, `/scarf_slam/slam_poses`, and `/scarf_slam/images` while it runs. |

The camera driver on the robot must use the calibrated mode, 3840x1080. A
frame rate of 15 fps is enough for RTAB-Map. If LiveKit lowers the stream
resolution, `stereo_rectify_node` scales the intrinsics, and the rectified
images keep the same size. A frame with a different aspect ratio is dropped.

`stereo_slam.launch.py` has these arguments:

| Argument | Default | Description |
| --- | --- | --- |
| `calibration_path` | `/remote_ros_slam/calibration/stereo_calibration.json` | The stereo calibration. |
| `sbs_topic` | `/stereo_camera` | The SBS JPEG topic. |
| `output_dir` | `$SLAM_RUNS_DIR/<date>_<time>` | The run folder. It gets `slam_bag` and `rtabmap.db`. |
| `decode_reduction` | `2` | Decode the JPEG at 1/2 size. Use 1, 2, 4, or 8. |
| `rect_scale` | `0.5` | Size of the rectified images. 0.5 gives 960x540. |
| `rtabmap_viz` | `false` | Start the RTAB-Map GUI. The GUI needs a display. |
| `scarf_live` | `false` | Run ScaRF-SLAM live, and save the map on `Ctrl+C`. See [Replay Or Live](#replay-or-live). |
| `scarf_config` | `config/scarf_slam/cobra_stereo.yaml` | ScaRF-SLAM configuration of the live mapping. The node copies it to `<output_dir>/scarf_config.yaml`. |
| `scarf_live_params` | empty | Optional ROS parameter file for the live node, for example the `submap_*` settings of [Map cloud over LiveKit](#map-cloud-over-livekit). Use the node name `scarf_live_mapping` or `/**`. |

Launch arguments use `:=`, for example `output_dir:=/remote_ros_slam/runs/run1`.

The image sets `SLAM_RUNS_DIR` to `/remote_ros_slam/runs`. The `runs/` folder
of the repository is mounted there, so the run output stays on the host.
Without `SLAM_RUNS_DIR`, the default is `~/scarf_slam_runs`.

The launch file does not start ROS Portal. ROS Portal runs in the
`ros_portal` container of the repository.

The static transforms are `stereo_link` to `cam0_optical` (the left lens)
to `cam0_rect_optical` (the rectified left image). `stereo_link` has x
forward, y left, and z up. To connect the camera to the rover body, publish a
transform from your robot base frame to `stereo_link`.

## Run

1. On the robot, start the camera driver, so that it publishes
   `/stereo_camera`.

2. On the robot, from the repository root, start ROS Portal with the robot
   configuration:

   ```bash
   LIVEKIT_URL=<url> LIVEKIT_TOKEN=<edge_robot token> \
     LIVEKIT_CONFIG=./src/livekit_scarf_slam/config/ros_portal/edge_robot.yaml \
     docker compose up ros_portal
   ```

3. On the SLAM host, from the repository root, start the `slam` container and
   ROS Portal:

   ```bash
   LIVEKIT_URL=<url> LIVEKIT_TOKEN=<slam_host token> docker compose up
   ```

4. On the SLAM host, open a shell in the `slam` container, and start the SLAM
   nodes:

   ```bash
   docker exec -it slam bash
   ros2 launch livekit_scarf_slam stereo_slam.launch.py
   ```

5. Drive the rover. Keep the first two seconds slow, so that the odometry
   can start.

6. Stop the launch with `Ctrl+C`. The recorder writes the final trajectory
   and closes the bag. The log line `Closed <path> with <n> keyframes`
   shows the bag path.

7. Build the dense map from the bag, in the `slam` container:

   ```bash
   cd /opt/scarf/ScaRF-SLAM
   python3 run_mapping.py --slam_folder <output_dir> \
     --input_bag <output_dir>/slam_bag \
     --config /remote_ros_slam/src/livekit_scarf_slam/config/scarf_slam/cobra_stereo.yaml
   ```

If the container runs on the CPU, Depth Anything 3 is slow in step 7, and
live mapping cannot keep up. For
the real-time demo, use an `x86_64` host with an NVIDIA GPU and
`ENABLE_GPU=true`.

To test the SLAM host without the robot, record the SBS topic once with
`ros2 bag record /stereo_camera`. Then start only the `slam`
container with `docker compose up slam`, start the launch, and play the bag.

If RTAB-Map logs `quality=0` for many frames, the odometry lost track. Drive
slower, add light, or point the camera at surfaces with more texture.

## Replay Or Live

The command decides the mode. Nothing is detected.

- **Replay:** `run_mapping.py --input_bag <bag>`, as in step 7. ScaRF-SLAM
  reads the closed bag and stops when it runs out of keyframes. This is the
  unchanged upstream code path.
- **Live:** `stereo_slam.launch.py scarf_live:=true`. The launch starts
  `scarf_live_mapping_node` instead of `slam_bag_recorder_node`. The node
  writes the same `slam_bag`, and also gives each keyframe to ScaRF-SLAM in
  the same process, so the dense map grows while the rover drives:

  ```bash
  ros2 launch livekit_scarf_slam stereo_slam.launch.py scarf_live:=true \
    output_dir:=/remote_ros_slam/runs/live1
  ```

  ScaRF-SLAM loads its model after the first keyframe, and then maps a batch
  of 6 keyframes at a time. A keyframe goes to ScaRF-SLAM only after the
  next RTAB-Map update (see below), so the map trails the rover by a few
  seconds. It
  publishes `/scarf_slam/clouds`, `/scarf_slam/slam_poses`, and
  `/scarf_slam/images`. The map also goes out in small chunks on
  `/scarf_slam/submaps/...` (see below), and ROS Portal sends the chunks and the
  poses to the room. `/scarf_slam/images` stays local. Press `Ctrl+C`
  once. ScaRF-SLAM maps the rest of the keyframes and saves the map to
  `<output_dir>/recon/rtabmap_slam`. The launch waits up to 120 seconds for
  this before it stops the node.

The live node and a replay of its bag process the same keyframes, with the
same trajectory snapshot for each batch, so they give the same map:

- The final trajectory has the key of the last snapshot, and a replay uses
  it in place of that snapshot. So the live input holds the keyframes of a
  snapshot until a later snapshot arrives, or until `Ctrl+C`. No live batch
  uses a snapshot that the final trajectory replaces.
- The node copies `scarf_config` to `<output_dir>/scarf_config.yaml` before
  it maps. The launch reads `scarf_config` from the install space, which is a
  copy of `src/` made at the last `colcon build` (without
  `--symlink-install`). Replay with the copy in the run folder, not with the
  file in `src/`.

To compare a live run with its replay, write the replay to another folder,
because the replay also saves to `<slam_folder>/recon`:

```bash
cd /opt/scarf/ScaRF-SLAM
python3 run_mapping.py --slam_folder /remote_ros_slam/runs/live1_replay \
  --input_bag /remote_ros_slam/runs/live1/slam_bag \
  --config /remote_ros_slam/runs/live1/scarf_config.yaml
```

Depth Anything 3 and the matchers run on the GPU, where the results can
change in the last bits from run to run. Two replays of one bag can also
differ that little, so compare the live run to the replay with a tolerance.

`patches/scarf_slam_live_hooks.patch` adds the two input hooks that the live
node overrides in `livekit_scarf_slam/live_mapper.py`. Replay does not use
them.

### Map cloud over LiveKit

LiveKit DataTracks are not reliable, and a frame is lost when one of its
packets is lost, so a multi-megabyte map message rarely arrives. ScaRF-SLAM
republishes its whole map after every batch, about 14 MB per message as a
`PointCloud2` after a minute of mapping. That `/scarf_slam/clouds` stays
local (RViz). For the room, the live node sends the map as small messages
instead (`livekit_scarf_slam/submap_cloud.py`):

- Each submap is voxelized (5 cm by default) from its full-resolution points,
  moved to `map`, and split into spatially compact Draco chunks of at most
  32 KB.
- Each chunk has its own `foxglove_msgs/CompressedPointCloud` topic in the
  `map` frame. The Foxglove 3D panel keeps the latest message of each topic,
  so the chunks together show the whole map, and a resend replaces a chunk in
  place. The topics are numbered slots, `/scarf_slam/submaps/chunk_000` and
  up, in the order the chunks first appear, so every run uses the same names
  (see [Foxglove layout](#foxglove-layout)).
- A submap is sent when it is new, when its scale changes by more than 2%, or
  when its anchor moves by more than 1 cm or 0.5 degrees.
- A sender thread (`livekit_scarf_slam/paced_sender.py`) sends the chunk
  messages at most 2 Mbps and 5 ms apart, and sends the last message of a new topic again after
  2 s. Every second, it also sends the least recently sent chunk again, so a
  lost chunk or a late viewer catches up without acknowledgements, also after
  the mapping stops.

Voxelization replaces the random 5% sample of `/scarf_slam/clouds`, so the
point density is even and the map grows with the mapped surface, not with
the run time. Each voxel holds the mean position and color of its points,
and, as in the ScaRF-SLAM sample, only points with a valid confidence count.
On a recorded 10-minute run with 13 submaps, 10 cm voxels without colors
sent 97 messages on 13 chunk topics: 0.9 KB median, 10.8 KB largest, 0.18 MB
for the whole run (1.2 MB with 3 cm voxels). Every valid submap point is
within 59 mm (p99) of a 10 cm voxel point, and half are within 22 mm.

Delivery, measured through LiveKit Cloud on the `camera-only` bag (84 s),
with a second ROS Portal as the viewer. With the defaults (5 cm, colors), the
map was 83k points on 23 chunk topics, the chunks 14.6 KB median and 31 KB
largest, up to 190 KB per batch, and the publish step took about 50 ms per
batch on the ScaRF-SLAM thread. 1.5% of the messages were lost, and all 23
topics ended with the right content on the viewer. At 10 cm with colors, the
map was 23k points on 20 topics, 5.5 KB median chunks.

The other topics that the SLAM host sends compete with the chunks. With
video tracks from the host (about 5 Mbps), three runs lost 34% to 46% of the
chunks while mapping, and with `/rtabmap/odom_last_frame` (15 KB at 5 Hz), a
synthetic chunk stream lost about twice as many. So `slam_host.yaml` sends
no video and none of the RTAB-Map point clouds. The resend timer brought
every chunk topic to the right content on the viewer in every run, also in
the runs that lost 46%.

Parameters of `scarf_live_mapping_node` (set them with
`scarf_live_params:=<file>`):

| Parameter | Default | Description |
| --- | --- | --- |
| `submap_voxel_m` | `0.05` | Voxel size. 0.10 sends about a third of the bytes, with points within about 5 cm (p95) of the surface. 0.03 sends about 2.5x the bytes of 0.05. |
| `submap_max_bytes` | `32000` | Size cap of one chunk message. |
| `submap_scale_tolerance` | `0.02` | Relative scale change that resends a submap. |
| `submap_translation_tolerance_m` | `0.01` | Anchor translation that resends a submap. |
| `submap_rotation_tolerance_deg` | `0.5` | Anchor rotation that resends a submap. |
| `submap_refresh_count` | `0` | Submaps sent again after each batch, least recently sent first. Off, because the timer below resends every chunk. |
| `submap_colors` | `true` | Send the mean voxel colors (5 bits per channel). About doubles the bytes. With `false`, the chunks carry positions only, and Foxglove colors the points. |
| `submap_send_gap_ms` | `5.0` | Time between two chunk messages. ROS Portal loses most messages that are published back to back (see `livekit_scarf_slam/paced_sender.py`). |
| `submap_warmup_s` | `2.0` | A new chunk topic gets its last message again this long after its first one, because ROS Portal loses the first message on a new topic. |
| `submap_refresh_interval_s` | `1.0` | Every interval, the chunk topic sent least recently gets its last message again, so lost messages and late viewers catch up, also after the mapping stops. At most one extra message per interval: a 20-topic map is fully resent every 20 s, about 3 KB/s. `0` turns it off. |
| `submap_max_rate_kbps` | `2000` | Rate cap of the chunk messages. After a message, the sender waits for its bytes at this rate, at least `submap_send_gap_ms`. ROS Portal loses more messages the faster they go out: 20% of the 5 cm map in 8 KB chunks at about 10 Mbps. |
| `publish_global_cloud` | `true` | Also publish the local `/scarf_slam/clouds` `PointCloud2`. |

ROS Portal on the SLAM host sends `/scarf_slam/submaps/.*` (a regex, because
the chunk topics appear while ScaRF-SLAM maps). The receiving ROS Portal needs
the same inbound entry:

```yaml
    - topic: "/scarf_slam/submaps/.*"
      direction: "in"
```

#### Foxglove layout

Foxglove keeps the 3D panel settings per topic, so a chunk slot that the
layout does not list starts with the default settings. To give every slot the
same settings, show `/scarf_slam/submaps/chunk_000` in a 3D panel, set it up
(for example the point size, and Color mode "RGBA (separate fields)" for the
voxel colors), and export the layout (Layouts > Export). Copy
the file to `runs/`, then copy the settings to the first 256 slots:

```bash
ros2 run livekit_scarf_slam expand_foxglove_layout.py \
  /remote_ros_slam/runs/layout.json
```

Import the result. `--count` sets the number of slots. A 10-minute run with
13 submaps used 13 (19 with 3 cm voxels).

Every ROS Portal on the track, and the ROS graph that Foxglove reads, needs
`foxglove_msgs`: ROS Portal rejects a data track whose type it cannot render,
and matches the type by a hash of the `.msg` text, so use the same
`foxglove_msgs` version (Jazzy apt: 3.6.0) everywhere. The compose
`ros_portal` service builds `ros_portal.Dockerfile`, which adds it to
`ROS_PORTAL_IMAGE`.

For more ScaRF-SLAM detail, see the upstream wiki:

- [Online Reconstruction with SLAM](https://github.com/ori-drs/ScaRF-SLAM/wiki/4.-%F0%9F%9A%80-Online-Reconstruction-with-SLAM)
- [Configuration](https://github.com/ori-drs/ScaRF-SLAM/wiki/6.-%E2%9A%99%EF%B8%8F-Configuration)

## Add An IMU Later

With an IMU on the robot, OpenVINS can give the poses instead of RTAB-Map.
OpenVINS uses the IMU for scale and fast motion. Then you need these steps:

1. Calibrate the camera to the IMU. See the IMU part of
   [calibration.md](calibration.md).
2. Clone `open_vins` and `ov_secondary_scarf` into `src/externals`. On
   Jazzy, use the branch of
   [open_vins pull request #543](https://github.com/rpng/open_vins/pull/543).
3. Add `ov_msckf` and `ov_secondary_loop_fusion` to `package.xml`.
