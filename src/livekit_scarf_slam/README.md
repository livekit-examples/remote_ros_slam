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
   on `/stereo/image_raw/compressed`. An SBS frame holds the left and right
   images next to each other, so both eyes have one stamp.
2. `ros_portal` sends the topic as the LiveKit video track `/stereo/image_raw`.
3. `ros_portal` on the SLAM host publishes the track on
   `/stereo/image_raw/compressed`.
4. `stereo_rectify_node` splits each frame and rectifies both fisheye images
   for RTAB-Map. Rectified images have matching points on the same row.
5. RTAB-Map calculates the poses.
6. `slam_bag_recorder_node` writes the ScaRF-SLAM input bag. For each RTAB-Map
   keyframe, the bag has the left fisheye image, the odometry pose, and the
   loop-corrected trajectory.
7. After the run, ScaRF-SLAM builds the dense map from the bag.

ScaRF-SLAM always reads a recorded bag, also in its upstream online mode.
RTAB-Map runs live on the SLAM host. The dense map comes after the run.

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
| `config/ros_portal/edge_robot.yaml` | ROS Portal on the robot. Sends `/stereo/image_raw/compressed` as a video track at 8000 kbps. |
| `config/ros_portal/slam_host.yaml` | ROS Portal on the SLAM host. Receives the `/stereo/image_raw` track. This file is the default `LIVEKIT_CONFIG`. |
| `config/rtabmap/stereo.yaml` | RTAB-Map parameters. `frame_id` is `stereo_link`. RTAB-Map adds at most 3 keyframes per second. |
| `config/scarf_slam/cobra_stereo.yaml` | ScaRF-SLAM configuration. The camera blocks come from the calibration. |

The camera driver on the robot must use the calibrated mode, 3840x1080. A
frame rate of 15 fps is enough for RTAB-Map. If LiveKit lowers the stream
resolution, `stereo_rectify_node` scales the intrinsics, and the rectified
images keep the same size. A frame with a different aspect ratio is dropped.

`stereo_slam.launch.py` has these arguments:

| Argument | Default | Description |
| --- | --- | --- |
| `calibration_path` | `/remote_ros_slam/calibration/stereo_calibration.json` | The stereo calibration. |
| `sbs_topic` | `/stereo/image_raw/compressed` | The SBS JPEG topic. |
| `output_dir` | `$SLAM_RUNS_DIR/<date>_<time>` | The run folder. It gets `slam_bag` and `rtabmap.db`. |
| `decode_reduction` | `2` | Decode the JPEG at 1/2 size. Use 1, 2, 4, or 8. |
| `rect_scale` | `0.5` | Size of the rectified images. 0.5 gives 960x540. |
| `rtabmap_viz` | `false` | Start the RTAB-Map GUI. The GUI needs a display. |

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
   `/stereo/image_raw/compressed`.

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

If the container runs on the CPU, Depth Anything 3 is slow in step 7. For
the real-time demo, use an `x86_64` host with an NVIDIA GPU and
`ENABLE_GPU=true`.

To test the SLAM host without the robot, record the SBS topic once with
`ros2 bag record /stereo/image_raw/compressed`. Then start only the `slam`
container with `docker compose up slam`, start the launch, and play the bag.

If RTAB-Map logs `quality=0` for many frames, the odometry lost track. Drive
slower, add light, or point the camera at surfaces with more texture.

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
