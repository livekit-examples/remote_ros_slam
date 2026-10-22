# Remote ROS SLAM

> [!IMPORTANT]
> This repository is currently in Developer Preview mode and not ready for production use.
> There can be bugs, and APIs and configuration options can change during this period.

This repository runs SLAM (simultaneous localization and mapping) and scene
reconstruction on a remote host. The robot and the SLAM host each run
[ROS Portal](https://github.com/livekit/ros-portal). ROS Portal on the robot
sends the camera as a LiveKit video track. ROS Portal on the SLAM host
publishes the track as a ROS topic, and the SLAM nodes read that topic.

## Containers

`docker-compose.yml` has two services. Both use host networking, so DDS can
discover the ROS graph of the host.

| Service | Container | Purpose |
| --- | --- | --- |
| `slam` | `slam` | Development container with ROS 2 Jazzy and PyTorch. It idles, and you open a shell in it. |
| `ros_portal` | `ros_portal` | The `livekit/ros-portal` image. It connects to LiveKit and moves the topics in the portal configuration. |

The `slam` image has a Python venv (an isolated Python package folder) at
`/opt/slam_venv`. The venv is active in every shell, so you do not source
anything for Python. The venv uses the system Python 3.12 and sees the ROS
Python packages. `PIP_CONSTRAINT` keeps numpy below 2 and setuptools below 80
for each `pip install`.

## Prerequisites

1. Install Docker with the Compose plugin.
2. On a host with an NVIDIA GPU, install the NVIDIA driver and the
   [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).
3. Get a LiveKit server URL and one access token for each side. Each token
   needs its own identity in the same room.

For example, you can create the tokens with the LiveKit CLI:

```bash
lk token create --join --room robo_room --identity slam_host --valid-for 24h
lk token create --join --room robo_room --identity edge_robot --valid-for 24h
```

## Build

From the repository root, build the `slam` image:

```bash
docker compose build
```

The first build takes a long time, because it installs PyTorch. The build
also builds the colcon workspace in the image.

## Run The SLAM Host

From the repository root, start the `slam` container and ROS Portal:

```bash
LIVEKIT_URL=wss://<project>.livekit.cloud LIVEKIT_TOKEN=<slam_host token> docker compose up
```

By default, the `slam` container gets all NVIDIA GPUs of the host. If the host
has no NVIDIA GPU, for example a Mac with Apple silicon, set `ENABLE_GPU=false`:

```bash
ENABLE_GPU=false LIVEKIT_URL=<url> LIVEKIT_TOKEN=<token> docker compose up
```

If `ENABLE_GPU=true` and the host has no NVIDIA GPU or no NVIDIA Container
Toolkit, the `slam` container does not start, and Docker shows this error:

```text
could not select device driver "nvidia" with capabilities: [[gpu]]
```

Open a shell in the `slam` container:

```bash
docker exec -it slam bash
```

The shell sources ROS 2 and the workspace overlay.

To stop the containers, run this command from the repository root:

```bash
docker compose down
```

## Run The Robot Side

On the robot, only ROS Portal runs. From the repository root on the robot,
start ROS Portal with a robot configuration that sends the camera topic:

```bash
LIVEKIT_URL=<url> LIVEKIT_TOKEN=<edge_robot token> \
  LIVEKIT_CONFIG=<robot configuration file> \
  docker compose up ros_portal
```

## Environment Variables

| Variable | Default | Description |
| --- | --- | --- |
| `LIVEKIT_URL` | (required) | LiveKit server URL for ROS Portal. |
| `LIVEKIT_TOKEN` | (required) | LiveKit access token for ROS Portal. |
| `LIVEKIT_CONFIG` | `./config/livekit.yaml` | ROS Portal configuration file on the host. |
| `ENABLE_GPU` | `true` | `true` gives the `slam` container all NVIDIA GPUs. `false` runs it on the CPU. Use only `true` or `false`. |
| `ROS_PORTAL_IMAGE` | `livekit/ros-portal:jazzy-v1.0.0-rc3` | ROS Portal image. |

The default `config/livekit.yaml` receives the video track `/camera/image_raw`
and publishes it on `/camera/image_raw/compressed`.

Compose reads all variables of the file, also for `docker compose build` and
`docker compose down`. If `LIVEKIT_URL` or `LIVEKIT_TOKEN` is not set, these
commands fail. For commands that do not connect to LiveKit, placeholder
values are enough:

```bash
LIVEKIT_URL=x LIVEKIT_TOKEN=x docker compose down
```

The SLAM host receives a video track and publishes it as a
`sensor_msgs/CompressedImage` topic. The robot sends a `CompressedImage` topic
as a video track. The `v1.0.0-rc3` image does not have these two
video features. Until a release includes them, build a ROS Portal image from
the video branches of the ROS Portal repository. Then set `ROS_PORTAL_IMAGE`
to that image. See
[Build a runtime image locally](https://github.com/livekit/ros-portal/blob/main/docs/docker.md#build-a-runtime-image-locally).

## Development

The `slam` service bind-mounts `./src` to `/remote_ros_slam/src` for live
editing. After you change the sources, build and test the workspace in the
`slam` container:

```bash
cd /remote_ros_slam
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

Host networking in this form targets Linux hosts. On Docker Desktop, turn on
host networking in the Docker Desktop configuration.
