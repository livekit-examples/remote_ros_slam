FROM ros:jazzy

ARG DEBIAN_FRONTEND=noninteractive

WORKDIR /remote_ros_slam

RUN apt-get update \
    && apt-get install -y --no-install-recommends python3-venv python3-pip python3-dev \
        libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# Python environment for the SLAM and reconstruction tools. Not conda: ROS 2
# Jazzy (rclpy, colcon) needs the system Python 3.12, so the venv is built on
# it with --system-site-packages to keep the apt ROS packages visible.
# VIRTUAL_ENV + PATH make the venv active in every shell and `docker exec`
# without sourcing anything; PIP_CONSTRAINT applies constraints.txt to every
# later `pip install`, including the ones you run by hand.
ENV VIRTUAL_ENV=/opt/slam_venv
ENV PATH=${VIRTUAL_ENV}/bin:${PATH} \
    PIP_CONSTRAINT=${VIRTUAL_ENV}/constraints.txt
RUN python3 -m venv --system-site-packages "${VIRTUAL_ENV}" \
    && touch "${VIRTUAL_ENV}/COLCON_IGNORE"
COPY constraints.txt ${VIRTUAL_ENV}/constraints.txt

# PyTorch. The x86_64 PyPI wheels are CUDA builds and use the GPU when the
# container runs with ENABLE_GPU=true. xformers is optional for Depth
# Anything 3 and has no aarch64 wheel, so it is skipped there.
RUN pip install --no-cache-dir "torch>=2" torchvision \
    && if [ "$(uname -m)" = "x86_64" ]; then pip install --no-cache-dir xformers; fi

# ScaRF-SLAM and Depth Anything 3 (plain Python, kept outside the colcon
# workspace). pycolmap has no Linux aarch64 wheel and Depth Anything 3 only
# needs it for COLMAP export, so the patch makes its import optional and
# aarch64 installs every other dependency by hand. vismatch needs poselib,
# which has no Linux aarch64 wheel either, so aarch64 builds it from source
# (its stub step runs the venv python, so pybind11-stubgen goes in first).
# vismatch>=1.3.2 requires setuptools>=83, which colcon cannot use.
# gtsam 4.3 lists pytest>=9 as a runtime dependency, and pytest 9 breaks the
# launch_testing plugin of colcon test, so it goes in with --no-deps (numpy is
# its only other dependency).
COPY scarf_slam.repos /opt/scarf/scarf_slam.repos
COPY patches /opt/scarf/patches
RUN vcs import /opt/scarf < /opt/scarf/scarf_slam.repos \
    && git -C /opt/scarf/Depth-Anything-3 apply \
        /opt/scarf/patches/depth_anything_3_optional_pycolmap.patch \
    && cd /opt/scarf/Depth-Anything-3 \
    && if [ "$(uname -m)" = "x86_64" ]; then \
        pip install --no-cache-dir -e . addict; \
    else \
        pip install --no-cache-dir pre-commit trimesh einops huggingface_hub \
            imageio opencv-python open3d fastapi uvicorn requests typer pillow \
            omegaconf evo e3nn "moviepy==1.0.3" plyfile pillow_heif safetensors \
            addict \
        && pip install --no-cache-dir --no-deps -e .; \
    fi \
    && if [ "$(uname -m)" != "x86_64" ]; then \
        pip install --no-cache-dir pybind11-stubgen \
        && pip install --no-cache-dir \
            "poselib @ git+https://github.com/PoseLib/PoseLib.git@v2.0.5"; \
    fi \
    && pip install --no-cache-dir rosbags open3d "vismatch==1.3.1" \
    && pip install --no-cache-dir --no-deps gtsam

# Workspace sources. The dev compose service bind-mounts src/ and
# calibration/ over these copies for live editing.
COPY src src
COPY calibration calibration

# Install every dependency the package manifests declare (rtabmap_ros &c).
RUN apt-get update \
    && rosdep update --rosdistro "${ROS_DISTRO}" \
    && rosdep install --from-paths src --ignore-src -y --rosdistro "${ROS_DISTRO}" \
    && rm -rf /var/lib/apt/lists/*

RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" && colcon build --packages-up-to livekit_scarf_slam

# stereo_slam.launch.py writes each run under here (bind-mounted to ./runs).
ENV SLAM_RUNS_DIR=/remote_ros_slam/runs

# Pre-source ROS and the overlay for interactive shells (docker exec -it slam
# bash). The venv needs no sourcing, see VIRTUAL_ENV above.
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
    && echo '[ -f /remote_ros_slam/install/setup.bash ] && source /remote_ros_slam/install/setup.bash' >> /root/.bashrc
