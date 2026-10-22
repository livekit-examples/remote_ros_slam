FROM ros:jazzy

ARG DEBIAN_FRONTEND=noninteractive

WORKDIR /remote_ros_slam

RUN apt-get update \
    && apt-get install -y --no-install-recommends python3-venv python3-pip python3-dev \
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
# container runs with ENABLE_GPU=true.
RUN pip install --no-cache-dir "torch>=2" torchvision

# Workspace sources. The dev compose service bind-mounts src/ over this copy
# for live editing.
COPY src src

# Install every dependency the package manifests declare.
RUN apt-get update \
    && rosdep update --rosdistro "${ROS_DISTRO}" \
    && rosdep install --from-paths src --ignore-src -y --rosdistro "${ROS_DISTRO}" \
    && rm -rf /var/lib/apt/lists/*

RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" && colcon build

# Pre-source ROS and the overlay for interactive shells (docker exec -it slam
# bash). The venv needs no sourcing, see VIRTUAL_ENV above.
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
    && echo '[ -f /remote_ros_slam/install/setup.bash ] && source /remote_ros_slam/install/setup.bash' >> /root/.bashrc
