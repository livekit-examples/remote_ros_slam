# ROS Portal with foxglove_msgs, so it can send and receive the
# foxglove_msgs/CompressedPointCloud submap chunks of scarf_live_mapping_node.
# ROS Portal renders a data track type from its installed interface package
# and matches peers by a hash of the .msg text, so every ROS Portal on the
# track, and the ROS graph that Foxglove reads, needs the same foxglove_msgs.
ARG ROS_PORTAL_BASE=livekit/ros-portal:jazzy-v0.3.0
FROM ${ROS_PORTAL_BASE}

RUN apt-get update \
    && apt-get install -y --no-install-recommends ros-jazzy-foxglove-msgs \
    && rm -rf /var/lib/apt/lists/*
