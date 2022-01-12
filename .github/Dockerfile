ARG FROM_IMAGE=ros:noetic-ros-base-focal
ARG CATKIN_WS=/home/ci/catkin_ws


FROM $FROM_IMAGE AS cacher

# Install vcstool
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install -q -y git python3-vcstool && \
    rm -rf /var/lib/apt/lists/*

# Copy in the project's source
ARG CATKIN_WS
WORKDIR $CATKIN_WS
COPY . src/wavemap_2d/

# Import from-source dependencies with vcstool
RUN vcs import --recursive --input src/wavemap_2d/wavemap_2d_https.yml src

# Copy manifests for caching
RUN mkdir -p /tmp/$CATKIN_WS && \
    find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp/$CATKIN_WS && \
    find ./ -name "CATKIN_IGNORE" | \
      xargs cp --parents -t /tmp/$CATKIN_WS || true


FROM $FROM_IMAGE AS builder

# Load the cached manifests
ARG CATKIN_WS
WORKDIR $CATKIN_WS
COPY --from=cacher /tmp/$CATKIN_WS .

# Install general and ROS-related system dependencies
# NOTE: Manually installing opencv_viz is a temporary workaround to satisfy
#       opencv3's dependencies (seemingly required but missing on headless
#       systems and not declared through rosdep).
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install -q -y git tree python3-catkin-tools python3-vcstool ccache libopencv-viz-dev && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src --skip-keys="numpy_eigen catkin_boost_python_buildtool" -q -y && \
    rm -rf /var/lib/apt/lists/*

# Load the rest of the catkin workspace
COPY --from=cacher $CATKIN_WS .

# Configure and bootstrap catkin
# NOTE: Build any small package to create catkin_ws/devel/setup.bash, removing the need
#       to source /opt/ros/noetic/setup.bash before the future first "real" catkin build call
RUN . /opt/ros/noetic/setup.sh && \
    catkin init && \
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build catkin_simple --no-status

# Update the entrypoint to source the workspace
ENV CATKIN_WS $CATKIN_WS
RUN sed --in-place \
      's|^source .*|source "$CATKIN_WS/devel/setup.bash"|' \
      /ros_entrypoint.sh
