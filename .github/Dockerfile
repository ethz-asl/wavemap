ARG FROM_IMAGE=ros:noetic-ros-base-focal
ARG CATKIN_WS=/home/ci/catkin_ws
ARG ROS_HOME=/home/ci/.ros

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


FROM $FROM_IMAGE AS workspace

# Load the cached manifests
ARG CATKIN_WS
WORKDIR $CATKIN_WS
COPY --from=cacher /tmp/$CATKIN_WS .

# Install general and ROS-related system dependencies
# NOTE: Manually installing opencv_viz is a temporary workaround to satisfy
#       opencv3's dependencies (seemingly required but missing on headless
#       systems and not declared through rosdep).
ARG DEBIAN_FRONTEND=noninteractive
ENV ROS_HOME $ROS_HOME
RUN apt-get update && \
    apt-get install -q -y git python3-catkin-tools ccache libopencv-viz-dev && \
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


FROM workspace AS builder
# TODO(victorr): Load ccache and build, then adapt build-dependencies to only
#                pull in the built catkin_ws and add an output stage to pull out
#                the updated ccache cacche
# TODO(victorr): Add mechanism to only pull cache out when the dependencies
#                changed
# TODO(victorr): Improve caching by pulling in and building source deps before
#                pulling in package code (since the deps don't change often,
#                this might even remove the need for ccache)


FROM workspace AS build-dependencies

WORKDIR $CATKIN_WS
RUN export PATH="/usr/lib/ccache:$PATH" && \
    ccache --max-size=1G && \
    ccache --set-config=compiler_check=content && \
    catkin build --no-status --force-color $(catkin list --deps --directory $(catkin locate wavemap_2d) -u | grep -oP '(?<= - ).*?(?=$)')
