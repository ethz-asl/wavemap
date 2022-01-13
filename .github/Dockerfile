ARG FROM_IMAGE=ros:noetic-ros-base-focal
ARG CATKIN_WS=/home/ci/catkin_ws
ARG ROS_HOME=/home/ci/.ros

FROM $FROM_IMAGE AS cacher

# Install vcstool
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install -q -y git python3-catkin-tools python3-vcstool && \
    rm -rf /var/lib/apt/lists/*

# Copy in the project's source
ARG CATKIN_WS
WORKDIR $CATKIN_WS
COPY . src/wavemap_2d/

# Import from-source dependencies with vcstool
RUN mkdir src/dependencies && \
    vcs import --recursive --input src/wavemap_2d/wavemap_2d_https.yml src/dependencies

# Cache the manifests for use in subsequent stages
RUN mkdir -p /tmp/$CATKIN_WS && \
    find ./ -name "package.xml" | \
      xargs cp --parents -t /tmp/$CATKIN_WS && \
    find ./ -name "CATKIN_IGNORE" | \
      xargs cp --parents -t /tmp/$CATKIN_WS || true

# Cache the package dependencies for use in subsequent stages
RUN catkin init && \
    echo $(catkin list --deps --directory $(catkin locate wavemap_2d) -u | grep -oP '(?<= - ).*?(?=$)') > /tmp/wavemap_2d_deps


FROM $FROM_IMAGE AS system-deps-installer

# Load the cached manifests
ARG CATKIN_WS
WORKDIR $CATKIN_WS
COPY --from=cacher /tmp/$CATKIN_WS .

# Install general and ROS-related system dependencies
# NOTE: Manually installing opencv_viz is a temporary workaround to satisfy
#       opencv3's dependencies (required but seemingly missing on headless
#       systems and not declared through rosdep).
ARG DEBIAN_FRONTEND=noninteractive
ENV ROS_HOME $ROS_HOME
RUN apt-get update && \
    apt-get install -q -y git python3-catkin-tools ccache libopencv-viz-dev && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src --skip-keys="numpy_eigen catkin_boost_python_buildtool" -q -y && \
    rm -rf /var/lib/apt/lists/*

# Add ccache to the path
ENV PATH="/usr/lib/ccache:${PATH}"


FROM system-deps-installer AS workspace-underlay

# Load the rest of the catkin workspace
ARG CATKIN_WS
WORKDIR $CATKIN_WS
COPY --from=cacher $CATKIN_WS/src/dependencies src/dependencies

# Update the entrypoint to source the workspace that will be built in the following steps
ENV CATKIN_WS $CATKIN_WS
RUN sed --in-place \
      's|^source .*|source "$CATKIN_WS/devel/setup.bash"|' \
      /ros_entrypoint.sh


FROM workspace-underlay AS workspace

# Load the package source code
COPY --from=cacher $CATKIN_WS/src/wavemap_2d src/wavemap_2d

# Configure and bootstrap catkin
# NOTE: Build any small package to create catkin_ws/devel/setup.bash, removing the need
#       to source /opt/ros/noetic/setup.bash before the future first "real" catkin build call
RUN . /opt/ros/noetic/setup.sh && \
    catkin init && \
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build catkin_simple --no-status


FROM workspace-underlay AS workspace-built-deps

# Build the from-source dependencies
ARG CATKIN_WS
WORKDIR $CATKIN_WS
COPY --from=cacher /tmp/wavemap_2d_deps wavemap_2d_deps
RUN . /opt/ros/noetic/setup.sh && \
    catkin init && \
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build --no-status --force-color $(cat wavemap_2d_deps)

# Load the package source code
COPY --from=cacher $CATKIN_WS/src/wavemap_2d src/wavemap_2d


FROM workspace-built-deps AS workspace-built-full

# Build the package
ARG CATKIN_WS
WORKDIR $CATKIN_WS
RUN catkin build --no-status --force-color wavemap_2d
