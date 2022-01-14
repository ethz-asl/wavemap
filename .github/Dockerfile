ARG FROM_IMAGE=ros:noetic-ros-base-focal
ARG CATKIN_WS=/home/ci/catkin_ws
ARG CCACHE_DIR=/home/ci/ccache
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
COPY wavemap_2d src/wavemap_2d/

# Import from-source dependencies with vcstool
RUN mkdir src/dependencies && \
    vcs import --recursive --input src/wavemap_2d/wavemap_2d_https.yml \
      src/dependencies

# Cache the manifests of all packages for use in subsequent stages
RUN mkdir -p /tmp/manifests && \
    find ./src -name "package.xml" | \
      xargs cp --parents -t /tmp/manifests && \
    find ./src -name "CATKIN_IGNORE" | \
      xargs cp --parents -t /tmp/manifests || true && \
    echo "Manifests hash:" && \
    find /tmp/manifests -type f -print0 | sort -z | \
      xargs -0 sha1sum | sha1sum

# Cache the dependencies source code for use in subsequent stages
# NOTE: We filter out the git histories since these change even if the code
#       didn't, which causes unnecessary Docker build cache misses.
RUN mkdir -p /tmp/filtered_sources && \
    find ./src/dependencies -type d -name .git -prune -o -type f -print | \
      xargs cp --parents -t /tmp/filtered_sources || true && \
    echo "Filtered dependencies sources hash:" && \
    find /tmp/filtered_sources -type f -print0 | sort -z | \
      xargs -0 sha1sum | sha1sum


FROM $FROM_IMAGE AS system-deps-installer

# Load the cached manifests
ARG CATKIN_WS
WORKDIR $CATKIN_WS
COPY --from=cacher /tmp/manifests .

# Install general and ROS-related system dependencies
# NOTE: Manually installing opencv_viz is a temporary workaround to satisfy
#       opencv3's dependencies (required but seemingly missing on headless
#       systems and not declared through rosdep).
ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_HOME
ENV ROS_HOME=$ROS_HOME
RUN apt-get update && \
    apt-get install -q -y git python3-catkin-tools ccache libopencv-viz-dev && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src \
      --skip-keys="numpy_eigen catkin_boost_python_buildtool" -q -y && \
    rm -rf /var/lib/apt/lists/*

# Add ccache to the path and set where it stores its cache
ARG CCACHE_DIR
ENV PATH="/usr/lib/ccache:${PATH}" CCACHE_DIR=$CCACHE_DIR


FROM system-deps-installer AS workspace-deps-builder

# Load the dependencies source code and our package's manifest
# to resolve which dependencies should be built
ARG CATKIN_WS
WORKDIR $CATKIN_WS
COPY --from=cacher /tmp/filtered_sources/src/dependencies src/dependencies
COPY --from=cacher $CATKIN_WS/src/wavemap_2d/wavemap_2d/package.xml \
                   src/wavemap_2d/wavemap_2d/package.xml

# Pull in ccache's cache
ARG CCACHE_DIR
COPY ccache $CCACHE_DIR

# Setup the catkin workspace and build the dependencies
RUN . /opt/ros/noetic/setup.sh && \
    catkin init && \
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build --no-status --force-color \
      $(catkin list --deps --directory $(catkin locate wavemap_2d) -u | \
        grep -oP '(?<= - ).*?(?=$)')


FROM workspace-deps-builder AS workspace-full-builder

# Load package source code
ARG CATKIN_WS
WORKDIR $CATKIN_WS
COPY --from=cacher $CATKIN_WS/src/wavemap_2d src/wavemap_2d

# Build the package
RUN . /opt/ros/noetic/setup.sh && \
    catkin build --no-status --force-color wavemap_2d


FROM system-deps-installer AS workspace-underlay

# Update the entrypoint to source the workspace
# NOTE: The devel/setup.bash will only be pulled in in a subsequent stage, so
#       images built only up to the current stage (i.e. with
#       --target=workspace-underlay) cannot yet successfully be booted.
RUN sed --in-place \
      's|^source .*|source "'$CATKIN_WS'/devel/setup.bash"|' \
      /ros_entrypoint.sh

# Load the workspace sources
COPY --from=cacher $CATKIN_WS/src src


FROM workspace-underlay AS workspace

# Configure and bootstrap catkin
# NOTE: We build an (arbitrary) small package to create
#       catkin_ws/devel/setup.bash, such that we can directly
#       source our workspace in the container entrypoint script
ARG CATKIN_WS
WORKDIR $CATKIN_WS
RUN . /opt/ros/noetic/setup.sh && \
    catkin init && \
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build catkin_setup --no-status && \
    ccache --clear


FROM workspace-underlay AS workspace-built-deps

# Pull in the compiled workspace
ARG CATKIN_WS
WORKDIR $CATKIN_WS
COPY --from=workspace-deps-builder $CATKIN_WS .


FROM workspace-underlay AS workspace-built-full

# Pull in the compiled workspace
ARG CATKIN_WS
WORKDIR $CATKIN_WS
COPY --from=workspace-full-builder $CATKIN_WS .


FROM scratch AS workspace-deps-builder-ccache-extractor

ARG CCACHE_DIR
COPY --from=workspace-deps-builder $CCACHE_DIR .


FROM scratch AS workspace-full-builder-ccache-extractor

ARG CCACHE_DIR
COPY --from=workspace-full-builder $CCACHE_DIR .
