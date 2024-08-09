ARG FROM_IMAGE
ARG CATKIN_WS_PATH
ARG CCACHE_DIR
ARG ROS_HOME
ARG REPOSITORY_NAME
ARG PACKAGE_NAME

# hadolint ignore=DL3006
FROM $FROM_IMAGE AS source-filter

# Copy in the project's source
ARG CATKIN_WS_PATH
ARG REPOSITORY_NAME
WORKDIR $CATKIN_WS_PATH
COPY $REPOSITORY_NAME src/$REPOSITORY_NAME/

# Cache the manifests of all packages for use in subsequent stages
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN mkdir -p /tmp/manifests && \
    find ./src -name "package.xml" -exec \
      cp --parents -t /tmp/manifests {} \; && \
    find ./src -name "CATKIN_IGNORE" -exec \
      cp --parents -t /tmp/manifests {} \; && \
    echo "Manifests hash:" && \
    find /tmp/manifests -type f -print0 | sort -z | \
      xargs -0 sha1sum | sha1sum

# hadolint ignore=DL3006
FROM $FROM_IMAGE AS dependency-installer

# Load the catkin package manifest files
ARG CATKIN_WS_PATH
WORKDIR $CATKIN_WS_PATH
COPY --from=source-filter /tmp/manifests .

# Install general and ROS-related system dependencies
# NOTE: Manually installing opencv_viz is a temporary workaround to satisfy
#       opencv3's dependencies (required but seemingly missing on headless
#       systems and not declared through rosdep).
ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_HOME
ENV ROS_HOME=$ROS_HOME
# hadolint ignore=DL3008
RUN apt-get update && \
    apt-get install -q -y --no-install-recommends git python3-catkin-tools ccache libopencv-viz-dev libtool && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src \
      --skip-keys="numpy_eigen catkin_boost_python_buildtool" -q -y && \
    rm -rf /var/lib/apt/lists/*

# Add ccache to the path and set where it stores its cache
ARG CCACHE_DIR
ENV PATH="/usr/lib/ccache:${PATH}" CCACHE_DIR=$CCACHE_DIR


FROM dependency-installer AS workspace

# Load the catkin workspace's source files
COPY --from=source-filter $CATKIN_WS_PATH/src src

# Configure and bootstrap catkin
# NOTE: We build a small dummy package to create
#       catkin_ws/devel/setup.bash, such that we can directly
#       source our workspace in the container entrypoint script
RUN . /opt/ros/noetic/setup.sh && \
    catkin init && \
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build catkin_setup --no-status && \
    ccache --clear

# Update the entrypoint to source the workspace
# hadolint ignore=SC2086
RUN sed --in-place \
      's|^source .*|source "'$CATKIN_WS_PATH'/devel/setup.bash"|' \
      /ros_entrypoint.sh && \
    echo "source '$CATKIN_WS_PATH'/devel/setup.bash" >> ~/.bashrc


FROM workspace AS workspace-builder

# Build our package
ARG CATKIN_WS_PATH
ARG PACKAGE_NAME
WORKDIR $CATKIN_WS_PATH
RUN catkin build --no-status --force-color $PACKAGE_NAME


FROM workspace AS workspace-built

# Pull in the compiled catkin workspace (but without ccache files etc)
ARG CATKIN_WS_PATH
WORKDIR $CATKIN_WS_PATH
COPY --from=workspace-builder $CATKIN_WS_PATH .


FROM scratch AS workspace-builder-ccache-extractor

# Extract the ccache cache directory from the workspace-builder stage
ARG CCACHE_DIR
WORKDIR /
COPY --from=workspace-builder $CCACHE_DIR .
