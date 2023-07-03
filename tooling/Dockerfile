ARG FROM_IMAGE
ARG CATKIN_WS_PATH
ARG CCACHE_DIR
ARG ROS_HOME
ARG REPOSITORY_NAME
ARG PACKAGE_NAME

# hadolint ignore=DL3006
FROM $FROM_IMAGE AS cacher

# Install vcstool
ARG DEBIAN_FRONTEND=noninteractive
# hadolint ignore=DL3008
RUN apt-get update && \
    apt-get install -q -y --no-install-recommends git python3-vcstool && \
    rm -rf /var/lib/apt/lists/*

# Copy in the project's source
ARG CATKIN_WS_PATH
ARG REPOSITORY_NAME
WORKDIR $CATKIN_WS_PATH
COPY $REPOSITORY_NAME src/$REPOSITORY_NAME/

# Import from-source dependencies with vcstool
RUN mkdir src/dependencies && \
    vcs import --recursive --input src/$REPOSITORY_NAME/tooling/vcstool/wavemap_https.yml \
      src/dependencies

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

# Cache the dependencies source code for use in subsequent stages
# NOTE: We filter out the git histories since these change even if the code
#       didn't, which causes unnecessary Docker build cache misses.
RUN mkdir -p /tmp/filtered_sources && \
    find ./src/dependencies -type d -name .git -prune -o -type f -exec \
      cp --parents -t /tmp/filtered_sources {} \; && \
    echo "Filtered dependencies sources hash:" && \
    find /tmp/filtered_sources -type f -print0 | sort -z | \
      xargs -0 sha1sum | sha1sum


# hadolint ignore=DL3006
FROM $FROM_IMAGE AS system-deps-installer

# Load the cached manifests
ARG CATKIN_WS_PATH
WORKDIR $CATKIN_WS_PATH
COPY --from=cacher /tmp/manifests .

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


FROM system-deps-installer AS workspace-deps-builder

# Load the dependencies source code and our package's manifest
# to resolve which dependencies should be built
ARG CATKIN_WS_PATH
ARG REPOSITORY_NAME
WORKDIR $CATKIN_WS_PATH
COPY --from=cacher /tmp/filtered_sources/src/dependencies src/dependencies
COPY --from=cacher /tmp/manifests/src/$REPOSITORY_NAME src/$REPOSITORY_NAME

# Pull in ccache's cache
ARG CCACHE_DIR
COPY ccache $CCACHE_DIR

# Setup the catkin workspace and build the dependencies
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
# hadolint ignore=SC2046
RUN . /opt/ros/noetic/setup.sh && \
    catkin init && \
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    dependencies=$(catkin list --deps --directory src/$REPOSITORY_NAME -u | grep -oP '(?<= - ).*?(?=$)' | grep -v $REPOSITORY_NAME | sort -u) && \
    catkin_packages=$(catkin list -u | sort -u) && \
    dependencies_to_catkin_build=$(comm -12 <(echo "$dependencies") <(echo "$catkin_packages")) && \
    catkin build --no-status --force-color $(echo "$dependencies_to_catkin_build")


FROM workspace-deps-builder AS workspace-full-builder

# Load package source code
ARG CATKIN_WS_PATH
ARG REPOSITORY_NAME
ARG PACKAGE_NAME
WORKDIR $CATKIN_WS_PATH
COPY --from=cacher $CATKIN_WS_PATH/src/$REPOSITORY_NAME src/$REPOSITORY_NAME

# Build the package
RUN . /opt/ros/noetic/setup.sh && \
    catkin build --no-status --force-color $PACKAGE_NAME


FROM system-deps-installer AS workspace-underlay

# Update the entrypoint to source the workspace
# NOTE: The devel/setup.bash will only be pulled in in a subsequent stage, so
#       images built only up to the current stage (i.e. with
#       --target=workspace-underlay) cannot yet successfully be booted.
# hadolint ignore=SC2086
RUN sed --in-place \
      's|^source .*|source "'$CATKIN_WS_PATH'/devel/setup.bash"|' \
      /ros_entrypoint.sh && \
    echo "source '$CATKIN_WS_PATH'/devel/setup.bash" >> ~/.bashrc

# Load the workspace sources
COPY --from=cacher $CATKIN_WS_PATH/src src


FROM workspace-underlay AS workspace

# Configure and bootstrap catkin
# NOTE: We build an (arbitrary) small package to create
#       catkin_ws/devel/setup.bash, such that we can directly
#       source our workspace in the container entrypoint script
ARG CATKIN_WS_PATH
WORKDIR $CATKIN_WS_PATH
RUN . /opt/ros/noetic/setup.sh && \
    catkin init && \
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build catkin_setup --no-status && \
    ccache --clear


FROM workspace-underlay AS workspace-built-deps

# Pull in the compiled workspace
ARG CATKIN_WS_PATH
WORKDIR $CATKIN_WS_PATH
COPY --from=workspace-deps-builder $CATKIN_WS_PATH .


FROM workspace-underlay AS workspace-built-full

# Pull in the compiled workspace
ARG CATKIN_WS_PATH
WORKDIR $CATKIN_WS_PATH
COPY --from=workspace-full-builder $CATKIN_WS_PATH .


FROM scratch AS workspace-deps-builder-ccache-extractor

ARG CCACHE_DIR
WORKDIR /
COPY --from=workspace-deps-builder $CCACHE_DIR .


FROM scratch AS workspace-full-builder-ccache-extractor

ARG CCACHE_DIR
WORKDIR /
COPY --from=workspace-full-builder $CCACHE_DIR .
