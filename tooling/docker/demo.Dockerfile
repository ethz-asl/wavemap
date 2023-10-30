ARG VERSION=latest

FROM ghcr.io/ethz-asl/wavemap:${VERSION}

# Checkout wavemap's demo branch
RUN rm -rf src/wavemap && \
    git clone -b feature/srd_demo https://github.com/ethz-asl/wavemap src/wavemap

# Install vcstool
ARG DEBIAN_FRONTEND=noninteractive
# hadolint ignore=DL3008
RUN apt-get update && \
    apt-get install -q -y --no-install-recommends git python3-vcstool && \
    rm -rf /var/lib/apt/lists/*

# Import from-source dependencies with vcstool
RUN mkdir src/demo_dependencies && \
    vcs import --recursive --input src/wavemap/tooling/vcstool/demo_https.yml \
      src/demo_dependencies

# Configure the Livox ROS driver for ROS1
# hadolint ignore=DL3003
RUN cd src/demo_dependencies/livox_ros_driver2 && \
    ln -s package_ROS1.xml package.xml && \
    sed -i '1s/^/set(ROS_EDITION ROS1)\n/' CMakeLists.txt

# Install system dependencies
ARG DEBIAN_FRONTEND=noninteractive
# hadolint ignore=DL3008
RUN apt-get update && \
    apt-get install -q -y --no-install-recommends ros-noetic-image-proc tmux && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src \
           --skip-keys="numpy_eigen catkin_boost_python_buildtool" -q -y && \
    rm -rf /var/lib/apt/lists/*

# Build the Livox SDK
# hadolint ignore=DL3003
RUN cd src/demo_dependencies/livox_sdk2 && \
    mkdir build && cd build && \
    cmake .. && make -j8 && \
    make install

# Pull in the Pico Flexx SDK
# hadolint ignore=DL3003
RUN cd src/demo_dependencies/pico_flexx_driver/royale && \
    curl -L "https://drive.google.com/uc?export=download&id=1Xm0K37KRJJG7usGykTUoHwog897G6EgG" \
         -o libroyale.tar.xz && \
    tar -xf libroyale.tar.xz

# Setup udev rules for the Pico Monstar
RUN cp src/demo_dependencies/pico_flexx_driver/royale/libroyale-*/driver/udev/10-royale-ubuntu.rules \
       /etc/udev/rules.d/ && \
    usermod -a -G plugdev root

# Build catkin dependencies and wavemap
RUN catkin build --force-color ouster_ros pico_flexx_driver livox_ros_driver2 fast_lio && \
    catkin build --force-color wavemap_all
