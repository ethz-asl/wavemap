FROM ros:noetic-ros-base-focal

ARG DEBIAN_FRONTEND=noninteractive

# Install general and ROS-related APT packages
RUN apt-get update && \
    apt-get install -y git tree python3-catkin-tools python3-vcstool ccache

COPY . /home/ci/catkin_ws/src/wavemap_2d/

# Create catkin_ws and configure catkin
WORKDIR /home/ci/catkin_ws
RUN . /opt/ros/noetic/setup.sh && \
    catkin init && \
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

# Import from-source dependencies with vcstool
RUN vcs import --recursive --input src/wavemap_2d/wavemap_2d_https.yml src

# Install system dependencies with rosdep
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src --skip-keys="numpy_eigen catkin_boost_python_buildtool" -y

# Manually install opencv_viz (missing opencv3 dependency)
RUN apt-get install -y libopencv-viz-dev

# Bootstrap catkin build
# NOTE: Build any small package to create catkin_ws/devel/setup.bash, removing the need
#       to source /opt/ros/noetic/setup.bash before the future first "real" catkin build call
RUN . /opt/ros/noetic/setup.sh && \
    catkin build catkin_simple --no-status
