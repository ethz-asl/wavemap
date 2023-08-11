Installation
############
.. highlight:: bash

If you would like to quickly try out wavemap or if you're not already using Ubuntu Noetic and ROS1, we recommend running wavemap in Docker.
For active wavemap development, it is probably easiest to install wavemap directly on your system.


Docker
******
To build the Docker image, simply run::

    docker build --tag=wavemap - < tooling/docker/incremental.Dockerfile

This will create a local image on your machine based on the latest version of wavemap. You can give the local image a different name by modifying the ``--tag=wavemap`` argument. The ``-`` that follows tells Docker that this job does not need a build context and ``< ...`` feeds in the relevant Dockerfile. By default, the image will be built using the latest wavemap release. To specify a specific release, such as v1.0.0, add the ``--build-arg="VERSION=v1.0.0"`` argument.



System install with ROS
***********************
We recommend using `ROS Noetic <http://wiki.ros.org/noetic/Installation>`_, as installed following the `standard instructions <http://wiki.ros.org/noetic/Installation>`_.

Make sure these required dependencies are installed::

    sudo apt update
    sudo apt install git build-essential  # General
    sudo apt install python3-catkin-tools python3-vcstool python3-rosdep  #ROS

Then create a catkin workspace with::

    mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/
    source /opt/ros/noetic/setup.sh
    catkin init
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

Next, clone the code for wavemap and its catkin dependencies. If you're using `SSH keys for github <https://docs.github.com/en/authentication/connecting-to-github-with-ssh>`_ (recommended)::

    cd ~/catkin_ws/src
    git clone git@github.com:ethz-asl/wavemap.git
    vcs import --recursive . --input wavemap/tooling/vcstool/wavemap_ssh.yml

If **not using SSH** keys, but https instead::

    cd ~/catkin_ws/src
    git clone https://github.com/ethz-asl/wavemap.git
    vcs import --recursive . --input wavemap/tooling/vcstool/wavemap_https.yml

Make sure rosdep is initialized::

    sudo rosdep init

Then install the remaining system dependencies using::

    cd ~/catkin_ws/src
    rosdep update
    rosdep install -y --from-paths . --ignore-src --skip-keys="numpy_eigen catkin_boost_python_buildtool"

Build all of wavemap's packages, including its ROS interface and the Rviz plugin used to visualize its maps, with::

    cd ~/catkin_ws/
    catkin build wavemap_all


System install without ROS
**************************
Wavemap's core libraries (in the `library` folder) are ROS agnostic and only have a few lightweight dependencies. They can be built and integrated into your project using CMake.
