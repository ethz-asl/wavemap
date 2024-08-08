ROS1 (catkin)
#############
.. highlight:: bash
.. rstcheck: ignore-directives=tab-set-code
.. rstcheck: ignore-roles=gh_file

If you would like to quickly try out wavemap or if you're not already using Ubuntu Noetic and ROS1, we recommend running wavemap in Docker.
For active wavemap development, it is probably easiest to install wavemap directly on your system.

.. _installation-ros1-docker:

Docker
******

If you have not yet installed Docker on your computer, please follow `these instructions <https://docs.docker.com/engine/install/>`_. We also recommend executing the `post-installation steps for Linux <https://docs.docker.com/engine/install/linux-postinstall/>`_, to make Docker available without ``sudo`` priviliges.

To build wavemap's Docker image, simply run::

    docker build --tag=wavemap --pull - <<< $(curl -s https://raw.githubusercontent.com/ethz-asl/wavemap/main/tooling/docker/incremental.Dockerfile)

This will create a local image on your machine containing the latest version of wavemap. You can give the local image a different name by modifying the ``--tag=wavemap`` argument. By default, the image will be built using the latest wavemap release. To specify a specific release, such as v1.0.0, add the ``--build-arg="VERSION=v1.0.0"`` argument.

There are many ways to work with Docker containers, with different pros and cons depending on the application.

One documented example of how to run wavemap containers with GUI (e.g. Rviz) support is provided in this :gh_file:`run_in_docker.sh <tooling/scripts/run_in_docker.sh>` script. This example should suffice to run all the :doc:`demos <../demos>`.

Native install
**************
We recommend using `ROS Noetic <http://wiki.ros.org/noetic/Installation>`_, as installed following the `standard instructions <http://wiki.ros.org/noetic/Installation>`_.

Make sure these required dependencies are installed::

    sudo apt update
    sudo apt install git build-essential  # General
    sudo apt install python3-catkin-tools python3-vcstool python3-rosdep  # ROS

Then create a catkin workspace with::

    mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/
    source /opt/ros/noetic/setup.sh
    catkin init
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

Next, clone the code for wavemap and its catkin dependencies. We recommend using `SSH <https://docs.github.com/en/authentication/connecting-to-github-with-ssh>`_. Alternatively, HTTPS can be used without requiring keys to be set up.

.. tab-set-code::

    .. code-block:: SSH
      :class: no-header

      cd ~/catkin_ws/src
      git clone git@github.com:ethz-asl/wavemap.git
      vcs import --recursive . --input wavemap/tooling/vcstool/wavemap_ssh.yml

    .. code-block:: HTTPS
      :class: no-header

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
