C++ (CMake)
###########
.. highlight:: bash
.. rstcheck: ignore-directives=tab-set-code
.. rstcheck: ignore-roles=gh_file

Wavemap's C++ library can be used as standard CMake package. In the following sections, we'll present four ways in which you can include it in your own CMake project.

Note that if you intend to use wavemap with ROS1, you can skip this guide and proceed directly to the :doc:`ROS1 installation page <ros1>`.

Prerequisites
*************
Before you start, make sure you have the necessary tools installed to build C++ projects with CMake. On Ubuntu, we recommend installing::

    sudo apt install cmake build-essential git

.. note::

      If you are working in Docker, these dependencies are only required inside your container. Not on your host machine.

FetchContent
************
The fastest way to include wavemap in an existing CMake project is to use FetchContent, by adding the following lines to your project's `CMakeLists.txt`:

.. code-block:: cmake

  set(WAVEMAP_VERSION main)  # Select a git branch, tag or commit

  cmake_minimum_required(VERSION 3.18)
  message(STATUS "Loading wavemap from GitHub (ref ${WAVEMAP_VERSION})")
  include(FetchContent)
  FetchContent_Declare(wavemap
      GIT_REPOSITORY https://github.com/ethz-asl/wavemap.git
      GIT_TAG ${WAVEMAP_VERSION}
      GIT_SHALLOW 1
      SOURCE_SUBDIR library/cpp)
  FetchContent_MakeAvailable(wavemap)

Subdirectory
************
The second option is to load wavemap's library into a subfolder of your project. This might be convenient if you expect to modify its code in the future:

.. tab-set-code::

    .. code-block:: HTTPS
      :class: no-header

      cd <your_project>/dependencies
      git clone https://github.com/ethz-asl/wavemap.git

    .. code-block:: SSH
      :class: no-header

      cd <your_project>/dependencies
      git clone git@github.com:ethz-asl/wavemap.git

    .. code-block:: Submodule
      :class: no-header

      cd <your_project>/dependencies
      git submodule add git@github.com:ethz-asl/wavemap.git

You can then include it into your CMake project by adding the following lines to your `CMakeLists.txt` file:

.. code-block:: cmake

  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/dependencies/wavemap/library/cpp
                   ${CMAKE_CURRENT_BINARY_DIR}/wavemap

Docker
******
In case you like to work in Docker, we provide sample Dockerfiles that build and install wavemap's C++ library on either :gh_file:`Alpine Linux <tooling/docker/cpp/alpine.Dockerfile>` or :gh_file:`Debian <tooling/docker/cpp/debian.Dockerfile>`.

If you have not yet installed Docker on your computer, please follow `these instructions <https://docs.docker.com/engine/install/>`_. We also recommend executing the `post-installation steps for Linux <https://docs.docker.com/engine/install/linux-postinstall/>`_, to make Docker available without ``sudo`` priviliges.

To build wavemap's C++ Docker image, simply run:

.. tab-set-code::

    .. code-block:: Alpine
      :class: no-header

      docker build --tag=wavemap_cpp --pull - <<< $(curl -s https://raw.githubusercontent.com/ethz-asl/wavemap/main/tooling/docker/cpp/alpine.Dockerfile)

    .. code-block:: Debian
      :class: no-header

      docker build --tag=wavemap_cpp --pull - <<< $(curl -s https://raw.githubusercontent.com/ethz-asl/wavemap/main/tooling/docker/cpp/debian.Dockerfile)

This will create a local image on your machine containing the latest version of wavemap's C++ library. You can give the local image a different name by modifying the ``--tag=wavemap_cpp`` argument. By default, the image will be built using the latest code on wavemap's ``main`` branch. To specify a specific branch, commit or release, such as `v2.1.0`, add the ``--build-arg="WAVEMAP_VERSION=v2.1.0"`` argument.

Native install
**************
The last option is to build wavemap as a standalone project, install it on your system and then load it using CMake's ``find_package``. This option is convenient when you want to use wavemap in multiple projects on your machine, while only having to download and compile it once.

First, make sure that all of wavemap's dependencies are available as system libraries. On Ubuntu, we recommend installing::

      sudo apt install libeigen3-dev libgoogle-glog-dev libboost-dev

Next, download the code:

.. tab-set-code::

    .. code-block:: HTTPS
      :class: no-header

      git clone https://github.com/ethz-asl/wavemap.git

    .. code-block:: SSH
      :class: no-header

      git clone git@github.com:ethz-asl/wavemap.git

Build it by running::

    cd wavemap/library/cpp
    cmake -S . -B build
    cmake --build build -j $(nproc)

You can then install wavemap as a system library by running::

    cmake --install build  # possibly needs sudo

To load wavemap's library into your own CMake project, you can now simply call ``find_package`` in your `CMakeLists.txt`:

.. code-block:: cmake

  find_package(wavemap)
