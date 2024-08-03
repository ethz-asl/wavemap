C++ Library (CMake)
###################
.. highlight:: bash
.. rstcheck: ignore-directives=tab-set-code
.. rstcheck: ignore-roles=gh_file

Wavemap's C++ library can be used as standard CMake package.
In the following sections, we'll discuss how you can include it in your own CMake project and link it to your libraries or executables.

Note that if you intend to use wavemap with ROS1, you can skip this guide and proceed directly to the :doc:`ROS1 installation page <ros1>`.

Including it in your project
****************************

We'll cover three ways in which you can make wavemap's library available in an existing CMake project.

Before you start, make sure your system has the necessary tools installed to build C++ projects with CMake. On Ubuntu, we recommend installing::

    sudo apt install cmake build-essential git

FetchContent
============

The fastest way to make wavemap's C++ library available in an existing CMake project is to use FetchContent, by adding the following lines to your project's CMakeLists.txt:

.. code-block:: cmake

  set(WAVEMAP_TAG develop/v2.0)

  cmake_minimum_required(VERSION 3.18)
  message(STATUS "Fetching wavemap ${WAVEMAP_TAG} from GitHub")
  include(FetchContent)
  FetchContent_Declare(wavemap
      GIT_REPOSITORY https://github.com/ethz-asl/wavemap.git
      GIT_TAG ${WAVEMAP_TAG}
      GIT_SHALLOW 1
      SOURCE_SUBDIR library)
  FetchContent_MakeAvailable(wavemap)

Subdirectory
============

The second option is to load wavemap's library into a subfolder of your project. This might be more convenient if you expect to modify its code in the future:

.. tab-set-code::

    .. code-block:: HTTPS
      :class: no-header
      :language: bash

      cd <your_project>/dependencies
      git clone https://github.com/ethz-asl/wavemap.git

    .. code-block:: SSH
      :class: no-header
      :language: bash

      cd <your_project>/dependencies
      git clone git@github.com:ethz-asl/wavemap.git

    .. code-block:: Submodule
      :class: no-header
      :language: bash

      cd <your_project>/dependencies
      git submodule add git@github.com:ethz-asl/wavemap.git

You can then include it into your CMake project by adding the following lines to your CMakeLists.txt file:

.. code-block:: cmake

  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/dependencies/wavemap/library
                   ${CMAKE_CURRENT_BINARY_DIR}/wavemap

System install
==============

The last option is to build wavemap as a standalone project, install on your system and then load it using CMake's `find_package`. This option is convenient when you want to use wavemap in multiple projects on your machine, while only having to download and compile it once.

First, make sure that all of wavemap's dependencies are available as system libraries. On Ubuntu, we recommend installing::

      sudo apt install libeigen3-dev libgoogle-glog-dev libboost-dev

Next, download the code:

.. tab-set-code::

    .. code-block:: HTTPS
      :class: no-header
      :language: bash

      git clone https://github.com/ethz-asl/wavemap.git

    .. code-block:: SSH
      :class: no-header
      :language: bash

      git clone git@github.com:ethz-asl/wavemap.git

Build it by running::

    cd wavemap/library
    cmake -S . -B build
    cmake --build build -j $(nproc)

You can then install wavemap as a system library by running::

    cmake --install build  # possibly needs sudo

To load wavemap's library into your own CMake project, you can now simply call `find_package` in your CMakeLists.txt:

.. code-block:: cmake

  find_package(wavemap)


Linking it to your code
***********************

After following either of the methods above, you're ready to link wavemap's C++ library against your own CMake targets and start using it inside your code.

The library contains three main components:

* `wavemap_core`: The framework's core algorithms, data structures and utilities
* `wavemap_io`: Functions to read and write maps to streams and files
* `wavemap_pipeline`: A measurement integration and map management pipeline

For an example executable that performs some operations on a map after reading it from a file, you would only need to link:

.. code-block:: cmake

      add_executable(example_executable example.cc)
      target_link_libraries(example_executable
            PUBLIC wavemap::wavemap_core wavemap::wavemap_io)

Note that this will automatically make wavemap's headers (includes) available to `example.cc`.

Finally, we strongly recommend using the `set_wavemap_target_properties` helper function to ensure your target's compilation flags are compatible with those of wavemap:

.. code-block:: cmake

      set_wavemap_target_properties(example_executable)
