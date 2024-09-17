Python (pip)
############
.. highlight:: bash
.. rstcheck: ignore-directives=tab-set-code

We're still working on making pywavemap available through PyPI. In the meantime, you can build and install it locally with pip, which takes less than two minutes and optimizes the build for your specific computer.

If you plan to use pywavemap without changing its code, a regular installation is easiest. However, if you're modifying wavemap's C++ or Python libraries, we recommend using the editable installation method for fast, incremental rebuilds.

Regular install
***************
.. _python-install-build-deps:

First, make sure the necessary dependencies to build C++ and Python packages are available:

.. tab-set-code::

    .. code-block:: Debian/Ubuntu
      :class: no-header

      sudo apt update
      sudo apt install git build-essential python3-dev python3-pip
      sudo apt install python3-venv  # If you use virtual environments

    .. code-block:: Alpine
      :class: no-header

      apk update
      apk add git build-base python3-dev py3-pip
      apk add python3-venv  # If you use virtual environments

.. _python-install-setup-venv:

*Optional:* We recommend using a virtual environment to isolate your Python dependencies. Create and activate it with the following commands:

.. tab-set-code::

    .. code-block:: Debian/Ubuntu
      :class: no-header

      sudo apt install python3-venv  # If needed
      python3 -m venv <path_to_new_virtual_env>
      source <path_to_new_virtual_env>/bin/activate

    .. code-block:: Alpine
      :class: no-header

      apk add python3-venv  # If needed
      python3 -m venv <path_to_new_virtual_env>
      source <path_to_new_virtual_env>/bin/activate

You can then build and install the latest version or a specific version of pywavemap by running:

.. tab-set-code::

    .. code-block:: Latest
      :class: no-header

      pip3 install git+https://github.com/ethz-asl/wavemap#subdirectory=library/python

    .. code-block:: Specific
      :class: no-header

      # Select a specific git branch, tag or commit using @...
      # For example, to install version v2.1.0, run
      pip3 install git+https://github.com/ethz-asl/wavemap@v2.1.0#subdirectory=library/python

Editable install
****************
If you're interested in modifying wavemap's code, you can save time by enabling incremental rebuilds.

The general steps are similar to those for a regular installation. Ensure your machine is :ref:`ready to build C++ and Python packages <python-install-build-deps>` and, optionally, :ref:`set up a virtual environment <python-install-setup-venv>`.

Next, clone wavemap's code to your machine:

.. tab-set-code::

    .. code-block:: SSH
      :class: no-header

      cd ~/
      git clone git@github.com:ethz-asl/wavemap.git

    .. code-block:: HTTPS
      :class: no-header

      cd ~/
      git clone https://github.com/ethz-asl/wavemap.git

Since editable installs are no longer built in an isolated environment, all build dependencies must be available on your system::

      pip3 install nanobind scikit-build-core
      pip3 install typing_extensions  # Only needed for Python < 3.11

You can then install pywavemap with incremental rebuilds using::

      cd ~/wavemap/library/python
      pip3 install --no-build-isolation -ve .

When you change wavemap's code, the command above must manually be rerun to reinstall the updated package. For a more interactive experience, you can use::

      cd ~/wavemap/library/python
      rm -rf build  # Only needed if you previously built pywavemap differently
      pip3 install --no-build-isolation -Ceditable.rebuild=true -ve .

In this mode, code changes are automatically rebuilt whenever pywavemap is imported into a Python session. Note that the rebuild message is quite verbose. You can suppress it by passing ``-Ceditable.verbose=false`` as an additional argument to ``pip3 install``.
