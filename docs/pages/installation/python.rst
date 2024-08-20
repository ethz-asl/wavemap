Python (pip)
############
.. rstcheck: ignore-directives=tab-set-code

We will make pywavemap available through PyPI soon. In the meantime, you can install it directly from source using pip.

If you only plan to use pywavemap without modifying its code, a regular install is easiest. However, if you are actively working on wavemap's C++ or Python libraries, we recommend using the editable installation method for fast, incremental rebuilds.

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

.. _python-install-clone-repo:

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

You can then install pywavemap by running::

    cd ~/wavemap/library/python
    pip3 install .

Editable install
****************
If you're interested in modifying wavemap's code, you can save time by enabling incremental builds.

The general steps are similar to those for a regular installation. Ensure your machine is :ref:`ready to build C++ and Python packages <python-install-build-deps>` and that you've :ref:`cloned the code <python-install-clone-repo>`. Optionally, you can :ref:`set up a virtual environment <python-install-setup-venv>`.

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

In this mode, code changes are automatically rebuilt whenever pywavemap is imported into a Python session.
