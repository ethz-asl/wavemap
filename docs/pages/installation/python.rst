Python
######
.. rstcheck: ignore-directives=tab-set-code

We will make pywavemap available through PyPI soon. In the meantime, you can already pip install pywavemap from sources.

Regular install
***************
In case you'd like to use pywavemap but do not expect to change its code often, we recommend a regular install.

.. _python-install-build-deps:

First, make sure the necessary dependencies to build C++ and Python packages are available on your machine:

.. tab-set-code::

    .. code-block:: Debian/Ubuntu
      :class: no-header

      sudo apt update
      sudo apt install git build-essential python3-dev python3-pip

    .. code-block:: Alpine
      :class: no-header

      apk update
      apk add git build-base python3-dev py3-pip

.. _python-install-clone-repo:

Next, clone wavemap's code to your machine. We recommend using `SSH <https://docs.github.com/en/authentication/connecting-to-github-with-ssh>`_. Alternatively, HTTPS can be used without requiring keys to be set up.

.. tab-set-code::

    .. code-block:: SSH
      :class: no-header

      cd ~/
      git clone git@github.com:ethz-asl/wavemap.git

    .. code-block:: HTTPS
      :class: no-header

      cd ~/
      git clone https://github.com/ethz-asl/wavemap.git

You can then install pywavemap by running::

    cd ~/wavemap/library/python
    pip3 install .

Editable install
****************
By default, pip will recompile wavemap's C++ and Python libraries in a virtual environment from scratch on every install. If you're interested in modifying wavemap's code, you can save time by enabling incremental builds.

The general steps are similar to those of a regular install. Make sure your machine is :ref:`ready to build C++ and Python packages <python-install-build-deps>`, and :ref:`clone wavemap's code <python-install-clone-repo>`.

However, since the build no longer happens in a virtual environment, the pip packages required to build pywavemap must also available on your machine::

      pip3 install nanobind scikit-build-core
      pip3 install typing_extensions  # Only needed if python_version < 3.11

You can then install pywavemap without isolation to allow incremental rebuilds::

      cd ~/wavemap/library/python
      rm -rf build  # Only needed if you previously built pywavemap differently
      pip install --no-build-isolation -ve .

The command above needs to be run after every change to reinstall the updated package. For an even more interactive experience, use::

      cd ~/wavemap/library/python
      rm -rf build  # Only needed if you previously built pywavemap differently
      pip install --no-build-isolation -Ceditable.rebuild=true -ve .

In this mode, any code that changed is automatically rebuilt whenever pywavemap is imported into a Python session.
