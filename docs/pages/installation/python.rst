Python
######
.. rstcheck: ignore-directives=tab-set-code

We will make pywavemap available through PyPI soon. In the meantime, you can already pip install the package from sources.

Before you start, make sure you have the necessary dependencies installed to build Python and C++ packages::

    sudo apt update
    sudo apt install git build-essential python3-dev python3-pip

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
