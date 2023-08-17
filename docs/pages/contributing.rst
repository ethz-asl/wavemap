Contributing
############
.. highlight:: bash

We are extending wavemap's API and invite you to share requests for specific interfaces by opening a `GitHub issue <https://github.com/ethz-asl/wavemap/issues>`_. Additionally, we encourage code merge requests and would be happy to review and help you optimize contributed code.

To maintain code quality, we use the pre-commit framework to automatically format, lint and perform basic code checks. You can install pre-commit together with the dependencies required to run all of wavemap's checks with::

    rosrun wavemap_utils install_pre_commit.sh

After running the above script, pre-commit will automatically check changed code when it is committed to git. All the checks can also be run manually at any time by calling::

    pre-commit run --all

Wavemap's codebase includes a broad suite of tests. These are run in our Continuous Integration pipeline for active merge requests, `see here <https://github.com/ethz-asl/wavemap/actions/workflows/ci.yml>`_. You can also run the tests locally with::

    rosrun wavemap_utils build_and_test_all.sh
