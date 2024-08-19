General
#######
.. highlight:: cpp
.. rstcheck: ignore-directives=doxygenstruct

The settings in the config's ``general`` section control wavemap's general behavior. In the following sections, we cover what options are available depending on which wavemap interface is used.

C++ Library
***********
The behavior of wavemap's C++ library is fully controlled by configuring its individual components, such as the ``map`` and ``measurement_integrators``. Therefore, it does not have a general config.

The only exception is that the library relies on `glog` for logging, whose verbosity is set globally. We provide a ``LoggingLevel`` helper struct to simplify setting `glog`'s verbosity, which can be used in C++ as follows::

  LoggingLevel{LoggingLevel::kWarning}.applyToGlog();

ROS1 Interface
**************
When using wavemap's ROS1 server, the following ``general`` settings are available:

.. doxygenstruct:: wavemap::RosServerConfig
    :project: wavemap_ros1
    :members:
