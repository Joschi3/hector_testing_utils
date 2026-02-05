hector_testing_utils
====================

.. image:: https://img.shields.io/badge/ROS_2-Jazzy%20%7C%20Rolling-blue
   :alt: ROS 2 Version

.. image:: https://img.shields.io/badge/C%2B%2B-17-blue
   :alt: C++ Standard

.. image:: https://img.shields.io/badge/License-BSD--3--Clause-green
   :alt: License

| `GitHub Repository <https://github.com/tu-darmstadt-ros-pkg/hector_testing_utils>`_
| `Issue Tracker <https://github.com/tu-darmstadt-ros-pkg/hector_testing_utils/issues>`_

**Helper classes and utilities for writing robust ROS 2 integration tests using the real ROS graph and middleware.**

``hector_testing_utils`` is designed for testing ROS 2 systems **where mocking rclcpp is not sufficient**.
It enables deterministic, connection-aware Google Tests that interact with **actual DDS/RMW behavior**.

Key Features
------------

- **Connection-aware wrappers** - Publishers, subscribers, clients, and servers that track connections
- **Auto-suggestions on timeout** - Helpful error messages with similar topic/service names when connections fail
- **QoS diagnostics** - Detect and debug QoS compatibility issues
- **Graph monitoring** - Wait for and track changes to nodes, topics, and services
- **Remote parameter manipulation** - Set and get parameters on other nodes during tests
- **Test fixtures** - Ready-to-use GTest fixtures with executor management

User Guide
----------

.. toctree::
   :maxdepth: 2

   getting_started
   core_components
   usage_examples
   advanced_features
   tips
   troubleshooting

API Reference
-------------

.. toctree::
   :maxdepth: 2

   generated/index

Indices and tables
------------------

* :ref:`genindex`
* :ref:`search`
