Installation
============

To use this package, you need to install ROS 2 Humble, typedb, and typedb python driver.

Install ROS2 Humble
-------------------
Follow the `official ros install instructions <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html>`_ for installing ROS 2 Humble.

Install TypeDB
-------------------

Install typedb: follow the `official typedb install instructions <https://typedb.com/docs/typedb/2.x/installation.html>`_.

Install `typedb python driver <https://typedb.com/docs/clients/2.x/python/python-install.html>`_:

.. code-block:: console

  pip install typedb-driver


Install ros_typedb package
--------------------------

Create a ROS workspace and clone the repository:

.. code-block:: console

  mkdir -p ~/ros_typedb_ws/src/
  cd ~/ros_typedb_ws/src
  git clone https://github.com/Rezenders/ros_typedb.git


Install the dependencies:

.. code-block:: console

  source /opt/ros/humble/setup.bash
  cd ~/ros_typedb_ws/
  rosdep install --from-paths src --ignore-src -r -y

Build the project:

.. code-block:: console

  cd ~/ros_typedb_ws/
  colcon build --symlink-install
  source install/setup.bash
