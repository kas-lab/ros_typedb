Design
======


The integration between ROS and TypeDB is implemented with 2 classes,
`TypeDBInterface <TypeDBInterfaceLink>`_ and
`ROSTypeDBInterface <ROSTypeDBInterfaceLink>`_.

The `TypeDBInterface <TypeDBInterfaceLink>`_ class interacts with the typeDB
database using the `typedb python api <PythonAPILink>`_, and it contains basic
functionalities that are common for all applications,
such as `insert_database <InsertDatabaseLink>`_ and
`match_database <MatchDatabaseLink>`_.

The `ROSTypeDBInterface <ROSTypeDBInterfaceLink>`_ class is a ROS 2
`LifeCycle <LifeCycleLink>`_ Node, and it implements 2 ROS interfaces.
A ROS service server :code:`ros_typedb_interface/query` that is used to query
the database, which uses the `Query.srv <QuerySrvLink>`_ service type. And the
ROS topic :code:`ros_typedb_interface/events`, where it publishes insert and
delete events when data is inserted or deleted from the database with the query
service.

Class diagram:

.. raw:: html

  <embed>
    <p align="center">
      <img src="https://github.com/Rezenders/ros_typedb/assets/20564040/4cf4f799-3dab-40c4-a323-8d1e8e376e62" width="500">
    </p>
  </embed>

Overview:

.. raw:: html

  <embed>
    <p align="center">
      <img src="https://github.com/Rezenders/ros_typedb/assets/20564040/53793f23-0cb2-42c8-8c3b-fbfa5764ab5b" width="500">
    </p>
  </embed>

.. TypeDBInterfaceLink: https://github.com/Rezenders/ros_typedb/blob/main/ros_typedb/ros_typedb/typedb_interface.py
.. ROSTypeDBInterfaceLink: https://github.com/Rezenders/ros_typedb/blob/main/ros_typedb/ros_typedb/ros_typedb_interface.py
.. PythonAPILink: https://typedb.com/docs/clients/2.x/python/python-tutorial.html
.. InsertDatabaseLink: https://github.com/Rezenders/ros_typedb/blob/c16e3f8f1958f4ac2333c7b7d0612c8c79d698a0/ros_typedb/ros_typedb/typedb_interface.py#L153
.. MatchDatabaseLink: https://github.com/Rezenders/ros_typedb/blob/c16e3f8f1958f4ac2333c7b7d0612c8c79d698a0/ros_typedb/ros_typedb/typedb_interface.py#L175
.. LifeCycleLink: https://design.ros2.org/articles/node_lifecycle.html
.. QuerySrvLink: https://github.com/Rezenders/ros_typedb/blob/main/ros_typedb_msgs/srv/Query.srv
