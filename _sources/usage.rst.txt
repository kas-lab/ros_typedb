Usage
=====

Using the package
-----------------

To run the ros_typedb_interface:

.. code-block:: console

  ros2 run ros_typedb ros_typedb_interface -p schema_path:=<schema_path> -p data_path:=<data_path>


**Note:** Make sure to replace <schema_path> and <data_path> with the real path for your schema and data file
**Note 2:** Remember that ros_typedb_interface is a `LifeCycle <https://design.ros2.org/articles/node_lifecycle.html>`_ node, so you need to change its state to active before using it. Check the [lifecycle tutorial](https://github.com/ros2/demos/tree/rolling/lifecycle).

Extend the package
------------------

To extend this package with custom functionalities, you can create a new ROS Node inheriting from ROSTypeDBInterface and a new typedb interface inheriting from TypeDBInterface. Then you simply need to add the new functionalities you need into your class.

Example:

New typedb interface:

.. code-block:: python

  class MyModelInterface(TypeDBInterface):
      def __init__(self, address, database_name, schema_path, data_path=None,
                   force_database=False, force_data=False):

          super().__init__(
              address,
              database_name,
              schema_path,
              data_path,
              force_database,
              force_data
          )

New ROS interface:

.. code-block:: python

  class MyModelROSInterface(ROSTypeDBInterface):
      def __init__(self, node_name, schema_path='', data_path='', **kwargs):
          super().__init__(node_name, schema_path, data_path, **kwargs)
          self.typedb_interface_class = MyModelInterface

Sping ROS node:

.. code-block:: python

  def main():
      rclpy.init()
      traceback_logger = rclpy.logging.get_logger(
          'mymodel_kb_traceback_logger')

      lc_node = MyModelROSInterface('mymodel_kb')

      executor = rclpy.executors.MultiThreadedExecutor()
      executor.add_node(lc_node)
      try:
          executor.spin()
      except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
          pass
      except Exception as exception:
          traceback_logger.error(traceback.format_exc())
          raise exception
      finally:
          lc_node.destroy_node()
