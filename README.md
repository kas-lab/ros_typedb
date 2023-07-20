# ros_typedb
[![tests](https://github.com/Rezenders/ros_typedb/actions/workflows/test.yml/badge.svg)](https://github.com/Rezenders/ros_typedb/actions/workflows/test.yml)

This package provides a basic generic integration between ROS and [typeDB](https://typedb.com/).
The package was designed in a way to enable users to easily extend it to fulfill their needs, the package design is explained in the [Package Design](#package-design) section.

This package was tested in Ubuntu 22.04 with ROS Humble and typedb v2.19.

**This repository is in an initial stage and under constant development**

## Install

To use this package, you need to install ROS 2 Humble, typedb, and typedb python driver.

#### Install ROS2 Humble

Follow the [official instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) for installing ROS 2 Humble.

#### Install TypeDB

Install typedb: follow the[official instructions](https://typedb.com/docs/typedb/2.x/installation.html).

Install [typedb python driver](https://typedb.com/docs/clients/2.x/python/python-install.html):

```bash
pip install typedb-client
```

#### Install ros_typedb package

Create a ROS workspace and clone the repository:
```Bash
mkdir -p ~/ros_typedb_ws/src/
cd ~/ros_typedb_ws/src
git clone https://github.com/Rezenders/ros_typedb.git
```

Install the dependencies:
```Bash
source /opt/ros/humble/setup.bash
cd ~/ros_typedb_ws/
rosdep install --from-paths src --ignore-src -r -y
```

Build the project:
```Bash
cd ~/ros_typedb_ws/
colcon build --symlink-install
source install/setup.bash
```

## Package Design

The integration between ROS and TypeDB is implemented with 2 classes, [TypeDBInterface](https://github.com/Rezenders/ros_typedb/blob/main/ros_typedb/ros_typedb/typedb_interface.py) and [ROSTypeDBInterface](https://github.com/Rezenders/ros_typedb/blob/main/ros_typedb/ros_typedb/ros_typedb_interface.py).

The [TypeDBInterface](https://github.com/Rezenders/ros_typedb/blob/main/ros_typedb/ros_typedb/typedb_interface.py) class interacts with the typeDB database using the [typedb python api](https://typedb.com/docs/clients/2.x/python/python-tutorial.html), and it contains basic functionalities that are common for all applications, such as [insert_database](https://github.com/Rezenders/ros_typedb/blob/c16e3f8f1958f4ac2333c7b7d0612c8c79d698a0/ros_typedb/ros_typedb/typedb_interface.py#L153) and [match_database](https://github.com/Rezenders/ros_typedb/blob/c16e3f8f1958f4ac2333c7b7d0612c8c79d698a0/ros_typedb/ros_typedb/typedb_interface.py#L175).

The [ROSTypeDBInterface](https://github.com/Rezenders/ros_typedb/blob/main/ros_typedb/ros_typedb/ros_typedb_interface.py) class is a ROS 2 [LifeCycle](https://design.ros2.org/articles/node_lifecycle.html) Node, and it implements 2 ROS interfaces. A ROS service server `ros_typedb_interface/query` that is used to query the database, which uses the [Query.srv](https://github.com/Rezenders/ros_typedb/blob/main/ros_typedb_msgs/srv/Query.srv) service type. And the ROS topic `ros_typedb_interface/events`, where it publishes insert and delete events when data is inserted or deleted from the database with the query service.

## Using the package

To run the ros_typedb_interface:
```bash
ros2 run ros_typedb ros_typedb_interface -p schema_path:=<schema_path> -p data_path:=<data_path>
```

**Note:** Make sure to replace <schema_path> and <data_path> with the real path for your schema and data file
**Note 2:** Remember that ros_typedb_interface is a [LifeCycle](https://design.ros2.org/articles/node_lifecycle.html) node, so you need to change its state to active before using it. Check the [lifecycle tutorial](https://github.com/ros2/demos/tree/rolling/lifecycle).

## Extend the package

To extend this package with custom functionalities, you can create a new ROS Node inheriting from ROSTypeDBInterface and a new typedb interface inheriting from TypeDBInterface. Then you simply need to add the new functionalities you need into your class.

Example:

New typedb interface:
```python
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
```

New ROS interface:
```python
class MyModelROSInterface(ROSTypeDBInterface):
    def __init__(self, node_name, schema_path='', data_path='', **kwargs):
        super().__init__(node_name, schema_path, data_path, **kwargs)
        self.typedb_interface_class = MyModelInterface
```

Sping ROS node:
```python
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
```
