# ros_typedb
[![tests](https://github.com/Rezenders/ros_typedb/actions/workflows/test.yml/badge.svg)](https://github.com/Rezenders/ros_typedb/actions/workflows/test.yml)
[![documentation](https://github.com/Rezenders/ros_typedb/actions/workflows/doc.yml/badge.svg)](https://github.com/Rezenders/ros_typedb/actions/workflows/doc.yml)

This package provides a first generic integration between ROS and [typeDB](https://typedb.com/), specifically for students of the course Knowledge Representation and Reasoning, and was tested in Ubuntu 22.04 with ROS Humble and typedb v2.28.0.

**This repository is in an initial stage and under constant development**

## Install

### From within the student's Singularity image

Since the package and the dependencies are already pre-installed, you may go to the root folder of this pacakge in your image and run `git pull` followed by `git checkout course_krr`, which will provide you with the latest examples for getting started with TypeDB in your ROS2 system.

Then build the project:
```Bash
> cd ~/krr_ws/
> colcon build --symlink-install
> source install/setup.bash
```

### From scratch

To use this package, you need to install ROS 2 Humble, typedb, and typedb python driver.

#### Install ROS2 Humble

Follow the [official instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) for installing ROS 2 Humble.

#### Install TypeDB

Install typedb: follow the [official instructions](https://typedb.com/docs/typedb/2.x/installation.html).

Install [typedb python driver](https://typedb.com/docs/clients/2.x/python/python-install.html):

```bash
pip install typedb-driver
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

## Using the package

**NOTE: It is assumed that the TypeDB server is already running**

#### Package elements

The package contains 2 functional nodes and 1 interface node:

- `ros_typedb` being a basic ROS2 wrapper of the TypeDB client. It is used for loading the schema and the data into the TypeDB server and then support communication with the database via the ROS2 service on the topic `/ros_typedb/query`. This functionality is implemented as a so-called lifecycle node, which means that when it is launched it can be activated and deactived using the following commands in a terminal: `ros2 lifecycle set /ros_typedb activate`, `ros2 lifecycle set /ros_typedb deactivate`, and possibly `ros2 lifecycle set /ros_typedb configure` and `ros2 lifecycle set /ros_typedb shutdown`. However, when using the launching script (mentioned below) the node is immediately configures and activated, so you can assume it runs as a typical ROS2 node when launched.
- `ros_typedb_msgs` being a set of specific ROS2 messages so that you may request queries to the TypeDB database via ROS2 services.
- `ros_db_client` being a simple ROS2 client for calling the query service of the `ros_typdb` node.

#### Running a first example
The `ros_typedb` comes with the schema and the data that was used in the course, although it is limited to the data related to the topology. To create or update the keyspace and use it in your ROS2 network please consider the following steps.

Launch `ros_typedb`:
```bash
ros2 launch ros_typedb ros_typedb.launch.py
```
This will load the schema and data files that were specified in the launch-file into the TypeDB database. The files themselves can be found in the config folder of the `ros_typedb` package, while the ones that are launched are `schema_floorplan.tql` and `data_floorplan_topology.tql`.

You may then run the illustrative example of the `ros_db_client` using
```bash
ros2 run ros_db_client ros_db_client
```
This will call the query service of the `ros_typedb` node for getting all rooms in the TypeDB database. The results are printed in the terminal after which the node is immediately shutdown. The implementation of `ros_db_client` follows the Minimal Service Client of the the ROS2 examples, which is found here [ROS2 Minimal Example Service and Client Node](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)

#### Continue developing

To embed TypeDB into the course practicum you may update the schema and data files that are located in `repo_root\ros_typdb\config`. To ensure that your latest, updated files are used you should re-build the workspace and source it.

You may then extend the example node for querying the database by making ROS2 service calls. The request contains two fields:

- `query_type` defining the type of query you want to call, which may either be "fetch", "get", "insert", "delete", or "get_aggregate"
- `query` defining the actual query you want to call, which can be any query that is viable for TypeDB. For example "match $x isa room, get $x" (and selecting `get_type = "get"`)

Please study `ros_db_client` before extending it. When extending the node with your own implementation, please test your queries first in TYpeDB Studio, while running the `ros_typedb` node, so that you are sure that the query is correct and about the results that you get back from the query.


## Package Design

The integration between ROS and TypeDB is implemented with 2 classes, [TypeDBInterface](https://github.com/Rezenders/ros_typedb/blob/main/ros_typedb/ros_typedb/typedb_interface.py) and [ROSTypeDBInterface](https://github.com/Rezenders/ros_typedb/blob/main/ros_typedb/ros_typedb/ros_typedb_interface.py).

The [TypeDBInterface](https://github.com/Rezenders/ros_typedb/blob/main/ros_typedb/ros_typedb/typedb_interface.py) class interacts with the typeDB database using the [typedb python api](https://typedb.com/docs/clients/2.x/python/python-tutorial.html), and it contains basic functionalities that are common for all applications, such as [insert_database](https://github.com/Rezenders/ros_typedb/blob/c16e3f8f1958f4ac2333c7b7d0612c8c79d698a0/ros_typedb/ros_typedb/typedb_interface.py#L153) and [match_database](https://github.com/Rezenders/ros_typedb/blob/c16e3f8f1958f4ac2333c7b7d0612c8c79d698a0/ros_typedb/ros_typedb/typedb_interface.py#L175).

The [ROSTypeDBInterface](https://github.com/Rezenders/ros_typedb/blob/main/ros_typedb/ros_typedb/ros_typedb_interface.py) class is a ROS 2 [LifeCycle](https://design.ros2.org/articles/node_lifecycle.html) Node, and it implements 2 ROS interfaces. A ROS service server `ros_typedb_interface/query` that is used to query the database, which uses the [Query.srv](https://github.com/Rezenders/ros_typedb/blob/main/ros_typedb_msgs/srv/Query.srv) service type. And the ROS topic `ros_typedb_interface/events`, where it publishes insert and delete events when data is inserted or deleted from the database with the query service.

Class diagram:
<p align="center">
  <img src="https://github.com/Rezenders/ros_typedb/assets/20564040/4cf4f799-3dab-40c4-a323-8d1e8e376e62" width="500">
</p>

Overview:
<p align="center">
  <img src="https://github.com/Rezenders/ros_typedb/assets/20564040/53793f23-0cb2-42c8-8c3b-fbfa5764ab5b" width="500">
</p>

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

Spin ROS node:
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

## Example of packages using ros_type
- [ROSA](https://github.com/kas-lab/rosa/tree/main/rosa_kb)
- [navigation_graph_map](https://github.com/kas-lab/navigation_graph_map/tree/main/navigation_kb)

## Acknowledgments

<a href="https://remaro.eu/">
    <img height="60" alt="REMARO Logo" src="https://remaro.eu/wp-content/uploads/2020/09/remaro1-right-1024.png">
</a>

This work is part of the Reliable AI for Marine Robotics (REMARO) Project. For more info, please visit: <a href="https://remaro.eu/">https://remaro.eu/

<br>

<a href="https://research-and-innovation.ec.europa.eu/funding/funding-opportunities/funding-programmes-and-open-calls/horizon-2020_en">
    <img align="left" height="60" alt="EU Flag" src="https://remaro.eu/wp-content/uploads/2020/09/flag_yellow_low.jpg">
</a>

This project has received funding from the European Union's Horizon 2020 research and innovation programme under the Marie Skłodowska-Curie grant agreement No. 956200.
