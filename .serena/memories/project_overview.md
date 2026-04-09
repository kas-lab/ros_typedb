# ros_typedb Overview

`ros_typedb` is a ROS 2 interface library for TypeDB 3. It is a ROS 2 Humble `ament_python` package that provides a reusable DB access layer and a lifecycle node exposing query services.

## Purpose

The project bridges ROS 2 and TypeDB. It provides:
- a Python `TypeDBInterface` for connection management, schema/data loading, and query execution
- a ROS lifecycle node `ROSTypeDBInterface` that exposes query/delete services and event publishing

## Main components

- `ros_typedb/ros_typedb/typedb_interface.py`: `TypeDBInterface`, the core TypeDB access layer.
- `ros_typedb/ros_typedb/ros_typedb_interface.py`: `ROSTypeDBInterface`, ROS lifecycle node + query service layer.
- `ros_typedb/ros_typedb/typedb_helpers.py`: helper functions for common TypeQL work.
- `ros_typedb/ros_typedb/ros_typedb_node.py`: node entrypoint.
- `ros_typedb/test/`: unit/integration/style tests.
- `scripts/`: Docker-based test and mandatory-check helpers.
- `ros_typedb_msgs/`: message/service package used by the interface.

## Tech stack

- Python
- ROS 2 Humble
- `ament_python` / `colcon`
- TypeDB 3.x
- Docker-based development workflow
- Pytest
- `ament_flake8` and `ament_pep257`

## Key architecture/design rules

- `ROSTypeDBInterface` is the ROS wiring layer; keep query/domain logic in plain Python classes where possible.
- Preserve `TypeDBQueryError` propagation semantics in `TypeDBInterface`.
- In ROS service callbacks, backend query exceptions should be converted into `success = False` responses.
- Preserve current test-data loading behavior for `match ... insert ...` blocks.
