Run from the ROS workspace root unless noted:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_cohesion+ --packages-up-to ros_typedb
colcon test --event-handlers console_cohesion+ --packages-select ros_typedb_tools
```

Run core node:
```bash
ros2 run ros_typedb ros_typedb_interface -p schema_path:=<schema> -p data_path:=<data>
```

Build docs from docs/ with `make html`. Prefer `rg`/`rg --files` for repository search. In this environment, use the running `ros_typedb` container for ROS/TypeDB-dependent commands when needed.