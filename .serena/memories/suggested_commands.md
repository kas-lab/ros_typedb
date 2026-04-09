# Suggested Commands For ros_typedb

## Basic navigation

- `cd /home/gus/ros_workspaces/tactical_retreat_3_ws/src/ros_typedb`
- `ls`
- `find . -maxdepth 2 -type d`
- `rg PATTERN .`
- `git status`

## Build the Docker image

```bash
docker build -t ros_typedb .
```

## Start the documented TypeDB container

```bash
docker run -d --rm --name ros_typedb -v /etc/localtime:/etc/localtime:ro -v $PWD:/home/ubuntu-user/typedb_ws/src/ros_typedb ros_typedb sudo typedb server
```

## Open a shell in the container

```bash
docker exec -it ros_typedb bash
```

## Build the package

```bash
docker exec ros_typedb bash -lc "source /opt/ros/humble/setup.bash && cd /home/ubuntu-user/typedb_ws && colcon build --symlink-install --packages-select ros_typedb"
```

## Run tests and checks

```bash
scripts/run-tests-docker.sh
scripts/run-mandatory-checks-docker.sh
colcon test --event-handlers console_cohesion+ --packages-select ros_typedb
docker exec ros_typedb bash -lc "source /opt/ros/humble/setup.bash && source /home/ubuntu-user/typedb_ws/install/setup.bash && cd /home/ubuntu-user/typedb_ws/src/ros_typedb/ros_typedb && python3 -m pytest -q test/test_typedb_interface.py test/test_typedb_helpers.py test/test_ros_typedb_interface.py"
docker exec ros_typedb bash -lc "source /opt/ros/humble/setup.bash && source /home/ubuntu-user/typedb_ws/install/setup.bash && cd /home/ubuntu-user/typedb_ws/src/ros_typedb/ros_typedb && python3 -m pytest -q -k malformed_query"
```

## Run the node

```bash
ros2 launch ros_typedb ros_typedb.launch.py schema_path:="['/absolute/path/schema.tql']" data_path:="['/absolute/path/data.tql']"
ros2 run ros_typedb ros_typedb
```

## Helpful Docker commands

- `docker ps --format '{{.Names}}\t{{.Status}}'`
- `docker logs --tail 200 ros_typedb`
- `docker stop ros_typedb`
