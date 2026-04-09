#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="ros_typedb"

if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}\$"; then
  echo "Container '${CONTAINER_NAME}' is not running." >&2
  echo "Start it first, e.g.:" >&2
  echo "  docker run -d --rm --name ros_typedb -v /etc/localtime:/etc/localtime:ro -v \$PWD:/home/ubuntu-user/typedb_ws/src/ros_typedb ros_typedb sudo typedb server" >&2
  exit 1
fi

echo "[1/2] Running flake8 test..."
docker exec "${CONTAINER_NAME}" bash -lc \
  "source /opt/ros/humble/setup.bash && \
   source /home/ubuntu-user/typedb_ws/install/setup.bash && \
   cd /home/ubuntu-user/typedb_ws && \
   colcon test --event-handlers console_cohesion+ --packages-select typedb_utils ros_typedb ros_typedb_tools --pytest-args -k test_flake8"

echo "[2/2] Running pep257 test..."
docker exec "${CONTAINER_NAME}" bash -lc \
  "source /opt/ros/humble/setup.bash && \
   source /home/ubuntu-user/typedb_ws/install/setup.bash && \
   cd /home/ubuntu-user/typedb_ws && \
   colcon test --event-handlers console_cohesion+ --packages-select typedb_utils ros_typedb ros_typedb_tools --pytest-args -k test_pep257"

echo "Mandatory checks passed."
