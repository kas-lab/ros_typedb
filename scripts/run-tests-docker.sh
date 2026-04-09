#!/usr/bin/env bash
set -euo pipefail

# Run tests inside the ros_typedb container (non-interactive).
CONTAINER_NAME="ros_typedb"

if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}\$"; then
  echo "Container '${CONTAINER_NAME}' is not running." >&2
  echo "Start it first, e.g.:" >&2
  echo "  docker run -d --rm --name ros_typedb -v /etc/localtime:/etc/localtime:ro -v \$PWD:/home/ubuntu-user/typedb_ws/src/ros_typedb ros_typedb sudo typedb server" >&2
  exit 1
fi

pytest_args=()
if [[ "${1:-}" == "-k" ]]; then
  pytest_args=("--pytest-args" "-k" "${2:-}")
fi

docker exec "${CONTAINER_NAME}" bash -lc \
  "source /opt/ros/humble/setup.bash && \
   source /home/ubuntu-user/typedb_ws/install/setup.bash && \
   colcon test --event-handlers console_cohesion+ --packages-select typedb_utils ros_typedb ros_typedb_tools ${pytest_args[*]}"
