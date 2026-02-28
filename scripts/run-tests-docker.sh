#!/usr/bin/env bash
set -euo pipefail

# Run tests inside the ros_typedb container (non-interactive).

if ! docker ps --format '{{.Names}}' | grep -q '^ros_typedb$'; then
  echo "Container 'ros_typedb' is not running." >&2
  echo "Start it first, e.g.:" >&2
  echo "  docker run -d --rm --name ros_typedb ..." >&2
  exit 1
fi

pytest_args=()
if [[ "${1:-}" == "-k" ]]; then
  pytest_args=("--pytest-args" "-k" "${2:-}")
fi

docker exec ros_typedb bash -lc \
  "source /opt/ros/humble/setup.bash && \
   source /home/ubuntu-user/typedb_ws/install/setup.bash && \
   colcon test --event-handlers console_cohesion+ --packages-select ros_typedb ${pytest_args[*]}"
