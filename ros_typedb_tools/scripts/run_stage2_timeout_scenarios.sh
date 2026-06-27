#!/usr/bin/env bash

set -euo pipefail

CLIENTS="${CLIENTS:-20}"
DURATION_S="${DURATION_S:-60}"
TIMEOUT_S="${TIMEOUT_S:-10}"
EXECUTOR_THREADS="${EXECUTOR_THREADS:-4}"
REAL_SERVICE_NAME="${REAL_SERVICE_NAME:-/ros_typedb_interface/query}"
FAKE_SERVICE_NAME="${FAKE_SERVICE_NAME:-/ros_typedb_fake_query}"
OUTPUT_DIR="${OUTPUT_DIR:-$HOME/results/ros_typedb_stage2_timeout_scenarios}"

mkdir -p "$OUTPUT_DIR"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 was not found. Source ROS and the workspace before running." >&2
  exit 1
fi

run_stress() {
  local name="$1"
  local service_name="$2"
  shift 2

  echo
  echo "==> Running $name"
  ros2 run ros_typedb_tools ros_typedb_stress_experiment \
    --service-name "$service_name" \
    --clients "$CLIENTS" \
    --duration-s "$DURATION_S" \
    --timeout-s "$TIMEOUT_S" \
    --debug-events-output "$OUTPUT_DIR/${name}_events.jsonl" \
    --output "$OUTPUT_DIR/${name}_results.json" \
    "$@"
}

start_fake_service() {
  echo
  echo "==> Starting fake Query service on $FAKE_SERVICE_NAME"
  ros2 run ros_typedb_tools ros_typedb_fake_query_service \
    --service-name "$FAKE_SERVICE_NAME" \
    >"$OUTPUT_DIR/fake_query_service.log" 2>&1 &
  FAKE_SERVICE_PID="$!"
  sleep 1
}

stop_fake_service() {
  if [[ -n "${FAKE_SERVICE_PID:-}" ]]; then
    kill "$FAKE_SERVICE_PID" 2>/dev/null || true
    wait "$FAKE_SERVICE_PID" 2>/dev/null || true
  fi
}

trap stop_fake_service EXIT

echo "Writing results to $OUTPUT_DIR"

run_stress "real_read_global" "$REAL_SERVICE_NAME" \
  --mode read

run_stress "real_read_multi" "$REAL_SERVICE_NAME" \
  --mode read \
  --executor multi \
  --executor-threads "$EXECUTOR_THREADS"

start_fake_service

run_stress "fake_read_global" "$FAKE_SERVICE_NAME" \
  --mode read

run_stress "fake_read_multi" "$FAKE_SERVICE_NAME" \
  --mode read \
  --executor multi \
  --executor-threads "$EXECUTOR_THREADS"

stop_fake_service
unset FAKE_SERVICE_PID

run_stress "real_entity_get_global" "$REAL_SERVICE_NAME" \
  --query 'match $x isa entity; get $x;' \
  --query-type get

echo
echo "Finished. Results are in $OUTPUT_DIR"
