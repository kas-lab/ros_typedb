# ros_typedb_benchmark

Stress-testing, fault injection, and benchmark tools for `ros_typedb`.

## ros_typedb_stress_experiment

Runs fixed-count or duration-based stress experiments against the real
`/ros_typedb_interface/query` service.

Prerequisites:

- TypeDB is running.
- `ros_typedb_interface` is running.
- The lifecycle node has been configured so the query service exists.

### Experiment Stages

The stress tool is organized as staged experiments. Earlier stages are useful
on their own; later stages build on the same result format and command-line
interface.

| Stage | Status | Purpose |
| --- | --- | --- |
| 1. Minimal real service load | Implemented | Send repeated requests to the real ROS `Query` service and measure latency, success, errors, and timeouts. |
| 2. Concurrent client stress | Implemented | Run many ROS service clients concurrently to find service/executor transport limits before blaming TypeDB or driver code. |
| 3. Correctness invariants | Implemented | Check that expected baseline facts remain visible during and after load, not only that requests returned. |
| 4. Mixed read/write load | Implemented | Read while inserting, updating, deleting, and cleaning up experiment-owned temporary data. |
| 5. Delete-database fault injection | Implemented | Delete the database during load and measure whether the driver fails loudly, reloads correctly, and recovers invariants. |
| 6. TypeDB restart fault injection | Implemented | Stop/restart TypeDB during load and measure outage, reconnect behavior, and post-recovery correctness. |
| 7. Lifecycle stress | Implemented | Trigger ROS lifecycle cleanup/configure/activate transitions during load to check teardown races. |
| 8. External schema/data profiles | Planned | Run the same harness with application-specific schema/data, query mixes, invariants, setup, and cleanup profiles. |

Use Stage 1 when validating a single query path. Use Stage 2 when probing ROS
service throughput and timeout behavior. Use Stage 3 when checking that load
does not corrupt or hide baseline data. Use Stage 4 when validating robustness
under real read/write activity. Use Stage 5 when validating recovery after a
database deletion fault. Use Stage 6 when validating driver reconnect behavior
after the TypeDB server process or container is restarted. Use Stage 7 when
validating that ROS lifecycle cleanup waits for active query callbacks before
tearing down TypeDB resources and that the node can be configured and activated
again afterward.

### Stage 1: Minimal Real Service Load

Example:

```bash
ros2 run ros_typedb_benchmark ros_typedb_stress_experiment \
  --query 'match $p isa Plan; (plan: $p, action: $a) isa has_action; fetch $a: attribute;' \
  --query-type fetch \
  --requests 2000 \
  --timeout-s 5 \
  --output ~/results/ros_typedb_stress_results.json
```

This is the smallest useful run. It is best for confirming that the service,
query type, timeout handling, result conversion, and JSON output work for one
known query before adding concurrency.

### Stage 2: Concurrent Client Stress

Concurrent read stress example:

```bash
ros2 run ros_typedb_benchmark ros_typedb_stress_experiment \
  --clients 20 \
  --duration-s 60 \
  --timeout-s 10 \
  --mode read \
  --debug-events-output ~/results/ros_typedb_read_stress_events.jsonl \
  --output ~/results/ros_typedb_read_stress_results.json
```

```bash
ros2 run ros_typedb_benchmark ros_typedb_stress_experiment \
  --clients 20 \
  --duration-s 60 \
  --timeout-s 10 \
  --query 'match $x isa entity; get $x;' \
  --query-type get \
  --debug-events-output ~/results/ros_typedb_read_stress_events.jsonl \
  --output ~/results/ros_typedb_get_stress_results.json
```

This stage is for separating ROS service transport behavior from database
behavior. It reports throughput and latency percentiles across concurrent
clients and can write debug JSONL events for request/future/executor analysis.

When `--output` is set, timeout-only records are also written next to the main
result file using the suffix `_timeouts.json`. Use `--timeout-output` to choose
a different path. Timeout records include request index, client id, query index,
query type, query text, latency, and whether future cancellation was requested.
The command exits successfully when the experiment runs to completion, even if
some requests fail. Use `--fail-on-failure` when a nonzero exit code is useful
for CI or scripted thresholds.

### Stage 3: Correctness Invariants

An invariant is a correctness check that should remain true while the stress
load is running. Request metrics show whether the service answered; invariants
show whether the database still contains expected baseline facts.

Correctness invariant example for the bundled `ros_typedb` test schema/data:

```bash
ros2 run ros_typedb_benchmark ros_typedb_stress_experiment \
  --clients 10 \
  --duration-s 60 \
  --timeout-s 10 \
  --mode read \
  --invariant-profile test-data \
  --output ~/results/ros_typedb_read_invariants_results.json
```

Use `--invariant-profile test-data` only when `ros_typedb_interface` was started
with the schema/data from `ros_typedb/test/typedb_test_data/`. The profile is
stored in `ros_typedb_benchmark/ros_typedb_benchmark/profiles/test_data_invariants.json`
and checks stable aggregate counts for `person`, `company`, and `employment`,
plus the `boss@tudelft.nl` sentinel person. This profile is not appropriate for
other schemas, such as the `ros_typedb_examples` plan schema.

For `ros_typedb_examples/data/plan_schema.tql` and `plan_data.tql`, use:

```bash
ros2 run ros_typedb_benchmark ros_typedb_stress_experiment \
  --clients 10 \
  --duration-s 60 \
  --timeout-s 10 \
  --mode read \
  --invariant-profile plan-schema \
  --output ~/results/ros_typedb_plan_invariants_results.json
```

The `plan-schema` profile is stored in
`ros_typedb_benchmark/ros_typedb_benchmark/profiles/plan_schema_invariants.json` and
checks counts for `Plan`, `Action`, `Proposition`, and the plan/action relation
types, plus a sentinel action named `collect-water-sample`.

Checks run every 10 seconds by default and once at the end. Use
`--invariant-period-s` to adjust the periodic interval, or
`--invariant-period-s 0` to run only the final check. Invariant failures are
reported separately from request failures and always make the command exit with
status 1.

### Stage 4: Mixed Read/Write Load

Mixed read/write stress for the bundled `ros_typedb` test schema:

```bash
ros2 run ros_typedb_benchmark ros_typedb_stress_experiment \
  --clients 10 \
  --duration-s 120 \
  --timeout-s 10 \
  --mode mixed \
  --invariant-profile test-data \
  --request-gap-s 0.01 \
  --max-in-flight 10 \
  --output ~/results/ros_typedb_mixed_stress_results.json
```

The packaged `test-data` mixed profile reads the database while inserting,
updating, and deleting experiment-owned temporary `robot` entities. It is
intended for the schema in `ros_typedb/test/typedb_test_data/schema.tql` and
cleans up remaining temporary robots before the final invariant check.

Mixed read/write stress for the `ros_typedb_examples` plan schema:

```bash
ros2 run ros_typedb_benchmark ros_typedb_stress_experiment \
  --clients 10 \
  --duration-s 120 \
  --timeout-s 10 \
  --mode mixed \
  --mixed-profile plan-schema \
  --invariant-profile plan-schema-mixed \
  --request-gap-s 0.01 \
  --max-in-flight 10 \
  --output ~/results/ros_typedb_plan_mixed_stress_results.json
```

Mixed profiles are packaged JSON files under
`ros_typedb_benchmark/ros_typedb_benchmark/profiles/`. The `plan-schema` mixed profile
writes temporary `Action` entities, so use `plan-schema-mixed` invariants when
periodic invariant checks run during that mixed workload. That invariant
profile keeps stable checks for plans, propositions, relations, and the
`collect-water-sample` sentinel action without requiring an exact total
`Action` count while temporary actions may exist.

### Stage 5: Delete-Database Fault Injection

This stage calls the real `delete_database` service while normal query clients
are still active. It is meant to check that a missing database does not produce
silent success against an uninitialized empty database. After the delete call,
periodic invariant passes during the active load record recovery as soon as all
checks pass once. If periodic checks do not observe recovery, the runner uses a
post-load recovery check until invariants pass or `--fault-recovery-timeout-s`
expires.

For a reproducible run, start the bundled test-data launch file in one terminal.
It configures and activates `/ros_typedb_interface`, loads
`ros_typedb/test/typedb_test_data/schema.tql` and `data.tql`, and leaves the
schema/data paths available for recovery:

```bash
docker exec -it ros_typedb bash -lc '
cd /home/ubuntu-user/typedb_ws &&
source /opt/ros/humble/setup.bash &&
source install/setup.bash &&
ros2 launch ros_typedb_examples test_data_example.launch.py'
```

Then run the fault-injection experiment from another terminal:

```bash
docker exec -it ros_typedb bash -lc '
cd /home/ubuntu-user/typedb_ws &&
source /opt/ros/humble/setup.bash &&
source install/setup.bash &&
ros2 run ros_typedb_benchmark ros_typedb_stress_experiment \
  --clients 10 \
  --duration-s 180 \
  --timeout-s 10 \
  --mode read \
  --fault delete-database \
  --fault-at-s 60 \
  --invariant-profile test-data \
  --request-gap-s 0.01 \
  --max-in-flight 10 \
  --output /tmp/stage5_delete_database.json \
  --debug-events-output /tmp/stage5_delete_database_debug.jsonl'
```

The result JSON always includes a `fault` object with:

- `fault`: configured fault name, such as `delete-database`
- `fault_at_s`: configured trigger time
- `fault_triggered_at_s`: monotonic timestamp when the fault was triggered
- `fault_delete_success`, `fault_delete_error`, and `fault_delete_latency_s`
- `fault_recovered_at_s`: first timestamp when all recovery invariants passed
- `fault_recovery_s`: elapsed time from trigger to successful recovery

If the fault is triggered but invariants do not recover, the CLI exits with
status 1. Keep `--invariant-profile test-data` paired with the bundled
test-data launch; use a matching invariant profile for any other schema/data
pair. Keep `--max-in-flight 10` and `--request-gap-s 0.01` for robustness
testing so the experiment stays inside the known stable ROS service envelope.

### Stage 6: TypeDB Restart Fault Injection

This stage restarts the TypeDB server while normal query clients continue
sending requests. It is meant to check that server outages produce bounded,
explicit failures and that later requests reconnect to TypeDB without
restarting the ROS lifecycle node. Recovery is still correctness-based: the
first complete post-fault invariant pass marks `fault_recovered_at_s`.

Run TypeDB separately from the ROS node and stress process. Do not pass the
same Docker container that is running `ros_typedb_interface` or the experiment
to `--typedb-container`, because stopping that container stops the test itself.
By default, the process fault uses `pkill -f "typedb/core/server"` to stop
TypeDB and `typedb server` to start it again:

```bash
ros2 run ros_typedb_benchmark ros_typedb_stress_experiment \
  --clients 10 \
  --duration-s 240 \
  --timeout-s 10 \
  --mode read \
  --fault restart-typedb \
  --typedb-restart-delay-s 2 \
  --fault-at-s 60 \
  --invariant-profile test-data \
  --request-gap-s 0.01 \
  --max-in-flight 10 \
  --output /tmp/stage6_restart_typedb.json \
  --debug-events-output /tmp/stage6_restart_typedb_debug.jsonl
```

Override the process commands with `--typedb-stop-command` and
`--typedb-start-command` when needed. For a TypeDB-only Docker container, pass
`--typedb-container`; it overrides the process commands:

```bash
--typedb-container typedb_server
```

The restart controller runs in a background thread, so the main experiment loop
keeps spinning ROS futures and sending requests during the outage. The result
JSON adds restart fields to the `fault` object:

- `fault_restart_stop_success`, `fault_restart_stop_error`, and
  `fault_restart_stop_latency_s`
- `fault_restart_start_success`, `fault_restart_start_error`, and
  `fault_restart_start_latency_s`
- `fault_restart_delay_s`: configured delay between stop and start
- `fault_restart_outage_s`: elapsed controller time from stop start to start
  completion
- `fault_observed_outage_s`: elapsed time from the first failed post-fault
  request result to the first later successful request result

If the fault is triggered but invariants do not recover, the CLI exits with
status 1. Request failures during the outage remain recorded in the JSON output;
use `--fail-on-failure` only when you want any request failure to fail the
command even if invariants later recover.

### Stage 7: Lifecycle Stress

This stage calls the ROS lifecycle `change_state` service while normal query
clients continue sending requests. The default sequence is deactivate, cleanup,
configure, then activate. Cleanup destroys the query and delete services and
waits for active service callbacks before closing TypeDB resources; the
experiment verifies that this does not race with in-flight requests and that
the node recovers after reactivation. The controller checks `get_state` after
each transition and only sends the next transition after the expected lifecycle
state is observed.

When using a launch file that automatically reactivates the node after every
deactivate, disable that behavior for this experiment. The bundled test-data
launch file defaults to normal auto-reactivation, so start it for Stage 7 with:

```bash
ros2 launch ros_typedb_examples test_data_example.launch.py \
  reactivate_on_deactivate:=False
```

Then run the stress experiment:

```bash
ros2 run ros_typedb_benchmark ros_typedb_stress_experiment \
  --clients 10 \
  --duration-s 180 \
  --timeout-s 10 \
  --mode read \
  --fault lifecycle-cleanup \
  --fault-at-s 60 \
  --invariant-profile test-data \
  --request-gap-s 0.01 \
  --max-in-flight 10 \
  --output /tmp/stage7_lifecycle_cleanup.json \
  --debug-events-output /tmp/stage7_lifecycle_cleanup_debug.jsonl
```

The lifecycle `change_state` and `get_state` services default to the
`--service-name` prefix with `/change_state` and `/get_state` appended. For
example, the default query service `/ros_typedb_interface/query` maps to
`/ros_typedb_interface/change_state` and `/ros_typedb_interface/get_state`.
Override them with `--lifecycle-change-state-service-name` and
`--lifecycle-get-state-service-name` when the launched lifecycle node uses a
different name, such as `/ros_typedb/change_state` and
`/ros_typedb/get_state`.

By default the node is configured and activated again after cleanup. Use
`--no-lifecycle-reactivate` only when you intentionally want to leave the node
cleaned up; in that mode recovery invariants are not expected to pass unless
something else reactivates the node.

The result JSON adds lifecycle fields to the `fault` object:

- `fault_lifecycle_deactivate_success`, `fault_lifecycle_deactivate_error`,
  and `fault_lifecycle_deactivate_latency_s`
- `fault_lifecycle_cleanup_success`, `fault_lifecycle_cleanup_error`, and
  `fault_lifecycle_cleanup_latency_s`
- `fault_lifecycle_configure_success`, `fault_lifecycle_configure_error`, and
  `fault_lifecycle_configure_latency_s`
- `fault_lifecycle_activate_success`, `fault_lifecycle_activate_error`, and
  `fault_lifecycle_activate_latency_s`
- `fault_lifecycle_reactivate`: whether configure/activate was requested
- `fault_observed_outage_s`: elapsed time from the first failed post-fault
  request result to the first later successful request result

If a lifecycle transition is rejected or times out, the CLI exits with status
1. If transitions succeed but invariants do not recover, the CLI also exits
with status 1. Request failures while the lifecycle node is inactive or cleaned
up remain recorded in the JSON output.

### Planned Fault-Injection Stages

Stage 8 is not implemented yet. It is a placeholder for the next robustness
check:

- Stage 8, external profiles: load schema-specific query mixes, invariants,
  setup, and cleanup from user-provided JSON/YAML profile files instead of only
  the packaged examples.

### Diagnostics and Boundaries

For ROS client/executor debugging, `--debug-events-output` writes JSONL events
for request sends, `spin_once()` calls, completed futures, timeouts, and periodic
pending-request snapshots. Use `--executor single` or `--executor multi` to
compare explicit executors against the default `rclpy.spin_once(node)` behavior.
Use `--request-gap-s` to add a small per-client delay between requests, or
`--max-in-flight` to cap total outstanding service requests independently of the
number of clients.

For `ros_typedb` robustness experiments, prefer bounded service concurrency.
High-rate unbounded Query service calls produced timeout waves even when TypeDB
was removed from the path by using fake Query services. The same behavior was
observed with Python and C++ clients/servers, Fast DDS and CycloneDDS, and ROS
Humble and Lyrical. Treat that as a ROS service transport/executor stress limit,
not as a database or result-conversion failure.

A practical bounded baseline is:

```bash
ros2 run ros_typedb_benchmark ros_typedb_stress_experiment \
  --clients 20 \
  --duration-s 60 \
  --timeout-s 10 \
  --mode read \
  --request-gap-s 0.01 \
  --max-in-flight 10 \
  --output ~/results/ros_typedb_read_bounded_results.json
```

Use larger `--max-in-flight` values only when intentionally probing ROS service
transport saturation. In local tests, about 12-15 in-flight requests started to
produce timeout waves, while 10 or fewer stayed stable.

To isolate ROS service/client behavior from TypeDB work, run a fake Query
service in one terminal:

```bash
ros2 run ros_typedb_benchmark ros_typedb_fake_query_service
```

Then run the same stress command against it from another terminal.

To run the common timeout-debug scenarios in sequence, use:

```bash
bash src/ros_typedb/ros_typedb_benchmark/scripts/run_stage2_timeout_scenarios.sh
```

The script runs real read/global, real read/multi, fake read/global, fake
read/multi, and real explicit entity-get/global scenarios. Override defaults
with environment variables such as `CLIENTS`, `DURATION_S`, `TIMEOUT_S`, and
`OUTPUT_DIR`.

Useful options:

- `--service-name`: query service name. Defaults to `/ros_typedb_interface/query`
- `--query`: optional TypeDB query to send on each request
- `--query-type`: one of `define`, `delete`, `fetch`, `get`,
  `get_aggregate`, `insert`, or `update`; required when `--query` is used
- `--requests`: number of requests to send when `--duration-s` is omitted
- `--clients`: number of concurrent service clients
- `--duration-s`: run for this many seconds instead of a fixed request count
- `--mode`: built-in query mix to use when `--query` is omitted; supports
  `read` and `mixed`
- `--mixed-profile`: schema-specific mixed-mode profile, such as `test-data`
  or `plan-schema`; `auto` derives it from the invariant profile when possible
- `--request-gap-s`: minimum delay between requests from the same client
- `--max-in-flight`: maximum number of outstanding requests across all clients
- `--executor`: client executor mode, one of `global`, `single`, or `multi`
- `--debug-events-output`: optional JSONL debug event path
- `--invariant-profile`: optional correctness profile, such as `test-data`,
  `plan-schema`, or `plan-schema-mixed`
- `--invariant-period-s`: seconds between periodic invariant checks
- `--timeout-s`: per-request client and server timeout
- `--output`: optional JSON results path
- `--timeout-output`: optional timeout-only JSON results path
- `--fail-on-failure`: return exit code 1 when any request fails

## Tests

```Bash
colcon test --event-handlers console_cohesion+ --packages-select ros_typedb_benchmark
```

## Docker

The commands below assume you are running them from the workspace root:

```Bash
cd <workspace_root>
```

Start dev container **without** display and the `ros_typedb` directory mounted:

```Bash
docker run -it --rm --name ros_typedb -v /etc/localtime:/etc/localtime:ro -v $PWD/src/ros_typedb:/home/ubuntu-user/typedb_ws/src/ros_typedb  -v $PWD/ros_typedb_results/:/home/ubuntu-user/results/ ros_typedb
```

Start new terminal in the container:

```Bash
docker exec -it ros_typedb bash
```


 export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
