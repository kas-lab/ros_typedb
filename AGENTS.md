# Repository Guidelines

## Project Structure & Module Organization

This repository contains four ROS 2 packages. `ros_typedb/` is the core
`ament_python` package, with runtime code in `ros_typedb/ros_typedb/`, launch
files in `ros_typedb/launch/`, and tests plus TypeDB fixtures in
`ros_typedb/test/`. `ros_typedb_msgs/` is the `ament_cmake` interface package;
message definitions live in `msg/` and services in `srv/`. `ros_typedb_tools/`
contains Python command-line tools for TypeDB schema and rule diagrams, with
tests and fixtures under `ros_typedb_tools/test/`. `ros_typedb_benchmark/`
contains stress-testing, fault-injection, and benchmark tools. Sphinx
documentation is in `docs/source/`.

## Build, Test, and Development Commands

Run commands from the ROS workspace root unless noted:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_cohesion+ --packages-up-to ros_typedb
colcon test --event-handlers console_cohesion+ --packages-select ros_typedb_tools
colcon test --event-handlers console_cohesion+ --packages-select ros_typedb_benchmark
```

Use `ros2 run ros_typedb ros_typedb_interface -p schema_path:=<schema> -p data_path:=<data>`
to run the node after building. From `docs/`, run `make html` to build docs.

## Coding Style & Naming Conventions

Python code follows ROS 2 `ament_flake8` and `ament_pep257` checks. Use 4-space
indentation, snake_case for modules/functions, PascalCase for classes, and
descriptive ROS parameter names. Keep console entry point names aligned with
`setup.py`, for example `typedb_schema_diagram`. Preserve existing package
layout and avoid mixing generated ROS interfaces with Python runtime modules.

Use Serena for code navigation before and after non-trivial Python changes.
Prefer symbol overviews, symbol lookup, and reference searches to broad file
reads, and re-check Serena's symbol view after adding or renaming methods.

## Testing Guidelines

Tests use `pytest` through `colcon test`, with ROS lint wrappers in
`test/test_flake8.py`, `test/test_pep257.py`, and `test/test_copyright.py`.
Name new tests `test_*.py` and keep TypeDB sample data in the relevant
`test/fixtures/` or `test/typedb_test_data/` directory. Add focused tests for
query behavior, lifecycle behavior, and diagram output when those areas change.

When the `ros_typedb` Docker container is running, prefer it for tests that need
the correct ROS Humble and TypeDB driver/server environment. The package test
fixtures use paths relative to the `ros_typedb/` package directory, so run
direct pytest from there, for example:

```bash
docker exec ros_typedb bash -lc '
cd /home/ubuntu-user/typedb_ws/src/ros_typedb/ros_typedb &&
source /opt/ros/humble/setup.bash &&
source /home/ubuntu-user/typedb_ws/install/setup.bash &&
PYTHONPATH=/home/ubuntu-user/typedb_ws/src/ros_typedb/ros_typedb \
  python3 -m pytest -q test/test_typedb_interface_unit.py'
```

For ROS-facing direct pytest runs, preserve the ROS-provided `PYTHONPATH` and
prepend the package path instead of replacing it:

```bash
PYTHONPATH=/home/ubuntu-user/typedb_ws/src/ros_typedb/ros_typedb:$PYTHONPATH \
  python3 -m pytest -q test/test_ros_typedb_interface_unit.py
```

For package-level verification, run from the container workspace root:

```bash
docker exec ros_typedb bash -lc '
cd /home/ubuntu-user/typedb_ws &&
source /opt/ros/humble/setup.bash &&
source install/setup.bash &&
colcon test --event-handlers console_cohesion+ --packages-select ros_typedb'
```

Keep TypeDB interface tests split by dependency style:

- `test_typedb_interface_unit.py`: fake drivers/sessions, timeout control flow,
  formatting helpers, and Python-only failure paths.
- `test_typedb_interface_integration.py`: live TypeDB server tests using real
  schema/data files.
- `test_ros_typedb_interface_unit.py`: ROS message conversion, direct callback
  plumbing, and mocked lifecycle cleanup helpers.
- `test_ros_typedb_interface_integration.py`: launch tests, lifecycle service
  transitions, and live ROS query service behavior.

## TypeDB Runtime Notes

Do not let missing-database recovery silently create an empty database. Recreate
from configured schema/data paths, or fail loudly when no initialization files
are configured.

`reload_schema=False` should skip schema redefinition only when reusing an
existing database. Newly created, force-recreated, or missing-and-recreated
databases must still load the configured schema before data is loaded.

`delete_all_data()` must fail loudly if any delete step fails. Do not load new
data on top of potentially residual data after a failed cleanup.

For TypeDB `fetch` queries, do not fetch a bare concept variable directly
(`fetch $x;`). Fetch attributes explicitly; to fetch all attributes owned by a
matched concept, use `fetch $x: attribute;`.

TypeDB `fetch` result shapes differ between explicit and wildcard attribute
fetches. Explicit fetches such as `fetch $x: email;` are keyed by the concrete
attribute label and usually contain a homogeneous list of values. Wildcard
fetches such as `fetch $x: attribute;` are keyed by `attribute` and can contain
mixed labels and value types in the same list. Normalize wildcard results by
grouping them by concrete `(label, value_type)` before converting them to ROS
`Attribute` messages.

When changing ROS fetch-result conversion, cover both unit-level JSON/dict
shapes and launch integration tests against the real TypeDB service.

ROS service callbacks should catch and log query execution or result conversion
exceptions, then return `success=False` with `error_message` populated. Do not
let malformed or unexpected query results escape the callback and crash the
`ros_typedb_interface` node.

## ros_typedb_benchmark Stress Diagnostics

When debugging Stage 2 stress timeouts, use the fake query service and the
scenario runner before changing `ros_typedb_interface` behavior. If the fake
service reproduces the same timeout-wave pattern, suspect ROS/rclpy/RMW/DDS or
stress-harness behavior rather than TypeDB queries, schema/data, or result
conversion.

High-rate ROS `Query` service calls can produce timeout waves even with fake
services that do no TypeDB work. Treat this as a ROS service
transport/executor stress limit, not as evidence of a TypeDB/query bug. For
database robustness tests, stay inside the bounded envelope: in local testing
`--max-in-flight 10` with `--request-gap-s 0.01` was stable.

Stress invariants are correctness checks that run alongside load to verify the
database still contains expected baseline facts. Keep invariant and mixed
read/write profiles as packaged JSON files under
`ros_typedb_benchmark/ros_typedb_benchmark/profiles/`, not as hard-coded Python data.
Update `setup.py` package data when adding new profile file patterns.

Do not use a built-in invariant profile with a different schema/data pair. A
passing request-load test only proves the service answered; passing invariants
prove the expected baseline data remained visible.

Stage 5 delete-database fault injection deletes the database through the real
`/delete_database` service while read load continues. The database is not
recreated by the delete service itself; recreation is lazy and happens when a
later `database_query()` calls `ensure_database_exists()`. That recovery only
works when the node was configured with valid schema/data paths.

Stage 5 recovery should be measured from the delete fault trigger to the first
complete successful invariant pass after the fault. Periodic invariants during
active load are the primary recovery signal; the post-load recovery loop is
only a fallback when periodic checks did not observe recovery. Do not implement
recovery timing that starts only after the main load duration ends, because it
can falsely report `recovery: not recovered` even if ordinary post-delete
queries already recreated and reloaded the database.

`ros_typedb_examples` can include launch files for reusable schema/data demos.
If a launch file needs data that normally lives under another package's `test/`
tree, install only the specific runtime fixture files through the examples
package `setup.py`; do not rely on source-tree-relative paths after
`colcon build`.

Useful stress diagnostics:

```bash
bash src/ros_typedb/ros_typedb_benchmark/scripts/run_stage2_timeout_scenarios.sh
```

The script runs real read/global, real read/multi, fake read/global, fake
read/multi, and real explicit entity-get/global scenarios. Use
`--debug-events-output` traces to inspect `request_sent`, `future_done`,
`request_timeout`, `timeout_barrier`, and `pending_snapshot` events.

When changing `setup.py` console entry points in an `ament_python` package,
`colcon build --symlink-install` may leave stale generated entry-point scripts.
If a `ros2 run` command still imports an old module after a source change,
remove that package's generated `build/<pkg>` and `install/<pkg>` directories
inside the workspace and rebuild the package.

## Commit & Pull Request Guidelines

Recent history uses short imperative commits such as `fix lint` and
`Serialize TypeDB database queries`. Keep subjects concise and action-oriented;
include scope when it helps, such as `ros_typedb_tools: fix rule parsing`.
Pull requests should describe behavior changes, list the `colcon test` commands
run, link related issues, and include generated diagram screenshots or samples
when tool output changes.

## Security & Configuration Tips

Do not commit local TypeDB databases, credentials, or generated build/install/log
directories. This project targets Ubuntu 22.04, ROS Humble, and TypeDB 2.x; note
version changes in PRs because driver compatibility affects runtime behavior.
