# Repository Guidelines

## Project Structure & Module Organization

This repository contains three ROS 2 packages. `ros_typedb/` is the core
`ament_python` package, with runtime code in `ros_typedb/ros_typedb/`, launch
files in `ros_typedb/launch/`, and tests plus TypeDB fixtures in
`ros_typedb/test/`. `ros_typedb_msgs/` is the `ament_cmake` interface package;
message definitions live in `msg/` and services in `srv/`. `ros_typedb_tools/`
contains Python command-line tools for TypeDB schema and rule diagrams, with
tests and fixtures under `ros_typedb_tools/test/`. Sphinx documentation is in
`docs/source/`.

## Build, Test, and Development Commands

Run commands from the ROS workspace root unless noted:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_cohesion+ --packages-up-to ros_typedb
colcon test --event-handlers console_cohesion+ --packages-select ros_typedb_tools
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

Do not let `ensure_database_exists()` silently create an empty database. If the
configured database is missing, recreate it only by replaying stored schema/data
paths, or fail loudly when no initialization files are configured. Recovery code
that runs while `_database_query_lock` is held should use internal unlocked
helpers instead of calling the public `database_query()` path again; this avoids
recursive health checks and avoids needing `RLock`.

`reload_schema=False` should skip schema redefinition only when reusing an
existing database. Newly created, force-recreated, or missing-and-recreated
databases must still load the configured schema before data is loaded.

`delete_all_data()` must fail loudly if any delete step fails. Do not load new
data on top of potentially residual data after a failed cleanup. The current
live fixture confirms entity-first deletion works for the bundled schema/data,
but that does not prove the order is universal for every TypeDB schema.

For TypeDB `fetch` queries, do not fetch a bare concept variable directly
(`fetch $x;`). Fetch attributes explicitly; to fetch all attributes owned by a
matched concept, use `fetch $x: attribute;`.

For PR support notes under `docs-agents/`, keep changelogs brief: summary,
compact bullet list of changes, and verification commands. Avoid copying the
more verbose style from planning documents.

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
