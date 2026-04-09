# Agent Guide

This file is for agents implementing and validating new features in this repository.

## Serena Startup

At the beginning of a new session in this repository, call `serena.activate_project` for `/home/gus/ros_workspaces/tactical_retreat_3_ws/src/ros_typedb`, then call `serena.check_onboarding_performed`.

Do not rerun Serena onboarding if it is already complete.

## Agent docs location

- Keep agent process/checklist docs in `agents-doc/`.
- `doc/` and `docs/` are project/user documentation.
- Do not load `doc/` or `docs/` unless explicitly asked.

## Project layout

- `ros_typedb/ros_typedb/typedb_interface.py`: core TypeDB access layer.
- `ros_typedb/ros_typedb/ros_typedb_interface.py`: ROS lifecycle node + query service.
- `ros_typedb/ros_typedb/typedb_helpers.py`: helper utilities for common TypeQL operations.
- `ros_typedb/test/`: unit/integration tests.
- `ros_typedb_tools/`: ament-python tools package for schema/function diagram generation.

## Development environment

Use Docker container `ros_typedb`.

Before executing build, test, launch, or query commands, ensure the TypeDB server is running inside the container.

Start container with TypeDB server:

```bash
docker run -d --rm --name ros_typedb -v /etc/localtime:/etc/localtime:ro -v $PWD:/home/ubuntu-user/typedb_ws/src/ros_typedb ros_typedb sudo typedb server
```

## Required workflow for code changes

1. Build in container:

```bash
docker exec ros_typedb bash -lc "source /opt/ros/humble/setup.bash && cd /home/ubuntu-user/typedb_ws && colcon build --symlink-install --packages-select ros_typedb ros_typedb_tools"
```

2. Run relevant feature tests (targeted or full):

```bash
scripts/run-tests-docker.sh
```

3. Run mandatory quality checks:

```bash
scripts/run-mandatory-checks-docker.sh
```

4. Final response must include:
- Commands executed
- Pass/fail summary
- Blockers (if any command could not be run)

## Testing commands

Run all `ros_typedb` tests:

```bash
docker exec ros_typedb bash -lc "source /opt/ros/humble/setup.bash && source /home/ubuntu-user/typedb_ws/install/setup.bash && cd /home/ubuntu-user/typedb_ws/src/ros_typedb/ros_typedb && python3 -m pytest -q test/test_typedb_interface.py test/test_typedb_helpers.py test/test_ros_typedb_interface.py"
docker exec ros_typedb bash -lc "source /opt/ros/humble/setup.bash && source /home/ubuntu-user/typedb_ws/install/setup.bash && cd /home/ubuntu-user/typedb_ws/src/ros_typedb && python3 -m pytest -q ros_typedb_tools/test/test_schema_diagram.py ros_typedb_tools/test/test_rule_diagram.py ros_typedb_tools/test/test_typedb_schema_diagram_cli.py ros_typedb_tools/test/test_typedb_rule_diagram_cli.py"
```

Run one test pattern:

```bash
docker exec ros_typedb bash -lc "source /opt/ros/humble/setup.bash && source /home/ubuntu-user/typedb_ws/install/setup.bash && cd /home/ubuntu-user/typedb_ws/src/ros_typedb/ros_typedb && python3 -m pytest -q -k malformed_query"
```

## Implementation notes

- Keep `TypeDBQueryError` propagation behavior intact in `TypeDBInterface`.
- In ROS service callbacks, catch backend query exceptions and return `success = False`.
- Preserve test data loading behavior for `match ... insert ...` blocks.

## Helper scripts

- `scripts/run-tests-docker.sh`
- `scripts/run-mandatory-checks-docker.sh`
- `scripts/find-text2typeql-examples.py` (only when doing NL->TypeQL tasks)
- `ros_typedb_tools/README.md` documents the schema/function diagram CLIs.

## Local skills

- `.claude/skills/new-typedb-fun/SKILL.md`: repo-local guidance for writing new TypeDB 3 `fun` functions.
- `.claude/skills/typedb/SKILL.md`: general TypeDB 3 query/function reference.
- Mirror copies also live under `.codex/skills/` for Codex-oriented workflows.
