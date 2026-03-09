# CLAUDE.md

Claude-specific guidance for this repository.

## Read first

- `AGENTS.md`
- `agents-doc/agent-notes.md`

## Scope

- Use agent docs in `agents-doc/` for implementation workflow.
- `doc/` and `docs/` are for project/user documentation and are ignored by default.

## Package

- ROS 2 package providing a TypeDB 3 client interface for robot knowledge bases.
- Main entry point: `ros_typedb/typedb_interface.py`
- Uses TypeDB 3 — do not use TypeDB 2 syntax (rules, `isa!`, old driver API).

## Completion rule

For code changes:

1. Run feature tests (`scripts/run-tests-docker.sh` or equivalent targeted tests).
2. Run mandatory checks (`scripts/run-mandatory-checks-docker.sh`).
3. Report commands and results in final response.
