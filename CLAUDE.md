# CLAUDE.md

This file provides Claude Code-specific guidance. For full project documentation see [AGENTS.md](AGENTS.md) and [agents-doc/agent-notes.md](agents-doc/agent-notes.md).

## Primary Goal

Migrate `ros_typedb` from the TypeDB 2 Python driver to the **TypeDB 3 Python driver**, and redesign the package architecture to use composition instead of inheritance for user-defined query classes. See the architecture design doc at [docs/plans/2026-02-27-typedb3-architecture-design.md](docs/plans/2026-02-27-typedb3-architecture-design.md).

## Read These First

- **[AGENTS.md](AGENTS.md)** — project overview, dependencies, common commands, Docker setup, TypeDB 2/3 migration notes, TypeQL pitfalls, and agent helpers.
- **[agents-doc/agent-notes.md](agents-doc/agent-notes.md)** — quick checklist, code style rules, common failure checks, and mandatory completion checks.

## Architecture Summary

The redesigned package splits responsibilities across four files:

| File | Responsibility |
|------|----------------|
| `typedb_interface.py` | `TypeDBInterface`: backend connection + raw query execution |
| `typedb_helpers.py` | Standalone helper functions (insert_entity, fetch_attribute_from_thing, etc.) |
| `ros_typedb_interface.py` | `ROSTypeDBInterface`: ROS lifecycle node + generic `/query` service |
| `ros_typedb_helpers.py` | ROS conversion helpers (query result → ROS msgs) |

Users inject `TypeDBInterface` into plain Python query classes — no inheritance from `TypeDBInterface`. ROS wiring lives in a separate node class that inherits `ROSTypeDBInterface` only for lifecycle boilerplate.

## Claude Code Specifics

- **Ignored paths:** `.claudeignore` at the project root excludes `__pycache__`, build artifacts, and test caches.
- **Memory:** persistent session notes live in `~/.claude/projects/…/memory/`. Consult and update these across sessions.
- **Working branch:** `typedb3_py` — all migration work goes here; `main` holds the stable TypeDB 2 implementation.
- **Before claiming work is done:** run lint and tests as described in `agents-doc/agent-notes.md` and use the `superpowers:verification-before-completion` skill.
