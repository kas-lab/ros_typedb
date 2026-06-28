# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Serena

At the start of every session, activate Serena for this project:

```python
mcp__plugin_serena_serena__activate_project("/home/gus/ros_workspaces/suave_rebetmc_ws/src/ros_typedb")
mcp__plugin_serena_serena__initial_instructions()
```

Then read relevant memories (e.g. `project_overview`, `style_and_conventions`) before touching code.

For ALL code exploration and editing, use Serena's symbolic tools instead of `Bash` grep/cat or `Read`:
- `get_symbols_overview` — survey a file's symbols
- `find_symbol` — locate a class/function by name path
- `find_referencing_symbols` — trace callers/usages
- `replace_symbol_body` — rewrite an entire symbol
- `replace_content` — targeted regex replacement within a file

## Project guidelines

All project conventions, TypeDB rules, testing guidelines, stress tooling notes,
architecture notes, commit guidelines, and security tips are maintained in **`AGENTS.md`**
at the repo root. Read that file now before touching any code.

## ROSTypeDBInterface Gotchas

**Lifecycle state check (rclpy Humble):** `get_current_state()` does not exist. The only
way to read lifecycle state programmatically is:
```python
node._state_machine.current_state[1] == 'active'
```

**Service naming in `__init__`:** When creating services inside `LifecycleNode.__init__`,
use `self.get_name()` — not the `node_name` constructor argument. ROS name remapping is
applied during `super().__init__()`, so `node_name` is the un-remapped name.
`self.get_name()` is correct immediately after `super().__init__()` returns.

**`destroy_service()` race with `MultiThreadedExecutor`:** Calling `destroy_service()` from
`on_cleanup()` while the executor is spinning causes `InvalidHandle` — the executor accesses
service handles in `_take_service()` before any application-level lock. Design pattern:
create services once in `__init__`, never destroy them; gate work in callbacks by checking
lifecycle state and `typedb_interface is None` inside `_service_callback_lock`.
