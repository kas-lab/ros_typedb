# ros_typedb Style And Conventions

## Python conventions

- Use snake_case naming for modules, functions, and variables.
- Use explicit type hints in public interfaces.
- Use PEP 257-style docstrings with `:param` / `:return:` sections.
- Keep the ROS layer thin; keep reusable query/domain logic in plain Python classes.

## Enforced checks

- `flake8` compliance is required.
- `pep257` compliance is required.
- Pytest-based validation is part of the standard workflow.

## Architectural rules

- Preserve `TypeDBQueryError` propagation in `TypeDBInterface`.
- In ROS service callbacks, catch backend query exceptions and return `success = False`.
- Preserve current test data loading behavior for `match ... insert ...` blocks.
- `ROSTypeDBInterface` should remain the ROS lifecycle/service wrapper around the core DB interface.
