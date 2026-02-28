# Agent Notes

This file is a short checklist for agents working in this repo.

## Quick test entrypoint

Preferred focused run inside Docker:

```bash
scripts/run-tests-docker.sh
```

## Code style

- Python code must pass flake8 and pep257 tests
- Add the following copyright to python files

```Python
# Copyright 2026 KAS Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
```

## Common failure checks

- Is TypeDB running on `localhost:1729`?
- Is the ROS + workspace environment sourced?
- Is the `tactical_retreat` container running (if using Docker)?

## Rule authoring reminders

- Avoid negation cycles: do not read/write the same predicate under `not {}`.
- Variables in `then` must be bound in `when` outside nested blocks.
- If you need to assign attributes to a relation, bind or create the relation first.

## Mandatory completion checks

- Always run and pass both lint/style tests before marking work as done:
  - `colcon test --event-handlers console_cohesion+ --packages-select typedb_tactics --pytest-args -k test_flake8`
  - `colcon test --event-handlers console_cohesion+ --packages-select typedb_tactics --pytest-args -k test_pep257`