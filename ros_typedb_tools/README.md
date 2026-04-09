# ros_typedb_tools

Reusable TypeDB 3 helper scripts for generating schema and function diagrams.

## Overview

This package provides two console scripts:

- `typedb_schema_diagram`: generate a diagram for one schema file or a merged diagram for multiple schema files.
- `typedb_rule_diagram`: generate a function dependency/read diagram for one or more schema files that contain TypeDB 3 `fun` definitions.

Both tools can write `.svg` or `.dot` output.

## Prerequisites

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Graphviz is required for `.svg` output. `.dot` output does not require Graphviz rendering.

## typedb_schema_diagram

Generates a visual summary of TypeDB schema structure.

What it extracts:

- `entity`, `relation`, and `attribute` types
- `sub` inheritance
- `owns`
- `plays`
- relation `relates`

Examples:

```bash
ros2 run ros_typedb_tools typedb_schema_diagram \
  --input src/typedb_tactics/schemas/feature_model/feature_model.tql

ros2 run ros_typedb_tools typedb_schema_diagram \
  --input src/typedb_tactics/schemas/feature_model/feature_model.tql \
  --format dot \
  --output /tmp/feature_model.dot

ros2 run ros_typedb_tools typedb_schema_diagram \
  --input src/typedb_tactics/schemas/feature_model/feature_model.tql \
          src/typedb_tactics/schemas/data_structure/data_structure.tql \
  --output /tmp/merged_schema.svg
```

Useful options:

- `--input`: one or more `.tql` schema files
- `--output`: explicit output path
- `--format`: `svg` or `dot`
- `--orientation`: `vertical` or `horizontal`
- `--include-functions`: include lightweight function notes in the schema diagram

## typedb_rule_diagram

Generates a function-centric diagram from TypeDB 3 functions.

What it shows:

- function nodes
- concepts read in function signatures and bodies
- return-type edges
- optional function-to-function dependency edges when one function calls another

Examples:

```bash
ros2 run ros_typedb_tools typedb_rule_diagram \
  --input src/typedb_tactics/schemas/discover_tactics_model/discover_tactics_model.tql

ros2 run ros_typedb_tools typedb_rule_diagram \
  --input src/typedb_tactics/schemas/discover_tactics_model/discover_tactics_model.tql \
  --format dot \
  --output /tmp/discover_tactics_model_functions.dot

ros2 run ros_typedb_tools typedb_rule_diagram \
  --input src/typedb_tactics/schemas/discover_tactics_model/discover_tactics_model.tql \
          src/typedb_tactics/schemas/discover_hypothetical_req_fulfillment/discover_hypothetical_req_fulfillment.tql \
  --no-rule-dependencies \
  --output /tmp/combined_functions.svg
```

Useful options:

- `--input`: one or more `.tql` files with function definitions
- `--output`: explicit output path
- `--format`: `svg` or `dot`
- `--orientation`: `vertical` or `horizontal`
- `--no-rule-dependencies`: suppress function-to-function dependency edges

## Running Without ros2 run

If needed, you can also invoke the modules directly from the workspace root:

```bash
PYTHONPATH=src/ros_typedb/ros_typedb_tools python3 -m ros_typedb_tools.typedb_schema_diagram --help
PYTHONPATH=src/ros_typedb/ros_typedb_tools python3 -m ros_typedb_tools.typedb_rule_diagram --help
```
