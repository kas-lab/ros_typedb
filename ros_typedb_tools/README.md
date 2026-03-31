# ros_typedb_tools

Reusable TypeDB 2 helper scripts for generating schema and rule diagrams.

## Overview

This package currently provides two console scripts:

- `typedb_schema_diagram`: generate a diagram for one schema file or a merged diagram for multiple schema files.
- `typedb_rule_diagram`: generate a rule dependency/read-write diagram for one or more schema files that contain rules.

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
  --input src/typedb_tactics/schemas/data_structure/data_structure.tql \
          src/typedb_tactics/schemas/context_model/context_model.tql \
  --output /tmp/merged_schema.svg
```

Useful options:

- `--input`: one or more `.tql` schema files
- `--output`: explicit output path
- `--format`: `svg` or `dot`
- `--orientation`: `vertical` or `horizontal`
- `--include-rules`: include lightweight rule notes in the schema diagram

## typedb_rule_diagram

Generates a rule-centric diagram from TypeDB rules.

What it shows:

- rule nodes
- concepts read in the `when` block
- concepts written in the `then` block
- optional rule-to-rule dependency edges when one rule writes a concept another rule reads

Examples:

```bash
ros2 run ros_typedb_tools typedb_rule_diagram \
  --input src/typedb_tactics/schemas/feature_model/feature_model.tql

ros2 run ros_typedb_tools typedb_rule_diagram \
  --input src/typedb_tactics/schemas/feature_model/feature_model.tql \
  --format dot \
  --output /tmp/feature_model_rules.dot

ros2 run ros_typedb_tools typedb_rule_diagram \
  --input src/typedb_tactics/schemas/feature_model/feature_model.tql \
          src/typedb_tactics/schemas/discover_tactics_model/discover_tactics_model.tql \
  --no-rule-dependencies \
  --output /tmp/combined_rules.svg
```

Useful options:

- `--input`: one or more `.tql` files with rule definitions
- `--output`: explicit output path
- `--format`: `svg` or `dot`
- `--orientation`: `vertical` or `horizontal`
- `--no-rule-dependencies`: suppress rule-to-rule dependency edges

## Running Without ros2 run

If needed, you can also invoke the modules directly from the workspace root:

```bash
PYTHONPATH=src/ros_typedb/ros_typedb_tools python3 -m ros_typedb_tools.typedb_schema_diagram --help
PYTHONPATH=src/ros_typedb/ros_typedb_tools python3 -m ros_typedb_tools.typedb_rule_diagram --help
```

## Installed README Location

After rebuilding the package, this README is installed to:

```text
install/ros_typedb_tools/share/ros_typedb_tools/README.md
```
