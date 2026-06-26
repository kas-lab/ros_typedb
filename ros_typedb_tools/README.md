# ros_typedb_tools

Reusable TypeDB 2 helper scripts for generating schema and rule diagrams.

## Overview

This package currently provides three console scripts:

- `typedb_schema_diagram`: generate a diagram for one schema file or a merged diagram for multiple schema files.
- `typedb_rule_diagram`: generate a rule dependency/read-write diagram for one or more schema files that contain rules.
- `ros_typedb_stress_experiment`: send repeated requests to the real `ros_typedb` query service and report request metrics.

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

## ros_typedb_stress_experiment

Runs fixed-count or duration-based stress experiments against the real
`/ros_typedb_interface/query` service.

Prerequisites:

- TypeDB is running.
- `ros_typedb_interface` is running.
- The lifecycle node has been configured so the query service exists.

Example:

```bash
ros2 run ros_typedb_tools ros_typedb_stress_experiment \
  --query 'match $p isa Plan; (plan: $p, action: $a) isa has_action; fetch $a: attribute;' \
  --query-type fetch \
  --requests 2000 \
  --timeout-s 5 \
  --output ~/results/ros_typedb_stress_results.json
```

Concurrent read stress example:

```bash
ros2 run ros_typedb_tools ros_typedb_stress_experiment \
  --clients 20 \
  --duration-s 60 \
  --timeout-s 10 \
  --mode read \
  --output ~/results/ros_typedb_read_stress_results.json
```

Useful options:

- `--service-name`: query service name. Defaults to `/ros_typedb_interface/query`
- `--query`: optional TypeDB query to send on each request
- `--query-type`: one of `define`, `delete`, `fetch`, `get`,
  `get_aggregate`, `insert`, or `update`; required when `--query` is used
- `--requests`: number of requests to send when `--duration-s` is omitted
- `--clients`: number of concurrent service clients
- `--duration-s`: run for this many seconds instead of a fixed request count
- `--mode`: built-in query mix to use when `--query` is omitted; currently
  supports `read`
- `--timeout-s`: per-request client and server timeout
- `--output`: optional JSON results path
