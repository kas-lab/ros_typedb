# Agent guide

This file provides guidance for Agents to work with this repository

## Project overview

This repository packages a ROS 2 package that interfaces TypeDB 3 with ROS 2 using the TypeDB 3 Python driver. It contains 2 ROS packages: `ros_typedb` and `ros_typedb_msgs`. This is a migration from the original TypeDB 2.28.3 Python driver implementation.

In summary:

- the `ros_typedb` package provides classes to interface with TypeDB. `TypeDBInterface` handles reading/writing schemas/data from/to the database. `ROSTypeDBInterface` provides ROS interfaces for the methods in `TypeDBInterface`
- the `ros_typedb_msgs` provides ROS 2 messages that are used by `ROSTypeDBInterface`

## Dependencies and environment

- TypeDB 3.x
- TypeDB 3 Python driver (installed in Dockerfile-TypeDB3 image)
- Python dependencies:
  - setuptools==75.8.2
  - packaging==24.1
  - empy==3.3.4
  - pandas==2.0.2
  - scipy==1.15.2
  - numpy==1.23.5
- ROS 2 Humble
- ROS dependencies: check `package.xml`

## Common commands

Source ROS and workspace before anything:

```Bash
source /opt/ros/humble/setup.bash
```

From the workspace folder:

```Bash
source ../../install/setup.bash
```

Build repo:

```Bash
colcon build --symlink-install
```

Test the whole repo:

```Bash
colcon test --event-handlers console_cohesion+ --packages-select ros_typedb
```

Run specific tests:

```Bash
colcon test --event-handlers console_cohesion+ --packages-select ros_typedb --pytest-args -k test_register_method
```

```Bash
colcon test --event-handlers console_cohesion+ --packages-select ros_typedb --pytest-args -k test_fetch_query
```

And so on.

**TypeDB note:** Tests and the KB node expect a running TypeDB 3 server (default `localhost:1729`). Start TypeDB 3 before running tests. Use the `Dockerfile-TypeDB3` image which has both TypeDB 3 server and python driver pre-installed.

## TypeDB 2 rules: common pitfalls

- **Relation creation vs attribute assignment:** you cannot create a new relation and assign attributes to it in the same `then` clause. Create the relation in one rule, then assign attributes in another rule, or bind the relation in the `when` and only add attributes in the `then`.
- **Variables in `then`:** any variable used in the `then` must be bound in the `when` **outside nested patterns** (e.g., not only inside `or {}` or `not {}` blocks). Otherwise you will get `[TQL33]`.
- **Negation cycles:** if a rule both reads and writes the same predicate under `not {}`, TypeDB rejects it (`[RUW02]`). Stratify by writing to a different relation/attribute layer and then copy results in a positive rule.
- **Minimal schema edits:** prefer subtyping existing relations over adding parallel relations, to keep fetch/query code stable.

## Docker test tips

- **Container running:** `docker exec` fails if the container is not running. Check `docker ps` or start the container first.
- **Non-interactive exec:** for CI/agents, prefer `docker exec tactical_retreat ...` (no `-it`).
- **Timeouts:** schema-heavy `colcon test` runs can exceed default agent timeouts; allow 5–7 minutes.

## Docker

### Primary: Dockerfile-TypeDB3 (with TypeDB 3 Python driver)

This is the recommended image for TypeDB 3 migration work. It includes TypeDB 3 server and the Python driver.

Build:

```Bash
docker build -f Dockerfile-TypeDB3 -t ros_typedb_typedb3 .
```

Start dev container with display and the `ros_typedb` directory mounted:
```Bash
docker run -it --rm --name ros_typedb_typedb3 -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -v /dev/dri:/dev/dri -v /tmp/.X11-unix:/tmp/.X11-unix -v /etc/localtime:/etc/localtime:ro -v $HOME/typedb_ws/src/ros_typedb:/home/ubuntu-user/typedb_ws/src/ros_typedb ros_typedb_typedb3
```

Start dev container **without** display and the `ros_typedb` directory mounted:
```Bash
docker run -it --rm --name ros_typedb_typedb3 -v /etc/localtime:/etc/localtime:ro -v $HOME/typedb_ws/src/ros_typedb:/home/ubuntu-user/typedb_ws/src/ros_typedb ros_typedb_typedb3
```

Start new terminal in the container:
```Bash
docker exec -it ros_typedb_typedb3 bash
```

Start container in the background with TypeDB 3 server running:
```Bash
docker run -d --rm --name ros_typedb_typedb3 -v /etc/localtime:/etc/localtime:ro -v $HOME/typedb_ws/src/ros_typedb:/home/ubuntu-user/typedb_ws/src/ros_typedb ros_typedb_typedb3 typedb server
```

**Note:** replace the path `$HOME/typedb_ws/src/ros_typedb` with the path of the `ros_typedb` repo in your host machine.

## Information on TypeDB 2

the documentation for typedb2 lives here: https://typedb.com/docs/home/2.x/

## Information on TypeDB 3

Read this curated list of typedb 3 contents for agents: https://typedb.com/docs/llms.txt

the documentation for typedb 3 lives here: https://typedb.com/docs/home/

## TypeQL Example Source (text2typeql)

Use the vendored local mirror first for NL->TypeQL tasks:

- Local path: `agent-data/text2typeql/`
- Primary dataset file: `agent-data/text2typeql/dataset/all_queries.csv`
- Domain files: `agent-data/text2typeql/dataset/synthetic-1/<domain>/queries.csv` and `agent-data/text2typeql/dataset/synthetic-2/<domain>/queries.csv`

Rules:

- For natural-language-to-TypeQL generation, consult local `agent-data/text2typeql` before web/docs.
- Prefer examples from matching `domain` and `source` when possible.
- Reuse query patterns, but always adapt variables, labels, and attributes to this repository's schema.
- Cite local file paths used when returning TypeQL suggestions.
- If no close local match exists, fallback to TypeDB docs (`https://typedb.com/docs/llms.txt` then official docs).
- Do not rely on remote GitHub fetch for examples when local mirror exists.

Deterministic retrieval workflow:

```Bash
python3 scripts/find-text2typeql-examples.py "list all companies in texas" --top-k 5 --min-score 2
python3 scripts/find-text2typeql-examples.py "find movies by actor" --domain movies --source synthetic-1 --top-k 3 --min-score 2
python3 scripts/find-text2typeql-examples.py "delete a supplier by id" --regex "delete|match" --top-k 5 --min-score 2
```

The helper outputs normalized JSON records with:

- `domain`
- `english`
- `typeql`
- `source_file`
- `line`
- `record_id`

Submodule bootstrap and refresh:

```Bash
git submodule update --init --recursive
git submodule update --remote agent-data/text2typeql
```

Refresh cadence: monthly or on-demand when TypeQL coverage is insufficient. Commit the updated submodule SHA intentionally.

### Examples of TypeDB 3 projects:

- https://github.com/lolski/typedb-audio-knowledge-graph-ml
- https://github.com/jmsfltchr/semantic-search
- https://github.com/krishnangovindraj/rusty-foil
- https://github.com/alexjpwalker/typedb-donuts
- https://github.com/flyingsilverfin/video-to-typedb
- https://github.com/flyingsilverfin/text2typeql
- https://github.com/sam-butcher/somerandomdndnotething

## TypeDB 2 to TypeDB 3 migration (In Progress)

migration information lives here: https://typedb.com/docs/reference/typedb-2-vs-3/process/

### TypeDB 2 Python Driver API (Legacy Reference)

For reference only; deprecated in favor of C driver:
- Documentation: https://typedb.com/docs/home/2.x/
- This served as the basis for the original implementation but is being replaced by the C driver.

## Agent helpers

- `agent-docs/agent-notes.md` has a quick checklist and common failure checks.
- `scripts/run-tests-docker.sh` runs tests in the `tactical_retreat` container (supports `-k`).
- `scripts/find-text2typeql-examples.py` retrieves local `text2typeql` examples for NL->TypeQL prompting.
