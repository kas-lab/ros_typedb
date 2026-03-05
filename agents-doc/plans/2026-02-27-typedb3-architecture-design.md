# ros_typedb TypeDB 3 Architecture Design

**Date:** 2026-02-27
**Branch:** typedb3_py
**Status:** Approved

## Goal

Migrate `ros_typedb` from TypeDB 2 to TypeDB 3 using the TypeDB 3 Python driver, and redesign the package architecture to use composition instead of inheritance for user-defined query classes.

## Motivation

The current design requires users to inherit from `TypeDBInterface` to add custom queries, and from `ROSTypeDBInterface` to add custom ROS interfaces. This couples query logic to both the DB layer and the ROS layer. The new design decouples them so that query classes are plain Python and can be used without ROS.

## Architecture

### Package Files

| File | Responsibility |
|------|----------------|
| `typedb_interface.py` | `TypeDBInterface`: backend connection and raw query execution (`insert_database`, `fetch_database`, `get_database`, `get_aggregate_database`, `update_database`, `delete_from_database`, `load_schema`, `load_data`, etc.) |
| `typedb_helpers.py` | Standalone helper functions that take a `TypeDBInterface` instance: `insert_entity`, `insert_relationship`, `fetch_attribute_from_thing`, `delete_attribute_from_thing`, `update_attribute_in_thing`, `delete_attributes_from_thing`, `insert_attributes_in_thing`, `update_attributes_in_thing`, `dict_to_query`, `create_match_query`, `create_relationship_query`, `attribute_dict_to_query` |
| `ros_typedb_interface.py` | `ROSTypeDBInterface`: ROS 2 lifecycle node, owns `TypeDBInterface`, handles lifecycle boilerplate and the generic `/query` service |
| `ros_typedb_helpers.py` | ROS conversion helpers: `fetch_query_result_to_ros_msg`, `get_query_result_to_ros_msg`, `get_aggregate_query_result_to_ros_msg`, `query_result_to_ros_msg`, `convert_attribute_dict_to_ros_msg`, `fetch_result_to_ros_result_tree`, `set_query_result_value` |

### User Pattern

Users define query logic in plain Python classes (no inheritance from `TypeDBInterface`) and wire ROS interfaces in a node subclass (inheriting from `ROSTypeDBInterface` only for lifecycle boilerplate).

```python
# my_component_queries.py — pure Python, usable without ROS
from ros_typedb.typedb_interface import TypeDBInterface
from ros_typedb.typedb_helpers import insert_entity, fetch_attribute_from_thing

class ComponentQueries:
    def __init__(self, db: TypeDBInterface):
        self.db = db

    def get_active_components(self):
        return self.db.fetch_database(
            "match $c isa component, has status 'active'; fetch $c;")

    def add_component(self, name: str):
        return insert_entity(self.db, 'component', [('name', name)])


# my_mission_queries.py — pure Python, usable without ROS
class MissionQueries:
    def __init__(self, db: TypeDBInterface):
        self.db = db

    def get_pending_missions(self):
        return self.db.fetch_database(
            "match $m isa mission, has status 'pending'; fetch $m;")


# my_app_node.py — inherits only for ROS lifecycle boilerplate
from ros_typedb.ros_typedb_interface import ROSTypeDBInterface
from ros_typedb.ros_typedb_helpers import fetch_query_result_to_ros_msg

class MyAppNode(ROSTypeDBInterface):
    def on_configure(self, state):
        super().on_configure(state)  # creates self.typedb_interface
        self.component_queries = ComponentQueries(self.typedb_interface)
        self.mission_queries = MissionQueries(self.typedb_interface)
        self.create_service(MyQuery, '~/components', self.components_cb)
        self.create_service(MyQuery, '~/missions', self.missions_cb)

    def components_cb(self, req, res):
        result = self.component_queries.get_active_components()
        return fetch_query_result_to_ros_msg(result)

    def missions_cb(self, req, res):
        result = self.mission_queries.get_pending_missions()
        return fetch_query_result_to_ros_msg(result)
```

### What Changes vs Current Code

- `TypeDBInterface` loses its helper methods → moved to `typedb_helpers.py`
- `ros_typedb_interface.py` loses its conversion helpers → moved to `ros_typedb_helpers.py`
- Users no longer inherit from `TypeDBInterface` — they inject it via constructor
- `register_method` is removed (design-time composition replaces it)
- `ROSTypeDBInterface` remains a base class but is purely lifecycle boilerplate + generic `/query` service
- Multiple query class instances are assigned explicitly in `on_configure` (`self.component_queries`, `self.mission_queries`, etc.)

### What Does Not Change

- The ROS 2 message types in `ros_typedb_msgs` are unchanged
- The generic `/query` service on `ROSTypeDBInterface` is retained
- The backend abstraction (`typedb_backend.py`) is unchanged
- The TypeDB 3 Python driver integration is unchanged

## Testing Strategy

- `test_typedb_interface.py` — tests `TypeDBInterface` raw query methods
- `test_typedb_helpers.py` — tests each helper function using a live TypeDB instance
- `test_ros_typedb_interface.py` — tests `ROSTypeDBInterface` lifecycle and `/query` service
- Existing tests are updated to import helpers from their new modules
