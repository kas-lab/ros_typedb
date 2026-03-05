# TypeDB 3 Refactor Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Migrate `ros_typedb` to the TypeDB 3 Python driver and refactor the architecture so user-defined query classes use composition (not inheritance) over `TypeDBInterface`.

**Architecture:** `TypeDBInterface` uses the TypeDB 3 Python driver directly (no abstraction layer). Helpers move to `typedb_helpers.py` and `ros_typedb_helpers.py` as standalone functions. User query classes inject `TypeDBInterface`; ROS nodes inherit only `ROSTypeDBInterface` for lifecycle boilerplate.

**Tech Stack:** Python 3.10+, ROS 2 Humble, TypeDB 3 Python driver (`typedb-driver` pip package), pytest, colcon.

---

## Baseline: Clean the branch

Before any task below, restore the modified files to origin/main:

```bash
git checkout origin/main -- \
  ros_typedb/ros_typedb/ros_typedb_interface.py \
  ros_typedb/ros_typedb/typedb_interface.py \
  ros_typedb/test/test_ros_typedb_interface.py \
  ros_typedb/test/test_typedb_interface.py \
  ros_typedb/setup.py
```

Confirm no tracked-file modifications remain:
```bash
git diff --name-only
# expected: empty (untracked new files like CLAUDE.md, docs/, agents-doc/ are fine)
```

---

### Task 1: Rewrite `typedb_interface.py` for TypeDB 3

**Files:**
- Modify: `ros_typedb/ros_typedb/typedb_interface.py`
- Modify: `ros_typedb/test/test_typedb_interface.py`

Replace the TypeDB 2 Python driver (`from typedb.driver import TypeDB, SessionType, ...`) with the TypeDB 3 Python driver API. Remove all helper methods (`insert_entity`, `insert_relationship`, etc.) — they move to `typedb_helpers.py` in Task 2. Remove `register_method`.

**TypeDB 3 driver API differences from TypeDB 2:**
- Connect: `TypeDB.core_driver(address=address)` (was `TypeDB.core_client`)
- Databases: `driver.databases.contains(name)`, `driver.databases.create(name)`, `driver.databases.get(name).delete()`
- Session: `driver.session(name, SessionType.DATA)` (same)
- Transaction: `session.transaction(TransactionType.WRITE)` (same)
- Query methods: `transaction.query.fetch(q)`, `transaction.query.get(q)`, `transaction.query.insert(q)`, etc. (no parentheses on `.query`)
- Fetch results: returns list of plain Python dicts directly
- Get results: returns stream of ConceptMap objects (need normalisation — see Step 3)
- Get aggregate: returns a value object, call `.as_long()` / `.as_float()` or check `isinstance`

**Step 1: Trim `test_typedb_interface.py` — remove helper-method tests**

Remove these test functions (they move to `test_typedb_helpers.py` in Task 2):
- `test_insert_entity`
- `test_delete_thing`
- `test_insert_attribute_in_thing`
- `test_fetch_attribute_from_thing`
- `test_delete_attribute_from_thing`
- `test_update_attribute_in_thing`
- `test_insert_relationship`
- `test_dict_to_query`
- `test_insert_attributes`
- `test_delete_attributes`
- `test_update_attributes`
- `test_register_method`

Keep only:
- `test_create_and_delete_database`
- `test_get_query`
- `test_fetch_query`

**Step 2: Run kept tests to confirm they still pass with TypeDB 2 before touching the implementation**
```bash
colcon test --event-handlers console_cohesion+ --packages-select ros_typedb \
  --pytest-args -k test_typedb_interface
# Expected: PASS (still TypeDB 2 code at this point)
```

**Step 3: Rewrite `typedb_interface.py`**

Replace the entire file. Key points:
- Import TypeDB 3 driver: `from typedb.driver import SessionType, TransactionType, TypeDB, TypeDBOptions`
- `connect_driver` creates `self.driver = TypeDB.core_driver(address=address)`
- `database_query` uses sessions and transactions directly (no backend dispatch)
- `_normalize_get_result` converts ConceptMap stream to list of dicts
- `_normalize_attribute` converts attribute concept to `{'type': {...}, 'value': ...}` dict
- Remove all helper methods
- Remove `register_method`
- Remove `create_session` shim

```python
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
"""typedb_interface - Python interface to interact with TypeDB 3."""

from datetime import datetime
import logging
from typing import Any
from typing import Literal
from typing import Optional
from typing import TypedDict

from typedb.driver import SessionType
from typedb.driver import TransactionType
from typedb.driver import TypeDB
from typedb.driver import TypeDBOptions


def convert_query_type_to_py_type(
        value_dict: Optional[dict] = None,
        value: Optional[str] = None,
        value_type: Optional[str] = None) -> datetime | int | str | float:
    """
    Convert typedb 'value_type' to python type.

    :param value: Data to be converted.
    :param value_type: Data type string (e.g. 'long', 'string').
    :param value_dict: Typedb value dict, overrides value and value_type.
    :return: Converted data.
    """
    if value_dict is not None:
        value_type = value_dict.get('type').get('value_type')
        value = value_dict.get('value')
    if value_type == 'datetime':
        return datetime.fromisoformat(value)
    elif value_type == 'long':
        return int(value)
    elif value_type == 'string':
        return str(value)
    elif value_type == 'double':
        return float(value)
    elif value_type == 'long_array':
        return list(map(int, value))
    elif value_type == 'double_array':
        return list(map(float, value))
    elif value_type == 'string_array':
        return list(map(str, value))
    return value


def convert_py_type_to_query_type(data: datetime | str | bool) -> str:
    """
    Convert a Python value to a properly formatted TypeQL string.

    :param data: Data to be converted.
    :return: Converted data as string.
    """
    if isinstance(data, str):
        if len(data) > 0 and data[0] != '$':
            return "'{}'".format(data)
    elif isinstance(data, datetime):
        return data.isoformat(timespec='milliseconds')
    elif isinstance(data, bool):
        return str(data).lower()
    return data


def string_to_string_array(string: str) -> list[str]:
    """
    Convert a comma-separated string to a list of strings.

    :param string: string to be converted
    :return: list of strings
    """
    return [s.strip(" '") for s in string.strip('[]').split(',')]


def recursively_sort_dict(obj):
    """
    Recursively sort dict keys in a dict or list.

    :param obj: dictionary or list
    :return: new sorted dictionary (does not mutate original).
    """
    if isinstance(obj, dict):
        return {k: recursively_sort_dict(obj[k]) for k in sorted(obj)}
    elif isinstance(obj, list):
        return [recursively_sort_dict(item) for item in obj]
    else:
        return obj


def _value_type_to_str(value_type: Any) -> str:
    """Normalise a TypeDB value type object to a lowercase string."""
    value = str(value_type).lower()
    if 'boolean' in value or value == 'bool':
        return 'boolean'
    if 'long' in value or value.endswith('int'):
        return 'long'
    if 'double' in value or 'float' in value:
        return 'double'
    if 'datetime' in value:
        return 'datetime'
    if 'string' in value or value == 'str':
        return 'string'
    return value


def _query_value_to_json(value: Any, value_type: str) -> Any:
    """Convert a TypeDB attribute value to a JSON-serialisable form."""
    if isinstance(value, datetime):
        return value.isoformat(timespec='milliseconds')
    if value_type == 'datetime' and isinstance(value, str):
        return value
    return value


class MatchResultDict(TypedDict):
    """TypedDict for match result."""

    type_: str
    value_type: str
    value: str


class TypeDBInterface:
    """Class used to interact with TypeDB 3 databases."""

    def __init__(
            self,
            address: str,
            database_name: str,
            schema_path: Optional[list[str] | str] = None,
            data_path: Optional[list[str] | str] = None,
            force_database: Optional[bool] = False,
            force_data: Optional[bool] = False,
            infer: Optional[bool] = False) -> None:
        """
        Connect to a TypeDB server and initialise the database.

        :param address: TypeDB server address.
        :param database_name: database name.
        :param schema_path: list of paths to schema files (.tql).
        :param data_path: list of paths to data files (.tql).
        :param force_database: delete and recreate the database if it exists.
        :param force_data: clear all data before loading data_path files.
        :param infer: enable the inference engine.
        """
        self.logger = logging.getLogger()
        self._infer = infer
        self.connect_driver(address)
        self.create_database(database_name, force=force_database)
        if isinstance(schema_path, str):
            schema_path = string_to_string_array(schema_path)
        if isinstance(schema_path, list):
            for path in schema_path:
                self.load_schema(path)
        if force_data:
            self.delete_all_data()
        if isinstance(data_path, str):
            data_path = string_to_string_array(data_path)
        if isinstance(data_path, list):
            for path in data_path:
                self.load_data(path)

    def __del__(self):
        try:
            self.driver.close()
        except AttributeError:
            pass

    def connect_driver(self, address: str) -> None:
        """
        Connect to TypeDB server.

        :param address: TypeDB server address.
        """
        self.driver = TypeDB.core_driver(address=address)

    def _build_options(self) -> TypeDBOptions:
        options = TypeDBOptions()
        if hasattr(options, 'infer'):
            options.infer = self._infer
        if hasattr(options, 'parallel'):
            options.parallel = True
        return options

    def _session(self, database_name: str, session_type):
        options = self._build_options()
        try:
            return self.driver.session(database_name, session_type, options)
        except TypeError:
            return self.driver.session(database_name, session_type)

    def _transaction(self, session, transaction_type):
        options = self._build_options()
        try:
            return session.transaction(transaction_type, options)
        except TypeError:
            return session.transaction(transaction_type)

    def delete_database(self, database_name: str = None) -> None:
        """
        Delete database.

        :param database_name: database name (defaults to current database).
        """
        if database_name is None:
            database_name = self.database_name
        if self.driver.databases.contains(database_name):
            self.driver.databases.get(database_name).delete()

    def database_exists(self, database_name: str | None = None) -> bool:
        """
        Check if database exists.

        :param database_name: database name.
        :return: True if database exists.
        """
        if database_name is None:
            database_name = self.database_name
        return self.driver.databases.contains(database_name)

    def create_database(
            self, database_name: str, force: Optional[bool] = False) -> None:
        """
        Create database.

        :param database_name: database name.
        :param force: delete existing database before creating.
        """
        if force:
            self.delete_database(database_name)
        self.database_name = database_name
        if self.driver.databases.contains(database_name):
            print('Database', database_name, 'already exists. Skipping create.')
            return
        self.driver.databases.create(database_name)

    def _normalize_attribute(self, attribute) -> dict[str, Any]:
        """Convert a TypeDB 3 attribute concept to a normalised dict."""
        attr_type = attribute.get_type()
        value_type = _value_type_to_str(attr_type.get_value_type())
        return {
            'type': {
                'label': attr_type.get_label().name,
                'root': 'attribute',
                'value_type': value_type,
            },
            'value': _query_value_to_json(attribute.get_value(), value_type),
        }

    def _normalize_thing(self, thing, root: str) -> dict[str, Any]:
        """Convert a TypeDB 3 entity/relation concept to a normalised dict."""
        return {
            'type': {
                'label': thing.get_type().get_label().name,
                'root': root,
            }
        }

    def _normalize_get_result(self, result_iterable) -> list[dict[str, Any]]:
        """Convert a stream of ConceptMaps to a list of normalised dicts."""
        result_rows = []
        for concept_map in result_iterable:
            row = {}
            for variable in concept_map.variables():
                var_name = str(variable).lstrip('$')
                concept = concept_map.get(variable)
                if concept is None:
                    continue
                if concept.is_attribute():
                    row[var_name] = self._normalize_attribute(concept.as_attribute())
                elif concept.is_entity():
                    row[var_name] = self._normalize_thing(concept.as_entity(), 'entity')
                elif concept.is_relation():
                    row[var_name] = self._normalize_thing(concept.as_relation(), 'relation')
            result_rows.append(row)
        return result_rows

    def database_query(
            self,
            session_type: str,
            transaction_type: str,
            query_type: Literal[
                'define', 'insert', 'delete', 'fetch', 'get', 'get_aggregate', 'update'],
            query: str,
            options: Optional[dict[str, Any]] = None
    ) -> Literal[True] | list[dict[str, Any]] | None | int | float:
        """
        Execute a query against the database.

        :param session_type: 'schema' or 'data'.
        :param transaction_type: 'read' or 'write'.
        :param query_type: one of define/insert/delete/fetch/get/get_aggregate/update.
        :param query: TypeQL query string.
        :param options: unused, kept for API compatibility.
        :return: query result.
        """
        del options
        tdb_session_type = (
            SessionType.SCHEMA if session_type == 'schema' else SessionType.DATA)
        tdb_transaction_type = (
            TransactionType.READ if transaction_type == 'read' else TransactionType.WRITE)

        with self._session(self.database_name, tdb_session_type) as session:
            with self._transaction(session, tdb_transaction_type) as transaction:
                if query_type == 'define':
                    transaction.query.define(query)
                    transaction.commit()
                    return True
                elif query_type == 'insert':
                    transaction.query.insert(query)
                    transaction.commit()
                    return True
                elif query_type == 'update':
                    transaction.query.update(query)
                    transaction.commit()
                    return True
                elif query_type == 'delete':
                    transaction.query.delete(query)
                    transaction.commit()
                    return True
                elif query_type == 'fetch':
                    return list(transaction.query.fetch(query))
                elif query_type == 'get':
                    return self._normalize_get_result(transaction.query.get(query))
                elif query_type == 'get_aggregate':
                    answer = transaction.query.get_aggregate(query)
                    if hasattr(answer, 'resolve'):
                        answer = answer.resolve()
                    if hasattr(answer, 'is_long') and answer.is_long():
                        return answer.as_long()
                    if hasattr(answer, 'is_float') and answer.is_float():
                        return answer.as_float()
                    if isinstance(answer, (int, float)):
                        return answer
                    return None
                else:
                    raise ValueError('Unsupported query_type {}'.format(query_type))

    def write_database_file(
            self,
            query_type: Literal['define', 'insert'],
            file_path: str) -> None:
        """
        Write a .tql file to the database.

        :param query_type: 'define' or 'insert'.
        :param file_path: path to .tql file.
        """
        with open(file_path, mode='r') as file:
            query = file.read()
        if query_type == 'define':
            self.database_query('schema', 'write', query_type, query)
            return
        self.database_query('data', 'write', query_type, query)

    def load_schema(self, schema_path: str) -> None:
        """
        Load a .tql schema file into the database.

        :param schema_path: path to .tql file.
        """
        if schema_path is not None and schema_path != '':
            self.write_database_file('define', schema_path)

    def delete_all_data(self) -> None:
        """Delete all data from the database."""
        self.delete_from_database(
            'match $e isa entity; delete $e isa entity;')
        self.delete_from_database(
            'match $r isa relation; delete $r isa relation;')
        self.delete_from_database(
            'match $a isa attribute; delete $a isa attribute;')

    def load_data(self, data_path: str) -> None:
        """
        Load a .tql data file into the database.

        :param data_path: path to .tql file.
        """
        if data_path is not None and data_path != '':
            try:
                self.write_database_file('insert', data_path)
            except Exception as err:
                print('Error in load_data:', err)

    def insert_data_event(self):
        """Insert data event hook (override to react to inserts)."""
        print('Data has been inserted!')

    def delete_data_event(self):
        """Delete data event hook (override to react to deletes)."""
        print('Data has been deleted!')

    def insert_database(self, query: str) -> bool | None:
        """
        Perform insert query.

        :param query: TypeQL insert query.
        :return: True on success, None on failure.
        """
        try:
            return self.database_query('data', 'write', 'insert', query)
        except Exception as err:
            print('Error with insert query:', err)
            return None

    def update_database(self, query: str) -> bool | None:
        """
        Perform update query.

        :param query: TypeQL update query.
        :return: True on success, None on failure.
        """
        try:
            return self.database_query('data', 'write', 'update', query)
        except Exception as err:
            print('Error with update query:', err)
            return None

    def delete_from_database(self, query: str) -> Literal[True] | None:
        """
        Perform delete query.

        :param query: TypeQL delete query.
        :return: True on success, None on failure.
        """
        try:
            return self.database_query('data', 'write', 'delete', query)
        except Exception as err:
            print('Error with delete query:', err)
            return None

    def fetch_database(self, query: str) -> list[dict[str, MatchResultDict]]:
        """
        Perform fetch query.

        :param query: TypeQL fetch query.
        :return: list of result dicts; empty list on failure.
        """
        try:
            return self.database_query('data', 'read', 'fetch', query)
        except Exception as err:
            print('Error with fetch query:', err)
            return []

    def get_database(self, query: str) -> list[dict[str, MatchResultDict]] | None:
        """
        Perform get query.

        :param query: TypeQL get query.
        :return: list of result dicts; None on failure.
        """
        try:
            return self.database_query('data', 'read', 'get', query)
        except Exception as err:
            print('Error with get query:', err)
            return None

    def get_aggregate_database(self, query: str) -> int | float | None:
        """
        Perform get aggregate query.

        :param query: TypeQL get aggregate query.
        :return: numeric result; None on failure.
        """
        try:
            return self.database_query('data', 'read', 'get_aggregate', query)
        except Exception as err:
            print('Error with get_aggregate query:', err)
            return None
```

**Step 4: Run tests**
```bash
colcon test --event-handlers console_cohesion+ --packages-select ros_typedb \
  --pytest-args -k test_typedb_interface
# Expected: PASS for test_create_and_delete_database, test_get_query, test_fetch_query
```

**Step 5: Run style checks**
```bash
colcon test --event-handlers console_cohesion+ --packages-select ros_typedb \
  --pytest-args -k "test_flake8 or test_pep257"
```

**Step 6: Commit**
```bash
git add ros_typedb/ros_typedb/typedb_interface.py \
        ros_typedb/test/test_typedb_interface.py
git commit -m "feat: migrate TypeDBInterface to TypeDB 3 Python driver, remove helpers"
```

---

### Task 2: Create `typedb_helpers.py`

**Files:**
- Create: `ros_typedb/ros_typedb/typedb_helpers.py`
- Create: `ros_typedb/test/test_typedb_helpers.py`

Helpers are standalone functions. Every function that was a method on `TypeDBInterface` becomes a module-level function with `db: TypeDBInterface` as its first argument.

**Step 1: Write the failing tests**

```python
# ros_typedb/test/test_typedb_helpers.py
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

from datetime import datetime
import pytest

from ros_typedb.typedb_interface import TypeDBInterface
from ros_typedb.typedb_helpers import (
    delete_attribute_from_thing,
    delete_thing,
    fetch_attribute_from_thing,
    insert_attribute_in_thing,
    insert_entity,
    insert_relationship,
    update_attribute_in_thing,
)


@pytest.fixture
def db():
    """Provide a TypeDBInterface with test schema and data loaded."""
    interface = TypeDBInterface(
        'localhost:1729',
        'test_helpers_db',
        force_database=True,
        schema_path=['test/typedb_test_data/schema.tql'],
        data_path=['test/typedb_test_data/data.tql'],
        force_data=True,
    )
    yield interface
    interface.delete_database()


def test_insert_entity(db):
    insert_entity(db, 'person', [('email', 'helper@test.test'), ('nickname', 'h')])
    result = db.get_aggregate_database(
        'match $e isa person, has email "helper@test.test"; get $e; count;')
    assert result > 0


def test_delete_thing(db):
    insert_entity(db, 'person', [('email', 'del@test.test')])
    delete_thing(db, 'person', 'email', 'del@test.test')
    result = db.fetch_database('match $e isa person, has email "del@test.test"; fetch $e;')
    assert len(result) == 0


@pytest.mark.parametrize('attr, attr_value', [
    ('nickname', 't'),
    ('alive', True),
    ('age', 33),
    ('height', 3.237),
    ('birth-date', datetime.now()),
])
def test_insert_attribute_in_thing(db, attr, attr_value):
    insert_entity(db, 'person', [('email', 'attr@test.test')])
    insert_attribute_in_thing(db, 'person', 'email', 'attr@test.test', attr, attr_value)
    result = fetch_attribute_from_thing(db, 'person', [('email', 'attr@test.test')], attr)
    assert result


@pytest.mark.parametrize('attr, attr_value', [
    ('nickname', 't'),
    ('alive', True),
    ('age', 33),
    ('height', 3.237),
    ('birth-date', datetime.fromisoformat(datetime.now().isoformat(timespec='milliseconds'))),
])
def test_fetch_attribute_from_thing(db, attr, attr_value):
    insert_entity(db, 'person', [('email', 'fetch@test.test'), ('gender', 'male')])
    insert_attribute_in_thing(db, 'person', 'email', 'fetch@test.test', attr, attr_value)
    result = fetch_attribute_from_thing(
        db, 'person', [('email', 'fetch@test.test'), ('gender', 'male')], attr)
    assert result[0] == attr_value


@pytest.mark.parametrize('attr, attr_value', [
    ('nickname', 't'),
    ('alive', True),
    ('age', 33),
    ('height', 3.237),
    ('birth-date', datetime.fromisoformat(datetime.now().isoformat(timespec='milliseconds'))),
])
def test_delete_attribute_from_thing(db, attr, attr_value):
    insert_entity(db, 'person', [('email', 'delattr@test.test')])
    insert_attribute_in_thing(db, 'person', 'email', 'delattr@test.test', attr, attr_value)
    delete_attribute_from_thing(db, 'person', 'email', 'delattr@test.test', attr)
    result = fetch_attribute_from_thing(db, 'person', [('email', 'delattr@test.test')], attr)
    assert len(result) == 0


@pytest.mark.parametrize('attr, attr_value, new_v', [
    ('nickname', 't', 'new_t'),
    ('alive', True, False),
    ('age', 33, 56),
    ('height', 3.237, 1.66),
    ('birth-date',
     datetime.fromisoformat(datetime.now().isoformat(timespec='milliseconds')),
     datetime.fromisoformat(datetime.now().isoformat(timespec='milliseconds'))),
])
def test_update_attribute_in_thing(db, attr, attr_value, new_v):
    insert_entity(db, 'person', [('email', 'upd@test.test')])
    insert_attribute_in_thing(db, 'person', 'email', 'upd@test.test', attr, attr_value)
    update_attribute_in_thing(db, 'person', 'email', 'upd@test.test', attr, new_v)
    result = fetch_attribute_from_thing(db, 'person', [('email', 'upd@test.test')], attr)
    assert result[0] == new_v


def test_insert_relationship(db):
    insert_entity(db, 'person', [('email', 'e1@test.test')])
    insert_entity(db, 'person', [('email', 'e2@test.test')])
    insert_entity(db, 'person', [('email', 'e3@test.test')])
    insert_entity(db, 'person', [('email', 'e4@test.test')])
    insert_relationship(db, 'employment', {
        'employee': [('person', 'email', 'e1@test.test'),
                     ('person', 'email', 'e2@test.test')],
        'employer': [('person', 'email', 'e3@test.test'),
                     ('person', 'email', 'e4@test.test')],
    }, [('salary', 2333), ('role-name', 'boss')])
    result = db.get_aggregate_database(
        'match $r (employee:$ee, employer:$er) isa employment,'
        ' has salary 2333, has role-name "boss"; get $r; count;')
    assert result > 0
```

**Step 2: Run to confirm failure**
```bash
colcon test --event-handlers console_cohesion+ --packages-select ros_typedb \
  --pytest-args -k test_typedb_helpers
# Expected: ImportError — typedb_helpers does not exist yet
```

**Step 3: Implement `typedb_helpers.py`**

Take the helper methods from the **current main version** of `typedb_interface.py` (i.e., the file before your Task 1 rewrite — retrieve it with `git show origin/main:ros_typedb/ros_typedb/typedb_interface.py`), convert each method to a standalone function by replacing `self` with `db: TypeDBInterface` as first argument.

Functions to create:
```python
def attribute_dict_to_query(db, attribute_dict) -> str
def dict_to_query(db, things_dict, attribute_str='attributes',
                  delete_attribute_str='delete-attributes') -> str
def create_match_query(db, things_list, prefix='t') -> tuple[str, list[str]]
def create_relationship_query(db, relationship, related_dict,
                              attribute_list=[], prefix='r') -> str
def delete_thing(db, thing, key, key_value) -> Literal[True] | None
def insert_entity(db, entity, attribute_list=[]) -> bool | None
def insert_relationship(db, relationship, related_dict, attribute_list=[]) -> bool | None
def fetch_attribute_from_thing_raw(db, thing, key_attr_list, attr) -> list
def fetch_attribute_from_thing(db, thing, key_attr_list, attr) -> list
def delete_attribute_from_thing(db, thing, key, key_value, attr) -> Literal[True] | None
def delete_attributes_from_thing(db, match_dict, attribute_str='delete_attributes')
def insert_attribute_in_thing(db, thing, key, key_value, attr, attr_value) -> bool | None
def insert_attributes_in_thing(db, match_dict, attribute_str='insert_attributes') -> bool | None
def update_attribute_in_thing(db, thing, key, key_value, attr, attr_value) -> bool | None
def update_attributes_in_thing(db, match_dict) -> bool | None
```

File header:
```python
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
"""typedb_helpers - standalone helper functions for TypeDB operations."""

from datetime import datetime
from typing import Literal
from typing import Optional
from typing import Tuple

from ros_typedb.typedb_interface import convert_py_type_to_query_type
from ros_typedb.typedb_interface import convert_query_type_to_py_type
from ros_typedb.typedb_interface import TypeDBInterface
```

**Step 4: Run tests**
```bash
colcon test --event-handlers console_cohesion+ --packages-select ros_typedb \
  --pytest-args -k test_typedb_helpers
# Expected: all PASS
```

**Step 5: Run style checks**
```bash
colcon test --event-handlers console_cohesion+ --packages-select ros_typedb \
  --pytest-args -k "test_flake8 or test_pep257"
```

**Step 6: Commit**
```bash
git add ros_typedb/ros_typedb/typedb_helpers.py ros_typedb/test/test_typedb_helpers.py
git commit -m "feat: add typedb_helpers.py with standalone helper functions"
```

---

### Task 3: Create `ros_typedb_helpers.py`

**Files:**
- Create: `ros_typedb/ros_typedb/ros_typedb_helpers.py`
- Modify: `ros_typedb/ros_typedb/ros_typedb_interface.py`
- Modify: `ros_typedb/test/test_ros_typedb_interface.py`

Move these from `ros_typedb_interface.py` to `ros_typedb_helpers.py`:
- `_PARAM_TYPE_MAP`
- `_TYPEDB_ROOT_TYPE_TO_QUERY_RESULT_TYPE`
- `_TYPEDB_ROOT_TYPE_TO_THING_TYPE`
- `set_query_result_value`
- `convert_attribute_dict_to_ros_msg`
- `fetch_result_to_ros_result_tree`
- `fetch_query_result_to_ros_msg`
- `get_query_result_to_ros_msg`
- `get_aggregate_query_result_to_ros_msg`
- `query_result_to_ros_msg`

**Step 1: Verify existing pure-unit tests pass before touching anything**
```bash
colcon test --event-handlers console_cohesion+ --packages-select ros_typedb \
  --pytest-args -k "test_convert_attribute or test_get_query_result or test_fetch_result"
# Expected: PASS (no ROS or TypeDB server required for these)
```

**Step 2: Create `ros_typedb_helpers.py`**

Copy the constants and functions listed above from `ros_typedb_interface.py` into a new file with the Apache 2026 KAS Lab header and the same imports.

Update imports in `test_ros_typedb_interface.py`:
```python
# Before:
from ros_typedb.ros_typedb_interface import convert_attribute_dict_to_ros_msg
from ros_typedb.ros_typedb_interface import fetch_result_to_ros_result_tree
from ros_typedb.ros_typedb_interface import get_query_result_to_ros_msg

# After:
from ros_typedb.ros_typedb_helpers import convert_attribute_dict_to_ros_msg
from ros_typedb.ros_typedb_helpers import fetch_result_to_ros_result_tree
from ros_typedb.ros_typedb_helpers import get_query_result_to_ros_msg
```

Remove the moved symbols from `ros_typedb_interface.py` and add:
```python
from ros_typedb.ros_typedb_helpers import query_result_to_ros_msg
```

**Step 3: Run unit tests**
```bash
colcon test --event-handlers console_cohesion+ --packages-select ros_typedb \
  --pytest-args -k "test_convert_attribute or test_get_query_result or test_fetch_result"
# Expected: PASS
```

**Step 4: Run style checks**
```bash
colcon test --event-handlers console_cohesion+ --packages-select ros_typedb \
  --pytest-args -k "test_flake8 or test_pep257"
```

**Step 5: Commit**
```bash
git add ros_typedb/ros_typedb/ros_typedb_helpers.py \
        ros_typedb/ros_typedb/ros_typedb_interface.py \
        ros_typedb/test/test_ros_typedb_interface.py
git commit -m "refactor: extract ros_typedb_helpers.py from ros_typedb_interface"
```

---

### Task 4: Update `setup.py` and rebuild

**Files:**
- Modify: `ros_typedb/setup.py`

**Step 1: Update `setup.py`** — add TypeDB Python driver as optional dep:

```python
extras_require={
    'test': ['pytest'],
    'typedb': ['typedb-driver'],
},
```

**Step 2: Rebuild**
```bash
colcon build --symlink-install --packages-select ros_typedb
source ../../install/setup.bash
```

**Step 3: Run full test suite**
```bash
colcon test --event-handlers console_cohesion+ --packages-select ros_typedb
```

**Step 4: Fix any remaining failures, then commit**
```bash
git add ros_typedb/setup.py
git commit -m "chore: update setup.py for TypeDB 3 and new modules"
```

---

### Task 5: Final — integration tests and lint

**Step 1: Run full test suite including launch tests** (requires TypeDB 3 server on localhost:1729):
```bash
colcon test --event-handlers console_cohesion+ --packages-select ros_typedb
```

**Step 2: Run lint**
```bash
colcon test --event-handlers console_cohesion+ --packages-select ros_typedb \
  --pytest-args -k "test_flake8 or test_pep257 or test_copyright"
```

**Step 3: All green — final commit**
```bash
git add -u
git commit -m "test: all tests passing with TypeDB 3 and refactored architecture"
```
