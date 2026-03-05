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

import ast
from datetime import datetime
import logging
import re
from typing import Any
from typing import Literal
from typing import TypedDict

from typedb.common.datetime import Datetime as TypeDBDatetime
from typedb.driver import Credentials
from typedb.driver import DriverOptions
from typedb.driver import TransactionType
from typedb.driver import TypeDB

from ros_typedb.typedb_helpers import convert_py_type_to_query_type
from ros_typedb.typedb_helpers import convert_query_type_to_py_type
from ros_typedb.typedb_helpers import AttributePair
from ros_typedb.typedb_helpers import create_match_query
from ros_typedb.typedb_helpers import create_relationship_query
from ros_typedb.typedb_helpers import RelatedThingsDict


class TypeDBQueryError(Exception):
    """Raised when a TypeDB query fails during execution."""

    def __init__(
            self,
            *,
            session_type: str,
            transaction_type: str,
            query_type: str,
            query: str) -> None:
        """
        Build a contextual error for failed TypeDB query execution.

        :param session_type: Requested API session type (schema/data).
        :param transaction_type: Requested transaction mode (read/write).
        :param query_type: Logical query kind (insert/get/fetch/etc.).
        :param query: Original TypeQL query text.
        :return: None.
        """
        self.session_type = session_type
        self.transaction_type = transaction_type
        self.query_type = query_type
        self.query = query
        query_preview = ' '.join(query.strip().split())
        message = (
            'TypeDB query failed '
            f'(session_type={session_type}, '
            f'transaction_type={transaction_type}, '
            f'query_type={query_type}, '
            f'query="{query_preview}")'
        )
        super().__init__(message)


def _string_to_string_array(string: str) -> list[str]:
    """
    Convert a path parameter string to a list of strings.

    Supports raw path strings (e.g. `/tmp/schema.tql`) and list literals
    (e.g. `['a.tql', 'b.tql']`).

    :param string: string to be converted.
    :return: list of path strings.
    :raises ValueError: If a list literal is malformed or contains non-strings.
    """
    raw = string.strip()
    if raw == '':
        return []
    if raw.startswith('['):
        try:
            parsed = ast.literal_eval(raw)
        except (SyntaxError, ValueError) as err:
            raise ValueError(
                'Invalid list literal for schema/data paths: {}'.format(raw)
            ) from err
        if not isinstance(parsed, list) or not all(
                isinstance(item, str) for item in parsed):
            raise ValueError(
                'Path list must be a list of strings: {}'.format(raw))
        return parsed
    try:
        parsed = ast.literal_eval(raw)
        if isinstance(parsed, str):
            return [parsed]
    except (SyntaxError, ValueError):
        pass
    return [raw]


def _value_type_to_str(value_type: Any) -> str:
    """
    Normalize a TypeDB value type object to a lowercase string label.

    :param value_type: TypeDB value-type object or equivalent representation.
    :return: Normalized value-type label.
    """
    value = str(value_type).lower()
    if 'boolean' in value or value == 'bool':
        return 'boolean'
    if 'long' in value or value.endswith('int') or 'integer' in value:
        return 'long'
    if 'double' in value or 'float' in value:
        return 'double'
    if 'datetime' in value:
        return 'datetime'
    if 'string' in value or value == 'str':
        return 'string'
    return value


def _query_value_to_json(value: Any, value_type: str) -> Any:
    """
    Convert a TypeDB attribute value to a JSON-serializable representation.

    :param value: Raw attribute value from the TypeDB driver.
    :param value_type: Normalized TypeDB value-type label.
    :return: JSON-serializable value.
    """
    if isinstance(value, TypeDBDatetime):
        millis = value.nanos // 1_000_000
        dt = datetime(value.year, value.month, value.day,
                      value.hour, value.minute, value.second,
                      millis * 1000)
        return dt.isoformat(timespec='milliseconds')
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
            schema_path: list[str] | str | None = None,
            data_path: list[str] | str | None = None,
            force_database: bool = False,
            force_data: bool = False,
            username: str = 'admin',
            password: str = 'password') -> None:
        """
        Connect to a TypeDB server and initialise the database.

        :param address: TypeDB server address.
        :param database_name: database name.
        :param schema_path: list of paths to schema files (.tql).
        :param data_path: list of paths to data files (.tql).
        :param force_database: delete and recreate the database if it exists.
        :param force_data: clear all data before loading data_path files.
        :param username: TypeDB username (default: 'admin').
        :param password: TypeDB password (default: 'password').
        :return: None.
        """
        self.logger = logging.getLogger(__name__)
        self.database_name = None
        self.connect_driver(address, username, password)
        self.create_database(database_name, force=force_database)
        if isinstance(schema_path, str):
            schema_path = _string_to_string_array(schema_path)
        if isinstance(schema_path, list):
            for path in schema_path:
                self.load_schema(path)
        if force_data:
            self.delete_all_data()
        if isinstance(data_path, str):
            data_path = _string_to_string_array(data_path)
        if isinstance(data_path, list):
            for path in data_path:
                self.load_data(path)

    def __del__(self):
        """Close the driver on deletion."""
        try:
            self.driver.close()
        except AttributeError:
            pass

    def connect_driver(
            self, address: str,
            username: str = 'admin',
            password: str = 'password') -> None:
        """
        Connect to TypeDB server.

        :param address: TypeDB server address.
        :param username: TypeDB username.
        :param password: TypeDB password.
        :return: None.
        """
        credentials = Credentials(username, password)
        driver_options = DriverOptions(is_tls_enabled=False)
        self.driver = TypeDB.driver(address, credentials, driver_options)

    def _transaction(self, transaction_type: TransactionType):
        """
        Open a transaction on the current database.

        :param transaction_type: TransactionType enum value.
        :return: transaction context manager.
        :raises ValueError: If no active database is configured.
        """
        if self.database_name is None:
            raise ValueError('No database selected.')
        return self.driver.transaction(self.database_name, transaction_type)

    def delete_database(self, database_name: str | None = None) -> None:
        """
        Delete database.

        :param database_name: database name (defaults to current database).
        :return: None.
        """
        if database_name is None:
            database_name = self.database_name
        if database_name is None:
            self.logger.warning('delete_database called but no database name set.')
            return
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
        if database_name is None:
            return False
        return self.driver.databases.contains(database_name)

    def create_database(
            self, database_name: str, force: bool = False) -> None:
        """
        Create database.

        :param database_name: database name.
        :param force: delete existing database before creating.
        :return: None.
        """
        if force:
            self.delete_database(database_name)
        self.database_name = database_name
        if self.driver.databases.contains(database_name):
            self.logger.warning(
                'Database %s already exists. Skipping create.', database_name)
            return
        self.driver.databases.create(database_name)

    def _normalize_attribute(self, attribute) -> dict[str, Any]:
        """
        Convert a TypeDB attribute concept to a normalized dictionary.

        :param attribute: TypeDB attribute concept.
        :return: Normalized attribute dictionary.
        """
        value_type = _value_type_to_str(attribute.get_value_type())
        return {
            'type': {
                'label': attribute.get_type().get_label(),
                'root': 'attribute',
                'value_type': value_type,
            },
            'value': _query_value_to_json(attribute.get_value(), value_type),
        }

    def _normalize_thing(self, thing, root: str) -> dict[str, Any]:
        """
        Convert a TypeDB entity/relation concept to a normalized dictionary.

        :param thing: TypeDB entity or relation concept.
        :param root: Root concept label (``entity`` or ``relation``).
        :return: Normalized thing dictionary.
        """
        return {
            'type': {
                'label': thing.get_type().get_label(),
                'root': root,
            }
        }

    def _normalize_get_result(self, result_iterable) -> list[dict[str, Any]]:
        """
        Convert a stream of concept rows to normalized dictionaries.

        :param result_iterable: Iterable of TypeDB concept rows.
        :return: List of normalized query rows.
        """
        result_rows = []
        for concept_row in result_iterable:
            row = {}
            for var_name in concept_row.column_names():
                concept = concept_row.get(var_name)
                if concept is None:
                    continue
                if concept.is_attribute():
                    row[var_name] = self._normalize_attribute(
                        concept.as_attribute())
                elif concept.is_entity():
                    row[var_name] = self._normalize_thing(
                        concept.as_entity(), 'entity')
                elif concept.is_relation():
                    row[var_name] = self._normalize_thing(
                        concept.as_relation(), 'relation')
                else:
                    row[var_name] = concept.try_get_value()
            result_rows.append(row)
        return result_rows

    def _resolve_transaction_type(
            self, session_type: str, transaction_type: str) -> TransactionType:
        """
        Map API session/transaction labels to ``TransactionType``.

        :param session_type: API session type label.
        :param transaction_type: API transaction mode label.
        :return: Matching TypeDB ``TransactionType``.
        :raises ValueError: If labels are unsupported.
        """
        if session_type == 'schema':
            return TransactionType.SCHEMA
        if session_type != 'data':
            raise ValueError('Unsupported session_type {}'.format(session_type))
        if transaction_type == 'read':
            return TransactionType.READ
        if transaction_type == 'write':
            return TransactionType.WRITE
        raise ValueError(
            'Unsupported transaction_type {}'.format(transaction_type))

    def _execute_write_query(
            self,
            transaction,
            query: str) -> Literal[True]:
        """
        Execute a write query and commit the transaction.

        :param transaction: Open TypeDB transaction.
        :param query: TypeQL write query.
        :return: Always ``True`` on success.
        """
        answer = transaction.query(query)
        answer.resolve()
        transaction.commit()
        return True

    def _execute_fetch_query(
            self, transaction, query: str) -> list[dict[str, Any]]:
        """
        Execute a fetch query and return concept documents as dictionaries.

        :param transaction: Open TypeDB transaction.
        :param query: TypeQL fetch query.
        :return: List of concept-document dictionaries.
        """
        answer = transaction.query(query).resolve()
        return list(answer.as_concept_documents())

    def _execute_get_query(
            self, transaction, query: str) -> list[dict[str, Any]]:
        """
        Execute a get/select query and normalize concept rows.

        :param transaction: Open TypeDB transaction.
        :param query: TypeQL get query.
        :return: List of normalized query rows.
        """
        answer = transaction.query(query).resolve()
        return self._normalize_get_result(answer.as_concept_rows())

    def _execute_get_aggregate_query(
            self, transaction, query: str) -> int | float | None:
        """
        Execute an aggregate query and return the first numeric value.

        :param transaction: Open TypeDB transaction.
        :param query: TypeQL aggregate query.
        :return: Numeric value or ``None`` when no rows are returned.
        """
        answer = transaction.query(query).resolve()
        rows = list(answer.as_concept_rows())
        if not rows:
            return None
        row = rows[0]
        for col in row.column_names():
            concept = row.get(col)
            if concept is None:
                continue
            if concept.is_integer():
                return concept.try_get_integer()
            if concept.is_double():
                return concept.try_get_double()
            val = concept.try_get_value()
            if isinstance(val, (int, float)):
                return val
        return None

    def database_query(
            self,
            session_type: str,
            transaction_type: str,
            query_type: Literal[
                'define', 'insert', 'delete', 'fetch', 'get',
                'get_aggregate', 'update'],
            query: str
    ) -> Literal[True] | list[dict[str, Any]] | None | int | float:
        """
        Execute a query against the database.

        :param session_type: 'schema' or 'data' (used to select transaction type).
        :param transaction_type: 'read' or 'write'.
        :param query_type: one of define/insert/delete/fetch/get/get_aggregate/update.
        :param query: TypeQL query string.
        :return: query result.
        :raises ValueError: If a provided query/session/transaction type is unsupported.
        :raises TypeDBQueryError: If query execution fails in the database driver.
        """
        query_handler_map = {
            'define': lambda tx: self._execute_write_query(tx, query),
            'insert': lambda tx: self._execute_write_query(tx, query),
            'update': lambda tx: self._execute_write_query(tx, query),
            'delete': lambda tx: self._execute_write_query(tx, query),
            'fetch': lambda tx: self._execute_fetch_query(tx, query),
            'get': lambda tx: self._execute_get_query(tx, query),
            'get_aggregate': lambda tx: self._execute_get_aggregate_query(
                tx, query),
        }
        query_handler = query_handler_map.get(query_type)
        if query_handler is None:
            raise ValueError('Unsupported query_type {}'.format(query_type))

        tdb_transaction_type = self._resolve_transaction_type(
            session_type, transaction_type)
        try:
            with self._transaction(tdb_transaction_type) as transaction:
                return query_handler(transaction)
        except Exception as err:
            query_preview = ' '.join(query.strip().split())
            self.logger.exception(
                'database_query failed (db=%s, session_type=%s, '
                'transaction_type=%s, query_type=%s, query="%s"): %s',
                self.database_name,
                session_type,
                transaction_type,
                query_type,
                query_preview,
                err)
            raise TypeDBQueryError(
                session_type=session_type,
                transaction_type=transaction_type,
                query_type=query_type,
                query=query) from err

    def write_database_file(
            self,
            query_type: Literal['define', 'insert'],
            file_path: str) -> None:
        """
        Write a .tql file to the database.

        :param query_type: 'define' or 'insert'.
        :param file_path: path to .tql file.
        :return: None.
        """
        with open(file_path, mode='r') as file:
            query = file.read()
        session_type = 'schema' if query_type == 'define' else 'data'
        self.database_query(session_type, 'write', query_type, query)

    def load_schema(self, schema_path: str) -> None:
        """
        Load a .tql schema file into the database.

        :param schema_path: path to .tql file.
        :return: None.
        """
        if not schema_path:
            return
        self.write_database_file('define', schema_path)

    def delete_all_data(self) -> None:
        """
        Delete all data from the database (entities and relations).

        :return: None.
        """
        self.delete_from_database(
            '''
                match $instance isa $instance_type;
                delete $instance;
            '''
        )

    def _split_data_statements(self, content: str) -> list[str]:
        """
        Split a .tql data file into individual statements.

        We split on top-level 'match' lines so each 'match ... insert ...'
        block remains intact. Any prelude before the first match (usually a
        plain 'insert' block) is kept as a single statement.
        Empty statements (whitespace/comments only) are discarded.

        :param content: raw file content.
        :return: list of statement strings, each including its keyword.
        """
        match_line_pattern = re.compile(r'(?m)^\s*match\b')
        match_positions = [m.start() for m in match_line_pattern.finditer(content)]
        parts = []
        if not match_positions:
            parts = [content]
        else:
            first_match = match_positions[0]
            if first_match > 0:
                parts.append(content[:first_match])
            for index, start in enumerate(match_positions):
                if index + 1 < len(match_positions):
                    end = match_positions[index + 1]
                else:
                    end = len(content)
                parts.append(content[start:end])

        statements = []
        for part in parts:
            # Strip full-line comments and skip empty/pure-comment chunks.
            cleaned = re.sub(r'(?m)^\s*#[^\n]*$', '', part).strip()
            if cleaned:
                statements.append(cleaned)
        return statements

    def load_data(self, data_path: str) -> None:
        """
        Load a .tql data file into the database.

        Files may contain multiple statements (insert or match...insert
        blocks).  Each statement is executed as a separate transaction so
        that later statements can match entities created by earlier ones.

        :param data_path: path to .tql file.
        :return: None.
        :raises TypeDBQueryError: If any statement fails during execution.
        """
        if not data_path:
            return
        with open(data_path, mode='r') as fh:
            content = fh.read()
        statements = self._split_data_statements(content)
        for statement in statements:
            self.database_query('data', 'write', 'insert', statement)

    def insert_data_event(self):
        """
        Insert data event hook.

        Override in subclasses to react when insert operations occur.

        :return: None.
        """
        self.logger.warning('Data has been inserted!')

    def delete_data_event(self):
        """
        Delete data event hook.

        Override in subclasses to react when delete operations occur.

        :return: None.
        """
        self.logger.warning('Data has been deleted!')

    def delete_thing(
            self,
            thing: str,
            key: str,
            key_value) -> Literal[True]:
        """
        Delete a thing from the database by its key attribute.

        :param thing: thing type (e.g. 'person').
        :param key: key attribute name (e.g. 'email').
        :param key_value: key attribute value.
        :return: True on success.
        """
        query = 'match $thing isa {}, has {} {}; delete $thing;'.format(
            thing, key, convert_py_type_to_query_type(key_value))
        return self.delete_from_database(query)

    def insert_entity(
            self,
            entity: str,
            attribute_list: list[AttributePair] | None = None) -> Literal[True]:
        """
        Insert an entity into the database with optional attributes.

        :param entity: entity type name.
        :param attribute_list: list of (attr_name, attr_value) tuples.
        :return: True on success.
        """
        if attribute_list is None:
            attribute_list = []
        query = 'insert $entity isa {}'.format(entity)
        for attr_name, attr_value in attribute_list:
            value = convert_py_type_to_query_type(attr_value)
            query += ', has {} {}'.format(attr_name, value)
        query += ';'
        return self.insert_database(query)

    def insert_relationship(
            self,
            relationship: str,
            related_dict: RelatedThingsDict,
            attribute_list: list[AttributePair] | None = None) -> Literal[True]:
        """
        Insert a relationship between existing things.

        :param relationship: relationship type name.
        :param related_dict: dict mapping role names to lists of (type, key, value) tuples.
        :param attribute_list: list of (attr_name, attr_value) tuples for the relationship.
        :return: True on success.
        """
        if attribute_list is None:
            attribute_list = []
        match_query = 'match '
        related_variables: dict[str, list[str]] = {}
        for key, things in related_dict.items():
            _match_query, _prefix_list = create_match_query(things, key)
            match_query += _match_query
            related_variables[key] = _prefix_list
        insert_query = 'insert ' + create_relationship_query(
            relationship, related_variables, attribute_list=attribute_list)
        return self.insert_database(match_query + insert_query)

    def fetch_attribute_from_thing(
            self,
            thing: str,
            key_attr_list: list[AttributePair],
            attr: str) -> list:
        """
        Fetch attribute values from a thing matched by one or more key attributes.

        :param thing: thing type name (e.g. 'person').
        :param key_attr_list: list of (attr_name, attr_value) pairs to match on.
        :param attr: attribute name to fetch.
        :return: list of attribute values (Python-typed).
        """
        query = 'match $thing isa {}'.format(thing)
        for key, value in key_attr_list:
            query += ', has {} {}'.format(key, convert_py_type_to_query_type(value))
        query += ', has {} $attribute; select $attribute;'.format(attr)
        result = self.get_database(query)
        return [
            convert_query_type_to_py_type(value_dict=row.get('attribute'))
            for row in result
        ]

    def fetch_attribute_from_thing_raw(
            self,
            thing: str,
            key_attr_list: list[AttributePair],
            attr: str) -> list:
        """
        Fetch raw normalised attribute dicts from a thing matched by key attributes.

        :param thing: thing type name.
        :param key_attr_list: list of (attr_name, attr_value) pairs to match on.
        :param attr: attribute name to fetch.
        :return: list of dicts with key 'attribute' containing normalised attribute dict.
        """
        query = 'match $thing isa {}'.format(thing)
        for key, value in key_attr_list:
            query += ', has {} {}'.format(key, convert_py_type_to_query_type(value))
        query += ', has {} $attribute; select $attribute;'.format(attr)
        return self.get_database(query)

    def delete_attribute_from_thing(
            self,
            thing: str,
            key: str,
            key_value,
            attr: str) -> Literal[True]:
        """
        Delete an attribute from a thing matched by a key attribute.

        :param thing: thing type name.
        :param key: key attribute name.
        :param key_value: key attribute value.
        :param attr: attribute name to delete.
        :return: True on success.
        """
        query = (
            'match $thing isa {}, has {} {}, has {} $attribute;'
            ' delete $attribute of $thing;'
        ).format(
            thing, key, convert_py_type_to_query_type(key_value), attr)
        return self.delete_from_database(query)

    def insert_attribute_in_thing(
            self,
            thing: str,
            key: str,
            key_value,
            attr: str,
            attr_value) -> Literal[True]:
        """
        Insert an attribute into a thing matched by a key attribute.

        :param thing: thing type name.
        :param key: key attribute name.
        :param key_value: key attribute value.
        :param attr: attribute name to insert.
        :param attr_value: attribute value to insert.
        :return: True on success.
        """
        query = 'match $thing isa {}, has {} {}; insert $thing has {} {};'.format(
            thing,
            key,
            convert_py_type_to_query_type(key_value),
            attr,
            convert_py_type_to_query_type(attr_value))
        return self.insert_database(query)

    def delete_attributes_from_thing(
            self,
            thing: str,
            key: str,
            key_value,
            attr_list: list[str]) -> None:
        """
        Delete multiple attributes from a thing matched by a key attribute.

        :param thing: thing type name.
        :param key: key attribute name.
        :param key_value: key attribute value.
        :param attr_list: list of attribute names to delete.
        :return: None.
        """
        for attr in attr_list:
            self.delete_attribute_from_thing(thing, key, key_value, attr)

    def insert_attributes_in_thing(
            self,
            thing: str,
            key: str,
            key_value,
            attribute_list: list[AttributePair]) -> None:
        """
        Insert multiple attributes into a thing matched by a key attribute.

        :param thing: thing type name.
        :param key: key attribute name.
        :param key_value: key attribute value.
        :param attribute_list: list of (attr_name, attr_value) tuples to insert.
        :return: None.
        """
        for attr, attr_value in attribute_list:
            self.insert_attribute_in_thing(
                thing, key, key_value, attr, attr_value)

    def update_attribute_in_thing(
            self,
            thing: str,
            key: str,
            key_value,
            attr: str,
            attr_value) -> Literal[True]:
        """
        Update an attribute in a thing by deleting the old value and inserting the new one.

        :param thing: thing type name.
        :param key: key attribute name.
        :param key_value: key attribute value.
        :param attr: attribute name to update.
        :param attr_value: new attribute value.
        :return: True on success.
        """
        self.delete_attribute_from_thing(thing, key, key_value, attr)
        return self.insert_attribute_in_thing(
            thing, key, key_value, attr, attr_value)

    def update_attributes_in_thing(
            self,
            thing: str,
            key: str,
            key_value,
            attribute_list: list[AttributePair]) -> None:
        """
        Update multiple attributes in a thing.

        :param thing: thing type name.
        :param key: key attribute name.
        :param key_value: key attribute value.
        :param attribute_list: list of (attr_name, new_attr_value) tuples.
        :return: None.
        """
        for attr, attr_value in attribute_list:
            self.update_attribute_in_thing(
                thing, key, key_value, attr, attr_value)

    def insert_database(self, query: str) -> Literal[True]:
        """
        Perform insert query.

        :param query: TypeQL insert query.
        :return: True on success.
        """
        return self.database_query('data', 'write', 'insert', query)

    def update_database(self, query: str) -> Literal[True]:
        """
        Perform update query.

        :param query: TypeQL update query.
        :return: True on success.
        """
        return self.database_query('data', 'write', 'update', query)

    def delete_from_database(self, query: str) -> Literal[True]:
        """
        Perform delete query.

        :param query: TypeQL delete query.
        :return: True on success.
        """
        return self.database_query('data', 'write', 'delete', query)

    def fetch_database(self, query: str) -> list[dict[str, MatchResultDict]]:
        """
        Perform fetch query.

        :param query: TypeQL fetch query.
        :return: list of result dicts.
        """
        return self.database_query('data', 'read', 'fetch', query)

    def get_database(self, query: str) -> list[dict[str, MatchResultDict]]:
        """
        Perform get query.

        :param query: TypeQL get query.
        :return: list of result dicts.
        """
        return self.database_query('data', 'read', 'get', query)

    def get_aggregate_database(self, query: str) -> int | float | None:
        """
        Perform get aggregate query.

        :param query: TypeQL get aggregate query.
        :return: numeric result or None when no aggregate row is returned.
        """
        return self.database_query('data', 'read', 'get_aggregate', query)
