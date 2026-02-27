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

from typedb.driver import Credentials
from typedb.driver import DriverOptions
from typedb.driver import TransactionType
from typedb.driver import TypeDB


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
        :param infer: enable the inference engine (unused in TypeDB 3).
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
        """Close the driver on deletion."""
        try:
            self.driver.close()
        except AttributeError:
            pass

    def connect_driver(self, address: str) -> None:
        """
        Connect to TypeDB server.

        :param address: TypeDB server address.
        """
        credentials = Credentials('admin', 'password')
        driver_options = DriverOptions(is_tls_enabled=False)
        self.driver = TypeDB.driver(address, credentials, driver_options)

    def _transaction(self, transaction_type: TransactionType):
        """
        Open a transaction on the current database.

        :param transaction_type: TransactionType enum value.
        :return: transaction context manager.
        """
        return self.driver.transaction(self.database_name, transaction_type)

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
        """Convert a TypeDB 3 entity/relation concept to a normalised dict."""
        return {
            'type': {
                'label': thing.get_type().get_label(),
                'root': root,
            }
        }

    def _normalize_get_result(self, result_iterable) -> list[dict[str, Any]]:
        """Convert a stream of ConceptRows to a list of normalised dicts."""
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

    def database_query(
            self,
            session_type: str,
            transaction_type: str,
            query_type: Literal[
                'define', 'insert', 'delete', 'fetch', 'get',
                'get_aggregate', 'update'],
            query: str,
            options: Optional[dict[str, Any]] = None
    ) -> Literal[True] | list[dict[str, Any]] | None | int | float:
        """
        Execute a query against the database.

        :param session_type: 'schema' or 'data' (used to select transaction type).
        :param transaction_type: 'read' or 'write'.
        :param query_type: one of define/insert/delete/fetch/get/get_aggregate/update.
        :param query: TypeQL query string.
        :param options: unused, kept for API compatibility.
        :return: query result.
        """
        del options
        if session_type == 'schema':
            tdb_transaction_type = TransactionType.SCHEMA
        elif transaction_type == 'read':
            tdb_transaction_type = TransactionType.READ
        else:
            tdb_transaction_type = TransactionType.WRITE

        with self._transaction(tdb_transaction_type) as transaction:
            if query_type == 'define':
                transaction.query(query).resolve()
                transaction.commit()
                return True
            elif query_type == 'insert':
                transaction.query(query).resolve()
                transaction.commit()
                return True
            elif query_type == 'update':
                transaction.query(query).resolve()
                transaction.commit()
                return True
            elif query_type == 'delete':
                transaction.query(query).resolve()
                transaction.commit()
                return True
            elif query_type == 'fetch':
                answer = transaction.query(query).resolve()
                return list(answer.as_concept_documents())
            elif query_type == 'get':
                answer = transaction.query(query).resolve()
                return self._normalize_get_result(answer.as_concept_rows())
            elif query_type == 'get_aggregate':
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
                        val = concept.try_get_integer()
                        return val
                    if concept.is_double():
                        val = concept.try_get_double()
                        return val
                    val = concept.try_get_value()
                    if isinstance(val, (int, float)):
                        return val
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
        try:
            with self._transaction(TransactionType.WRITE) as tx:
                answer = tx.query('match $e isa entity; select $e;').resolve()
                types_seen = set()
                for row in answer.as_concept_rows():
                    concept = row.get('e')
                    if concept and concept.is_entity():
                        label = concept.get_label()
                        types_seen.add(label)
            for label in types_seen:
                self.delete_from_database(
                    'match $e isa {}; delete $e;'.format(label))
        except Exception as err:
            print('Error in delete_all_data:', err)

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
