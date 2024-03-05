# Copyright 2023 Gustavo Rezende Silva
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
"""typedb_interface - python interface to interact with typedb."""

import functools

from datetime import datetime

from typedb.driver import ConceptMap
from typedb.driver import SessionType
from typedb.driver import TransactionType
from typedb.driver import TypeDB
from typedb.driver import TypeDBSession
from typedb.driver import TypeDBOptions
from typing import Iterator
from typing import Literal
from typing import Optional
from typing import Tuple
from typing import TypedDict


def string_to_string_array(string: str) -> list[str]:
    """
    Convert string to string array.

    :param string: string to be converted
    :return: converted string
    """
    return [s.strip(' \'') for s in string.strip('[]').split(',')]


class MatchResultDict(TypedDict):
    """TypedDict for match result."""

    type: str  #: attribute name, e.g., name, age, height etc
    value_type: str  #: value type, e.g., boolean, long etc
    value: str  #: value


class ThingPrefixAttrDict(TypedDict):
    """TypedDict for dict with thing prefix and attributes."""

    prefix: str  #: typedb variable prefix. E.g., person1 results in $person1
    #: attributes to match. E.g., {'email': 'test@test.test'} match email
    attributes: dict[str, str | bool | int | float | datetime]
    #: attributes to be inserted. E.g., {'age': '30'} inserts age = 30
    insert_attributes: dict[str, str | bool | int | float | datetime]
    #: attributes to be deleted. E.g., ['age'] deletes the 'age attribute'
    delete_attributes: list[str]
    #: attributes to be updated. E.g., {'age': '35'} changes age to 35
    update_attributes: dict[str, str | bool | int | float | datetime]
    #: related entities, used when inserting relationship.
    #: E.g., { 'employee': 'p1', 'employer': 'p2'} creates the relationship
    #: (employee:$p1, employer: $p2)
    relationship: dict[str, str]


class TypeDBInterface:
    """Class used to interact with typeDB databases."""

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
        Connect to a typeDB server and interacts with it.

        Connects TypeDBInterface to typeDB server, creating a database,
        loading a schema and a data file.


        :param address: TypeDB server address.
        :param database_name: database name.
        :param schema_path: list with paths to schema files (.tql).
        :param data_path: list with paths to data files (.tql).
        :param force_database: if database should override an existing database
        :param force_data: if the database data should be overriden.
        :param infer: if inference engine should be used.
        """
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
        Connect to typedb server.

        :param address: typedb server address.
        """
        self.driver = TypeDB.core_driver(address=address)

    def delete_database(self, database_name: str) -> None:
        """
        Delete database.

        :param database_name: database name.
        """
        if self.driver.databases.contains(database_name):
            self.driver.databases.get(database_name).delete()

    def create_database(
            self, database_name: str, force: Optional[bool] = False) -> None:
        """
        Create database.

        :param database_name: database name.
        :param force: if database should override an existing database
        """
        if force:
            self.delete_database(database_name)

        self.database_name = database_name
        if self.driver.databases.contains(database_name):
            print('The database with the name ', database_name,
                  ' already exists. Ignoring create_database request.')
            return

        self.driver.databases.create(database_name)

    def create_session(
        self,
        database_name: str,
        session_type: SessionType,
        options: Optional[TypeDBOptions] = TypeDBOptions()
    ) -> TypeDBSession:
        """
        Create session with the database.

        :param database_name: database name.
        :param session_type: session type, e.g., schema or data.
        :param options: typedb options.
        """
        return self.driver.session(database_name, session_type, options)

    # Read/write database
    # Generic query method
    def database_query(
            self,
            session_type: SessionType,
            transaction_type: TransactionType,
            query_type: Literal[
                'define', 'insert', 'delete', 'match', 'match_aggregate'],
            query: str,
            options: Optional[TypeDBOptions] = TypeDBOptions()
        ) -> Literal[True] | Iterator[ConceptMap] | \
            list[dict[str, MatchResultDict]] | None | int | float:
        """
        Query database.

        Helper method to query the database, it handles creating a session and
        managaging a transaction. It can perform queries of the type define,
        insert, match, match_aggregate, or delete.

        :param session_type: TypeDB session type.
        :param transaction_type: TypeDB transaction type.
        :param query_type: TypeDB query type.
        :param query: Query to be performed.
        :param options: TypeDB options.
        :return: Query result, type depends on the query_type.
        """
        with self.create_session(self.database_name, session_type) as session:
            options.infer = self._infer
            options.parallel = True
            with session.transaction(transaction_type, options) as transaction:
                transaction_query_function = getattr(
                    transaction.query, query_type)
                query_answer = transaction_query_function(query)
                if transaction_type == TransactionType.WRITE:
                    transaction.commit()
                    if query_type == 'delete' or query_type == 'define':
                        return True  # delete always return None
                    return query_answer
                elif transaction_type == TransactionType.READ:
                    if query_type == 'match':
                        answer_list = []
                        for answer in query_answer:
                            answer_list.append(answer.to_json())
                        return answer_list
                    elif query_type == 'match_aggregate':
                        answer = query_answer
                        if answer.is_nan():
                            return None
                        if answer.is_int():
                            return answer.as_int()
                        if answer.is_float():
                            return answer.as_float()

    def write_database_file(
            self,
            session_type: SessionType,
            query_type: Literal['define', 'insert'],
            file_path: str) -> None:
        """
        Write .tql schema or data file content to database.

        :param session_type: session type, e.g., schema or data.
        :param query_type: query type, e.g., 'define' or 'insert'.
        :param file_path: .tql file path.
        """
        with open(file_path, mode='r') as file:
            query = file.read()

        self.database_query(
            session_type, TransactionType.WRITE, query_type, query)

    def load_schema(self, schema_path: str) -> None:
        """
        Load .tql schema file to database.

        :param schema_path: .tql file path.
        """
        if schema_path is not None and schema_path != '':
            return self.write_database_file(
                SessionType.SCHEMA,
                'define',
                schema_path
            )

    def delete_all_data(self) -> None:
        """Delete all data from database."""
        self.delete_from_database(
            'match $e isa entity; delete $e isa entity;')
        self.delete_from_database(
            'match $r isa relation; delete $r isa relation;')
        self.delete_from_database(
            'match $a isa attribute; delete $a isa attribute;')

    def load_data(self, data_path: str, force: bool = False) -> None:
        """
        Load .tql data file to database.

        :param data_path: .tql file path.
        :param force: if database should be overwritten.
        """
        if data_path is not None and data_path != '':
            try:
                self.write_database_file(
                    SessionType.DATA,
                    'insert',
                    data_path
                )
            except Exception as err:
                print('Error in load_data method. Exception msg: ', err)

    # Events begining
    def insert_data_event(self):
        """Insert data event."""
        print('Data has been inserted!')

    def insert_data_event_(func):
        """Generate insert data event."""
        @functools.wraps(func)
        def insert_data_event_wrapper(*args, **kwargs):
            value = func(*args, **kwargs)
            args[0].insert_data_event()
            return value
        return insert_data_event_wrapper

    def delete_data_event(self):
        """Delete data event."""
        print('Data has been deleted!')

    def delete_data_event_(func):
        """Generate delete data event."""
        @functools.wraps(func)
        def delete_data_event_wrapper(*args, **kwargs):
            value = func(*args, **kwargs)
            args[0].delete_data_event()
            return value
        return delete_data_event_wrapper
    # Events end

    # @insert_data_event_
    def insert_database(self, query: str) -> Iterator[ConceptMap] | None:
        """
        Perform insert query.

        :param query: Query to be performed.
        :return: Query result, if query fails return None.
        """
        result = None
        try:
            result = self.database_query(
                SessionType.DATA, TransactionType.WRITE, 'insert', query)
        except Exception as err:
            print('Error with insert query! Exception retrieved: ', err)
        return result

    # @delete_data_event_
    def delete_from_database(self, query: str) -> Literal[True] | None:
        """
        Perform delete query.

        :param query: Query to be performed.
        :return: Query result, if query fails return None.
        """
        result = None
        try:
            result = self.database_query(
                SessionType.DATA, TransactionType.WRITE, 'delete', query)
        except Exception as err:
            print('Error with delete query! Exception retrieved: ', err)
        return result

    def match_database(
            self, query: str) -> list[dict[str, MatchResultDict]]:
        """
        Perform match query.

        :param query: Query to be performed.
        :return: Query result, if query fails return None.
        """
        result = None
        try:
            options = TypeDBOptions()
            options.infer = self._infer
            result = self.database_query(
                SessionType.DATA,
                TransactionType.READ,
                'match',
                query,
                options)
        except Exception as err:
            print('Error with match query! Exception retrieved: ', err)
            return []
        return result

    def match_aggregate_database(self, query: str) -> int | float | None:
        """
        Perform match aggregate query.

        :param query: Query to be performed.
        :return: Query result.
        """
        result = None
        try:
            options = TypeDBOptions()
            options.infer = self._infer
            result = self.database_query(
                SessionType.DATA,
                TransactionType.READ,
                'match_aggregate',
                query,
                options)
        except Exception as err:
            print(
                'Error with match_aggregate query! Exception retrieved: ', err)
        return result
    # Read/write database end

    def convert_query_type_to_py_type(
            self, data: str) -> datetime | int | str | float:
        """
        Convert typedb 'value_type' to python type.

        :param data: Data to be converted.
        :return: Converted data.
        """
        if data.get('value_type') == 'datetime':
            return datetime.fromisoformat(data.get('value'))
        elif data.get('value_type') == 'long':
            return int(data.get('value'))
        elif data.get('value_type') == 'string':
            return str(data.get('value'))
        elif data.get('value_type') == 'double':
            return float(data.get('value'))
        return data.get('value')

    def convert_py_type_to_query_type(
            self, data: datetime | str | bool) -> str:
        """
        Convert python type to string.

        Convert python type to a properly formatted string to be used with
        a typedb query.

        :param data: Data to be converted.
        :return: Converted data.
        """
        if isinstance(data, str):
            if len(data) > 0 and data[0] != '$':
                return "'{}'".format(data)
        elif isinstance(data, datetime):
            return data.isoformat(timespec='milliseconds')
        elif isinstance(data, bool):
            return str(data).lower()
        return data

    def attribute_dict_to_query(
        self,
        attribute_dict: dict[str, str | int | float | bool | datetime]
    ) -> str:
        """
        Convert python dict with typedb attributes to a query.

        :param attribute_dict: Dictionary with attributes to be converted.
        :return: Converted query.

        :Example:

        The following dict:

        .. code-block:: python

            {
                'email': 'test@test.test',
                'height': 1.8,
                'age': 18,
                'alive': True,
                'birth-date': datetime.datetime(2024, 1, 9, 15, 20, 0, 997315)
            }

        Converts to the string:

        .. code-block::

            has email 'test@test.test',
            has height 1.8,
            has age 18,
            has alive true,
            has birth-date 2024-01-09T15:27:10.385

        """
        _query = ''
        first = True
        for attr, attr_value in attribute_dict.items():
            if not isinstance(attr_value, list):
                attr_value = [attr_value]
            for v in attr_value:
                if first is False:
                    _query += ','
                _query += ' has {0} {1}'.format(
                    attr,
                    self.convert_py_type_to_query_type(v)
                )
            first = False
        return _query

    def dict_to_query(
            self,
            things_dict: dict[str, list[ThingPrefixAttrDict]],
            attribute_str: Optional[str] = 'attributes',
            delete_attribute_str: Optional[str] = 'delete-attributes') -> str:
        """
        Convert python dict to query.

        Convert python dict that describes how to insert a thing, or how to
        update a thing's attributes, or how to delete a thing's attributes.

        :param things_dict: Dictionary describing the thing operation.
        :return: Converted query.

        :Example:

        - Adding thing

        .. code-block:: python

            insert_dict = {
                'person': [
                    {
                        'prefix': 'p1',
                        'attributes': {
                            'email': 'test@email.test',
                            'nickname': 't',
                        }
                    },
                    {
                        'prefix': 'p2',
                        'attributes': {
                            'email': 'test2@email.test',
                            'nickname': 't2',
                            'height': 1.33,
                        }
                    },
                ],
                'employment': [
                    {
                        'prefix': 'e',
                        'attributes': {
                            'salary': 2333,
                            'role-name': ['boss', 'super boss'],
                        },
                        'relationship': {
                            'employee': 'p1',
                            'employer': 'p2'
                        }
                    },
                ]
            }

            query = typedb_interface.dict_to_query(insert_dict)
            insert_result = typedb_interface.insert_database("insert " + query)

        The output of dict_to_query is:

        .. code-block::

            $p1  isa person,  has email 'test@email.test';
            $p2  isa person,  has email 'test2@email.test', has height 1.33;
            $e (employee:$p1,employer:$p2) isa employment,  has salary 2333,
                has role-name 'boss', has role-name 'super boss';

        - Insert attribute to thing:

        .. code-block:: python

            insert_dict = {
                'person': [
                    {
                        'prefix': 'p1',
                        'attributes': {
                            'email': 'test_person@test.test',
                        },
                        'insert_attributes': {
                            'height': 1.80,
                            'alive': True,
                        }
                    },
                ],
            },

            match_query = self.dict_to_query(insert_dict)
            insert_query = self.dict_to_query(insert_dict, 'insert_attributes')

        .. code-block::

            # match_query
            $p1  isa person,  has email 'test_person@test.test';

            # insert_query
            $p1  isa person,  has height 1.8, has alive true;

        - Delete attribute from thing:

        .. code-block:: python

            delete_dict = {
                'person': [
                    {
                        'prefix': 'p1',
                        'attributes': {
                            'email': 'test@test.test',
                        },
                        'delete_attributes': ['height', 'age']
                    },
                ],
            },

            query = self.dict_to_query(
                delete_dict, delete_attribute_str='delete_attributes')

        .. code-block::

            # query
            $p1 isa person, has email 'test@test.test';
            $p1 has height $p1_height, has age $p1_age;
            delete $p1 has $p1_height, has $p1_age;

        - Update attribute from thing:

        .. code-block:: python

            update_dict = {
                'person': [
                    {
                        'prefix': 'p1',
                        'attributes': {
                            'email': 'test@test.test',
                        },
                        'update_attributes': {
                            'height': 1.50,
                            'age': 17,
                        }
                    },
                ],
            },

            typedb_interface.update_attributes_in_thing(update_dict)
        """
        query = ''
        delete_query = 'delete '
        for thing, prefix_attr_list in things_dict.items():
            thing_counter = 0
            _query = ''
            for prefix_attr in prefix_attr_list:
                prefix = 't_{}'.format(thing_counter)
                thing_counter += 1
                if 'prefix' in prefix_attr:
                    prefix = prefix_attr['prefix']
                _query += ' ${0} '.format(prefix)

                if 'relationship' in prefix_attr:
                    related_things = ''
                    for role, variables in prefix_attr['relationship'].items():
                        if not isinstance(variables, list):
                            variables = [variables]
                        for v in variables:
                            aux = '{0}:${1}'.format(role, v)
                            if related_things != '':
                                aux = "," + aux
                            related_things += aux
                    _query += '({})'.format(related_things)

                _query += ' isa {}, '.format(thing)

                if attribute_str in prefix_attr:
                    _query += self.attribute_dict_to_query(
                        prefix_attr[attribute_str])

                if delete_attribute_str in prefix_attr:
                    _delete_query = ''
                    _query += '; $' + prefix
                    _delete_query += '$' + prefix
                    first = True
                    for attr in prefix_attr[delete_attribute_str]:
                        if first is False:
                            _query += ','
                            _delete_query += ','
                        first = False
                        _query += ' has {0} ${1}_{0}'.format(
                            attr, prefix)
                        _delete_query += ' has ${1}_{0}'.format(attr, prefix)
                    _delete_query += ';'
                    delete_query += _delete_query
                _query += ';'
            query += _query

        if delete_query != 'delete ':
            query += delete_query
        return query

    def create_match_query(
            self,
            things_list: list[Tuple[str, str, str]],
            prefix: Optional[str] = 't') -> Tuple[str, list[str]]:
        """
        Create match query from a list of tuples specifying a Thing individual.

        Create match query from list of tuples specifying a Thing individual.
        A tuple has the following form: (THING_NAME, ATTR_NAME, ATTR_VALUE).

        :param things_list: list of tuples specifying a Thing individual.
            E.g., [('person', 'email', 'test@email.test'),
            ('person', 'email', 'test2@email.test')].
        :param prefix: prefix for the variable of each individual in the match
            query. E.g., `employee` results in the typedb variables
            `$employee_0`, `$employee_1` etc.
        :return: Converted query.

        :Example:

        The following list:

        .. code-block:: python

            my_list = [
                ('person', 'email', 'test@email.test'),
                ('person', 'email', 'test2@email.test')
            ]

            query, prefix_list = self.create_match_query(my_list, 'employee')

        Results in:

        .. code-block::

            # query
            $employee_0 isa person, has email 'test@email.test';
            $employee_1 isa person, has email 'test2@email.test';

            # prefix_list
            ['employee_0', 'employee_1']

        """
        match_query = ""
        prefix_list = []
        t_counter = 0
        for thing in things_list:
            match_query += " ${0}_{1} isa {2},".format(
                prefix, t_counter, thing[0])
            match_query += " has {0} {1};".format(
                thing[1], self.convert_py_type_to_query_type(thing[2]))
            prefix_list.append("{0}_{1}".format(prefix, t_counter))
            t_counter += 1
        return match_query, prefix_list

    def create_relationship_query(
            self,
            relationship: str,
            related_dict: dict[str, list[Tuple[str, str, str]]],
            attribute_list: Optional[list[
                Tuple[str, str | int | float | bool | datetime]]] = [],
            prefix: Optional[str] = 'r') -> str:
        """
        Create a query for relationships.

        :param relationship: relationship name.
        :param related_dict: dictionary with related things.
        :param attribute_list: list with the relationships attributes.
        :param prefix: prefix for query variables.

        :Example:

        .. code-block:: python

            related_dict = {
                'employee': [
                    ('person', 'email', 'test@email.test'),
                    ('person', 'email', 'test2@email.test')],
                'employer': [
                    ('person', 'email', 'test3@email.test'),
                    ('person', 'email', 'test4@email.test')
                ]
            }

            attribute_list = [
                ('salary', 2333), ('role-name', 'boss')]

            query = self.create_relationship_query(
                'employment', related_dict, attribute_list, 'employment')

        .. code-block::

            # query
            $employment (employee:$employee_0, employee:$employee_1,
                employer:$employer_0, employer:$employer_1) isa employment,
                has salary 2333 , has role-name 'boss';

        """
        related_things = ""
        for role, variables in related_dict.items():
            for v in variables:
                aux = "{0}:${1}".format(role, v)
                if related_things != "":
                    aux = "," + aux
                related_things += aux
        query = " ${0} ({1}) isa {2}".format(
            prefix, related_things, relationship)
        for attribute in attribute_list:
            if attribute[0] is not None:
                query += ", has {} {} ".format(
                    attribute[0],
                    self.convert_py_type_to_query_type(attribute[1])
                )
        query += ";"
        return query

    def delete_thing(
        self,
        thing: str,
        key: str,
        key_value: str | int | float | bool | datetime
    ) -> Literal[True] | None:
        """
        Delete thing individual in the database.

        :param thing: thing name.
        :param key: attribute name to identify the individual.
        :param key_value: attribute value to identify the individual.
        :return: True.
        """
        query = f"""
            match $thing isa {thing}, has {key} "{key_value}";
            delete $thing isa {thing};
        """
        return self.delete_from_database(query)

    def insert_entity(
        self,
        entity: str,
        attribute_list: Optional[
            list[Tuple[str, str | int | float | bool | datetime]]] = []
    ) -> Iterator[ConceptMap] | None:
        """
        Insert entity individual in the database.

        :param entity: entity name.
        :param attribute_list: list with attribute tuple (name, value).
        :return: query result.
        """
        query = f"""
            insert $entity isa {entity}
        """
        for attribute in attribute_list:
            if attribute[0] is not None:
                value = self.convert_py_type_to_query_type(attribute[1])
                query += f""", has {attribute[0]} {value} """
        query += ";"
        return self.insert_database(query)

    def insert_relationship(
        self,
        relationship: str,
        related_dict: dict[str, list[Tuple[str, str, str]]],
        attribute_list: Optional[list[
            Tuple[str, str | int | float | bool | datetime]]] = []
    ) -> Iterator[ConceptMap] | None:
        """
        Insert relationship individual in the database.

        :param relationship: relationship name.
        :param related_dict: dictionary with related things.
        :param attribute_list: list with the relationships attributes.
        :return: query result

        :Example:

        The following code:

        .. code-block:: python

            related_dict = {
                'employee': [
                    ('person', 'email', 'test@email.test'),
                    ('person', 'email', 'test2@email.test')],
                'employer': [
                    ('person', 'email', 'test3@email.test'),
                    ('person', 'email', 'test4@email.test')
                ]
            }

            attribute_list = [
                ('salary', 2333), ('role-name', 'boss')]

            self.insert_relationship(
                'employment', related_dict, attribute_list)

        Performs the following query:

        .. code-block::

            # query
            match $employee_0 isa person, has email 'test@email.test';
                  $employee_1 isa person, has email 'test2@email.test';
                  $employer_0 isa person, has email 'test3@email.test';
                  $employer_1 isa person, has email 'test4@email.test';
            insert  $employment (employee:$employee_0, employee:$employee_1,
                    employer:$employer_0,employer:$employer_1) isa employment,
                    has salary 2333 , has role-name 'boss';

        """
        match_query = "match "
        insert_query = "insert "
        _related_dict = dict()
        for key, things in related_dict.items():
            _match_query, _prefix_list = self.create_match_query(things, key)
            match_query += _match_query
            _related_dict[key] = _prefix_list

        insert_query += self.create_relationship_query(
            relationship,
            _related_dict,
            attribute_list=attribute_list,
            prefix=relationship
        )
        query = match_query + insert_query
        return self.insert_database(query)

    def get_attribute_from_thing_raw(
            self,
            thing: str,
            key_attr_list: list[
                Tuple[str, str | int | float | bool | datetime]],
            attr: str) -> list[dict[str, MatchResultDict]]:
        """
        Get raw attribute values from a instance of a thing.

        :param thing: thing name
        :param key_attr_list: list with attribute tuple (name, value)
        :param attr: attribute name to be fetched, e.g., 'person-name'
        :return: List of dictionary with the query result.
        """
        query = f"""
            match $thing isa {thing}
        """
        for (key, value) in key_attr_list:
            value = self.convert_py_type_to_query_type(value)
            query += f""", has {key} {value} """
        query += f"""
            , has {attr} $attribute;
            get $attribute;
        """
        return self.match_database(query)

    def get_attribute_from_thing(
            self,
            thing: str,
            key_attr_list: list[
                Tuple[str, str | int | float | bool | datetime]],
            attr: str) -> list[str | int | float | bool | datetime]:
        """
        Get attribute value from a instance of a thing.

        :param thing: thing name
        :param key_attr_list: list with attribute tuple (name, value)
        :param attr: attribute name to be fetched, e.g., 'person-name'
        :return: List with the attribute values of type attr.
        """
        result = self.get_attribute_from_thing_raw(
            thing, key_attr_list, attr)
        return [self.convert_query_type_to_py_type(r.get('attribute'))
                for r in result]

    def delete_attribute_from_thing(
            self,
            thing: str,
            key: str,
            key_value: str | int | float | bool | datetime,
            attr: str) -> Literal[True] | None:
        """
        Delete attribute value from a instance of a thing.

        :param thing: thing name
        :param key: attribute name to identify the instance
        :param key_value: attribute value to identify the instance
        :param attr: attribute name to be deleted
        :return: True.
        """
        key_value = self.convert_py_type_to_query_type(key_value)
        query = f"""
            match $thing isa {thing},
            has {key} {key_value},
            has {attr} $attribute;
            delete $thing has $attribute;
        """
        return self.delete_from_database(query)

    def delete_attributes_from_thing(
        self,
        match_dict: dict[str, list[ThingPrefixAttrDict]],
        attribute_str: Optional[str] = 'delete_attributes'
    ) -> Literal[True] | None:
        """
        Delete attributes from thing individual.

        :param match_dict: Dictionary describing the delete operation.
        :return: Delete query result.

        :Example:

        The following code:

        .. code-block:: python

            delete_dict = {
                'person': [
                    {
                        'prefix': 'p1',
                        'attributes': {
                            'email': 'test@test.test',
                        },
                        'delete_attributes': ['height', 'age']
                    },
                ],
            },

            self.delete_attributes_from_thing(
                delete_dict, delete_attribute_str='delete_attributes')

        Performs the query:

        .. code-block::

            # query
            match $p1 isa person, has email 'test@test.test';
                $p1 has height $p1_height, has age $p1_age;
            delete $p1 has $p1_height, has $p1_age;

        """
        match_query = 'match ' + self.dict_to_query(
            match_dict, delete_attribute_str=attribute_str)
        return self.delete_from_database(match_query)

    def insert_attribute_in_thing(
        self,
        thing: str,
        key: str,
        key_value: str | int | float | bool | datetime,
        attr: str,
        attr_value: str | int | float | bool | datetime
    ) -> Iterator[ConceptMap] | None:
        """
        Insert attribute value in a instance of a thing.

        :param thing: thing name
        :param key: attribute name to identify the instance
        :param key_value: attribute value to identify the instance
        :param attr: attribute name to be inserted
        :param attr_value: attribute value to be inserted
        :return: Insert query result.
        """
        key_value = self.convert_py_type_to_query_type(key_value)
        attr_value = self.convert_py_type_to_query_type(attr_value)
        query = f"""
            match $thing isa {thing},
            has {key} {key_value};
            insert $thing has {attr} {attr_value};
        """
        return self.insert_database(query)

    def insert_attributes_in_thing(
        self,
        match_dict: dict[str, list[ThingPrefixAttrDict]],
        attribute_str: Optional[str] = 'insert_attributes'
    ) -> Iterator[ConceptMap] | None:
        """
        Insert attributes to thing individual.

        :param match_dict: Dictionary describing the insert operation.
        :return: Insert query result.

        :Example:

        The following code:

        .. code-block:: python

            insert_dict = {
                'person': [
                    {
                        'prefix': 'p1',
                        'attributes': {
                            'email': 'test_person@test.test',
                        },
                        'insert_attributes': {
                            'height': 1.80,
                            'alive': True,
                        }
                    },
                ],
            },

            self.insert_attributes_in_thing(insert_dict, 'insert_attributes')

        Performs the following query:

        .. code-block::

            match $p1 isa person, has email 'test_person@test.test';
            insert $p1 isa person, has height 1.8, has alive true;

        """
        match_query = 'match ' + self.dict_to_query(match_dict)
        insert_query = 'insert ' + self.dict_to_query(
            match_dict, attribute_str)
        return self.insert_database(match_query + insert_query)

    def update_attribute_in_thing(
        self,
        thing: str,
        key: str,
        key_value: str | int | float | bool | datetime,
        attr: str,
        attr_value: str | int | float | bool | datetime
    ) -> Iterator[ConceptMap] | None:
        """
        Update attribute value in a instance of a thing.

        :param thing: thing name
        :param key: attribute name to identify the instance
        :param key_value: attribute value to identify the instance
        :param attr: attribute name to be updated
        :param attr_value: attribute value to be inserted
        :return: Insert query result.
        """
        self.delete_attribute_from_thing(
            thing, key, key_value, attr)
        return self.insert_attribute_in_thing(
            thing, key, key_value, attr, attr_value)

    def update_attributes_in_thing(
        self,
        match_dict: dict[str, list[ThingPrefixAttrDict]]
    ) -> Iterator[ConceptMap] | None:
        """
        Update attributes of a thing individual.

        :param match_dict: Dictionary describing the update operation.
        :return: Insert query result.

        :Example:

        The following code:

        .. code-block:: python

            update_dict = {
                'person': [
                    {
                        'prefix': 'p1',
                        'attributes': {
                            'email': 'test@test.test',
                        },
                        'update_attributes': {
                            'height': 1.50,
                            'age': 17,
                        }
                    },
                ],
            },

            typedb_interface.update_attributes_in_thing(update_dict)

        Performs the following two query:

        .. code-block::

            # delete query
            match $p1 isa person, has email 'test@test.test';
                $p1 has height $p1_height, has age $p1_age;
            delete $p1 has $p1_height, has $p1_age;

            # insert query
            match $p1 isa person, has email 'test@test.test';
            insert $p1 isa person, has height 1.50, has age 17;

        """
        self.delete_attributes_from_thing(match_dict, 'update_attributes')
        return self.insert_attributes_in_thing(match_dict, 'update_attributes')
