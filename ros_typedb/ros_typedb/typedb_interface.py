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

from typedb.driver import SessionType
from typedb.driver import TransactionType
from typedb.driver import TypeDB
from typedb.driver import TypeDBDriverException
from typedb.driver import TypeDBOptions
from datetime import datetime


class TypeDBInterface:
    """Class to interact with typedb."""

    def __init__(self, address, database_name, schema_path=None,
                 data_path=None, force_database=False, force_data=False):
        """
        Init TypeDBInterface.

        Init TypeDBInterface by connecting to typeDB server, creating a
        database, loading a schema and a data file.


        :param address: TypeDB server address.
        :type address: str
        :param database_name: database name.
        :type database_name: str
        :param schema_path: path to the schema file (.tql).
        :type schema_path: str
        :param data_path: path to the data file (.tql).
        :type data_path: str
        :param force_database: if database should override an existing database
        :type force_database: bool
        :param force_data: if the database data should be overriden.
        :type force_data: bool
        """
        self.connect_driver(address)
        self.create_database(database_name, force=force_database)

        if schema_path is not None and schema_path != '':
            self.load_schema(schema_path)

        if data_path is not None and data_path != '':
            self.load_data(data_path, force=force_data)

    def __del__(self):
        try:
            self.driver.close()
        except AttributeError:
            pass

    def connect_driver(self, address):
        self.driver = TypeDB.core_driver(address=address)

    def create_database(self, database_name, force=False):
        if self.driver.databases.contains(database_name) and force:
            self.driver.databases.get(database_name).delete()

        if not self.driver.databases.contains(database_name):
            self.driver.databases.create(database_name)
            self.database_name = database_name
        else:
            self.database_name = database_name
            print('The database with the name ', database_name,
                  ' already exists. Ignoring create_database request.')

    def create_session(
            self,
            database_name,
            session_type,
            options=TypeDBOptions()):
        return self.driver.session(database_name, session_type, options)

    # Read/write database
    # Generic query method
    def database_query(
            self,
            session_type,
            transaction_type,
            query_type,
            query,
            options=TypeDBOptions()):
        with self.create_session(self.database_name, session_type) as session:
            options.infer = True
            with session.transaction(transaction_type, options) as transaction:
                transaction_query_function = getattr(
                    transaction.query, query_type)
                query_answer = transaction_query_function(query)
                if transaction_type == TransactionType.WRITE:
                    transaction.commit()
                    if query_type == 'delete':
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

    # Load schema or data from file
    def write_database_file(
            self,
            session_type,
            transaction_type,
            query_type,
            file_path):
        with open(file_path, mode='r') as file:
            query = file.read()

        self.database_query(session_type, transaction_type, query_type, query)

    # Load schema from file
    def load_schema(self, schema_path):
        self.write_database_file(
            SessionType.SCHEMA, TransactionType.WRITE, 'define', schema_path)

    # Load data from file
    def load_data(self, data_path, force=False):
        if force:
            self.delete_from_database(
                'match $e isa entity; delete $e isa entity;')
            self.delete_from_database(
                'match $r isa relation; delete $r isa relation;')
            self.delete_from_database(
                'match $a isa attribute; delete $a isa attribute;')
        try:
            self.write_database_file(
                SessionType.DATA, TransactionType.WRITE, 'insert', data_path)
        except Exception as err:
            print('Error in load_data method. Exception msg: ', err)

    # Events begining
    def insert_data_event(self):
        print('Data has been inserted!')

    def insert_data_event_(func):
        @functools.wraps(func)
        def insert_data_event_wrapper(*args, **kwargs):
            value = func(*args, **kwargs)
            args[0].insert_data_event()
            return value
        return insert_data_event_wrapper

    def delete_data_event(self):
        print('Data has been deleted!')

    def delete_data_event_(func):
        @functools.wraps(func)
        def delete_data_event_wrapper(*args, **kwargs):
            value = func(*args, **kwargs)
            args[0].delete_data_event()
            return value
        return delete_data_event_wrapper
    # Events end

    # Insert query
    @insert_data_event_
    def insert_database(self, query):
        result = None
        try:
            result = self.database_query(
                SessionType.DATA, TransactionType.WRITE, 'insert', query)
        except Exception as err:
            print('Error with insert query! Exception retrieved: ', err)
        return result

    # Delete query
    @delete_data_event_
    def delete_from_database(self, query):
        result = None
        try:
            result = self.database_query(
                SessionType.DATA, TransactionType.WRITE, 'delete', query)
        except Exception as err:
            print('Error with delete query! Exception retrieved: ', err)
        return result

    # TODO: decorator?
    # Match query
    def match_database(self, query):
        result = None
        try:
            options = TypeDBOptions()
            options.infer = True
            result = self.database_query(
               SessionType.DATA, TransactionType.READ, 'match', query, options)
        except Exception as err:
            print('Error with match query! Exception retrieved: ', err)
        return result

    def match_aggregate_database(self, query):
        result = None
        try:
            options = TypeDBOptions()
            options.infer = True
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

    def create_match_query(self, things_list, prefix='t'):
        match_query = ""
        prefix_list = []
        t_counter = 0
        for thing in things_list:
            match_query += " ${0}_{1} isa {2},".format(
                prefix, t_counter, thing[0])
            if type(thing[2]) is str:
                match_query += " has {0} '{1}';".format(thing[1], thing[2])
            else:
                match_query += " has {0} {1};".format(thing[1], thing[2])
            prefix_list.append("{0}_{1}".format(prefix, t_counter))
            t_counter += 1
        return match_query, prefix_list

    def create_relationship_insert_query(
         self, relationship, related_dict, attribute_list=[], prefix='r'):
        related_things = ""
        for role, variables in related_dict.items():
            for v in variables:
                aux = "{0}:${1}".format(role, v)
                if related_things != "":
                    aux = "," + aux
                related_things += aux
        insert_query = " ${0} ({1}) isa {2}".format(
            prefix, related_things, relationship)
        for attribute in attribute_list:
            if attribute[0] is not None:
                if type(attribute[1]) is str:
                    insert_query += ", has {} '{}' ".format(
                        attribute[0], attribute[1])
                elif type(attribute[1]) is datetime:
                    insert_query += ", has {} {} ".format(
                        attribute[0],
                        attribute[1].isoformat(timespec='milliseconds')
                    )
                else:
                    insert_query += ", has {} {} ".format(
                        attribute[0], attribute[1])
        insert_query += ";"
        return insert_query

    def delete_thing(self, thing, key, key_value):
        query = f"""
            match $thing isa {thing}, has {key} "{key_value}";
            delete $thing isa {thing};
        """
        return self.delete_from_database(query)

    def insert_entity(self, entity, attribute_list=[]):
        """
        Insert entity in the database.

        :param entity: entity name
        :type address: str
        :param attribute_list: list with attribute tuple (name, value)
        :type attribute_list: list[(name, value)]
        """
        query = f"""
            insert $entity isa {entity}
        """
        for attribute in attribute_list:
            if attribute[0] is not None:
                if type(attribute[1]) is str:
                    query += f""", has {attribute[0]} "{attribute[1]}" """
                else:
                    query += f""", has {attribute[0]} {attribute[1]}"""
        query += ";"
        return self.insert_database(query)

    # related_dict is a dictionary with the keys being the roles and
    # the values being the entities/relationships related
    def insert_relationship(
            self, relationship, related_dict, attribute_list=[]):
        match_query = "match "
        insert_query = "insert "
        _related_dict = dict()
        for key, things in related_dict.items():
            _match_query, _prefix_list = self.create_match_query(things, key)
            match_query += _match_query
            _related_dict[key] = _prefix_list

        insert_query += self.create_relationship_insert_query(
            relationship,
            _related_dict,
            attribute_list=attribute_list,
            prefix=relationship
        )
        query = match_query + insert_query
        return self.insert_database(query)

    def get_attribute_from_thing_raw(self, thing, key, key_value, attr):
        query = f"""
            match $thing isa {thing},
            has {key} "{key_value}",
            has {attr} $attribute;
            get $attribute;
        """
        return self.match_database(query)

    def get_attribute_from_thing(self, thing, key, key_value, attr):
        result = self.get_attribute_from_thing_raw(
            thing, key, key_value, attr)
        return [r.get('attribute').get('value') for r in result]

    def delete_attribute_from_thing(self, thing, key, key_value, attr):
        query = f"""
            match $thing isa {thing},
            has {key} "{key_value}",
            has {attr} $attribute;
            delete $thing has $attribute;
        """
        return self.delete_from_database(query)

    def insert_attribute_in_thing(
            self, thing, key, key_value, attr, attr_value):
        query = f"""
            match $thing isa {thing},
            has {key} "{key_value}";
            insert $thing has {attr} {attr_value};
        """
        return self.insert_database(query)

    def update_attribute_in_thing(
            self, thing, key, key_value, attr, attr_value):
        self.delete_attribute_from_thing(
            thing, key, key_value, attr)
        return self.insert_attribute_in_thing(
            thing, key, key_value, attr, attr_value)
