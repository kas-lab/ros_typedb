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
import functools

from typedb.driver import SessionType
from typedb.driver import TransactionType
from typedb.driver import TypeDB
from typedb.driver import TypeDBDriverException
from typedb.driver import TypeDBOptions


class TypeDBInterface:

    def __init__(self, address, database_name, schema_path=None,
                 data_path=None, force_database=False, force_data=False):
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

    def delete_entity(self, entity, key, key_value):
        query = f"""
            match $entity isa {entity}, has {key} "{key_value}";
            delete $entity isa {entity};
        """
        return self.delete_from_database(query)

    def insert_entity(self, entity, attribute_list=[]):
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
        related_things = ""
        t_counter = 0
        for role, things in related_dict.items():
            for thing in things:
                match_query += f"""
                    $t_{t_counter} isa {thing[0]},
                """
                if type(thing[2]) is str:
                    match_query += f"""
                        has {thing[1]} "{thing[2]}";
                    """
                else:
                    match_query += f"""
                        has {thing[1]} {thing[2]};
                    """

                aux = f"""{role}:$t_{t_counter}"""
                if related_things != "":
                    aux = "," + aux
                related_things += aux
                t_counter += 1

        query = match_query
        query += f"""
            insert ({related_things}) isa {relationship}"""
        for attribute in attribute_list:
            if attribute[0] is not None:
                if type(attribute[1]) is str:
                    query += f""", has {attribute[0]} "{attribute[1]}" """
                else:
                    query += f""", has {attribute[0]} {attribute[1]}"""
        query += ";"
        return self.insert_database(query)

    def get_attribute_from_entity_raw(self, entity, key, key_value, attr):
        query = f"""
            match $entity isa {entity},
            has {key} "{key_value}",
            has {attr} $attribute;
            get $attribute;
        """
        return self.match_database(query)

    def get_attribute_from_entity(self, entity, key, key_value, attr):
        result = self.get_attribute_from_entity_raw(
            entity, key, key_value, attr)
        return [r.get('attribute').get('value') for r in result]

    def delete_attribute_from_entity(self, entity, key, key_value, attr):
        query = f"""
            match $entity isa {entity},
            has {key} "{key_value}",
            has {attr} $attribute;
            delete $entity has $attribute;
        """
        return self.delete_from_database(query)

    def insert_attribute_entity(
            self, entity, key, key_value, attr, attr_value):
        query = f"""
            match $entity isa {entity},
            has {key} "{key_value}";
            insert $entity has {attr} {attr_value};
        """
        return self.insert_database(query)

    def update_attribute_entity(
            self, entity, key, key_value, attr, attr_value):
        self.delete_attribute_from_entity(
            entity, key, key_value, attr)
        return self.insert_attribute_entity(
            entity, key, key_value, attr, attr_value)
