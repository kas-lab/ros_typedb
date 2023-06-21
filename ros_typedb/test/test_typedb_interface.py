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

import pytest
from ros_typedb.typedb_interface import TypeDBInterface


@pytest.fixture
def typedb_interface():
    typedb_interface = TypeDBInterface(
        "localhost:1729",
        "test_database",
        force_database=True,
        schema_path='test/typedb_test_data/schema.tql',
        data_path='test/typedb_test_data/data.tql',
        force_data=True)
    return typedb_interface


def test_insert_entity(typedb_interface):
    typedb_interface.insert_entity('person', 'email', 'test@email.test')
    query = '''
        match $entity isa person, has email "test@email.test";
        get $entity;
    '''
    result = typedb_interface.match_database(query)
    assert len(result) > 0


def test_delete_entity(typedb_interface):
    typedb_interface.insert_entity('person', 'email', 'test@email.test')
    typedb_interface.delete_entity('person', 'email', 'test@email.test')
    query = '''
        match $entity isa person, has email "test@email.test";
        get $entity;
    '''
    result = typedb_interface.match_database(query)
    assert len(result) == 0


def test_insert_attribute_entity(typedb_interface):
    typedb_interface.insert_entity('person', 'email', 'test@email.test')
    typedb_interface.insert_attribute_entity(
        'person', 'email', 'test@email.test', 'nickname', '"t"')
    typedb_interface.insert_attribute_entity(
        'person', 'email', 'test@email.test', 'graduated', 'true')
    typedb_interface.insert_attribute_entity(
        'person', 'email', 'test@email.test', 'age', '33')
    typedb_interface.insert_attribute_entity(
        'person', 'email', 'test@email.test', 'height', '3.237')


def test_get_attribute_from_entity(typedb_interface):
    typedb_interface.insert_entity('person', 'email', 'test@email.test')
    typedb_interface.insert_attribute_entity(
        'person', 'email', 'test@email.test', 'nickname', '"t"')
    result = typedb_interface.get_attribute_from_entity(
        'person', 'email', 'test@email.test', 'nickname')

    assert result[0] == 't'


def test_delete_attribute_from_entity(typedb_interface):
    typedb_interface.insert_entity('person', 'email', 'test@email.test')
    typedb_interface.insert_attribute_entity(
        'person', 'email', 'test@email.test', 'nickname', '"t"')
    typedb_interface.delete_attribute_from_entity(
        'person', 'email', 'test@email.test', 'nickname')
    result = typedb_interface.get_attribute_from_entity(
        'person', 'email', 'test@email.test', 'nickname')

    assert len(result) == 0


def test_update_attribute_entity(typedb_interface):
    typedb_interface.insert_entity('person', 'email', 'test@email.test')
    typedb_interface.insert_attribute_entity(
        'person', 'email', 'test@email.test', 'nickname', '"t"')
    typedb_interface.update_attribute_entity(
        'person', 'email', 'test@email.test', 'nickname', '"new_t"')
    result = typedb_interface.get_attribute_from_entity(
        'person', 'email', 'test@email.test', 'nickname')

    assert result[0] == 'new_t'
