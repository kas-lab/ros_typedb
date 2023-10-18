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
from datetime import datetime


@pytest.fixture
def typedb_interface():
    typedb_interface = TypeDBInterface(
        'localhost:1729',
        'test_database',
        force_database=True,
        schema_path='test/typedb_test_data/schema.tql',
        data_path='test/typedb_test_data/data.tql',
        force_data=True)
    return typedb_interface


def test_insert_entity(typedb_interface):
    typedb_interface.insert_entity(
        'person', [('email', 'test@email.test'), ('nickname', 't')])
    query = """
        match $entity isa person,
        has email "test@email.test",
        has nickname "t";
    """
    result = typedb_interface.match_database(query)
    assert len(result) > 0


def test_delete_thing(typedb_interface):
    typedb_interface.insert_entity('person', [('email', 'test@email.test')])
    typedb_interface.delete_thing('person', 'email', 'test@email.test')
    query = """
        match $entity isa person, has email "test@email.test";
        get $entity;
    """
    result = typedb_interface.match_database(query)
    assert len(result) == 0


@pytest.mark.parametrize("attr, attr_value", [
    ('nickname', 't'),
    ('alive', True),
    ('age', 33),
    ('height', 3.237),
    ('birth-date', datetime.now()),
])
def test_insert_attribute_in_thing(typedb_interface, attr, attr_value):
    typedb_interface.insert_entity('person', [('email', 'test@email.test')])
    typedb_interface.insert_attribute_in_thing(
        'person', 'email', 'test@email.test', attr, attr_value)
    result = typedb_interface.get_attribute_from_thing(
        'person', 'email', 'test@email.test', attr)
    assert result


@pytest.mark.parametrize("attr, attr_value", [
    ('nickname', 't'),
    ('alive', True),
    ('age', 33),
    ('height', 3.237),
    ('birth-date', datetime.fromisoformat(
        datetime.now().isoformat(timespec='milliseconds'))),
])
def test_get_attribute_from_thing(typedb_interface, attr, attr_value):
    typedb_interface.insert_entity('person', [('email', 'test@email.test')])
    typedb_interface.insert_attribute_in_thing(
        'person', 'email', 'test@email.test', attr, attr_value)

    result = typedb_interface.get_attribute_from_thing(
        'person', 'email', 'test@email.test', attr)

    assert result[0] == attr_value


@pytest.mark.parametrize("attr, attr_value", [
    ('nickname', 't'),
    ('alive', True),
    ('age', 33),
    ('height', 3.237),
    ('birth-date', datetime.fromisoformat(
        datetime.now().isoformat(timespec='milliseconds'))),
])
def test_delete_attribute_from_thing(typedb_interface, attr, attr_value):
    typedb_interface.insert_entity('person', [('email', 'test@email.test')])
    typedb_interface.insert_attribute_in_thing(
        'person', 'email', 'test@email.test', attr, attr_value)

    typedb_interface.delete_attribute_from_thing(
        'person', 'email', 'test@email.test', attr)

    result = typedb_interface.get_attribute_from_thing(
        'person', 'email', 'test@email.test', attr)

    assert len(result) == 0


@pytest.mark.parametrize("attr, attr_value, new_v", [
    ('nickname', 't', 'new_t'),
    ('alive', True, False),
    ('age', 33, 56),
    ('height', 3.237, 1.66),
    ('birth-date', datetime.fromisoformat(
        datetime.now().isoformat(timespec='milliseconds')),
        datetime.fromisoformat(
            datetime.now().isoformat(timespec='milliseconds'))),
])
def test_update_attribute_in_thing(typedb_interface, attr, attr_value, new_v):
    typedb_interface.insert_entity('person', [('email', 'test@email.test')])
    typedb_interface.insert_attribute_in_thing(
        'person', 'email', 'test@email.test', attr, attr_value)
    typedb_interface.update_attribute_in_thing(
        'person', 'email', 'test@email.test', attr, new_v)
    result = typedb_interface.get_attribute_from_thing(
        'person', 'email', 'test@email.test', attr)

    assert result[0] == new_v


def test_insert_relationship(typedb_interface):
    typedb_interface.insert_entity('person', [('email', 'test@email.test')])
    typedb_interface.insert_entity('person', [('email', 'test2@email.test')])
    typedb_interface.insert_entity('person', [('email', 'test3@email.test')])
    typedb_interface.insert_entity('person', [('email', 'test4@email.test')])

    related_things_dict = {
        'employee': [
            ('person', 'email', 'test@email.test'),
            ('person', 'email', 'test2@email.test')],
        'employer': [
            ('person', 'email', 'test3@email.test'),
            ('person', 'email', 'test4@email.test')
        ]
    }
    attribute_list = [
        ('salary', 2333), ('role-name', 'boss'), ('role-name', 'super boss')]
    insert_result = typedb_interface.insert_relationship(
        'employment', related_things_dict, attribute_list)

    query = """
        match $r (employee:$ee, employer:$er)isa employment,
        has salary 2333,
        has role-name "boss",
        has role-name "super boss";
    """
    result = typedb_interface.match_database(query)
    assert len(result) > 0
