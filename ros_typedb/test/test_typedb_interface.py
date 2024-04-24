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
        schema_path=['test/typedb_test_data/schema.tql'],
        data_path=['test/typedb_test_data/data.tql'],
        force_data=True,
    )
    return typedb_interface


def test_insert_entity(typedb_interface):
    typedb_interface.insert_entity(
        'person', [('email', 'test@email.test'), ('nickname', 't')])
    query = """
        match $entity isa person,
        has email "test@email.test",
        has nickname "t";
        get $entity;
        count;
    """
    result = typedb_interface.get_aggregate_database(query)
    assert result > 0


def test_delete_thing(typedb_interface):
    typedb_interface.insert_entity('person', [('email', 'test@email.test')])
    typedb_interface.delete_thing('person', 'email', 'test@email.test')
    query = """
        match $entity isa person, has email "test@email.test";
        get $entity;
    """
    result = typedb_interface.fetch_database(query)
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
    result = typedb_interface.fetch_attribute_from_thing(
        'person', [('email', 'test@email.test')], attr)
    assert result


@pytest.mark.parametrize("attr, attr_value", [
    ('nickname', 't'),
    ('alive', True),
    ('age', 33),
    ('height', 3.237),
    ('birth-date', datetime.fromisoformat(
        datetime.now().isoformat(timespec='milliseconds'))),
])
def test_fetch_attribute_from_thing(typedb_interface, attr, attr_value):
    typedb_interface.insert_entity(
        'person',
        [('email', 'test@email.test'), ('gender', 'male')]
    )
    typedb_interface.insert_attribute_in_thing(
        'person', 'email', 'test@email.test', attr, attr_value)

    result = typedb_interface.fetch_attribute_from_thing(
        'person', [('email', 'test@email.test'), ('gender', 'male')], attr)

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

    result = typedb_interface.fetch_attribute_from_thing(
        'person', [('email', 'test@email.test')], attr)

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
    result_update = typedb_interface.update_attribute_in_thing(
        'person', 'email', 'test@email.test', attr, new_v)
    result = typedb_interface.fetch_attribute_from_thing(
        'person', [('email', 'test@email.test')], attr)

    assert result_update is not None and result[0] == new_v


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
    typedb_interface.insert_relationship(
        'employment', related_things_dict, attribute_list)

    query = """
        match $r (employee:$ee, employer:$er)isa employment,
        has salary 2333,
        has role-name "boss",
        has role-name "super boss";
        get $r;
        count;
    """
    result = typedb_interface.get_aggregate_database(query)
    assert result > 0


@pytest.mark.parametrize("things_dict", [
    ({
        'person': [
            {
                'prefix': 'p1',
                'attributes': {
                    'email': 'test@email.test',
                    'nickname': 't',
                    'height': 1.80,
                    'age': 18,
                    'alive': True,
                    'birth-date': datetime.now()
                }
            },
            {
                'prefix': 'p2',
                'attributes': {
                    'email': 'test2@email.test',
                    'nickname': 't2',
                    'height': 1.33,
                    'age': 128,
                    'alive': False,
                    'birth-date': datetime.now()
                }
            },
        ],
        'robot': [
            {
                'prefix': 'r',
                'attributes': {
                    'full-name': 'robot123',
                    'height': 1.0,
                    'age': 0,
                    'alive': True,
                    'birth-date': datetime.now()
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
    }),
])
def test_dict_to_query(typedb_interface, things_dict):
    query = typedb_interface.dict_to_query(things_dict)
    insert_result = typedb_interface.insert_database("insert " + query)
    match_result = typedb_interface.fetch_database("match " + query)
    assert insert_result is not None and insert_result is not False \
        and match_result is not None and match_result is not False


@pytest.mark.parametrize("match_dict, r_dict", [
    (
        {
            'person': [
                {
                    'prefix': 'p1',
                    'attributes': {
                        'email': 'test_person@test.test',
                    },
                    'insert_attributes': {
                        'nickname': 't',
                        'height': 1.80,
                        'age': 18,
                        'alive': True,
                        'birth-date': datetime.now()
                    }
                },
            ],
        },
        {
            'person': [
                {
                    'prefix': 'p1',
                    'attributes': {
                        'email': 'test_person@test.test',
                        'nickname': 't',
                        'height': 1.80,
                        'age': 18,
                        'alive': True,
                        'birth-date': datetime.now()
                    }
                },
            ],
        },
    ),
])
def test_insert_attributes(typedb_interface, match_dict, r_dict):
    r = typedb_interface.insert_attributes_in_thing(match_dict)
    query = typedb_interface.dict_to_query(r_dict)
    match_result = typedb_interface.get_aggregate_database(
        "match " + query + "get; count;")
    assert r is not None and r is not False and match_result > 0


@pytest.mark.parametrize("insert_dict, match_dict", [
    (
        {
            'person': [
                {
                    'prefix': 'p1',
                    'attributes': {
                        'email': 'test@test.test',
                        'nickname': 't',
                        'height': 1.80,
                        'age': 18,
                        'alive': True,
                        'birth-date': datetime.now()
                    }
                },
            ],
            'robot': [
                {
                    'prefix': 'r',
                    'attributes': {
                        'full-name': 'robot123',
                        'height': 1.0,
                        'age': 0,
                        'alive': True,
                        'birth-date': datetime.now(),
                        'robot-type': 'auv'
                    }
                },
            ],
        },
        {
            'person': [
                {
                    'prefix': 'p1',
                    'attributes': {
                        'email': 'test@test.test',
                    },
                    'delete_attributes': [
                        'nickname', 'height', 'age', 'alive', 'birth-date']
                },
            ],
            'robot': [
                {
                    'attributes': {
                        'full-name': 'robot123',
                    },
                    'delete_attributes': ['robot-type']
                },
            ],
        },
    )
])
def test_delete_attributes(
   typedb_interface, insert_dict, match_dict):

    query = typedb_interface.dict_to_query(insert_dict)
    typedb_interface.insert_database("insert " + query)

    r = typedb_interface.delete_attributes_from_thing(match_dict)

    query = typedb_interface.dict_to_query(insert_dict)
    match_result = typedb_interface.fetch_database("match " + query)
    assert r is not None and r is not False and len(match_result) == 0


@pytest.mark.parametrize("insert_dict, update_dict, r_dict", [
    (
        {
            'person': [
                {
                    'prefix': 'p1',
                    'attributes': {
                        'email': 'test@test.test',
                        'nickname': 't',
                        'height': 1.80,
                        'age': 18,
                        'alive': True,
                        'birth-date': datetime.now()
                    }
                },
            ],
        },
        {
            'person': [
                {
                    'prefix': 'p1',
                    'attributes': {
                        'email': 'test@test.test',
                    },
                    'update_attributes': {
                        'nickname': 't2',
                        'height': 1.50,
                        'age': 17,
                        'alive': False,
                    }
                },
            ],
        },
        {
            'person': [
                {
                    'prefix': 'p1',
                    'attributes': {
                        'email': 'test@test.test',
                        'nickname': 't2',
                        'height': 1.50,
                        'age': 17,
                        'alive': False,
                    }
                },
            ],
        },
    ),
])
def test_update_attributes(typedb_interface, insert_dict, update_dict, r_dict):
    query = typedb_interface.dict_to_query(insert_dict)
    typedb_interface.insert_database("insert " + query)

    r = typedb_interface.update_attributes_in_thing(update_dict)
    query = typedb_interface.dict_to_query(r_dict)
    match_result = typedb_interface.get_aggregate_database(
        "match " + query + "get; count;")
    assert r is not None and r is not False and match_result > 0
