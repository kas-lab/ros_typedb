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
        'match $e isa person, has email "helper@test.test"; select $e; reduce $c = count;')
    assert result > 0


def test_delete_thing(db):
    insert_entity(db, 'person', [('email', 'del@test.test')])
    delete_thing(db, 'person', 'email', 'del@test.test')
    result = db.fetch_database(
        'match $e isa person, has email "del@test.test"; fetch { "email": $e.email };')
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
        ' has salary 2333, has role-name "boss"; select $r; reduce $c = count;')
    assert result > 0
