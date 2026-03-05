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
from ros_typedb.typedb_interface import TypeDBQueryError
from ros_typedb.typedb_interface import _string_to_string_array


@pytest.fixture
def typedb_interface():
    """Fixture that creates a TypeDBInterface with test data."""
    tdb = TypeDBInterface(
        'localhost:1729',
        'test_database',
        force_database=True,
        schema_path=['test/typedb_test_data/schema.tql'],
        data_path=['test/typedb_test_data/data.tql'],
    )
    yield tdb
    tdb.delete_database()


def test_create_and_delete_database():
    """Test database creation and deletion."""
    typedb_interface = TypeDBInterface(
        'localhost:1729',
        'test_database',
        force_database=True,
        schema_path=['test/typedb_test_data/schema.tql'],
        data_path=['test/typedb_test_data/data.tql'],
    )

    assert typedb_interface.driver.databases.contains('test_database')

    typedb_interface.delete_database()
    assert not typedb_interface.driver.databases.contains('test_database')


def test_get_query(typedb_interface):
    """Test that select query returns normalised attribute dicts."""
    query = """
        match
            $p isa person, has full-name $name, has email $email;
        select $name, $email;
        sort $name asc; limit 3;
    """
    result = typedb_interface.get_database(query)
    assert len(result) == 3
    name_val = result[0].get('name', {}).get('value')
    assert name_val == 'Ahmed Frazier'


def test_fetch_query(typedb_interface):
    """Test that fetch query returns plain Python dicts."""
    query = """
    match
        $company isa company, has name "TU Delft";
        (employer: $company, employee: $employee) isa employment,
        has salary $salary;
    fetch {
        "full-name": $employee.full-name,
        "email": $employee.email,
        "salary": $salary
    };
    """
    result = typedb_interface.fetch_database(query)
    assert len(result) == 3

    expected = [
        {
            'salary': 30000,
            'email': 'phd@tudelft.nl',
            'full-name': 'PhD candidate 1'
        },
        {
            'salary': 999999,
            'email': 'boss@tudelft.nl',
            'full-name': 'Big Boss'
        },
        {
            'salary': 0,
            'email': 'guest@tudelft.nl',
            'full-name': 'Random guest'
        }
    ]

    # Compare as sets so order does not matter
    assert {tuple(sorted(d.items())) for d in result} == \
        {tuple(sorted(d.items())) for d in expected}


def test_delete_all_data(typedb_interface):
    """Test that delete_all_data removes both entities and relations."""
    person_count_query = """
        match $p isa person;
        select $p; reduce $c = count;
    """
    employment_count_query = """
        match $e isa employment;
        select $e; reduce $c = count;
    """
    person_count_before = typedb_interface.get_aggregate_database(
        person_count_query)
    employment_count_before = typedb_interface.get_aggregate_database(
        employment_count_query)
    assert person_count_before > 0
    assert employment_count_before > 0

    typedb_interface.delete_all_data()

    person_count_after = typedb_interface.get_aggregate_database(
        person_count_query)
    employment_count_after = typedb_interface.get_aggregate_database(
        employment_count_query)
    assert person_count_after == 0
    assert employment_count_after == 0


def test_database_query_unsupported_query_type_raises_value_error(
        typedb_interface):
    """Unsupported query type should fail fast before query execution."""
    with pytest.raises(ValueError):
        typedb_interface.database_query(  # type: ignore[arg-type]
            'data', 'read', 'unknown',
            'match $p isa person; select $p;')


@pytest.mark.parametrize(
    'raw,expected',
    [
        ("['a.tql', 'b.tql']", ['a.tql', 'b.tql']),
        ("'/tmp/schema.tql'", ['/tmp/schema.tql']),
        ('/tmp/data,with,comma.tql', ['/tmp/data,with,comma.tql']),
        ('', []),
    ],
)
def test_string_to_string_array_parses_paths(raw, expected):
    """Path-string parser should support literals and raw path strings."""
    assert _string_to_string_array(raw) == expected


@pytest.mark.parametrize(
    'raw',
    [
        '[a.tql, b.tql]',
        "['a.tql', 123]",
    ],
)
def test_string_to_string_array_invalid_list_literal_raises(raw):
    """Malformed list-literal path values should raise a clear ValueError."""
    with pytest.raises(ValueError):
        _string_to_string_array(raw)


def test_split_data_statements_handles_insert_prelude_and_match_blocks(
        typedb_interface):
    """Split logic should preserve insert prelude and subsequent match blocks."""
    content = """
        # comment prelude
        insert
            $p isa person, has email 'split@test.test';

        match
            $p isa person, has email 'split@test.test';
        insert
            $p has nickname 'split';

        match
            $p isa person, has email 'split@test.test';
        delete $p;
    """
    statements = typedb_interface._split_data_statements(content)
    assert len(statements) == 3
    assert statements[0].lstrip().startswith('insert')
    assert statements[1].lstrip().startswith('match')
    assert statements[2].lstrip().startswith('match')


def test_split_data_statements_discards_comment_only_content(typedb_interface):
    """Split logic should return no statements for comment-only content."""
    content = """
        # comment 1
        # comment 2
    """
    assert typedb_interface._split_data_statements(content) == []


def test_split_data_statements_keeps_single_insert_block(typedb_interface):
    """Split logic should keep a plain insert-only content as one statement."""
    content = """

            insert
                $p isa person, has email 'single@test.test';

    """
    statements = typedb_interface._split_data_statements(content)
    assert len(statements) == 1
    assert statements[0].lstrip().startswith('insert')


def test_database_query_logs_and_raises_typedb_query_error(
        typedb_interface, capsys):
    """Invalid TypeQL should be logged and raised as TypeDBQueryError."""
    with pytest.raises(TypeDBQueryError):
        typedb_interface.fetch_database('this is not valid typeql')

    captured = capsys.readouterr()
    assert 'database_query failed' in captured.err
    assert 'query_type=fetch' in captured.err


def test_load_schema_raises_error_for_invalid_schema(typedb_interface, tmp_path):
    """Loading an invalid schema file should raise TypeDBQueryError."""
    bad_schema_path = tmp_path / 'bad_schema.tql'
    bad_schema_path.write_text('define ???')

    with pytest.raises(TypeDBQueryError):
        typedb_interface.load_schema(str(bad_schema_path))


@pytest.mark.parametrize('method_name', [
    'insert_database',
    'update_database',
    'delete_from_database',
    'fetch_database',
    'get_database',
    'get_aggregate_database',
])
def test_wrapper_methods_propagate_query_errors(typedb_interface, method_name):
    """All wrappers should propagate TypeDBQueryError on invalid query."""
    method = getattr(typedb_interface, method_name)
    with pytest.raises(TypeDBQueryError):
        method('this is not valid typeql')


def test_insert_entity(typedb_interface):
    typedb_interface.insert_entity(
        'person', [('email', 'helper@test.test'), ('nickname', 'h')])
    result = typedb_interface.get_aggregate_database(
        'match $e isa person, has email "helper@test.test"; '
        'select $e; reduce $c = count;')
    assert result > 0


def test_delete_thing(typedb_interface):
    typedb_interface.insert_entity('person', [('email', 'del@test.test')])
    typedb_interface.delete_thing('person', 'email', 'del@test.test')
    result = typedb_interface.fetch_database(
        'match $e isa person, has email "del@test.test"; '
        'fetch { "email": $e.email };')
    assert len(result) == 0


@pytest.mark.parametrize(
    'attr,attr_value',
    [
        ('nickname', 't'),
        ('alive', True),
        ('age', 33),
        ('height', 3.237),
        ('birth-date', datetime(2024, 1, 2, 3, 4, 5, 678000)),
    ],
)
def test_insert_attribute_in_thing(typedb_interface, attr, attr_value):
    typedb_interface.insert_entity('person', [('email', 'attr@test.test')])
    typedb_interface.insert_attribute_in_thing(
        'person', 'email', 'attr@test.test', attr, attr_value)
    result = typedb_interface.fetch_attribute_from_thing(
        'person', [('email', 'attr@test.test')], attr)
    assert result


@pytest.mark.parametrize(
    'attr,attr_value',
    [
        ('nickname', 't'),
        ('alive', True),
        ('age', 33),
        ('height', 3.237),
        ('birth-date', datetime(2024, 1, 2, 3, 4, 5, 678000)),
    ],
)
def test_fetch_attribute_from_thing(typedb_interface, attr, attr_value):
    typedb_interface.insert_entity(
        'person', [('email', 'fetch@test.test'), ('gender', 'male')])
    typedb_interface.insert_attribute_in_thing(
        'person', 'email', 'fetch@test.test', attr, attr_value)
    result = typedb_interface.fetch_attribute_from_thing(
        'person',
        [('email', 'fetch@test.test'), ('gender', 'male')],
        attr,
    )
    assert result[0] == attr_value


@pytest.mark.parametrize(
    'attr,attr_value',
    [
        ('nickname', 't'),
        ('alive', True),
        ('age', 33),
        ('height', 3.237),
        ('birth-date', datetime(2024, 1, 2, 3, 4, 5, 678000)),
    ],
)
def test_delete_attribute_from_thing(typedb_interface, attr, attr_value):
    typedb_interface.insert_entity('person', [('email', 'delattr@test.test')])
    typedb_interface.insert_attribute_in_thing(
        'person', 'email', 'delattr@test.test', attr, attr_value)
    typedb_interface.delete_attribute_from_thing(
        'person', 'email', 'delattr@test.test', attr)
    result = typedb_interface.fetch_attribute_from_thing(
        'person', [('email', 'delattr@test.test')], attr)
    assert len(result) == 0


@pytest.mark.parametrize(
    'attr,attr_value,new_value',
    [
        ('nickname', 't', 'new_t'),
        ('alive', True, False),
        ('age', 33, 56),
        ('height', 3.237, 1.66),
        (
            'birth-date',
            datetime(2024, 1, 2, 3, 4, 5, 678000),
            datetime(2025, 2, 3, 4, 5, 6, 123000),
        ),
    ],
)
def test_update_attribute_in_thing(
        typedb_interface, attr, attr_value, new_value):
    typedb_interface.insert_entity('person', [('email', 'upd@test.test')])
    typedb_interface.insert_attribute_in_thing(
        'person', 'email', 'upd@test.test', attr, attr_value)
    typedb_interface.update_attribute_in_thing(
        'person', 'email', 'upd@test.test', attr, new_value)
    result = typedb_interface.fetch_attribute_from_thing(
        'person', [('email', 'upd@test.test')], attr)
    assert result[0] == new_value


def test_insert_relationship(typedb_interface):
    typedb_interface.insert_entity('person', [('email', 'e1@test.test')])
    typedb_interface.insert_entity('person', [('email', 'e2@test.test')])
    typedb_interface.insert_entity('person', [('email', 'e3@test.test')])
    typedb_interface.insert_entity('person', [('email', 'e4@test.test')])
    typedb_interface.insert_relationship(
        'employment',
        {
            'employee': [
                ('person', 'email', 'e1@test.test'),
                ('person', 'email', 'e2@test.test'),
            ],
            'employer': [
                ('person', 'email', 'e3@test.test'),
                ('person', 'email', 'e4@test.test'),
            ],
        },
        [('salary', 2333), ('role-name', 'boss')],
    )
    result = typedb_interface.get_aggregate_database(
        'match $r (employee:$ee, employer:$er) isa employment, '
        'has salary 2333, has role-name "boss"; select $r; reduce $c = count;')
    assert result > 0


def test_fetch_attribute_from_thing_raw(typedb_interface):
    typedb_interface.insert_entity(
        'person', [('email', 'raw@test.test'), ('nickname', 'raw-nick')])
    result = typedb_interface.fetch_attribute_from_thing_raw(
        'person', [('email', 'raw@test.test')], 'nickname')
    assert len(result) == 1
    attribute = result[0]['attribute']
    assert attribute.get('type', {}).get('label') == 'nickname'
    assert attribute.get('value') == 'raw-nick'


def test_insert_attributes_in_thing(typedb_interface):
    typedb_interface.insert_entity('person', [('email', 'multi-insert@test.test')])
    typedb_interface.insert_attributes_in_thing(
        'person',
        'email',
        'multi-insert@test.test',
        [('nickname', 'inserted'), ('age', 21), ('alive', True)],
    )
    assert typedb_interface.fetch_attribute_from_thing(
        'person', [('email', 'multi-insert@test.test')], 'nickname') == ['inserted']
    assert typedb_interface.fetch_attribute_from_thing(
        'person', [('email', 'multi-insert@test.test')], 'age') == [21]
    assert typedb_interface.fetch_attribute_from_thing(
        'person', [('email', 'multi-insert@test.test')], 'alive') == [True]


def test_delete_attributes_from_thing(typedb_interface):
    typedb_interface.insert_entity('person', [('email', 'multi-delete@test.test')])
    typedb_interface.insert_attributes_in_thing(
        'person',
        'email',
        'multi-delete@test.test',
        [('nickname', 'to-delete'), ('age', 35), ('alive', True)],
    )
    typedb_interface.delete_attributes_from_thing(
        'person',
        'email',
        'multi-delete@test.test',
        ['nickname', 'age', 'alive'],
    )
    assert typedb_interface.fetch_attribute_from_thing(
        'person', [('email', 'multi-delete@test.test')], 'nickname') == []
    assert typedb_interface.fetch_attribute_from_thing(
        'person', [('email', 'multi-delete@test.test')], 'age') == []
    assert typedb_interface.fetch_attribute_from_thing(
        'person', [('email', 'multi-delete@test.test')], 'alive') == []


def test_update_attributes_in_thing(typedb_interface):
    typedb_interface.insert_entity('person', [('email', 'multi-update@test.test')])
    typedb_interface.insert_attributes_in_thing(
        'person',
        'email',
        'multi-update@test.test',
        [('nickname', 'before'), ('age', 40), ('alive', True)],
    )
    typedb_interface.update_attributes_in_thing(
        'person',
        'email',
        'multi-update@test.test',
        [('nickname', 'after'), ('age', 41), ('alive', False)],
    )
    assert typedb_interface.fetch_attribute_from_thing(
        'person', [('email', 'multi-update@test.test')], 'nickname') == ['after']
    assert typedb_interface.fetch_attribute_from_thing(
        'person', [('email', 'multi-update@test.test')], 'age') == [41]
    assert typedb_interface.fetch_attribute_from_thing(
        'person', [('email', 'multi-update@test.test')], 'alive') == [False]
