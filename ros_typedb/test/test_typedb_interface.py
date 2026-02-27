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

import pytest

from ros_typedb.typedb_interface import TypeDBInterface


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
