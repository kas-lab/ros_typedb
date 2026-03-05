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

from ros_typedb.typedb_helpers import attribute_dict_to_query
from ros_typedb.typedb_helpers import convert_py_type_to_query_type
from ros_typedb.typedb_helpers import convert_query_type_to_py_type
from ros_typedb.typedb_helpers import create_match_query
from ros_typedb.typedb_helpers import create_relationship_query
from ros_typedb.typedb_helpers import dict_to_query


def test_convert_query_type_to_py_type_from_value_dict():
    value_dict = {
        'type': {'value_type': 'double_array'},
        'value': ['1', '2.5', '3.0'],
    }
    assert convert_query_type_to_py_type(value_dict=value_dict) == [1.0, 2.5, 3.0]


@pytest.mark.parametrize(
    'value,value_type,expected',
    [
        ('5', 'long', 5),
        ('3.2', 'double', 3.2),
        ('hello', 'string', 'hello'),
        ('2024-01-02T03:04:05.678', 'datetime',
         datetime(2024, 1, 2, 3, 4, 5, 678000)),
        (['1', '2'], 'long_array', [1, 2]),
        (['1.2', '3.4'], 'double_array', [1.2, 3.4]),
        ([1, 'two'], 'string_array', ['1', 'two']),
        ('keep-me', 'unknown', 'keep-me'),
    ],
)
def test_convert_query_type_to_py_type(value, value_type, expected):
    assert convert_query_type_to_py_type(value=value, value_type=value_type) == expected


def test_convert_py_type_to_query_type():
    dt = datetime(2024, 1, 2, 3, 4, 5, 678999)
    assert convert_py_type_to_query_type('value') == "'value'"
    assert convert_py_type_to_query_type('$var') == '$var'
    assert convert_py_type_to_query_type(True) == 'true'
    assert convert_py_type_to_query_type(dt) == '2024-01-02T03:04:05.678'
    assert convert_py_type_to_query_type(42) == '42'


def test_convert_py_type_to_query_type_escapes_quotes_and_backslashes():
    assert convert_py_type_to_query_type("O'Brien") == "'O\\'Brien'"
    assert convert_py_type_to_query_type(r'c:\tmp\file') == r"'c:\\tmp\\file'"


@pytest.mark.parametrize(
    'bad_value_dict',
    [
        {},
        {'type': None, 'value': 'x'},
        {'type': {}, 'value': 'x'},
        {'type': {'value_type': 'string'}},
    ],
)
def test_convert_query_type_to_py_type_invalid_value_dict(bad_value_dict):
    with pytest.raises(ValueError):
        convert_query_type_to_py_type(value_dict=bad_value_dict)


def test_attribute_dict_to_query_with_scalars_and_lists():
    query = attribute_dict_to_query({
        'email': 'user@test.test',
        'tag': ['alpha', 'beta'],
        'active': True,
    })
    assert query == (
        " has email 'user@test.test', has tag 'alpha', has tag 'beta',"
        ' has active true'
    )


def test_attribute_dict_to_query_escapes_apostrophes():
    query = attribute_dict_to_query({'last-name': "O'Brien"})
    assert query == " has last-name 'O\\'Brien'"


def test_create_match_query_builds_match_fragment_and_variable_names():
    match_query, variable_names = create_match_query(
        [
            ('person', 'email', 'person@test.test'),
            ('company', 'name', 'TU Delft'),
        ],
        prefix='node',
    )
    assert match_query == (
        " $node_0 isa person, has email 'person@test.test';"
        " $node_1 isa company, has name 'TU Delft';"
    )
    assert variable_names == ['node_0', 'node_1']


def test_create_relationship_query_with_attributes():
    query = create_relationship_query(
        'employment',
        {
            'employee': ['emp_0', 'emp_1'],
            'employer': ['org_0'],
        },
        attribute_list=[('salary', 1200), ('role-name', 'boss')],
    )
    assert query == (
        " (employee:$emp_0,employee:$emp_1,employer:$org_0) isa employment,"
        " has salary 1200, has role-name 'boss';"
    )


def test_dict_to_query_matches_attribute_dict_to_query():
    data_dict = {'nickname': 'neo', 'age': 33}
    assert dict_to_query(data_dict) == attribute_dict_to_query(data_dict)
