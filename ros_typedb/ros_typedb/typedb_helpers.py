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
"""typedb_helpers - utility functions for TypeDB query/value conversions."""

from datetime import datetime


def convert_query_type_to_py_type(
        value_dict: dict | None = None,
        value: str | None = None,
        value_type: str | None = None) -> 'datetime | int | str | float':
    """
    Convert typedb 'value_type' to python type.

    :param value: Data to be converted.
    :param value_type: Data type string (e.g. 'long', 'string').
    :param value_dict: Typedb value dict, overrides value and value_type.
    :return: Converted data.
    :raises ValueError: If value_dict misses required typedb value fields.
    """
    if value_dict is not None:
        value_type_dict = value_dict.get('type')
        if not isinstance(value_type_dict, dict) or 'value_type' not in value_type_dict:
            raise ValueError(
                "value_dict must include a 'type' dict with 'value_type'.")
        if 'value' not in value_dict:
            raise ValueError("value_dict must include a 'value' key.")
        value_type = value_type_dict.get('value_type')
        value = value_dict.get('value')
    if value_type == 'datetime':
        return datetime.fromisoformat(value)
    elif value_type == 'long':
        return int(value)
    elif value_type == 'string':
        return str(value)
    elif value_type == 'double':
        return float(value)
    elif value_type == 'long_array':
        return list(map(int, value))
    elif value_type == 'double_array':
        return list(map(float, value))
    elif value_type == 'string_array':
        return list(map(str, value))
    return value


def convert_py_type_to_query_type(data: 'datetime | str | bool') -> str:
    """
    Convert a Python value to a properly formatted TypeQL string.

    :param data: Data to be converted.
    :return: Converted data as string.
    """
    if isinstance(data, str):
        if len(data) > 0 and data[0] != '$':
            escaped_data = data.replace('\\', '\\\\').replace("'", "\\'")
            return "'{}'".format(escaped_data)
    elif isinstance(data, datetime):
        return data.isoformat(timespec='milliseconds')
    elif isinstance(data, bool):
        return str(data).lower()
    return data


def attribute_dict_to_query(attribute_dict: dict) -> str:
    """
    Convert an attribute dict to a TypeQL 'has' clause fragment.

    :param attribute_dict: dict mapping attribute names to values.
    :return: TypeQL string fragment like " has email 'x', has age 5".
    """
    _query = ''
    first = True
    for attr, attr_value in attribute_dict.items():
        if not isinstance(attr_value, list):
            attr_value = [attr_value]
        for v in attr_value:
            if first is False:
                _query += ','
            _query += ' has {0} {1}'.format(attr, convert_py_type_to_query_type(v))
            first = False
    return _query


def create_match_query(
        things_list: list,
        prefix: str = 't') -> tuple[str, list]:
    """
    Build a match clause from a list of (type, attr, value) tuples.

    :param things_list: list of (thing_type, attr_name, attr_value) tuples.
    :param prefix: variable name prefix.
    :return: tuple of (match_clause_string, list_of_variable_names).
    """
    match_query = ''
    prefix_list = []
    t_counter = 0
    for thing in things_list:
        match_query += ' ${0}_{1} isa {2},'.format(prefix, t_counter, thing[0])
        match_query += ' has {0} {1};'.format(
            thing[1], convert_py_type_to_query_type(thing[2]))
        prefix_list.append('{0}_{1}'.format(prefix, t_counter))
        t_counter += 1
    return match_query, prefix_list


def create_relationship_query(
        relationship: str,
        related_dict: dict,
        attribute_list: list | None = None,
        prefix: str = 'r') -> str:
    """
    Build a TypeQL insert clause for a relationship.

    :param relationship: relationship type name.
    :param related_dict: dict mapping role names to lists of variable name strings.
    :param attribute_list: list of (attr_name, attr_value) tuples.
    :param prefix: variable name prefix for the relationship variable.
    :return: TypeQL insert clause string.
    """
    del prefix
    if attribute_list is None:
        attribute_list = []
    related_things = ''
    for role, variables in related_dict.items():
        for v in variables:
            aux = '{0}:${1}'.format(role, v)
            if related_things != '':
                aux = ',' + aux
            related_things += aux
    query = ' ({0}) isa {1}'.format(related_things, relationship)
    for attribute in attribute_list:
        if attribute[0] is not None:
            query += ', has {} {} '.format(
                attribute[0], convert_py_type_to_query_type(attribute[1]))
    query += ';'
    return query


def dict_to_query(data_dict: dict) -> str:
    """
    Convert a dict of attributes to a TypeQL 'has' clause fragment.

    This is a simple alias for :func:`attribute_dict_to_query`.

    :param data_dict: dict mapping attribute names to values.
    :return: TypeQL string fragment.
    """
    return attribute_dict_to_query(data_dict)
