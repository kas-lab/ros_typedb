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
from typing import Any

AttributePair = tuple[str, Any]
ThingMatchTuple = tuple[str, str, Any]
RelatedThingsDict = dict[str, list[ThingMatchTuple]]
RelatedVariablesDict = dict[str, list[str]]


def convert_query_type_to_py_type(
        value_dict: dict[str, Any] | None = None,
        value: Any = None,
        value_type: str | None = None) -> Any:
    """
    Convert a TypeDB value representation to a native Python type.

    :param value_dict: Normalised TypeDB value dict.
    :param value: Raw scalar/array value to convert.
    :param value_type: TypeDB value type label (e.g. ``long``, ``datetime``).
    :return: Converted Python value.
    :raises ValueError: If ``value_dict`` is missing required keys.
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
    if value_type is None:
        return value

    converters = {
        'datetime': datetime.fromisoformat,
        'long': int,
        'string': str,
        'double': float,
    }
    array_converters = {
        'long_array': int,
        'double_array': float,
        'string_array': str,
    }
    if value_type == 'boolean':
        if isinstance(value, str):
            return value.lower() == 'true'
        return bool(value)
    converter = converters.get(value_type)
    if converter is not None:
        return converter(value)
    array_converter = array_converters.get(value_type)
    if array_converter is not None:
        return [array_converter(v) for v in value]
    return value


def convert_py_type_to_query_type(data: Any) -> str:
    """
    Convert a Python value to a properly formatted TypeQL string.

    :param data: Python value to serialise into a TypeQL literal/variable.
    :return: TypeQL-ready string representation.
    """
    if isinstance(data, str):
        if len(data) > 0 and data[0] != '$':
            escaped_data = data.replace('\\', '\\\\').replace("'", "\\'")
            return "'{}'".format(escaped_data)
        return data
    elif isinstance(data, datetime):
        return data.isoformat(timespec='milliseconds')
    elif isinstance(data, bool):
        return str(data).lower()
    return str(data)


def attribute_dict_to_query(attribute_dict: dict[str, Any]) -> str:
    """
    Convert an attribute dict to a TypeQL 'has' clause fragment.

    :param attribute_dict: Mapping of attribute labels to scalar/list values.
    :return: ``has`` clause fragment prefixed with a space when non-empty.
    """
    fragments = []
    for attr, attr_value in attribute_dict.items():
        if not isinstance(attr_value, list):
            attr_value = [attr_value]
        for v in attr_value:
            fragments.append(
                'has {0} {1}'.format(attr, convert_py_type_to_query_type(v))
            )
    if not fragments:
        return ''
    return ' ' + ', '.join(fragments)


def create_match_query(
        things_list: list[ThingMatchTuple],
        prefix: str = 't') -> tuple[str, list[str]]:
    """
    Build a match clause from a list of (type, attr, value) tuples.

    :param things_list: Sequence of ``(thing_type, attr_name, attr_value)`` tuples.
    :param prefix: Prefix used to generate TypeQL variable names.
    :return: Tuple with match fragment and generated variable names.
    """
    fragments = []
    prefix_list = []
    for index, (thing_type, attr_name, attr_value) in enumerate(things_list):
        var_name = '{0}_{1}'.format(prefix, index)
        fragments.append(
            ' ${0} isa {1}, has {2} {3};'.format(
                var_name,
                thing_type,
                attr_name,
                convert_py_type_to_query_type(attr_value))
        )
        prefix_list.append(var_name)
    return ''.join(fragments), prefix_list


def create_relationship_query(
        relationship: str,
        related_dict: RelatedVariablesDict,
        attribute_list: list[AttributePair] | None = None) -> str:
    """
    Build a TypeQL insert clause for a relationship.

    :param relationship: Relationship type label.
    :param related_dict: Role-to-variable-name mapping.
    :param attribute_list: Optional relationship attribute tuples.
    :return: Relationship insert fragment ending with ``;``.
    """
    if attribute_list is None:
        attribute_list = []

    related_things = ','.join(
        '{0}:${1}'.format(role, variable_name)
        for role, variables in related_dict.items()
        for variable_name in variables
    )
    query = ' ({0}) isa {1}'.format(related_things, relationship)
    for attr_name, attr_value in attribute_list:
        query += ', has {} {}'.format(
            attr_name, convert_py_type_to_query_type(attr_value))
    query += ';'
    return query


def dict_to_query(data_dict: dict[str, Any]) -> str:
    """
    Convert a dict of attributes to a TypeQL 'has' clause fragment.

    This is a simple alias for :func:`attribute_dict_to_query`.

    :param data_dict: dict mapping attribute names to values.
    :return: TypeQL string fragment.
    """
    return attribute_dict_to_query(data_dict)
