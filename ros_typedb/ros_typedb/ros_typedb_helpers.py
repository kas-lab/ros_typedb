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
"""ros_typedb_helpers - ROS conversion helpers for TypeDB query results."""

from datetime import datetime
from typing import Any
from typing import Dict
from typing import List
from typing import Literal
from typing import Union

import rcl_interfaces

from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue

from ros_typedb.typedb_interface import convert_query_type_to_py_type
from ros_typedb.typedb_interface import MatchResultDict

import ros_typedb_msgs

from ros_typedb_msgs.msg import Attribute
from ros_typedb_msgs.msg import IndexList
from ros_typedb_msgs.msg import QueryResult
from ros_typedb_msgs.msg import ResultTree
from ros_typedb_msgs.msg import Thing
from ros_typedb_msgs.srv import Query


_PARAM_TYPE_MAP = {
    'boolean': (ParameterType.PARAMETER_BOOL, 'bool_value'),
    'bool': (ParameterType.PARAMETER_BOOL, 'bool_value'),
    'long': (ParameterType.PARAMETER_INTEGER, 'integer_value', 'long'),
    'int': (ParameterType.PARAMETER_INTEGER, 'integer_value', 'long'),
    'double': (ParameterType.PARAMETER_DOUBLE, 'double_value', 'double'),
    'float': (ParameterType.PARAMETER_DOUBLE, 'double_value', 'double'),
    'string': (ParameterType.PARAMETER_STRING, 'string_value', 'string'),
    'str': (ParameterType.PARAMETER_STRING, 'string_value', 'string'),
    'datetime': (ParameterType.PARAMETER_STRING, 'string_value', 'string'),
    'boolean_array': (ParameterType.PARAMETER_BOOL_ARRAY, 'bool_array_value'),
    'long_array': (
        ParameterType.PARAMETER_INTEGER_ARRAY, 'integer_array_value', 'long_array'),
    'double_array': (
        ParameterType.PARAMETER_DOUBLE_ARRAY, 'double_array_value', 'double_array'),
    'string_array': (
        ParameterType.PARAMETER_STRING_ARRAY, 'string_array_value', 'string_array'),
}

_TYPEDB_ROOT_TYPE_TO_QUERY_RESULT_TYPE = {
    'entity': QueryResult.THING,
    'relation': QueryResult.THING,
    'attribute': QueryResult.ATTRIBUTE
}
_TYPEDB_ROOT_TYPE_TO_THING_TYPE = {
    'entity': Thing.ENTITY,
    'relation': Thing.RELATION,
}


def _py_type_to_value_type(value) -> str:
    """Infer TypeDB value type string from a Python native value."""
    if isinstance(value, bool):
        return 'boolean'
    if isinstance(value, int):
        return 'long'
    if isinstance(value, float):
        return 'double'
    return 'string'


def _native_value_to_ros_attr(key: str, value) -> 'Attribute':
    """
    Convert a TypeDB3 fetch native value to :class:`ros_typedb_msgs.msg.Attribute`.

    :param key: fetch result key (used as variable_name and label).
    :param value: native Python value returned by TypeDB3 fetch.
    :return: Attribute message.
    """
    attr = Attribute()
    attr.variable_name = key
    attr.label = key
    if value is None:
        return attr
    try:
        from typedb.common.datetime import Datetime as TypeDBDatetime
        if isinstance(value, TypeDBDatetime):
            millis = value.nanos // 1_000_000
            dt = datetime(value.year, value.month, value.day,
                          value.hour, value.minute, value.second,
                          millis * 1000)
            value = dt.isoformat(timespec='milliseconds')
            attr.value = set_query_result_value(value, 'datetime')
            return attr
    except ImportError:
        pass
    if isinstance(value, datetime):
        value = value.isoformat(timespec='milliseconds')
        attr.value = set_query_result_value(value, 'datetime')
        return attr
    attr.value = set_query_result_value(value, _py_type_to_value_type(value))
    return attr


def _native_list_to_ros_attr(key: str, values: list) -> 'Attribute':
    """
    Convert a TypeDB3 multi-valued fetch list to :class:`ros_typedb_msgs.msg.Attribute`.

    :param key: fetch result key (used as variable_name and label).
    :param values: list of native Python values.
    :return: Attribute message.
    """
    attr = Attribute()
    attr.variable_name = key
    attr.label = key
    if not values:
        return attr
    first = values[0]
    if isinstance(first, bool):
        vtype = 'boolean_array'
    elif isinstance(first, int):
        vtype = 'long_array'
    elif isinstance(first, float):
        vtype = 'double_array'
    else:
        vtype = 'string_array'
    attr.value = set_query_result_value(values, vtype)
    return attr


def set_query_result_value(
    value: Union[bool, int, float, str, List[bool], List[int], List[float], List[str]],
    value_type: str
) -> rcl_interfaces.msg.ParameterValue:
    """
    Convert value to :class:`rcl_interfaces.msg.ParameterValue`.

    :param value: value to be converted
    :param value_type: value type, e.g., `boolean`, `float` etc.
    :return: converted value
    """
    param_value = ParameterValue()

    param_info = _PARAM_TYPE_MAP.get(value_type)
    if not param_info:
        raise ValueError(f'Unsupported value_type: {value_type}')

    param_value.type = param_info[0]

    if len(param_info) > 2:
        value = convert_query_type_to_py_type(
            value=value, value_type=param_info[2])

    setattr(param_value, param_info[1], value)

    return param_value


def convert_attribute_dict_to_ros_msg(
    attr_name: str,
    attribute_value: List[Dict[str, Any]] | Dict[str, Any]
) -> Attribute:
    """
    Convert a TypeDB attribute dict to :class:`ros_typedb_msgs.msg.Attribute`.

    :param attr_name: attribute variable name.
    :param attribute_value: normalised attribute dict or list of dicts.
    :return: converted Attribute message.
    """
    attr = Attribute()
    attr.variable_name = attr_name

    if isinstance(attribute_value, list):
        if not attribute_value:
            return attr

        first_type = attribute_value[0]['type']
        value_type = first_type['value_type']
        attr_label = first_type['label']

        value_list = []

        for value in attribute_value:
            current_type = value['type']
            if current_type['value_type'] != value_type or current_type['label'] != attr_label:
                raise ValueError(
                    'Inconsistent types or labels in attribute list')

            value_list.append(value['value'])

        attr.value = set_query_result_value(value_list, value_type + '_array')
        attr.label = attr_label
        return attr

    attr.label = attribute_value['type']['label']
    if 'value' in attribute_value:
        attr.value = set_query_result_value(
            attribute_value['value'],
            attribute_value['type']['value_type']
        )

    return attr


def fetch_result_to_ros_result_tree(json_obj, start_index=0):
    """
    Convert a TypeDB fetch result dict to a :class:`ros_typedb_msgs.msg.ResultTree`.

    :param json_obj: single fetch result dict.
    :param start_index: starting index for result indexing.
    :return: tuple of (ResultTree, next_index).
    """
    result_tree = ResultTree()
    index = start_index

    for key, values in json_obj.items():
        query_result = QueryResult()
        query_result.result_index = index
        index += 1

        if isinstance(values, dict) and 'type' in values:
            result_type_info = values['type']
            result_type = result_type_info['root']
            result_label = result_type_info['label']

            query_result.type = _TYPEDB_ROOT_TYPE_TO_QUERY_RESULT_TYPE[result_type]

            if result_type == 'attribute':
                attr = convert_attribute_dict_to_ros_msg(key, values)
                query_result.attribute = attr
            else:
                thing = Thing()
                thing.type = _TYPEDB_ROOT_TYPE_TO_THING_TYPE[result_type]
                thing.variable_name = key
                thing.type_name = result_label
                thing.attributes = [
                    convert_attribute_dict_to_ros_msg(attr_name, attr_result_list)
                    for attr_name, attr_result_list in values.items()
                    if attr_name != 'type'
                ]
                query_result.thing = thing

            result_tree.results.append(query_result)
        elif isinstance(values, list):
            if values and not isinstance(values[0], dict):
                # TypeDB3 multi-valued attribute: list of native Python values
                query_result.type = QueryResult.ATTRIBUTE
                query_result.attribute = _native_list_to_ros_attr(key, values)
                result_tree.results.append(query_result)
            else:
                # TypeDB2 sub-query: list of result dicts (or empty list)
                query_result.type = QueryResult.SUB_QUERY
                query_result.sub_query_name = key

                children_results = []
                for value_dict in values:
                    child_result_tree, child_last_index = fetch_result_to_ros_result_tree(
                        value_dict, index)
                    sub_tree_index_list = IndexList()
                    sub_tree_index_list.index = list(
                        range(index, child_last_index))
                    index = child_last_index
                    query_result.children_index.append(sub_tree_index_list)
                    children_results.extend(child_result_tree.results)

                result_tree.results.append(query_result)
                result_tree.results.extend(children_results)
        else:
            # TypeDB3 single native value (int, float, str, bool, datetime)
            query_result.type = QueryResult.ATTRIBUTE
            query_result.attribute = _native_value_to_ros_attr(key, values)
            result_tree.results.append(query_result)

    return result_tree, index


def fetch_query_result_to_ros_msg(
    query_result: list[dict[str, MatchResultDict]] | None
) -> ros_typedb_msgs.srv.Query.Response:
    """
    Convert typedb fetch query result to :class:`ros_typedb_msgs.srv.Query`.

    :param query_result: typedb fetch query result.
    :return: converted query response.
    """
    response = Query.Response()

    if query_result is None:
        return response

    for result in query_result:
        result_tree, _ = fetch_result_to_ros_result_tree(result)
        response.results.append(result_tree)
    response.success = True
    return response


def get_query_result_to_ros_msg(
    query_result: int | float | None
) -> ros_typedb_msgs.srv.Query.Response:
    """
    Convert get query result to :class:`ros_typedb_msgs.srv.Query`.

    :param query_result: typedb get query result.
    :return: converted query response.
    """
    response = Query.Response()

    if query_result is None:
        return response

    for result in query_result:
        result_tree = ResultTree()
        for var_name, attr_dict in result.items():
            if not isinstance(attr_dict, dict):
                continue
            query_result_ros = QueryResult()
            query_result_ros.type = QueryResult.ATTRIBUTE
            attr = convert_attribute_dict_to_ros_msg(var_name, attr_dict)
            query_result_ros.attribute = attr
            result_tree.results.append(query_result_ros)
        response.results.append(result_tree)
    response.success = True
    return response


def get_aggregate_query_result_to_ros_msg(
    query_result: int | float | None
) -> ros_typedb_msgs.srv.Query.Response:
    """
    Convert get aggregate query result to :class:`ros_typedb_msgs.srv.Query`.

    :param query_result: typedb get aggregate query result.
    :return: converted query response.
    """
    response = Query.Response()
    attr = Attribute()
    attr.value = set_query_result_value(
        query_result,
        type(query_result).__name__)

    query_result_ros_msg = QueryResult()
    query_result_ros_msg.type = QueryResult.ATTRIBUTE
    query_result_ros_msg.attribute = attr
    result_tree = ResultTree()
    result_tree.results.append(query_result_ros_msg)
    response.results.append(result_tree)
    return response


def query_result_to_ros_msg(
    query_type: Literal[2, 3, 4],
    query_result: list[dict[str, MatchResultDict]] | int | float | None
) -> ros_typedb_msgs.srv.Query.Response:
    """
    Convert typedb query result to :class:`ros_typedb_msgs.srv.Query`.

    :param query_type: query_type, e.g., 'fetch' or 'get_aggregate'
    :param query_result: typedb query result.
    :return: converted query response.
    """
    response = Query.Response()
    if query_type == Query.Request.FETCH:
        response = fetch_query_result_to_ros_msg(query_result)
    elif query_type == Query.Request.GET:
        response = get_query_result_to_ros_msg(query_result)
    elif query_type == Query.Request.GET_AGGREGATE:
        response = get_aggregate_query_result_to_ros_msg(query_result)
    return response
