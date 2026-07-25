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
"""ros_typedb_interface - python interface to interact with typedb via ROS."""

from functools import wraps
import threading
import traceback

from typing import Any
from typing import Dict
from typing import List
from typing import Literal
from typing import Optional
from typing import Union

import rcl_interfaces

from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

from ros_typedb.typedb_interface import convert_query_type_to_py_type
from ros_typedb.typedb_interface import MatchResultDict
from ros_typedb.typedb_interface import TypeDBInterface

import ros_typedb_msgs

from ros_typedb_msgs.msg import Attribute
from ros_typedb_msgs.msg import IndexList
from ros_typedb_msgs.msg import QueryResult
from ros_typedb_msgs.msg import ResultTree
from ros_typedb_msgs.msg import Thing
from ros_typedb_msgs.srv import Query

from std_msgs.msg import String

import std_srvs
from std_srvs.srv import Empty


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
    'long_array': (ParameterType.PARAMETER_INTEGER_ARRAY, 'integer_array_value', 'long_array'),
    'double_array': (ParameterType.PARAMETER_DOUBLE_ARRAY, 'double_array_value', 'double_array'),
    'string_array': (ParameterType.PARAMETER_STRING_ARRAY, 'string_array_value', 'string_array'),
    'datetime_array': (
        ParameterType.PARAMETER_STRING_ARRAY,
        'string_array_value',
        'string_array'),
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
    Convert one concrete TypeDB attribute fetch result to a ROS attribute.

    ``attribute_value`` may be a single attribute dict or a homogeneous list of
    attribute dicts. Homogeneous means every item has the same concrete
    attribute label and value type. Wildcard fetch results such as
    ``fetch $x: attribute;`` must be grouped by concrete attribute type before
    calling this function.

    Example single attribute shape from ``fetch $age;``::

        {'value': 33,
         'type': {'label': 'age',
                  'root': 'attribute',
                  'value_type': 'long'}}

    Example homogeneous list shape from ``fetch $x: phone-number;``::

        [{'value': '+1-202-555-0106',
          'type': {'label': 'phone-number',
                   'root': 'attribute',
                   'value_type': 'string'}},
         {'value': '+44-1632-960007',
          'type': {'label': 'phone-number',
                   'root': 'attribute',
                   'value_type': 'string'}}]
    """
    attr = Attribute()
    attr.variable_name = attr_name

    if isinstance(attribute_value, list):
        if not attribute_value:
            return attr  # Early exit for empty list

        first_type = attribute_value[0]['type']
        value_type = first_type['value_type']
        attr_label = first_type['label']

        value_list = []

        for value in attribute_value:
            current_type = value['type']
            if (
                current_type['value_type'] != value_type or
                current_type['label'] != attr_label
            ):
                raise ValueError(
                    f"""Inconsistent types or labels in attribute list:
                    attribute name: {attr_name}
                    attribute value: {attribute_value}
                    current_type: {current_type}
                    value type: {value_type}
                    attribute label: {attr_label}""")

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


def _attribute_type_key(
    attribute_result: Dict[str, Any]
) -> tuple[str, str]:
    """Return the concrete attribute label and value type for grouping."""
    attr_type = attribute_result['type']
    return attr_type['label'], attr_type['value_type']


def _group_wildcard_attribute_results(
    attribute_results: List[Dict[str, Any]]
) -> Dict[tuple[str, str], List[Dict[str, Any]]]:
    """
    Group ``fetch $x: attribute;`` results by concrete attribute type.

    TypeDB places all wildcard-owned attributes under an ``attribute`` key, so
    one list can contain ``age`` longs, ``nickname`` strings, datetimes, etc.
    The ordinary attribute converter expects homogeneous lists, so this groups
    the wildcard list into homogeneous lists keyed by ``(label, value_type)``.
    """
    grouped_attribute_values = {}
    for attr_result in attribute_results:
        attr_key = _attribute_type_key(attr_result)
        grouped_attribute_values.setdefault(attr_key, []).append(attr_result)
    return grouped_attribute_values


def _iter_thing_attribute_groups(
    thing_result: Dict[str, Any]
):
    """
    Yield normalized ``(variable_name, attribute_result)`` pairs for a thing.

    TypeDB fetch returns two different shapes for thing attributes:

    * explicit fetches, e.g. ``fetch $x: email;``, are keyed by the concrete
      attribute label (``email``) and already contain homogeneous lists;
    * wildcard fetches, e.g. ``fetch $x: attribute;``, are keyed as
      ``attribute`` and may contain mixed concrete labels and value types.

    This iterator hides that difference from the ROS message converter.

    Example explicit fetch shape::

        {'type': {'label': 'person', 'root': 'entity'},
         'email': [{'value': 'test@test.com',
                    'type': {'label': 'email',
                             'root': 'attribute',
                             'value_type': 'string'}}]}

    Example wildcard fetch shape::

        {'type': {'label': 'person', 'root': 'entity'},
         'attribute': [
             {'value': 'test@test.com',
              'type': {'label': 'email',
                       'root': 'attribute',
                       'value_type': 'string'}},
             {'value': 33,
              'type': {'label': 'age',
                       'root': 'attribute',
                       'value_type': 'long'}}]}
    """
    for attr_name, attr_result in thing_result.items():
        if attr_name == 'type':
            continue

        if attr_name != 'attribute':
            yield attr_name, attr_result
            continue

        # A wildcard attribute may still be represented as one concrete
        # attribute dict. Normalize its variable name to the concrete label.
        if isinstance(attr_result, dict):
            attr_label, _ = _attribute_type_key(attr_result)
            yield attr_label, attr_result
            continue

        # The common wildcard shape is a mixed list. Split it before conversion
        # so each ROS Attribute contains only one concrete TypeDB attribute.
        for attr_key, attr_group in _group_wildcard_attribute_results(
                attr_result).items():
            attr_label, _ = attr_key
            yield attr_label, attr_group


def convert_thing_attribute_dicts_to_ros_msgs(
    attribute_values: Dict[str, Any]
) -> List[Attribute]:
    """
    Convert TypeDB fetch thing attributes to ROS attributes.

    This function accepts a full entity/relation fetch result dict, including
    its ``type`` key. Attribute shape normalization happens in
    ``_iter_thing_attribute_groups``; this function only maps normalized groups
    into ROS messages.
    """
    return [
        convert_attribute_dict_to_ros_msg(attr_name, attr_result)
        for attr_name, attr_result in _iter_thing_attribute_groups(
            attribute_values)
    ]


def _is_typedb_concept_fetch_result(values: Any) -> bool:
    """Return true for fetch results that describe a TypeDB concept."""
    return isinstance(values, dict) and 'type' in values


def _is_subquery_fetch_result(values: Any) -> bool:
    """Return true for fetch results that contain nested fetch results."""
    return isinstance(values, list)


def _concept_fetch_result_to_ros_msg(
    variable_name: str,
    values: Dict[str, Any],
    result_index: int
) -> QueryResult:
    """
    Convert an attribute/entity/relation fetch result to a QueryResult.

    TypeDB fetch result dicts use a ``type.root`` field to distinguish
    top-level attributes from things. Top-level attributes become
    ``QueryResult.ATTRIBUTE``. Entities and relations become
    ``QueryResult.THING`` with their owned attributes attached.
    """
    result_type_info = values['type']
    result_type = result_type_info['root']
    result_label = result_type_info['label']

    query_result = QueryResult()
    query_result.result_index = result_index
    query_result.type = _TYPEDB_ROOT_TYPE_TO_QUERY_RESULT_TYPE[result_type]

    if result_type == 'attribute':
        query_result.attribute = convert_attribute_dict_to_ros_msg(
            variable_name, values)
        return query_result

    thing = Thing()
    thing.type = _TYPEDB_ROOT_TYPE_TO_THING_TYPE[result_type]
    thing.variable_name = variable_name
    thing.type_name = result_label
    thing.attributes = convert_thing_attribute_dicts_to_ros_msgs(values)
    query_result.thing = thing
    return query_result


def _subquery_fetch_result_to_ros_msg(
    subquery_name: str,
    nested_results: List[Dict[str, Any]],
    result_index: int
) -> tuple[QueryResult, List[QueryResult], int]:
    """
    Convert a nested fetch result and return its flattened children.

    ROS result trees store subquery children in a flat ``results`` list.
    ``children_index`` stores the index ranges for each nested match returned by
    TypeDB. The returned integer is the next unused result index.
    """
    query_result = QueryResult()
    query_result.result_index = result_index
    query_result.type = QueryResult.SUB_QUERY
    query_result.sub_query_name = subquery_name

    next_index = result_index + 1
    children_results = []
    for value_dict in nested_results:
        child_result_tree, child_last_index = fetch_result_to_ros_result_tree(
            value_dict, next_index)
        sub_tree_index_list = IndexList()
        # The child tree uses a contiguous index range starting at next_index.
        sub_tree_index_list.index = list(range(next_index, child_last_index))
        next_index = child_last_index
        query_result.children_index.append(sub_tree_index_list)
        children_results.extend(child_result_tree.results)

    return query_result, children_results, next_index


def fetch_result_to_ros_result_tree(json_obj, start_index=0):
    """
    Convert one TypeDB fetch result dict to a flattened ROS result tree.

    ``json_obj`` is one item from the TypeDB fetch response list. Each key is
    either a fetched variable/thing name or a subquery name. Concept results are
    appended directly; subquery results are appended with their descendants
    flattened immediately after the subquery node.
    """
    result_tree = ResultTree()
    index = start_index

    for key, values in json_obj.items():
        if _is_typedb_concept_fetch_result(values):
            query_result = _concept_fetch_result_to_ros_msg(
                key, values, index)
            result_tree.results.append(query_result)
            index += 1
            continue

        if _is_subquery_fetch_result(values):
            query_result, children_results, index = (
                _subquery_fetch_result_to_ros_msg(key, values, index))
            result_tree.results.append(query_result)
            result_tree.results.extend(children_results)

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

    :param query_result: typedb get aggreate query result.
    :return: converted query response.
    """
    response = Query.Response()

    if query_result is None:
        return response

    for result in query_result:
        result_tree = ResultTree()
        variables = result.variables()
        for variable in variables:
            variable_value = result.get(variable)
            if variable_value.is_attribute():
                typedb_attr = variable_value.as_attribute()

                query_result_ros = QueryResult()
                query_result_ros.type = QueryResult.ATTRIBUTE

                attr = Attribute()
                attr.variable_name = variable
                attr.label = typedb_attr.get_type().get_label().name
                attr.value = set_query_result_value(
                    typedb_attr.get_value(),
                    str(typedb_attr.get_type().get_value_type()))
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

    :param query_result: typedb get aggreate query result.
    :return: converted query response.
    """
    response = Query.Response()

    if query_result is None:
        return response

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
    response.success = True
    return response


def query_result_to_ros_msg(
    query_type: Literal[2, 3, 4],
    query_result: list[dict[str, MatchResultDict]] | int | float | None
) -> ros_typedb_msgs.srv.Query.Response:
    """
    Convert typedb query result to :class:`ros_typedb_msgs.srv.Query`.

    :param query_type: query_type, e.g., 'fetch' or 'get_aggregate'
    :param query_result: typedb  query result.
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


def lifecycle_state_is_active(node):
    """Return True when the lifecycle node is in the active state."""
    return node._state_machine.current_state[1] == 'active'


def when_lifecycle_active(func):
    """Decorate a method to run only when its lifecycle node is active."""
    @wraps(func)
    def inner(self, *args, **kwargs):
        if lifecycle_state_is_active(self):
            return func(self, *args, **kwargs)
    return inner


class ROSTypeDBInterface(Node):
    """ROS lifecycle node to interact with typedb."""

    def __init__(self, node_name: str, **kwargs):
        """Create ROSTypeDBInterface node, inherits from lifecycle node."""
        super().__init__(node_name, **kwargs)
        self.declare_parameter('address', 'localhost:1729')
        self.declare_parameter('database_name', 'ros_typedb')
        self.declare_parameter('force_database', True)
        self.declare_parameter('force_data', True)
        self.declare_parameter('reload_schema', True)
        self.declare_parameter('infer', True)
        self.declare_parameter('driver_timeout_s', 10.0)
        self.declare_parameter('query_timeout_s', 0.0)

        self.default_schema_path = ''
        self.declare_parameter('schema_path', [''])
        self.declare_parameter('data_path', [''])

        self.declare_parameter('sort_fetch_results', False)

        self.typedb_interface_class = TypeDBInterface

        self.query_cb_group = MutuallyExclusiveCallbackGroup()
        self._service_callback_lock = threading.Lock()

        self.query_service = self.create_service(
            Query,
            self.get_name() + '/query',
            self.query_service_cb,
            callback_group=self.query_cb_group)

        self.delete_db_service = self.create_service(
            Empty,
            self.get_name() + '/delete_database',
            self.delete_db_cb,
            callback_group=self.query_cb_group)

    def _get_service_callback_lock(self):
        """Return the lock used to serialize service callbacks with cleanup."""
        lock = getattr(self, '_service_callback_lock', None)
        if lock is None:
            lock = threading.Lock()
            self._service_callback_lock = lock
        return lock

    def init_typedb_interface(
            self,
            address: str,
            database_name: str,
            schema_path: Optional[list[str] | str] = None,
            data_path: Optional[list[str] | str] = None,
            force_database: Optional[bool] = False,
            force_data: Optional[bool] = False,
            reload_schema: Optional[bool] = True,
            infer: Optional[bool] = False,
            sort_fetch_results: Optional[bool] = False,
            driver_timeout_s: Optional[float] = 10.0,
            query_timeout_s: Optional[float] = None) -> None:
        """
        Initialize self.typedb_interface.

        :param address: TypeDB server address.
        :param database_name: database name.
        :param schema_path: list with paths to schema files (.tql).
        :param data_path: list with paths to data files (.tql).
        :param force_database: if database should override an existing database
        :param force_data: if the database data should be overriden.
        :param reload_schema: if schema files should be reapplied when the
            database already exists.
        :param infer: if inference engine should be used.
        :param sort_fetch_results: if fetch query results should be
            recursively sorted.
        :param driver_timeout_s: seconds to wait for driver connection before
            timing out.
        :param query_timeout_s: default per-query timeout in seconds.
            None means no limit.
        """
        self.typedb_interface = self.typedb_interface_class(
            address,
            database_name,
            schema_path,
            data_path,
            force_database,
            force_data,
            infer,
            sort_fetch_results,
            driver_timeout_s,
            query_timeout_s=query_timeout_s,
            reload_schema=reload_schema
        )

        self.typedb_interface.insert_data_event = self.insert_data_event
        self.typedb_interface.delete_data_event = self.delete_data_event

    def close_typedb_interface(self) -> None:
        """Close the TypeDB driver if the interface was initialized."""
        typedb_interface = getattr(self, 'typedb_interface', None)
        driver = getattr(typedb_interface, 'driver', None)
        if driver is not None:
            driver.close()
        if typedb_interface is not None:
            self.typedb_interface = None

    @when_lifecycle_active
    def publish_data_event(self, event_type: str) -> None:
        """
        Publish message in the `/event` topic.

        :param event_type: event to be published, e.g., 'insert' or 'delete'.
        """
        self.event_pub.publish(String(data=event_type))

    def insert_data_event(self) -> None:
        """Publish 'insert' in the /event topic."""
        self.publish_data_event('insert')

    def delete_data_event(self) -> None:
        """Publish 'delete' in the /event topic."""
        self.publish_data_event('delete')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """
        Configure ROSTypeDBInterface when the configure transition is called.

        :return: transition result
        """
        self.get_logger().info(self.get_name() + ': on_configure() is called.')

        try:
            self.get_logger().info(
                self.get_name() + ': initializing TypeDB interface.')
            self.init_typedb_interface(
                address=self.get_parameter('address').value,
                database_name=self.get_parameter('database_name').value,
                schema_path=self.get_parameter('schema_path').value,
                data_path=self.get_parameter('data_path').value,
                force_database=self.get_parameter('force_database').value,
                force_data=self.get_parameter('force_data').value,
                reload_schema=self.get_parameter('reload_schema').value,
                infer=self.get_parameter('infer').value,
                sort_fetch_results=(
                    self.get_parameter('sort_fetch_results').value),
                driver_timeout_s=self.get_parameter('driver_timeout_s').value,
                query_timeout_s=(
                    self.get_parameter('query_timeout_s').value or None)
            )

            self.event_pub = self.create_lifecycle_publisher(
                String,
                self.get_name() + '/events',
                10,
                callback_group=ReentrantCallbackGroup())
        except Exception as exc:
            try:
                self.close_typedb_interface()
            except Exception:
                self.get_logger().error(
                    self.get_name() +
                    ': failed to close TypeDB interface after configure '
                    'failure:\n' + traceback.format_exc())
            self.get_logger().error(
                self.get_name() + ': on_configure() failed: ' + str(exc) +
                '\n' + traceback.format_exc())
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info(self.get_name() + ':on_configure() completed.')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """
        Cleanup ROSTypeDBInterface when the cleanup transition is called.

        :return: transition result
        """
        service_callback_lock = self._get_service_callback_lock()
        with service_callback_lock:
            if getattr(self, 'event_pub', None) is not None:
                self.destroy_publisher(self.event_pub)
                del self.event_pub

            self.close_typedb_interface()

        self.get_logger().info(self.get_name() + ' :on_cleanup() is called.')
        return TransitionCallbackReturn.SUCCESS

    def query_service_cb(
        self,
        req: ros_typedb_msgs.srv.Query.Request,
        response: ros_typedb_msgs.srv.Query.Response
    ) -> ros_typedb_msgs.srv.Query.Response:
        """
        Handle callback for ~/query service.

        Perform the query requested with the ~/query service.

        :param req: query to be performed
        :param response: query result
        :return: query result
        """
        service_callback_lock = self._get_service_callback_lock()
        with service_callback_lock:
            if not lifecycle_state_is_active(self):
                response.success = False
                response.error_message = 'Node is not active'
                return response
            if getattr(self, 'typedb_interface', None) is None:
                response.success = False
                response.error_message = 'Node is not configured'
                return response
            if req.query_type == Query.Request.INSERT:
                query_func = self.typedb_interface.insert_database
            elif req.query_type == Query.Request.DELETE:
                query_func = self.typedb_interface.delete_from_database
            elif req.query_type == Query.Request.FETCH:
                query_func = self.typedb_interface.fetch_database
            elif req.query_type == Query.Request.GET:
                query_func = self.typedb_interface.get_database
            elif req.query_type == Query.Request.GET_AGGREGATE:
                query_func = self.typedb_interface.get_aggregate_database
            elif req.query_type == Query.Request.UPDATE:
                query_func = self.typedb_interface.update_database
            elif req.query_type == Query.Request.DEFINE:
                query_func = self.typedb_interface.define_database
            else:
                self.get_logger().warning(
                    'Query type {} not recognized'.format(req.query_type))
                response.success = False
                response.error_message = (
                    f'Unknown query type: {req.query_type}')
                return response

            per_call_timeout = req.timeout_s if req.timeout_s > 0 else None
            try:
                query_result = query_func(req.query, timeout=per_call_timeout)
                response = query_result_to_ros_msg(
                    req.query_type, query_result)
                if query_result is None:
                    response.success = False
                    response.error_message = self.typedb_interface.last_error
                else:
                    response.success = True
                return response
            except Exception as exc:
                try:
                    self.get_logger().error(
                        'Query service failed: {}\n{}'.format(
                            exc, traceback.format_exc()))
                except Exception:
                    pass
                response.success = False
                response.error_message = str(exc)
                return response

    def delete_db_cb(
        self,
        req: std_srvs.srv.Empty.Request,
        response: std_srvs.srv.Empty.Response
    ) -> std_srvs.srv.Empty.Response:
        """
        Handle callback for ~/delete_database service.

        Delete the dabase.
        """
        service_callback_lock = self._get_service_callback_lock()
        with service_callback_lock:
            if not lifecycle_state_is_active(self):
                return response
            if getattr(self, 'typedb_interface', None) is None:
                return response
            self.typedb_interface.delete_database()
        return response
