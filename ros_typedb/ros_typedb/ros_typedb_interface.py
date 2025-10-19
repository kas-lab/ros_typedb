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

import rcl_interfaces
import ros_typedb_msgs

from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.msg import ParameterType
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

from ros_typedb.typedb_interface import MatchResultDict
from ros_typedb.typedb_interface import TypeDBInterface
from ros_typedb.typedb_interface import convert_query_type_to_py_type

from ros_typedb_msgs.msg import Attribute
from ros_typedb_msgs.msg import IndexList
from ros_typedb_msgs.msg import QueryResult
from ros_typedb_msgs.msg import ResultTree
from ros_typedb_msgs.msg import Thing
from ros_typedb_msgs.srv import Query

from std_msgs.msg import String

import std_srvs
from std_srvs.srv import Empty

from typing import Any
from typing import Dict
from typing import Literal
from typing import List
from typing import Optional
from typing import Union

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
}

_TYPEDB_ROOT_TYPE_TO_QUERY_RESULT_TYPE = {
    'entity' : QueryResult.THING,
    'relation' : QueryResult.THING,
    'attribute' : QueryResult.ATTRIBUTE
}
_TYPEDB_ROOT_TYPE_TO_THING_TYPE = {
    'entity' : Thing.ENTITY,
    'relation' : Thing.RELATION,
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
        raise ValueError(f"Unsupported value_type: {value_type}")

    param_value.type = param_info[0]

    if len(param_info) > 2:
        value = convert_query_type_to_py_type(value=value, value_type=param_info[2])

    setattr(param_value, param_info[1], value)

    return param_value


def convert_attribute_dict_to_ros_msg(
    attr_name:str,
    attribute_value: List[Dict[str, Any]] | Dict[str, Any]
    ) -> Attribute:

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
            if current_type['value_type'] != value_type or current_type['label'] != attr_label:
                raise ValueError("Inconsistent types or labels in attribute list")

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

def fetch_result_to_ros_result_tree(json_obj, start_index = 0):
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
                    if attr_name != "type"
                ]
                query_result.thing = thing

            result_tree.results.append(query_result)
        elif isinstance(values, list):
            query_result.type = QueryResult.SUB_QUERY
            query_result.sub_query_name = key

            children_results = list()
            for value_dict in values:
                child_result_tree, child_last_index = fetch_result_to_ros_result_tree(value_dict, index)
                sub_tree_index_list = IndexList()
                sub_tree_index_list.index = list(range(index, child_last_index))
                index = child_last_index
                query_result.children_index.append(sub_tree_index_list)
                children_results.extend(child_result_tree.results)

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


class ROSTypeDBInterface(Node):
    """ROS lifecycle node to interact with typedb."""

    def __init__(self, node_name: str, **kwargs):
        """Create ROSTypeDBInterface node, inherits from lifecycle node."""
        super().__init__(node_name, **kwargs)
        self.declare_parameter('address', 'localhost:1729')
        self.declare_parameter('database_name', 'ros_typedb')
        self.declare_parameter('force_database', True)
        self.declare_parameter('force_data', True)
        self.declare_parameter('infer', True)

        self.default_schema_path = ''
        self.declare_parameter('schema_path', [''])
        self.declare_parameter('data_path', [''])

        self.typedb_interface_class = TypeDBInterface

        self.query_cb_group = MutuallyExclusiveCallbackGroup()

    def init_typedb_interface(
            self,
            address: str,
            database_name: str,
            schema_path: Optional[list[str] | str] = None,
            data_path: Optional[list[str] | str] = None,
            force_database: Optional[bool] = False,
            force_data: Optional[bool] = False,
            infer: Optional[bool] = False) -> None:
        """
        Initialize self.typedb_interface.

        :param address: TypeDB server address.
        :param database_name: database name.
        :param schema_path: list with paths to schema files (.tql).
        :param data_path: list with paths to data files (.tql).
        :param force_database: if database should override an existing database
        :param force_data: if the database data should be overriden.
        :param infer: if inference engine should be used.
        """
        self.typedb_interface = self.typedb_interface_class(
            address,
            database_name,
            schema_path,
            data_path,
            force_database,
            force_data,
            infer
        )

        self.typedb_interface.insert_data_event = self.insert_data_event
        self.typedb_interface.delete_data_event = self.delete_data_event

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

        self.init_typedb_interface(
            address=self.get_parameter('address').value,
            database_name=self.get_parameter('database_name').value,
            schema_path=self.get_parameter('schema_path').value,
            data_path=self.get_parameter('data_path').value,
            force_database=self.get_parameter('force_database').value,
            force_data=self.get_parameter('force_data').value,
            infer=self.get_parameter('infer').value
        )

        self.event_pub = self.create_lifecycle_publisher(
            String,
            self.get_name() + '/events',
            10,
            callback_group=ReentrantCallbackGroup())

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

        self.get_logger().info(self.get_name() + ':on_configure() completed.')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """
        Cleanup ROSTypeDBInterface when the cleanup transition is called.

        :return: transition result
        """
        self.destroy_publisher(self.event_pub)
        self.destroy_service(self.insert_query_service)

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
        else:
            self.get_logger().warning(
                'Query type {} not recognized'.format(req.query_type))
            response.success = False
            return response

        query_result = query_func(req.query)
        response = query_result_to_ros_msg(req.query_type, query_result)
        if query_result is None:
            response.success = False
        else:
            response.success = True
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
        self.typedb_interface.delete_database()
        return response