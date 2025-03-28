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
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

from ros_typedb.typedb_interface import MatchResultDict
from ros_typedb.typedb_interface import TypeDBInterface
from ros_typedb.typedb_interface import convert_query_type_to_py_type
from ros_typedb_msgs.msg import Attribute
from ros_typedb_msgs.msg import QueryResult
from ros_typedb_msgs.srv import Query

from std_msgs.msg import String

from typing import Literal
from typing import Optional


def set_query_result_value(
    value: bool | int | float | str | list[bool] | list[int] | list[float]
    | list[str],
        value_type: str) -> rcl_interfaces.msg.ParameterValue:
    """
    Convert value to :class:`rcl_interfaces.msg.ParameterValue`.

    :param value: value to be converted
    :param value_type: value type, e.g., `boolean`, `float` etc.
    :return: converted value
    """
    _param_value = ParameterValue()
    _type_dict = {
        'boolean': (1, 'bool_value'),
        'bool': (1, 'bool_value'),
        'long': (2, 'integer_value', 'long'),
        'int': (2, 'integer_value', 'long'),
        'double': (3, 'double_value', 'double'),
        'float': (3, 'double_value', 'double'),
        'string': (4, 'string_value', 'string'),
        'str': (4, 'string_value', 'string'),
        'datetime': (4, 'string_value', 'string'),
        'boolean_array': (6, 'bool_array_value'),
        'long_array': (7, 'integer_array_value'),
        'double_array': (8, 'double_array_value'),
        'string_array': (9, 'string_array_value'),
    }
    if value_type in _type_dict:
        _param_value.type = _type_dict[value_type][0]
        try:
            value = convert_query_type_to_py_type(
                value=value, value_type=_type_dict[value_type][2])
        except IndexError:
            pass
        setattr(
            _param_value,
            _type_dict[value_type][1],
            value
        )
    return _param_value


def convert_attribute_dict_to_ros_msg(attr_name:str, attribute_dict: Attribute):
    _attr = Attribute()
    _attr.name = attr_name
    _attr.label = attribute_dict.get('type').get('label')
    if 'value' in attribute_dict:
        _attr.value = set_query_result_value(
            attribute_dict.get('value'),
            attribute_dict.get('type').get('value_type'))
    return _attr

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
        _result = QueryResult()
        for key, result_dict in result.items():
            # _result.variable_name = key

            result_type_info = result_dict.get('type')
            # _result.variable_type = result_type_info.get('label')
            result_type = result_type_info.get('root')
            if result_type == 'attribute':
                _result.attributes.append(convert_attribute_dict_to_ros_msg(key, result_dict))
            else:
                _result.variable_name = key
                _result.variable_type = result_type_info.get('label')
                _result.attributes = [
                    convert_attribute_dict_to_ros_msg(attr_name, attr_result_dict)
                    for attr_name, attr_result_list in result_dict.items()
                    if attr_name != "type"
                    for attr_result_dict in attr_result_list
                ]
        response.results.append(_result)
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
        _result = QueryResult()
        _variables = result.variables()
        for variable in _variables:
            variable_value = result.get(variable)
            if variable_value.is_attribute():
                _typedb_attr = variable_value.as_attribute()
                _attr = Attribute()
                _attr.name = variable
                _attr.label = _typedb_attr.get_type().get_label().name
                _attr.value = set_query_result_value(
                    _typedb_attr.get_value(),
                    str(_typedb_attr.get_type().get_value_type()))
                _result.attributes.append(_attr)
        response.results.append(_result)
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
    _attr = Attribute()
    _attr.value = set_query_result_value(
        query_result,
        type(query_result).__name__)

    _result = QueryResult()
    _result.attributes.append(_attr)
    response.results.append(_result)
    return response


def query_result_to_ros_msg(
    query_type: Literal['fetch', 'get_aggregate'],
    query_result: list[dict[str, MatchResultDict]] | int | float | None
) -> ros_typedb_msgs.srv.Query.Response:
    """
    Convert typedb query result to :class:`ros_typedb_msgs.srv.Query`.

    :param query_type: query_type, e.g., 'fetch' or 'get_aggregate'
    :param query_result: typedb  query result.
    :return: converted query response.
    """
    response = Query.Response()
    if query_type == 'fetch':
        response = fetch_query_result_to_ros_msg(query_result)
    elif query_type == 'get':
        response = get_query_result_to_ros_msg(query_result)
    elif query_type == 'get_aggregate':
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

        self.get_logger().info(f"schema: {self.get_parameter('schema_path').value} and data: {self.get_parameter('data_path').value}")

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
        if req.query_type == 'insert':
            query_func = self.typedb_interface.insert_database
        elif req.query_type == 'delete':
            query_func = self.typedb_interface.delete_from_database
        elif req.query_type == 'fetch':
            query_func = self.typedb_interface.fetch_database
        elif req.query_type == 'get':
            query_func = self.typedb_interface.get_database
        elif req.query_type == 'get_aggregate':
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
