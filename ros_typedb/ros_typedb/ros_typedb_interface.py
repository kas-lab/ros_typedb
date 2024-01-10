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
from ros_typedb_msgs.msg import Attribute
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
        'long': (2, 'integer_value'),
        'int': (2, 'integer_value'),
        'double': (3, 'double_value'),
        'float': (3, 'double_value'),
        'string': (4, 'string_value'),
        'str': (4, 'string_value'),
        'datetime': (4, 'string_value'),
        'boolean_array': (6, 'bool_array_value'),
        'long_array': (7, 'integer_array_value'),
        'double_array': (8, 'double_array_value'),
        'string_array': (9, 'string_array_value'),
    }
    if value_type in _type_dict:
        _param_value.type = _type_dict[value_type][0]
        setattr(
            _param_value,
            _type_dict[value_type][1],
            value
        )
    return _param_value


def match_query_result_to_ros_msg(
    query_result: list[dict[str, MatchResultDict]] | None
) -> ros_typedb_msgs.srv.Query.Response:
    """
    Convert typedb match query result to :class:`ros_typedb_msgs.srv.Query`.

    :param query_result: typedb match query result.
    :return: converted query response.
    """
    response = Query.Response()
    for result in query_result:
        for key, value_dict in result.items():
            _attr = Attribute()
            _attr.name = key
            if 'type' in value_dict:
                _attr.type = value_dict['type']
            if 'value' in value_dict:
                _attr.value = set_query_result_value(
                    value_dict['value'],
                    value_dict['value_type'])
            response.attributes.append(_attr)
    return response


def match_aggregate_query_result_to_ros_msg(
        query_result: int | float | None) -> ros_typedb_msgs.srv.Query.Response:
    """
    Convert match aggregate query result to :class:`ros_typedb_msgs.srv.Query`.

    :param query_result: typedb match aggreate query result.
    :return: converted query response.
    """
    response = Query.Response()
    _attr = Attribute()
    _attr.value = set_query_result_value(
        query_result,
        type(query_result).__name__)
    response.attributes.append(_attr)
    return response


def query_result_to_ros_msg(
    query_type: Literal['match', 'match_aggregate'],
    query_result: list[dict[str, MatchResultDict]] | int | float | None
) -> ros_typedb_msgs.srv.Query.Response:
    """
    Convert typedb query result to :class:`ros_typedb_msgs.srv.Query`.

    :param query_type: query_type, e.g., 'match' or 'match_aggregate'
    :param query_result: typedb  query result.
    :return: converted query response.
    """
    response = Query.Response()
    if query_type == 'match':
        response = match_query_result_to_ros_msg(query_result)
    elif query_type == 'match_aggregate':
        response = match_aggregate_query_result_to_ros_msg(query_result)
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

        self.get_logger().info(self.get_name() + ' :on_configure() completed.')
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
        elif req.query_type == 'match':
            query_func = self.typedb_interface.match_database
        elif req.query_type == 'match_aggregate':
            query_func = self.typedb_interface.match_aggregate_database
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
