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

from rcl_interfaces.msg import ParameterValue
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

from ros_typedb.typedb_interface import TypeDBInterface
from ros_typedb_msgs.msg import Attribute
from ros_typedb_msgs.srv import Query

from std_msgs.msg import String


def set_query_result_value(value, value_type):
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


def match_query_result_to_ros_msg(query_result):
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


def match_aggregate_query_result_to_ros_msg(query_result):
    response = Query.Response()
    _attr = Attribute()
    _attr.value = set_query_result_value(
        query_result,
        type(query_result).__name__)
    response.attributes.append(_attr)
    return response


def query_result_to_ros_msg(query_type, query_result):
    response = Query.Response()
    if query_type == 'match':
        response = match_query_result_to_ros_msg(query_result)
    elif query_type == 'match_aggregate':
        response = match_aggregate_query_result_to_ros_msg(query_result)
    return response


class ROSTypeDBInterface(Node):
    """ROS node to interact with typedb."""

    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)
        self.declare_parameter('address', 'localhost:1729')
        self.declare_parameter('database_name', 'ros_typedb')
        self.declare_parameter('force_database', True)
        self.declare_parameter('force_data', True)

        self.default_schema_path = ''
        self.declare_parameter('schema_path', [''])
        self.declare_parameter('data_path', [''])

        self.typedb_interface_class = TypeDBInterface

        self.query_cb_group = MutuallyExclusiveCallbackGroup()

    def init_typedb_interface(
            self,
            address,
            database_name,
            schema_path=None,
            data_path=None,
            force_database=False,
            force_data=False):

        self.typedb_interface = self.typedb_interface_class(
            address,
            database_name,
            schema_path,
            data_path,
            force_database,
            force_data
        )

        self.typedb_interface.insert_data_event = self.insert_data_event
        self.typedb_interface.delete_data_event = self.delete_data_event

    def publish_data_event(self, event_type):
        self.event_pub.publish(String(data=event_type))

    def insert_data_event(self):
        self.publish_data_event('insert')

    def delete_data_event(self):
        self.publish_data_event('delete')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(self.get_name() + ': on_configure() is called.')

        self.init_typedb_interface(
            address=self.get_parameter('address').value,
            database_name=self.get_parameter('database_name').value,
            schema_path=self.get_parameter('schema_path').value,
            data_path=self.get_parameter('data_path').value,
            force_database=self.get_parameter('force_database').value,
            force_data=self.get_parameter('force_data').value
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
        self.destroy_publisher(self.event_pub)
        self.destroy_service(self.insert_query_service)

        self.get_logger().info(self.get_name() + ' :on_cleanup() is called.')
        return TransitionCallbackReturn.SUCCESS

    def query_service_cb(self, req, response):
        # self.get_logger().info(
        #     '{} query requested: {}'.format(req.query_type, req.query))
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
