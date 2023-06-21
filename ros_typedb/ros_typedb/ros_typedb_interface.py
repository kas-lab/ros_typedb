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

import rclpy

from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

from rcl_interfaces.msg import ParameterValue
from ros_typedb.typedb_interface import TypeDBInterface
from ros_typedb_msgs.msg import QueryResult
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
        'datetime': (4, 'string_value')
    }
    value_type = str(value_type)
    if value_type in _type_dict:
        if value_type == 'datetime':
            value = value.strftime('%Y-%m-%d')
        _param_value.type = _type_dict[value_type][0]
        setattr(_param_value, _type_dict[value_type][1], value)
    return _param_value


class ROSTypeDBInterface(Node):
    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)

    def init_typedb_interface(
            self,
            address,
            database_name,
            schema_path=None,
            data_path=None,
            force_database=False,
            force_data=False):

        self.typedb_interface = TypeDBInterface(
            address,
            database_name,
            schema_path,
            data_path,
            force_database,
            force_data
        )

        self.typedb_interface.insert_data_event = self.insert_data_event
        self.typedb_interface.delete_data_event = self.delete_data_event

    def data_event(self, event_type):
        event = String()
        event.data = event_type
        self.event_pub.publish(event)

    def insert_data_event(self):
        self.data_event('insert')

    def delete_data_event(self):
        self.data_event('delete')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure() is called.")

        self.event_pub = self.create_lifecycle_publisher(
            String, 'typedb/events', 10)

        self.query_service = self.create_service(
            Query,
            'typedb/query',
            self.query_service_cb)

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.destroy_publisher(self.event_pub)
        self.destroy_service(self.insert_query_service)

        self.get_logger().info('on_cleanup() is called.')
        return TransitionCallbackReturn.SUCCESS

    def query_service_cb(self, req, response):
        self.get_logger().info(
            '{} query requested: {}'.format(req.query_type, req.query))
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
        if req.query_type == 'match':
            for result in query_result:
                for key, value in result.items():
                    if value.is_attribute():
                        _attr = QueryResult()
                        _attr.attribute_name = key
                        _attr.value = set_query_result_value(
                            value.get_value(),
                            value.get_type().get_value_type())
                        response.result.append(_attr)
        elif req.query_type == 'match_aggregate':
            _result = QueryResult()
            _result.value = set_query_result_value(
                query_result,
                type(query_result).__name__)
            response.result.append(_result)
        if query_result is None:
            response.success = False
        else:
            response.success = True
        return response


def main():
    rclpy.init()

    executor = rclpy.executors.MultiThreadedExecutor()
    lc_node = ROSTypeDBInterface('ros_typedb_interface')
    lc_node.init_typedb_interface(
        "localhost:1729",
        "test_database",
        force_database=True,
        schema_path='/home/gus/exp_metacontrol_ws/src/ros_typedb/ros_typedb/test/typedb_test_data/schema.tql',
        data_path='/home/gus/exp_metacontrol_ws/src/ros_typedb/ros_typedb/test/typedb_test_data/data.tql',
        force_data=True)
    executor.add_node(lc_node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        lc_node.destroy_node()


if __name__ == '__main__':
    main()
